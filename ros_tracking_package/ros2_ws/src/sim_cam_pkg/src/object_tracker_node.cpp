
// ----------------------------------------------------------------------------------
// Node: ObjectTrackerNode
// Functionality:
//   Subscribes to /object_detection and tracks detected objects over time using a
//   simple Kalman Filter-based tracker.
//   Publishes tracked object info (including ID and trajectory) to /object_tracking.
//   Resets tracking when a message is received on /tracking/reset.
//
// Features:
//   - Class-based association (only matches detections with same class)
//   - Per-object Kalman Filters for smoothing and prediction
//   - Track aging and removal for lost objects
//   - New track creation based on confidence threshold
//
// Reference: https://github.com/jvirico/kalman-tracker/blob/master/src/kalman.cpp
// ----------------------------------------------------------------------------------


#include "rclcpp/rclcpp.hpp"
#include "sim_cam_pkg/msg/detection_array.hpp"
#include "sim_cam_pkg/msg/tracked_object_array.hpp"
#include "sim_cam_pkg/msg/tracking_reset.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/header.hpp"

#include <vector>
#include <map>
#include <cmath> 
#include <opencv2/video/tracking.hpp> // For cv::KalmanFilter
#include <opencv2/core/types.hpp>  // To represent centroids, cv::Point2f 

using std::placeholders::_1;


// Internal struct used for a tracked object
struct TrackedObject { 
    int id;
    sim_cam_pkg::msg::Detection last_detection_info; // Stores latest bbox and their class, score
    std::vector<geometry_msgs::msg::Point> trajectory_history; // Holds past centroid positions
    int age_since_last_update = 0; // Frames since this track was last associated with a detection
    cv::KalmanFilter kf;           // Kalman filter instance for this track
    bool kf_ready = false;         // Flag: Whether KF is initialized
};

class ObjectTrackerNode : public rclcpp::Node
{
public:
    // Initialize node and next track ID
    ObjectTrackerNode() : Node("object_tracker_node"), next_id_counter_(1) 
    {
        // Configuring Tracking Parameters
        max_dist_ = 125.0;        // Max pixel distance for associating a detection to a track prediction
        max_unseen_frames_ = 10;   // Frames to keep track without update or being removed
        max_traj_len_ = 50;       // Max trajectory points to store
        kf_dt_ = 1.0/10.0;        // Time step for Kalman filter motion model (e.g., 10Hz)
        min_new_track_conf_ = 0.80; // Min confidence to initiate a new track
        
        // Subscribe to raw detections from Object Detector Node
        detection_subscriber_ = this->create_subscription<sim_cam_pkg::msg::DetectionArray>(
            "/object_detection", 10, std::bind(&ObjectTrackerNode::detections_received_cb, this, _1));
        // Subscribe to the reset signal publihed by simulate camera node
        reset_subscriber_ = this->create_subscription<sim_cam_pkg::msg::TrackingReset>(
            "/tracking/reset", 10, std::bind(&ObjectTrackerNode::reset_tracker_cb, this, _1));
        // Publish: currently or actively tracked objects
        tracked_objects_publisher_ = this->create_publisher<sim_cam_pkg::msg::TrackedObjectArray>(
            "/object_tracking", 10);

        RCLCPP_INFO(this->get_logger(), "Simple KF Tracker Initialized.");
    }

private:
    // Callback for reset signal
    // Clears all active tracks and resets ID counter
    void reset_tracker_cb(const sim_cam_pkg::msg::TrackingReset::SharedPtr msg) {
        if (msg->reset_triggered) {
            RCLCPP_WARN(this->get_logger(), "Tracker Resetting!");
            active_tracks_.clear(); // Remove all current tracks
            next_id_counter_ = 1;   // Reset to 1 for new episode i.e. video loop
            sim_cam_pkg::msg::TrackedObjectArray empty_msg; // Publish empty tracklets when reset is triggered to clear in window
            empty_msg.header.stamp = this->get_clock()->now();
            empty_msg.header.frame_id = last_processed_frame_id_; // Last known frame id
            tracked_objects_publisher_->publish(empty_msg);
        }
    }

    // Initializes Kalman filter for newly detected object
    // new_track: reference to TrackedObject struct to iniatialize kf
    void initialize_kf_for_track(TrackedObject& new_track, float cx, float cy) {
        // State: [x, y, vx, vy], Measurement: [x, y]
        new_track.kf = cv::KalmanFilter(4, 2, 0, CV_32F); // 4 state [x,y,vx,vy], 2 measure [x,y]
        
        // Initial state: position (cx, cy), velocity (0, 0)
        new_track.kf.statePost = (cv::Mat_<float>(4,1) << cx, cy, 0, 0);
        
        // Transition Matrix A for constant velocity model)
        cv::setIdentity(new_track.kf.transitionMatrix);
        new_track.kf.transitionMatrix.at<float>(0, 2) = static_cast<float>(kf_dt_); // x = x + vx*dt
        new_track.kf.transitionMatrix.at<float>(1, 3) = static_cast<float>(kf_dt_); // y = y+vy*dt

        // Measurement Matrix H; measure only position (x, y)
        cv::setIdentity(new_track.kf.measurementMatrix); 
        new_track.kf.measurementMatrix = cv::Mat::zeros(2, 4, CV_32F);
        new_track.kf.measurementMatrix.at<float>(0,0) = 1.0f;
        new_track.kf.measurementMatrix.at<float>(1,1) = 1.0f;

        // Noise Covariances (simple fixed values)
        cv::setIdentity(new_track.kf.processNoiseCov, cv::Scalar::all(1e-2)); // Q: Uncertainity in motion model
        cv::setIdentity(new_track.kf.measurementNoiseCov, cv::Scalar::all(1e-1)); // R: Uncertain, in detection measurement
        cv::setIdentity(new_track.kf.errorCovPost, cv::Scalar::all(1.0)); // P: Initial estimate error covariance

        new_track.kf_ready = true; // Flag: Kalman filter as intialized
    }

    // Callback to process incoming detections
    // Funtionality: Prediction, Association, Update and track management
    void detections_received_cb(const sim_cam_pkg::msg::DetectionArray::SharedPtr detections_msg) {
        last_processed_frame_id_ = detections_msg->header.frame_id;

        // Extract Centroids from current detections
        std::vector<cv::Point2f> current_det_centroids;
        for (const auto& det : detections_msg->detections) {
            current_det_centroids.push_back(cv::Point2f(
                det.box_x + det.box_width / 2.0f, det.box_y + det.box_height / 2.0f));
        }

        // Predict next state for all active tracks
        std::vector<int> track_ids_to_process; // IDS of those active tracks to predict

        std::vector<cv::Point2f> predicted_centroids; // Predicted Centroids for those track IDs
        for (auto& pair : active_tracks_) { // Predict existing tracks
            if (pair.second.kf_ready) {
                cv::Mat p = pair.second.kf.predict();
                predicted_centroids.push_back(cv::Point2f(p.at<float>(0), p.at<float>(1)));
                track_ids_to_process.push_back(pair.first); 
            }
        }

        std::vector<bool> det_is_matched(current_det_centroids.size(), false);
        // Store det_idx for each track_idx
        std::vector<int> track_assignment(track_ids_to_process.size(), -1); // track_idx -> det_idx

        // Data Association (Simple Greedy Approach)
        // Match current detections to existing track predictons
        if (!track_ids_to_process.empty() && !current_det_centroids.empty()) {
            for (size_t i = 0; i < track_ids_to_process.size(); ++i) { // For each existing track predictions
                double best_dist = max_dist_;
                int best_det_idx = -1;
                int current_track_id = track_ids_to_process[i]; // Get the ID of the current track being processed
                const std::string& existing_track_class = active_tracks_.at(current_track_id).last_detection_info.class_name;

                for (size_t j = 0; j < current_det_centroids.size(); ++j) { // For each current detection
                    // If this current detection isnt yet matched,
                    if (!det_is_matched[j]) {
                        const std::string& current_det_class = detections_msg->detections[j].class_name;
                        
                        // **** Consider matching only if their classes are same ****
                        // Otherwise: Same ID is being assigned to a different class
                        // In my case: initially ID 1: (car), then ID 2: (car), but when another class occurs it overrides like ID 1: (person) 
                        // which wasnt desirable, so this check handles it, but may not be robust to handle all cases, needs some refinement
                        if (existing_track_class != current_det_class) {
                            continue; 
                        }

                        double d = cv::norm(predicted_centroids[i] - current_det_centroids[j]);
                        // is this detection closer than previous best
                        if (d < best_dist) { 
                            best_dist = d;
                            best_det_idx = j;
                        }
                    }
                }
                // If a suitable match is found
                if (best_det_idx != -1) {
                    track_assignment[i] = best_det_idx; // asign this det. to track i 
                    det_is_matched[best_det_idx] = true; // Flag this detection as *matched*
                }
            }
        }

        // Update matched tracks and Manage Unmatched Tracks
        std::vector<int> tracks_to_remove;
        for (size_t i = 0; i < track_ids_to_process.size(); ++i) {
            int id = track_ids_to_process[i];
            TrackedObject& track = active_tracks_.at(id);
            
            if (track_assignment[i] != -1) { // if this track was Matched to a detection: Update KF
                int det_idx = track_assignment[i];

                // Measurement for Kalman filter
                cv::Mat_<float> measurement(2,1);
                measurement(0) = current_det_centroids[det_idx].x;
                measurement(1) = current_det_centroids[det_idx].y;
                track.kf.correct(measurement); // Correct KF with new measurement
                track.last_detection_info = detections_msg->detections[det_idx];// Updated with latest detection
                track.age_since_last_update = 0; // Reset age since when it was seen
            } else { // Not matched: Increment age
                track.age_since_last_update++; // Increment age of the track
            }
            
            geometry_msgs::msg::Point current_kf_pos; // Add current KF state to trajectory
            // statePost is the corected or estimated state of the track
            current_kf_pos.x = track.kf.statePost.at<float>(0);
            current_kf_pos.y = track.kf.statePost.at<float>(1);
            track.trajectory_history.push_back(current_kf_pos);
            // Keep the tracjectory length specified as above in params
            if (track.trajectory_history.size() > static_cast<size_t>(max_traj_len_)) {
                track.trajectory_history.erase(track.trajectory_history.begin());
            }

            if (track.age_since_last_update > max_unseen_frames_) {
                tracks_to_remove.push_back(id);
            }
        }

        // Remove tracks marked for delection
        for (int id : tracks_to_remove) {
            active_tracks_.erase(id);
            RCLCPP_INFO(this->get_logger(), "Removed track ID: %d", id);
        }

        // Register New Tracks for Unmatched Detecions
        for (size_t i = 0; i < detections_msg->detections.size(); ++i) {
            // not matched to any existing track
            if (!det_is_matched[i]) { 
                const auto& det = detections_msg->detections[i];
                if (det.score >= min_new_track_conf_) { 
                    TrackedObject new_track;
                    new_track.id = next_id_counter_++;
                    new_track.last_detection_info = det; // Store intial detection info
                    initialize_kf_for_track(new_track, current_det_centroids[i].x, current_det_centroids[i].y);
                    
                    // Tracjectory update
                    geometry_msgs::msg::Point initial_kf_pos; // Add first point to new trajectory
                    initial_kf_pos.x = new_track.kf.statePost.at<float>(0);
                    initial_kf_pos.y = new_track.kf.statePost.at<float>(1);
                    new_track.trajectory_history.push_back(initial_kf_pos);

                    active_tracks_[new_track.id] = new_track; // Add new_track to active tracks
                    RCLCPP_INFO(this->get_logger(), "New track ID: %d (%s)", new_track.id, det.class_name.c_str());
                }
            }
        }

        // Publish all active tracks 
        sim_cam_pkg::msg::TrackedObjectArray output_msg;
        // Header from incoming messages for sync with detections
        output_msg.header = detections_msg->header;
        for (const auto& pair : active_tracks_) {
            const TrackedObject& track = pair.second;
            sim_cam_pkg::msg::TrackedObject t_obj_msg; // Message to pubish for thi strack
            t_obj_msg.id = track.id;
            t_obj_msg.class_name = track.last_detection_info.class_name;
            t_obj_msg.score = track.last_detection_info.score;
            t_obj_msg.box_x = track.last_detection_info.box_x;
            t_obj_msg.box_y = track.last_detection_info.box_y;
            t_obj_msg.box_width = track.last_detection_info.box_width;
            t_obj_msg.box_height = track.last_detection_info.box_height;
            // Centroid from Kalman filter state
            t_obj_msg.centroid_x = track.kf.statePost.at<float>(0);
            t_obj_msg.centroid_y = track.kf.statePost.at<float>(1);
            t_obj_msg.trajectory_points = track.trajectory_history;
            output_msg.tracked_objects.push_back(t_obj_msg);
        }
        tracked_objects_publisher_->publish(output_msg);
    }

    // ROS Communication objects
    rclcpp::Subscription<sim_cam_pkg::msg::DetectionArray>::SharedPtr detection_subscriber_;
    rclcpp::Subscription<sim_cam_pkg::msg::TrackingReset>::SharedPtr reset_subscriber_;
    rclcpp::Publisher<sim_cam_pkg::msg::TrackedObjectArray>::SharedPtr tracked_objects_publisher_;

    // Internal Tracker state
    std::map<int, TrackedObject> active_tracks_; 
    int next_id_counter_;
    std::string last_processed_frame_id_ = "map"; // Default frame id for reset

    // Imp. Tracker parameters 
    double max_dist_;
    int max_unseen_frames_;
    int max_traj_len_;
    double kf_dt_;
    double min_new_track_conf_;
};

int main(int argc, char * argv[]) {
    // Initialize ROS2 
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<ObjectTrackerNode>(); // Create Object Tracker node
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown(); 
    return 0;
}