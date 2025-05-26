// ----------------------------------------------------------------------------------
// Node: VisualizationNode
// Functionality:
//   Subscribes to synchronized image, detection, and tracking topics using 
//   message_filters and overlays visualization elements (bounding boxes, track IDs,
//   trajectories) onto current image frames.
//
// Topics:
//   - Subscribed:
//       • /camera/image_raw (sensor_msgs::msg::Image)
//       • /object_detection (sim_cam_pkg::msg::DetectionArray)
//       • /object_tracking (sim_cam_pkg::msg::TrackedObjectArray)
//   - Published:
//       • /visualization/image (sensor_msgs::msg::Image) — image with overlays
//
// Features:
//   - Only draws bounding boxes for tracked objects seen in the current frame.
//   - Draws unique colored trajectories per track.
//   - Uses approximate time synchronization as its subscribed to multiple nodes.
//
// Reference:
//   1. https://docs.ros.org/en/humble/Tutorials/Intermediate/Understanding-Time/Time-ApproximateTime.html
//   2. https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html# 
// ----------------------------------------------------------------------------------


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sim_cam_pkg/msg/detection_array.hpp"       // For raw detections
#include "sim_cam_pkg/msg/tracked_object_array.hpp"  // For tracked objects
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"       // For publishing annotated img
#include "opencv2/opencv.hpp"

// Sync messages from different topics based on their timestamps
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <set>

using ImageMsg = sensor_msgs::msg::Image;
using DetectionArrayMsg = sim_cam_pkg::msg::DetectionArray;
using TrackedObjectArrayMsg = sim_cam_pkg::msg::TrackedObjectArray;


// Synchronisation policy for all three input message types
typedef message_filters::sync_policies::ApproximateTime<
    ImageMsg,
    DetectionArrayMsg, 
    TrackedObjectArrayMsg
> MySyncPolicy;

class VisualizationNode : public rclcpp::Node
{
public:
    VisualizationNode()
    : Node("visualization_node")
    {

        // ROS2 QOS - Get latest message, realtime
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        qos_profile.depth = 10;  // QOS buffer size for subscribed

        // Initialize Subscribers for message_filters
        image_sub_.subscribe(this, "/camera/image_raw", qos_profile);
        detection_sub_.subscribe(this, "/object_detection", qos_profile);
        tracking_sub_.subscribe(this, "/object_tracking", qos_profile);

        // Create the synchronizer instance
        sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
            MySyncPolicy(30), image_sub_, detection_sub_, tracking_sub_ // Queue size for policy
        );
        // Register the callback to be called when synchronized messages are avlble
        sync_->registerCallback(std::bind(
            &VisualizationNode::synchronized_callback, this,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
        ));
        // Publisher instance for the final ann. image that can be viewed in rqt_image_viewer
        annotated_image_pub_ = image_transport::create_publisher(this, "/visualization/image");
        // RCLCPP_INFO(this->get_logger(), "Visualization Node started. Draw raw detections: %s", draw_raw_detections_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Will only draw bounding boxes for tracks with frames_since_last_seen == 0.");
    }

private:
    // Callback for synchronised camera raw feed, detection and tracking messages
    void synchronized_callback(
        const ImageMsg::ConstSharedPtr& image_msg,
        const DetectionArrayMsg::ConstSharedPtr& detection_msg,
        const TrackedObjectArrayMsg::ConstSharedPtr& tracking_msg)

    {   // ROS Image -> OpenCV format
        cv_bridge::CvImagePtr cv_ptr;
        (void)detection_msg; // Unused
        try {
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat frame = cv_ptr->image.clone();

        // --- 1. Draw Tracked Objects ---
        for (const auto& track : tracking_msg->tracked_objects) {
            
            // Only draw bbox if currently seen from detector to avoid multiple overlays of past detections  
            if (track.frames_since_last_seen == 0) {
                cv::rectangle(frame, cv::Point(track.box_x, track.box_y),
                              cv::Point(track.box_x + track.box_width, track.box_y + track.box_height),
                              cv::Scalar(0, 255, 0), 2); // GREEN for currently visible tracked box

                std::string track_label = "ID:" + std::to_string(track.id) + " (" + track.class_name.substr(0,7) + ")"; // Shorter class name
                int baseline = 0;
                cv::Size text_size = cv::getTextSize(track_label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
                cv::Point text_org(track.box_x, track.box_y - 7);
                 if (text_org.y < text_size.height) text_org.y = track.box_y + track.box_height + text_size.height + 2;


                 // Draw white background rectangle for the text
                cv::rectangle(frame,
                      text_org + cv::Point(0, baseline),
                      text_org + cv::Point(text_size.width, -text_size.height),
                      cv::Scalar(255, 255, 255), cv::FILLED);
                cv::putText(frame, track_label, text_org,
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 200), 1, cv::LINE_AA); 
            }

            // Draw Trajectory for any active track present in the message
            if (track.trajectory_points.size() > 1) {
                for (size_t i = 0; i < track.trajectory_points.size() - 1; ++i) {
                    cv::Point p1(static_cast<int>(track.trajectory_points[i].x), 
                                 static_cast<int>(track.trajectory_points[i].y));
                    cv::Point p2(static_cast<int>(track.trajectory_points[i+1].x), 
                                 static_cast<int>(track.trajectory_points[i+1].y));
                    // Unique color per trajectory based on ID
                    cv::Scalar trajectory_color = cv::Scalar((track.id * 40 + 60) % 255, (track.id * 70 + 100) % 255, (track.id * 100 + 50) % 255);
                    // cv::Scalar trajectory_color = cv::Scalar(0, 0, 200); // Red
                    cv::line(frame, p1, p2, trajectory_color, 2); 
                }
            }
            // draw small circles at each trajectory point for neat visz.
            for (const auto& pt : track.trajectory_points) {
                 cv::circle(frame, cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)), 1, cv::Scalar(200,200,0), -1); // Cyan dots
            }
        }


        try {
            sensor_msgs::msg::Image::SharedPtr out_msg = cv_bridge::CvImage(
                image_msg->header, "bgr8", frame
            ).toImageMsg();
            annotated_image_pub_.publish(out_msg);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception when publishing: %s", e.what());
        } 
    }

    // ROS Communication objects
    message_filters::Subscriber<ImageMsg> image_sub_;
    message_filters::Subscriber<DetectionArrayMsg> detection_sub_;
    message_filters::Subscriber<TrackedObjectArrayMsg> tracking_sub_;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
    image_transport::Publisher annotated_image_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisualizationNode>();
    rclcpp::spin(node); // Keep node active and processing callbacks
    rclcpp::shutdown(); // shutdown ROS2
    return 0; 
}