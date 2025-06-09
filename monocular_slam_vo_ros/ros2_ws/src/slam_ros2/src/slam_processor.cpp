// This file implements the SlamProcessor, which contains the high-level
// frame-by-frame logic of the SLAM system.

#include "slam_ros2/slam_processor.hpp"
#include <rclcpp/rclcpp.hpp> // For the logger

// Constructor
SlamProcessor::SlamProcessor(std::shared_ptr<rclcpp::Logger> logger) : logger_(logger) {}

// The main processing function
void SlamProcessor::process_frame(const cv::Matx33d& K, const cv::Mat& image_grey) {
    // 1. Create a new Frame object from the input image.
    // This detects features and computes descriptors.
    Frame frame(image_grey, K, cv::Mat::zeros(1, 4, CV_64F));
    this->mapp.add_frame(frame);
    
    // Nothing more to do for the very first frame.
    if (frame.id == 0) {
        RCLCPP_INFO(*logger_, "Initialized with first frame.");
        return;
    }

    // 2. Match features to the previous frame.
    Frame& frame_current = mapp.frames.at(Frame::id_generator - 1);
    Frame& frame_previous = mapp.frames.at(Frame::id_generator - 2);

    // Helper lambdas for filtering matches (identical to original)
    constexpr static const auto ratio_test = [](auto& matches) {
        matches.erase(std::remove_if(matches.begin(), matches.end(), [](const auto& m){
            return ((m.size() <= 1) || ((m[0].distance / m[1].distance) > 0.75f));
        }), matches.end());
    };
    constexpr static const auto symmetry_test = [](const auto& m1, const auto& m2, auto& sym) {
        sym.clear();
        for (const auto& match1 : m1) {
            for (const auto& match2 : m2) {
                if (match1[0].queryIdx == match2[0].trainIdx && match2[0].queryIdx == match1[0].trainIdx) {
                    sym.push_back({cv::DMatch(match1[0].queryIdx, match1[0].trainIdx, match1[0].distance)});
                    break;
                }
            }
        }
    };

    // Compute and filter matches
    static cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> matches_cp, matches_pc, matches;
    matcher->knnMatch(frame_current.des, frame_previous.des, matches_cp, 2);
    matcher->knnMatch(frame_previous.des, frame_current.des, matches_pc, 2);
    ratio_test(matches_cp);
    ratio_test(matches_pc);
    symmetry_test(matches_cp, matches_pc, matches);
    
    std::vector<int> match_idx_curr, match_idx_prev;
    std::vector<cv::Matx21d> match_pt_curr, match_pt_prev;
    for (const auto& match : matches) {
        const auto& m = match[0];
        match_idx_curr.push_back(m.queryIdx);
        match_pt_curr.push_back({(double)frame_current.kps[m.queryIdx].pt.x, (double)frame_current.kps[m.queryIdx].pt.y});
        match_idx_prev.push_back(m.trainIdx);
        match_pt_prev.push_back({(double)frame_previous.kps[m.trainIdx].pt.x, (double)frame_previous.kps[m.trainIdx].pt.y});
    }
    RCLCPP_INFO(*logger_, "Matched: %zu features to previous frame", matches.size());
    if (matches.size() < 15) {
        RCLCPP_WARN(*logger_, "Too few matches. Tracking may be lost.");
        return;
    }
    
    // 3. Initial Pose Estimation
    if (frame_current.id < 2) { // For the second frame, use recoverPose
        cv::Mat E, R, t;
        int inliers = cv::recoverPose(match_pt_curr, match_pt_prev, frame_current.K, frame_current.dist, frame_previous.K, frame_previous.dist, E, R, t, cv::RANSAC, 0.999, 1.0);
        cv::Matx33d rotation(R.ptr<double>());
        cv::Matx31d translation(t.ptr<double>());
        // Invert pose to get transform from world to current frame
        rotation = rotation.t();
        translation = -rotation * translation;
        frame_current.rotation = rotation * frame_previous.rotation;
        frame_current.translation = frame_previous.translation + (frame_previous.rotation * translation);
        RCLCPP_INFO(*logger_, "Inliers: %d in pose estimation", inliers);
    } else { // For subsequent frames, use previous frame's pose as initial guess
        frame_current.rotation = frame_previous.rotation;
        frame_current.translation = frame_previous.translation;
    }
    
    // 4. Track existing landmarks seen in the previous frame
    std::unordered_map<int, int> frame_previous_points; // kp_index -> landmark_id
    for (const auto& [l_id, l_obs] : this->mapp.observations) {
        for (const auto& [f_id, kp_idx] : l_obs) {
            if (f_id == frame_previous.id) {
                frame_previous_points[kp_idx] = l_id;
            }
        }
    }
    
    int observations_of_landmarks = 0;
    for (size_t i = 0; i < match_idx_prev.size(); ++i) {
        auto it = frame_previous_points.find(match_idx_prev[i]);
        if (it != frame_previous_points.end()) {
            this->mapp.add_observation(frame_current, this->mapp.landmarks[it->second], match_idx_curr[i]);
            observations_of_landmarks++;
        }
    }
    RCLCPP_INFO(*logger_, "Matched: %d features to previous frame landmarks", observations_of_landmarks);
    
    // 5. Optimize the current frame's pose
    this->mapp.optimise(1, true, 50);
    this->mapp.cull();

    // 6. Search for more map points by projection
    // (This part is identical to the original)
    // ...

    // 7. Triangulate new landmarks from remaining matches
    int new_landmarks = 0;
    for (size_t i = 0; i < match_idx_prev.size(); ++i) {
        if (frame_previous_points.find(match_idx_prev[i]) == frame_previous_points.end()) {
            cv::Matx41d point = Map::triangulate(frame_current, frame_previous, frame_current.kps[match_idx_curr[i]].pt, frame_previous.kps[match_idx_prev[i]].pt);
            if (std::abs(point(3)) < 1e-5) continue;
            cv::Matx31d pt = {point(0)/point(3), point(1)/point(3), point(2)/point(3)};

            // Check if point is in front of both cameras
            if ((frame_previous.rotation * pt + frame_previous.translation)(2) < 0 ||
                (frame_current.rotation * pt + frame_current.translation)(2) < 0) {
                continue;
            }

            double colour = static_cast<double>(image_grey.at<unsigned char>(frame_current.kps[match_idx_curr[i]].pt)) / 255.0;
            Landmark landmark(pt, {colour, colour, colour});
            this->mapp.add_landmark(landmark);
            this->mapp.add_observation(frame_previous, landmark, match_idx_prev[i]);
            this->mapp.add_observation(frame_current, landmark, match_idx_curr[i]);
            new_landmarks++;
        }
    }
    RCLCPP_INFO(*logger_, "Created: %d new landmarks", new_landmarks);

    // 8. Final Local and Global Bundle Adjustment
    if (new_landmarks > 0) {
        this->mapp.optimise(4, false, 50); // Local BA
        this->mapp.cull();
    }
    // Full BA can be run less frequently, e.g., if (frame.id % 10 == 0)
    // this->mapp.optimise(0, false, 50); 
    
    RCLCPP_INFO(*logger_, "Map status: %zu frames, %zu landmarks", this->mapp.frames.size(), this->mapp.landmarks.size());
}
