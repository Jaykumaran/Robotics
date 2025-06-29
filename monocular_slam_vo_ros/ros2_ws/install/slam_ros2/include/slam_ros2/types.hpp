#pragma once

// This header defines the core data structures for the SLAM system.
// These classes are independent of ROS and form the "backend" logic.

// OpenCV
#include <opencv2/opencv.hpp>

// Standard Library
#include <cstdio> // For std::printf in Frame constructor
#include <unordered_map>
#include <vector>

// Forward declaration of Map class for use in other headers if needed
class Map;

// Represents a single camera view at a point in time.
// It contains the image, camera parameters, extracted features, and estimated pose.
class Frame {
public:
    // Static members to generate unique IDs for each frame
    static inline int id_generator = 0;
    int id;

    // Input data
    cv::Mat image_grey;
    cv::Matx33d K;
    cv::Mat dist;

    // Extracted features
    std::vector<cv::KeyPoint> kps;
    cv::Mat des;

    // Estimated pose (Rotation and Translation) in the world frame
    cv::Matx33d rotation;
    cv::Matx31d translation;

public:
    Frame() = default;

    // Constructor that takes an image and camera parameters, then extracts features.
    Frame(const cv::Mat& input_image_grey, const cv::Matx33d& input_K, const cv::Mat& input_dist) {
        static cv::Ptr<cv::ORB> extractor = cv::ORB::create();
        this->id = Frame::id_generator++;
        this->image_grey = input_image_grey;
        this->K = input_K;
        this->dist = input_dist;

        // Detect good features to track
        std::vector<cv::Point2f> corners;
        cv::goodFeaturesToTrack(image_grey, corners, 3000, 0.01, 7);
        this->kps.reserve(corners.size());
        for (const cv::Point2f& corner : corners) {
            this->kps.push_back(cv::KeyPoint(corner, 20));
        }

        // Compute ORB descriptors for the keypoints
        extractor->compute(image_grey, this->kps, this->des);
        
        // Initialize pose to identity
        this->rotation = cv::Matx33d::eye();
        this->translation = cv::Matx31d::zeros();
        
        std::printf("Detected: %zu features\n", this->kps.size());
    }
};

// Represents a 3D point in the world map.
class Landmark {
public:
    // Static member to generate unique IDs
    static inline int id_generator = 0;
    int id;

    // 3D position and color
    cv::Matx31d location;
    cv::Matx31d colour;

public:
    Landmark() = default;
    Landmark(const cv::Matx31d& input_location, const cv::Matx31d& input_colour) {
        this->id = Landmark::id_generator++;
        this->location = input_location;
        this->colour = input_colour;
    }
};

// Represents the entire world map, containing all frames, landmarks, and their relationships.
class Map {
public:
    // Data containers
    std::unordered_map<int, Frame> frames;
    std::unordered_map<int, Landmark> landmarks;
    // Maps a landmark ID to a list of its observations (frame_id, keypoint_index)
    std::unordered_map<int, std::vector<std::pair<int, int>>> observations;

public:
    // --- Method Implementations in map.cpp ---
    // These are declared here but defined in map.cpp to keep the header clean.

    // Triangulates a 3D point from two 2D observations in different frames.
    static cv::Matx41d triangulate(const Frame& lhs_frame, const Frame& rhs_frame, const cv::Point2d& lhs_point, const cv::Point2d& rhs_point);

    // Runs bundle adjustment using g2o to optimize poses and landmark positions.
    void optimise(int local_window, bool fix_landmarks, int rounds);

    // Removes bad landmarks from the map based on reprojection error or lack of recent observations.
    void cull();


    // --- Simple inline methods ---
    // These are simple enough to be defined directly in the header.

    void add_frame(const Frame& frame) {
        frames[frame.id] = frame;
    }
    
    void add_landmark(const Landmark& landmark) {
        this->landmarks[landmark.id] = landmark;
    }
    
    void add_observation(const Frame& frame, const Landmark& landmark, int kp_index) {
        this->observations[landmark.id].push_back({ frame.id, kp_index });
    }
};
