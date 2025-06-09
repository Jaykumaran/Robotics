#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include "cv_bridge/cv_bridge.h" // For CvImage, toCvCopy, image_encodings
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Your SLAM implementation headers
#include "slam_ros2/slam_processor.hpp" // Contains the SlamProcessor class
#include "slam_ros2/types.hpp"          // Contains Frame, Landmark, Map

class SlamNode : public rclcpp::Node
{
public:
    SlamNode() : Node("slam_node")
    {
        slam_processor_ = std::make_unique<SlamProcessor>(std::make_shared<rclcpp::Logger>(this->get_logger()));
        
        // Subscribers
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info", 10, std::bind(&SlamNode::camera_info_callback, this, std::placeholders::_1));
        
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&SlamNode::image_callback, this, std::placeholders::_1));

        // Publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/slam/camera_pose", 10);
        map_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/slam/map_points", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/slam/trajectory", 10);
        debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/slam/debug_image", 10);


        RCLCPP_INFO(this->get_logger(), "SLAM node initialized. Waiting for camera info and images...");
    }

private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (has_camera_info_) return;
        
        K_ = cv::Matx33d(
            msg->k[0], msg->k[1], msg->k[2],
            msg->k[3], msg->k[4], msg->k[5],
            msg->k[6], msg->k[7], msg->k[8]
        );
        has_camera_info_ = true;
        RCLCPP_INFO(this->get_logger(), "Received camera intrinsics (K matrix).");
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!has_camera_info_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No camera info received yet. Skipping frame.");
            return;
        }

        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
            
            auto start = std::chrono::steady_clock::now();
            slam_processor_->process_frame(K_, cv_ptr->image);
            auto end = std::chrono::steady_clock::now();
            std::chrono::duration<double> diff = end - start;
            RCLCPP_INFO(this->get_logger(), "Frame processing time: %.4f s", diff.count());

            // Publish results after processing
            publish_results(msg->header.stamp);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void publish_results(const rclcpp::Time& stamp)
    {
        if (slam_processor_->mapp.frames.empty()) return;

        // Get the latest frame
        const Frame& last_frame = slam_processor_->mapp.frames.at(Frame::id_generator - 1);

        // 1. Publish current camera pose
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = "map"; // Poses are in the map frame
        
        // Convert rotation matrix to quaternion
        cv::Matx33d R = last_frame.rotation;
        tf2::Matrix3x3 tf_R(R(0,0), R(0,1), R(0,2), R(1,0), R(1,1), R(1,2), R(2,0), R(2,1), R(2,2));
        tf2::Quaternion q;
        tf_R.getRotation(q);
        
        pose_msg.pose.orientation = tf2::toMsg(q);
        pose_msg.pose.position.x = last_frame.translation(0);
        pose_msg.pose.position.y = last_frame.translation(1);
        pose_msg.pose.position.z = last_frame.translation(2);
        pose_pub_->publish(pose_msg);

        // 2. Publish trajectory path
        path_msg_.header.stamp = stamp;
        path_msg_.header.frame_id = "map";
        path_msg_.poses.push_back(pose_msg);
        path_pub_->publish(path_msg_);
        
        // 3. Publish map points as a PointCloud2
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = stamp;
        cloud_msg.header.frame_id = "map";
        cloud_msg.height = 1;
        cloud_msg.width = slam_processor_->mapp.landmarks.size();
        cloud_msg.is_bigendian = false;
        cloud_msg.is_dense = true;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2Fields(
            4, 
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "rgb", 1, sensor_msgs::msg::PointField::FLOAT32
        );
        
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");

        for (const auto& [id, landmark] : slam_processor_->mapp.landmarks)
        {
            *iter_x = landmark.location(0);
            *iter_y = landmark.location(1);
            *iter_z = landmark.location(2);
            
            // Assuming color is 0-1, convert to 0-255
            uint8_t r = static_cast<uint8_t>(landmark.colour(0) * 255.0);
            uint8_t g = static_cast<uint8_t>(landmark.colour(1) * 255.0);
            uint8_t b = static_cast<uint8_t>(landmark.colour(2) * 255.0);
            
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            *reinterpret_cast<uint32_t*>(&*iter_r) = rgb;

            ++iter_x; ++iter_y; ++iter_z; ++iter_r;
        }
        map_points_pub_->publish(cloud_msg);


        // At the very end of the publish_results function:

        // 4. Publish debug image with keypoints
        cv::Mat debug_image;
        // Use the color version of the last frame's image if available, or greyscale
        // For simplicity, we'll just draw on the grey image
        cv::drawKeypoints(last_frame.image_grey, last_frame.kps, debug_image);

        // Convert to a ROS message and publish
        auto debug_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug_image).toImageMsg();
        debug_image_msg->header.stamp = stamp;
        debug_image_msg->header.frame_id = "camera_link";
        debug_image_pub_->publish(*debug_image_msg);
    }
    
    std::unique_ptr<SlamProcessor> slam_processor_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    // In the private: section of the SlamNode class declaration:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;

    cv::Matx33d K_;
    bool has_camera_info_ = false;
    nav_msgs::msg::Path path_msg_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamNode>());
    rclcpp::shutdown();
    return 0;
}
