#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "cv_bridge/cv_bridge.h" // For CvImage, toCvCopy, image_encodings
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std::chrono_literals;

class VideoPublisherNode : public rclcpp::Node
{
public:
    VideoPublisherNode() : Node("video_publisher_node")
    {
        // Declare parameters for video path and focal length
        this->declare_parameter<std::string>("video_path", "");
        this->declare_parameter<double>("focal_length", 525.0);
        this->declare_parameter<double>("publish_rate_hz", 30.0);

        std::string video_path = this->get_parameter("video_path").as_string();
        double focal_length = this->get_parameter("focal_length").as_double();
        double publish_rate = this->get_parameter("publish_rate_hz").as_double();

        if (video_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Video path parameter 'video_path' is not set or empty.");
            rclcpp::shutdown();
            return;
        }

        cap_.open(video_path);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file: %s", video_path.c_str());
            rclcpp::shutdown();
            return;
        }

        // Create publishers
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
        cam_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10);

        // Populate CameraInfo message (it's constant)
        cam_info_msg_.header.frame_id = "camera_link";
        cam_info_msg_.width = static_cast<uint32_t>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
        cam_info_msg_.height = static_cast<uint32_t>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
        cam_info_msg_.k = {
            focal_length, 0.0, cam_info_msg_.width / 2.0,
            0.0, focal_length, cam_info_msg_.height / 2.0,
            0.0, 0.0, 1.0
        };
        cam_info_msg_.p = {
            focal_length, 0.0, cam_info_msg_.width / 2.0, 0.0,
            0.0, focal_length, cam_info_msg_.height / 2.0, 0.0,
            0.0, 0.0, 1.0, 0.0
        };

        // Create a timer to publish frames at a fixed rate
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate),
            std::bind(&VideoPublisherNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Video publisher started. Publishing from '%s' at %.1f Hz.", video_path.c_str(), publish_rate);
    }
// Video loop is enabled to ensure I receive continuous stream of msgs to rviz
private:
   void timer_callback()
    {
        cv::Mat frame;
        if (!cap_.read(frame)) {
            // If reading fails (end of video), reset to the beginning
            RCLCPP_INFO(this->get_logger(), "End of video file reached. Looping...");
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0); 
            
            // Try to read the first frame again
            if (!cap_.read(frame)) {
                // If it still fails, there's a real problem with the video file
                RCLCPP_ERROR(this->get_logger(), "Failed to read from video after looping. Shutting down.");
                rclcpp::shutdown();
                return;
            }
        }

        // Convert cv::Mat to sensor_msgs::Image
        auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        auto now = this->get_clock()->now();
        image_msg->header.stamp = now;
        image_msg->header.frame_id = "camera_link";
        
        cam_info_msg_.header.stamp = now;

        image_pub_->publish(*image_msg);
        cam_info_pub_->publish(cam_info_msg_);
    }

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::CameraInfo cam_info_msg_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
