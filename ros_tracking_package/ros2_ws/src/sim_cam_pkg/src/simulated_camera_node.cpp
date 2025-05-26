// ----------------------------------------------------------------------------------
// Node: SimulatedCameraNode
// Functionality:
//   Publishes video frames from a local video file as ROS Image messages on
//   /camera/image_raw, simulating a live camera feed.
//   Also publishes a reset signal on /tracking/reset when the video loops, something like a "new episode"
//   Resetting the track ID when a video loops is to prevent incorrect tracjectory and ID switching for the same instance reappears
//
// Reference: 
//    1. https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
//    2.  https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html
// ----------------------------------------------------------------------------------


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp" // For cv::VideoCapture, cv::Mat
#include "image_transport/image_transport.hpp" // For image_transport::Publisher
#include "ament_index_cpp/get_package_share_directory.hpp" // To locate package resources
#include "sim_cam_pkg/msg/tracking_reset.hpp" // To signal tracker reset
#include "std_msgs/msg/header.hpp" // For std_msgs::msg::Header

// chrono: for timestamps and duration
#include <chrono>    // For std::chrono::milliseconds
#include <string>    // For std::string
#include <memory>    // For std::shared_ptr
#include <functional>// For std::bind
using namespace std::chrono_literals; 

class SimulatedCameraNode : public rclcpp::Node
{
public:
    SimulatedCameraNode()
    : Node("simulated_camera_node") // Initialize Node
    {
        // Declare Parameters
        this->declare_parameter<std::string>("video_path", "drift_car.mp4"); 
        this->declare_parameter<double>("publish_rate_hz", 10.0);
        this->declare_parameter<bool>("loop_video", true);

        // Passed via cli 
        std::string video_file = this->get_parameter("video_path").as_string();
        // Controls how often frames are published, in Hz (frames per second)
        double publish_rate = this->get_parameter("publish_rate_hz").as_double();
        // Flag to detect a video is looped, by default set to true
        loop_ = this->get_parameter("loop_video").as_bool();

        // Construct the full path to video, find it in the package's share directory `sim_cam_pkg/videos`
        std::string full_video_path = video_file;
        if (video_file.find('/') == std::string::npos && !video_file.empty()) {
            try {
                full_video_path = ament_index_cpp::get_package_share_directory("sim_cam_pkg") + "/videos/" + video_file;
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Could not find package share dir, trying video_path as is: %s. Error: %s", video_file.c_str(), e.what());
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Opening video: %s", full_video_path.c_str());
        cap_.open(full_video_path); // open the video using OpenCV

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video: %s. Node will not function.", full_video_path.c_str());
       
            if(rclcpp::ok()) rclcpp::shutdown(); // Trigger ROS Shutdown if reading video fails
            return; // Prevent further steps or initialization
        }

        // Initialize publisher to topic `/camera/image_raw`
        image_publisher_ = image_transport::create_publisher(this, "/camera/image_raw");
        // It is published for the tracking node to subscribe
        reset_publisher_ = this->create_publisher<sim_cam_pkg::msg::TrackingReset>("/tracking/reset", 10);
        

        if (publish_rate <= 0) publish_rate = 10.0;  // Set a default publish rate if an invalid rate is passed
        // Setup timer to call timer_callback at specified publish rate
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
        timer_ = this->create_wall_timer(
            timer_period, std::bind(&SimulatedCameraNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Simulated camera initialized, publishing at ~%.1f Hz.", publish_rate);
    }

private:
    // Execute Timer callback at regular intervals to read and publish a video frame
    void timer_callback()
    {
        if (!cap_.isOpened()) return; 

        cv::Mat frame;
        bool video_looped = false;
        // Try: Read next frame
        if (!cap_.read(frame)) { 
            if (loop_) {
                RCLCPP_INFO(this->get_logger(), "Video ended, looping.");
                cap_.set(cv::CAP_PROP_POS_FRAMES, 0); // Rewind video to beginning if loop is detected
                video_looped = true; // Flag that a loop has occured
                if (!cap_.read(frame)) { // Try reading the first frame again after rewind
                    RCLCPP_ERROR(this->get_logger(), "Failed to read frame after looping.");
                    return; // if fails, Skip this cycle
                }
            } else { // if loop is set to false, end the video
                RCLCPP_INFO(this->get_logger(), "Video ended, no loop. Stopping timer.");
                timer_->cancel(); // Stop timer if no looping
                return; 
            }
        }

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame read from video.");
            return;
        }

        // Create ROS image message
        auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now(); // Current timestamp
        header.frame_id = "camera_link"; // Standard frame_id used for cameras

        // Convert OpenCV image to ROS Image message using cv_bridge
     
        try {
            cv_bridge::CvImage cv_image(header, "bgr8", frame);
            cv_image.toImageMsg(*image_msg); // Fill the message
            image_publisher_.publish(std::move(image_msg)); // Publish
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return; // If an exemption occurs, Skip publishing this frame
        }

        // Publish a reset signal
        if (video_looped) {
            sim_cam_pkg::msg::TrackingReset reset_msg;
            reset_msg.header.stamp = this->get_clock()->now(); // Timestamp when the reset signal is occured
            reset_msg.reset_triggered = true;
            reset_publisher_->publish(reset_msg);
            RCLCPP_INFO(this->get_logger(), "Published reset signal.");
        }
    }

    // ROS2 Member Variables
    rclcpp::TimerBase::SharedPtr timer_;
    image_transport::Publisher image_publisher_; // Using image_transport
    rclcpp::Publisher<sim_cam_pkg::msg::TrackingReset>::SharedPtr reset_publisher_;

    cv::VideoCapture cap_; // Video Capture object
    bool loop_; // Flag
};

int main(int argc, char * argv[])
{   
    // Initialize ROS2 CLI lib.
    rclcpp::init(argc, argv);
    // Create the node instance
    auto node = std::make_shared<SimulatedCameraNode>(); // shared smart pointers
    if (rclcpp::ok()) { // Check if constructor called shutdown, if not keep node alive
        rclcpp::spin(node);
    }
    rclcpp::shutdown();  // Shutdown ROS2
    return 0;
}
