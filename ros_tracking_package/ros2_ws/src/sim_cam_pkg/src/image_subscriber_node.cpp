// ----------------------------------------------------------------------------------
// Node: ImageSubscriberNode
// Functionality:
//   Subscribes to raw camera images on /camera/image_raw,
//   converts them to OpenCV-compatible BGR8 format,
//   and republishes them (withput any preprocessing) on /image_processed.
//
// Use:
//   Acts as a preprocessing bridge for downstream nodes (e.g., DNNs using OpenCV).
//
// Reference: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h" // For CvImage, toCvCopy, image_encodings
#include "image_transport/image_transport.hpp" // For image_transport functionalities

#include <memory>     // For std::make_shared
#include <functional> // For std::bind
// Placeholder for first argument in std::bind when callback is invoked (ROS msg )
using std::placeholders::_1; 

// Subscription Node
class ImageSubscriberNode : public rclcpp::Node
{
public:
    ImageSubscriberNode()
    : Node("image_subscriber_node") // Initialize the node with its name
    {
        // Subscriber for raw images
        image_sub_ = image_transport::create_subscription(
            this, // Node instance;  similar to self, points the node instance
            "/camera/image_raw", // Input Topic to subscribe to
            std::bind(&ImageSubscriberNode::image_callback, this, _1), // Callback func
            "raw", // Underlying transport hint
            rmw_qos_profile_sensor_data); // QoS profile depth for sensor data

        // Publisher for "processed" (format-converted) images
        image_pub_ = image_transport::create_publisher(this, "/image_processed");

        RCLCPP_INFO(this->get_logger(), "Image Subscriber Node: /camera/image_raw -> /image_processed");
    }

private:
    // Callback function that will be triggered to process incoming image messages
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {
        // convert ROS Image message to OpenCV CvImage (BGR8 format)
        cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
        if (cv_ptr->image.empty()) {
            RCLCPP_WARN(this->get_logger(), "Converted OpenCV image is empty, skipping publish.");
            return; // Skip this frame due to conversion error
        }

        // Prepare the message for publishing
        // Image Resizing and Manipulation, The OpenCV DNN - Python node will take care, just publish as it is!
        sensor_msgs::msg::Image::SharedPtr out_msg = cv_bridge::CvImage(
                                                        msg->header,
                                                        sensor_msgs::image_encodings::BGR8,
                                                        cv_ptr->image // OpenCV Image data
                                                    ).toImageMsg();
        // Publish the OpenCV-Compatible Image over ROS 
        image_pub_.publish(out_msg);
    }

    // ROS2 communication members
    image_transport::Subscriber image_sub_; // Subscriber object
    image_transport::Publisher image_pub_;  // Publisher object
};

int main(int argc, char * argv[])
{
    // Initialize ROS client library with args passed via cli
    rclcpp::init(argc, argv);

    // Create and spin the node to process callbacks
    // shared_ptr to the node object
    rclcpp::spin(std::make_shared<ImageSubscriberNode>());

    rclcpp::shutdown(); // Shutdown ROS2 before exiting
    return 0;
}
