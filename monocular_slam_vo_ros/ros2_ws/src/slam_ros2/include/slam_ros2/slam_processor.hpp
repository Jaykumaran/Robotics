#pragma once

// ---- ADD THESE LINES ----
#include <rclcpp/rclcpp.hpp> // Provides rclcpp::Logger
#include <memory>           // Provides std::shared_ptr
// -------------------------

#include "types.hpp"

class SlamProcessor {
public:
    Map mapp;
    SlamProcessor(std::shared_ptr<rclcpp::Logger> logger);
    void process_frame(const cv::Matx33d& K, const cv::Mat& image_grey);
private:
    std::shared_ptr<rclcpp::Logger> logger_;
};
