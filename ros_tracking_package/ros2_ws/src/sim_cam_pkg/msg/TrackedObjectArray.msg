std_msgs/Header header # Timestamp and frame_id for this array of tracks
sim_cam_pkg/TrackedObject[] tracked_objects   # https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html#extra-use-an-existing-interface-definition



# Ref:https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html#
# *****-----------------------------**********
# The ros_idl_generators will try to build from msg such a C++ struct representation:

# #include <string>
# #include <vector>
# #include "geometry_msgs/msg/point.hpp"

# struct TrackedObject {
#   int32_t id;                  // Unique ID
#   std::string class_name;      // Object class
#   float score;                 // Confidence score
#
#   // Bounding box (pixel coordinates)
#   int32_t x;
#   int32_t y;
#   int32_t width;
#   int32_t height;
#
#   // Centroid (pixel coordinates, subpixel precision)
#   float centroid_x;
#   float centroid_y;
#
#   // Trajectory (list of past centroids)
#   std::vector<geometry_msgs::msg::Point> trajectory_points;
# };

# #include "std_msgs/msg/header.hpp"
# #include "package_name/msg/tracked_object.hpp" 

# struct TrackedObjectArray {
#   std_msgs::msg::Header header;                             // Timestamp + frame_id
#   std::vector<package_name::msg::TrackedObject> tracked_objects;
# };
# *****-----------------------------**********

