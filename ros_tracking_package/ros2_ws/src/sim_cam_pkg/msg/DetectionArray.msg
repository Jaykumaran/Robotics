std_msgs/Header header
sim_cam_pkg/Detection[] detections  # Array of Detection messages: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html#extra-use-an-existing-interface-definition



# Ref: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html#
# ------------------------------------------------
#  Message structure:

#struct Header {
 # Time stamp;
#  std::string frame_id;
#};

#struct DetectionArray {
 # Header header;
 # std::vector<Detection> detections;
#};

# <type> <field_name>

# O/P
# header:
#   stamp:
#     sec: 1748082954
#     nanosec: 442527318
#   frame_id: camera_link
# detections:
# - og_image_width: 640
#   og_image_height: 640
#   class_id: 7
#   class_name: truck
#   score: 0.6962231993675232
#   box_x: 360
#   box_y: 463
#   box_width: 1327
#   box_height: 379
# --------------------------------------------------
