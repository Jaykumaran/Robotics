cmake_minimum_required(VERSION 3.8)
project(sim_cam_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui imgcodecs dnn videoio video)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED) # For custom messages
find_package(CURL REQUIRED)
find_package(message_filters REQUIRED)


# === Custom Messages ===
set(msg_files
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/TrackedObject.msg"
  "msg/TrackedObjectArray.msg"
  "msg/TrackingReset.msg" # If video loop detected again reset id to 0
)

# https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html#
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files} # Above defined custom message list --> set(), unlike python set(), this will have duplicates and is ordered
  DEPENDENCIES sensor_msgs std_msgs geometry_msgs # # Add packages that above messages depend on
) 
# ========================

# ======== EXECUTABLES ================
add_executable(simulated_camera_node src/simulated_camera_node.cpp)
ament_target_dependencies(simulated_camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  ament_index_cpp)

target_link_libraries(simulated_camera_node
      ${PROJECT_NAME}__rosidl_typesupport_cpp  # https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html#link-against-the-interface
      ${OpenCV_LIBS}
      CURL::libcurl)

add_executable(image_subscriber_node src/image_subscriber_node.cpp)
ament_target_dependencies(image_subscriber_node
  rclcpp sensor_msgs cv_bridge image_transport ament_index_cpp)
target_link_libraries(image_subscriber_node
      ${OpenCV_LIBS}
      CURL::libcurl)

# --- Object Tracker Node ---
add_executable(object_tracker_node src/object_tracker_node.cpp)

ament_target_dependencies(object_tracker_node # Dependencies for compilation (headers)
  rclcpp
  std_msgs
  geometry_msgs
  cv_bridge
  image_transport 
  message_filters 
)

target_link_libraries(object_tracker_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp  # Custom messages that's defined
  ${OpenCV_LIBS}
)


# -------- For Visualization --------------
add_executable(visualization_node src/visualization_node.cpp)
ament_target_dependencies(visualization_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  message_filters
  )

target_link_libraries(visualization_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  ${OpenCV_LIBS}
)
# ==========================

install(TARGETS
  simulated_camera_node
  image_subscriber_node
  object_tracker_node
  visualization_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- Installations for shared resources ---
install(DIRECTORY images/ DESTINATION share/${PROJECT_NAME}/images)
install(DIRECTORY videos/ DESTINATION share/${PROJECT_NAME}/videos)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install Python node script
install(PROGRAMS nodes/object_detector_node.py DESTINATION lib/${PROJECT_NAME})
# **************************************

# --- Export dependencies for other packages and ensure messages are discoverable ---
ament_export_dependencies(rosidl_default_runtime) # Runtime dependency for messages

# Using ament_export_targets to export the message interfaces
if(rosidl_generate_interfaces_TARGETS) # Check if the target list variable is set
  ament_export_targets(${rosidl_generate_interfaces_TARGETS}
    HAS_LIBRARY_TARGET # Indicates that these targets include libraries
  )
endif()



ament_package()
