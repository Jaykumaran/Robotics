cmake_minimum_required(VERSION 3.8)
project(slam_ros2)

# C++ standard and warnings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to Release build
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ROS and core dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Try to find g2o normally
find_package(g2o QUIET)

# Manual fallback for g2o if not found via config
if(NOT g2o_FOUND)
  message(WARNING "g2oConfig.cmake not found, falling back to manual search.")

  find_path(G2O_INCLUDE_DIRS
    NAMES g2o/core/block_solver.h
    PATHS /usr/local/include /usr/include
  )

  find_library(G2O_CORE_LIBRARY NAMES g2o_core PATHS /usr/local/lib /usr/lib)
  find_library(G2O_STUFF_LIBRARY NAMES g2o_stuff PATHS /usr/local/lib /usr/lib)
  find_library(G2O_SBA_LIBRARY NAMES g2o_types_sba PATHS /usr/local/lib /usr/lib)

  if(NOT G2O_INCLUDE_DIRS OR NOT G2O_CORE_LIBRARY OR NOT G2O_STUFF_LIBRARY OR NOT G2O_SBA_LIBRARY)
    message(FATAL_ERROR "Missing G2O libraries. Please install G2O with SBA module.")
  endif()

  set(G2O_LIBRARIES
    ${G2O_CORE_LIBRARY}
    ${G2O_STUFF_LIBRARY}
    ${G2O_SBA_LIBRARY}
  )
else()
  # If g2oConfig.cmake defines targets, try to use them
  set(G2O_INCLUDE_DIRS "")
  if(TARGET g2o::core)
    list(APPEND G2O_LIBRARIES g2o::core g2o::stuff g2o::types_sba)
  endif()
endif()

# ===================================================================
# slam_core library
# ===================================================================
add_library(slam_core SHARED
  src/map.cpp
  src/slam_processor.cpp
)

target_include_directories(slam_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${G2O_INCLUDE_DIRS}
)

target_link_libraries(slam_core
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${G2O_LIBRARIES}
)

ament_target_dependencies(slam_core
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
)

# ===================================================================
# video_publisher_node executable
# ===================================================================
add_executable(video_publisher_node
  src/video_publisher_node.cpp
)

target_link_libraries(video_publisher_node
  ${OpenCV_LIBS}
)

ament_target_dependencies(video_publisher_node
  rclcpp
  cv_bridge
  sensor_msgs
  std_msgs
)

# ===================================================================
# slam_node executable
# ===================================================================
add_executable(slam_node
  src/slam_node.cpp
)

target_link_libraries(slam_node
  slam_core
  ${G2O_LIBRARIES}
)

ament_target_dependencies(slam_node
  rclcpp
  cv_bridge
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
)

# ===================================================================
# Installation
# ===================================================================
install(TARGETS
  slam_core
  video_publisher_node
  slam_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  # RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME}) # For RViz 



ament_package()

