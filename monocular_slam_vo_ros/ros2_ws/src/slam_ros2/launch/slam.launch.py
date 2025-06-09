import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to your video file
    video_path = 'videos/freiburgxyz_525.mp4'
    # Focal length for the camera
    focal_length = 525.0

    # RViz configuration file
        # --- Automatically find the rviz config file ---
    pkg_dir = get_package_share_directory('slam_ros2')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'slam.rviz')

    return LaunchDescription([
        Node(
            package='slam_ros2',
            executable='video_publisher_node',
            name='video_publisher',
            output='screen',
            parameters=[
                {'video_path': video_path},
                {'focal_length': focal_length}
            ]
        ),
        Node(
            package='slam_ros2',
            executable='slam_node',
            name='slam_node',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
        # Static transform for camera link to show in rviz
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'map', '--child-frame-id', 'camera_link']
        )
    ])




# For RQT Image View Debug

# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     # --- Parameters to set ---
#     # Note: Using absolute path is safer, but relative can work if launched from workspace root
#     video_path = 'videos/kitti_984.mp4' 
#     focal_length = 719.0

#     return LaunchDescription([
#         # Node to publish video frames
#         Node(
#             package='slam_ros2',
#             executable='video_publisher_node',
#             name='video_publisher',
#             output='screen',
#             parameters=[
#                 {'video_path': video_path},
#                 {'focal_length': focal_length}
#             ]
#         ),

#         # The main SLAM node
#         Node(
#             package='slam_ros2',
#             executable='slam_node',
#             name='slam_node',
#             output='screen'
#         ),

#         # The RQT Image Viewer, subscribed to our new debug topic
#         Node(
#             package='rqt_image_view',
#             executable='rqt_image_view',
#             name='image_viewer',
#             arguments=['/slam/debug_image']
#         ),
#     ])