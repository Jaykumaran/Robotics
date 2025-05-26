# Launch: `ros2 launch sim_cam_pkg object_tracking_pipeline.launch.py`

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration


# Ref: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html#change-via-a-launch-file

def generate_launch_description():

    # === Configurations ===
    video_filename = LaunchConfiguration('video_path', default='drift_car.mp4')
    publish_rate = LaunchConfiguration('publish_rate', default='30.0')
    
    # Detector params
    detector_confidence_threshold = LaunchConfiguration('confidence_threshold', default='0.4')    

    # === Node Definitions ===
    
    # T1: Simulated Camera Node
    sim_camera_node = Node(
        package='sim_cam_pkg',
        executable='simulated_camera_node',
        name='simulated_camera_node',
        output='screen', # Show output in the launch terminal
        parameters=[
            {'video_path': video_filename},
            {'publish_rate_hz': publish_rate},
            {'loop_video': True} 
        ]
    )

    # T2: Image Subscriber Node (Preprocessor)
    image_subscriber_node = Node(
        package='sim_cam_pkg',
        executable='image_subscriber_node',
        name='image_subscriber_node',
        output='screen'
    )

    # T3: Object Detector Node (Python)
    object_detector_node = Node(
        package='sim_cam_pkg',
        executable='object_detector_node.py', # Make sure this matches your Python script name
        name='object_detector_node',
        output='screen',
        parameters=[
            {'confidence_threshold': detector_confidence_threshold}
    
        ]
    )

    # T4: Object Tracker Node (C++)
    object_tracker_node = Node(
        package='sim_cam_pkg',
        executable='object_tracker_node',
        name='object_tracker_node',
        output='screen',
    )

    # T5: Visualization Node (C++)
    visualization_node = Node(
        package='sim_cam_pkg',
        executable='visualization_node',
        name='visualization_node',
        output='screen'
    )

    # T6: RQT Image View
    rqt_image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        arguments=['/visualization/image'], # Topic to view
        output='screen' 
       
    )


    return LaunchDescription([
        LogInfo(msg="Starting Simulated Camera Node..."),
        sim_camera_node,

        LogInfo(msg="Starting Image Subscriber Node..."),
        image_subscriber_node,
        
        LogInfo(msg="Starting Object Detector Node..."),
        object_detector_node,
        
        LogInfo(msg="Starting Object Tracker Node..."),
        object_tracker_node,
        
        LogInfo(msg="Starting Visualization Node..."),
        visualization_node,
        
        LogInfo(msg="Starting RQT Image View for /visualization/image..."),
        rqt_image_view_node,

    ])