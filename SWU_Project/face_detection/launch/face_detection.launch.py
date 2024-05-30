from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='SWU_Project',
            executable='face_detection_node',
            name='face_detection_node',
            output='screen',
            parameters=[{
                'config_path': '/home/ubuntu/SWU_Project/SWU_Project/face_detection/config/face_tracker.yaml'
            }]
        )
    ])
