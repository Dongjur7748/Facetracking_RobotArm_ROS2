from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='face_detection',
            executable='face_detection_node',  # 수정된 파일명
            name='face_detection_node',
            output='screen',
            parameters=[{
                'config_path': '/home/ubuntu/ros2_ws/install/face_detection/share/face_detection/config/face_tracker.yaml'
            }]
        )
    ])
