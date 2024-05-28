from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_arm',
            executable='arduino_control',
            name='arduino_control',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB0'},
                {'baudrate': 115200}
            ]
        )
    ])
