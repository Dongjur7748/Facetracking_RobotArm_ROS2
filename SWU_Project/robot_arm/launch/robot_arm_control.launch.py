from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_arm',
            executable='robot_arm_node',
            name='robot_arm_node',
            output='screen',
            parameters=[
                {'config_path': '/home/ubuntu/ros2_ws/src/robot_arm/config/joint_limits.yaml'},
                {'kinematics_path': '/home/ubuntu/ros2_ws/src/robot_arm/config/kinematics.yaml'}
            ]
        )
    ])
