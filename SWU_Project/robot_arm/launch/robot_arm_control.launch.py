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
                {'config_path': '/home/ubuntu/SWU_Project/SWU_Project/robot_arm/config/joint_limits.yaml'},
                {'kinematics_path': '/home/ubuntu/SWU_Proejct/SWU_Project/robot_arm/config/kinematics.yaml'}
            ]
        )
    ])
