from setuptools import setup
import os
from glob import glob

package_name = 'SWU_Project'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, 'SWU_Project.face_detection', 'SWU_Project.robot_arm'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('SWU_Project/face_detection/launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('SWU_Project/robot_arm/launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('SWU_Project/face_detection/config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('SWU_Project/robot_arm/config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('SWU_Project/face_detection/models/*.tflite')),
        (os.path.join('share', package_name, 'urdf'), glob('SWU_Project/robot_arm/urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Face tracking and robot arm control project.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_detection_node = SWU_Project.face_detection.face_detection_node:main',
            'robot_arm_node = SWU_Project.robot_arm.src.robot_arm_node:main',
        ],
    },
)
