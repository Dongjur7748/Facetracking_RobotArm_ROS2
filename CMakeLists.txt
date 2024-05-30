cmake_minimum_required(VERSION 3.5)
project(SWU_Project)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Python 패키지 설치
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  SWU_Project/face_detection/face_detection_node.py
  SWU_Project/robot_arm/src/robot_arm_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  SWU_Project/face_detection/config
  SWU_Project/robot_arm/config
  SWU_Project/face_detection/launch
  SWU_Project/robot_arm/launch
  SWU_Project/face_detection/models
  SWU_Project/robot_arm/urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
