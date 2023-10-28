cmake_minimum_required(VERSION 3.5)
project(agv_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config gazebo ros2_control urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
