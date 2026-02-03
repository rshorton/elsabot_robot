cmake_minimum_required(VERSION 3.5)
project(elsabot_robot)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config maps rviz meshes urdf gazebo_worlds nav_bt
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
