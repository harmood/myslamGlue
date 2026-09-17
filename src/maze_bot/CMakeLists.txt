cmake_minimum_required(VERSION 3.8)
project(maze_bot)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz resource config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/camera_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
