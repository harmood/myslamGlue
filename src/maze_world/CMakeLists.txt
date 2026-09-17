cmake_minimum_required(VERSION 3.8)
project(maze_world)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds launch scripts resource rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
