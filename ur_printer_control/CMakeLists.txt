cmake_minimum_required(VERSION 3.8)
project(ur_printer_control)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
