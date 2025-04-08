cmake_minimum_required(VERSION 3.8)
project(printing_manager)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)

# URSlicer dependencies
find_package(ur_slicer_interfaces REQUIRED)

# Include directories
include_directories(
  include
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)

# Node executable
add_executable(printing_manager_node
  src/printing_manager_node.cpp
)
target_include_directories(printing_manager_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(printing_manager_node
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
  ur_slicer_interfaces
)

# Install targets
install(TARGETS printing_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
