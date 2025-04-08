cmake_minimum_required(VERSION 3.8)
project(printer_manager)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ur_slicer_interfaces REQUIRED)

include_directories(
  include
)

# Create the printer_manager_component library
add_library(printer_manager_component SHARED
  src/printer_manager_component.cpp
)

target_compile_definitions(printer_manager_component
  PRIVATE "PRINTER_MANAGER_BUILDING_LIBRARY")

ament_target_dependencies(printer_manager_component
  rclcpp
  rclcpp_action
  rclcpp_components
  moveit
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
  std_msgs
  ur_slicer_interfaces
)

# Register the component with the component registry
rclcpp_components_register_nodes(printer_manager_component 
  "printer_manager::PrinterManagerComponent")

# Create the standalone node executable
add_executable(printer_manager_node
  src/standalone_printer_manager_node.cpp
)

target_link_libraries(printer_manager_node
  printer_manager_component
)

ament_target_dependencies(printer_manager_node
  rclcpp
)

# Install targets
install(TARGETS
  printer_manager_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  printer_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

