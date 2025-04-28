# UR Slicer Interfaces Package

## Overview

The `ur_slicer_interfaces` package defines custom ROS2 interfaces used for communication between various components of the URSlicer project. These interfaces include messages and actions tailored for 3D printing tasks.

## Core Structure

- **msg/**: Contains custom message definitions.
  - `Path.msg`: A list of geometry_msgs/Point objects representing a motion path.
  - `SlicerSettings.msg`: Configuration settings for slicing operations.
  - `NozzleTemperature.msg`: Represents the nozzle temperature.
- **action/**: Contains custom action definitions.
  - `PreparePrinter.action`: Action for preparing the printer for operation.
  - `Slicer.action`: Action for slicing STL files and generating motion paths.
- **srv/**: Contains custom service definitions.
  - `HeaterControl.srv`: Service for controlling the heater.

## Usage

1. Include the package in your `package.xml` and `CMakeLists.txt` to use the custom interfaces.
2. Import the interfaces in your ROS2 nodes:
   ```cpp
   #include "ur_slicer_interfaces/msg/path.hpp"
   #include "ur_slicer_interfaces/action/slicer.hpp"
   ```

## Development Instructions

- **Adding New Interfaces**:
  1. Create a new `.msg`, `.srv`, or `.action` file in the appropriate directory.
  2. Update the `CMakeLists.txt` and `package.xml` to include the new interface.
  3. Build the workspace:
     ```bash
     colcon build
     ```

- **Testing**: Use ROS2 CLI tools to test the interfaces:
  ```bash
  ros2 topic pub /topic_name ur_slicer_interfaces/msg/Path "{...}"
  ```

## Notes

- Refer to the ROS2 custom interface tutorials for creating new message, service, or action types.
- Ensure all collaborators are informed of changes to existing interfaces.