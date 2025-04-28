# Printing Manager Package

## Overview

The `printing_manager` package manages the 3D printing process, including motion planning, slicing integration, and hardware control.

## Core Structure

- **include/printer_manager/**: Header files for the printer manager component.
- **src/**: Source files for motion planning and hardware control.
- **launch/**: Launch files for simulation and real hardware testing.
- **rviz/**: RViz configuration files for visualization.
- **CMakeLists.txt**: Build configuration for the package.
- **package.xml**: ROS2 package metadata.

## Usage

1. Launch the printing manager:
   ```bash
   ros2 launch printing_manager real_slice_test.launch.py
   ```

2. Monitor the printing process in RViz.

## Development Instructions

- **Adding New Features**: Extend the `PrinterManagerComponent` class in `printer_manager_component.hpp`.
- **Motion Planning**: Use MoveIt2 for planning and executing motion paths.
- **Testing**: Use the provided launch files for simulation and hardware testing.

## Notes

- Ensure compatibility with the `ur_slicer_interfaces` package for message types.
- Follow the MoveIt2 documentation for motion planning.
