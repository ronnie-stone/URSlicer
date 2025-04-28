# UR Printer Control Package

## Overview

The `ur_printer_control` package provides control interfaces for the UR5e robotic arm and the custom 3D printing end effector.

## Core Structure

- **launch/**: Launch files for starting the robot and configuring hardware.
- **config/**: Configuration files for robot parameters.
- **urdf/**: URDF files for the robot model.
- **CMakeLists.txt**: Build configuration for the package.
- **package.xml**: ROS2 package metadata.

## Usage

1. Launch the robot:
   ```bash
   ros2 launch ur_printer_control start_robot.launch.py
   ```

2. Specify the robot IP and type as arguments.

## Development Instructions

- **Adding New Features**: Modify the URDF files in the `urdf/` directory for hardware changes.
- **Testing**: Use the provided launch files for simulation and hardware testing.

## Notes

- Ensure compatibility with the `ur_slicer_interfaces` package for message types.
- Follow the ROS2 and UR driver documentation for hardware integration.
