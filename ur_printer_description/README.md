# UR Printer Description Package

## Overview

The `ur_printer_description` package contains the URDF and mesh files for the UR5e robotic arm and the custom 3D printing end effector.

## Core Structure

- **urdf/**: URDF files defining the robot model.
- **meshes/**: Mesh files for the robot and end effector.
- **rviz/**: RViz configuration files for visualization.
- **CMakeLists.txt**: Build configuration for the package.
- **package.xml**: ROS2 package metadata.

## Usage

1. Launch the robot description in RViz:
   ```bash
   ros2 launch ur_printer_description view_robot.launch.py
   ```

2. Visualize the robot model and verify its configuration.

## Development Instructions

- **Modifying the Robot Model**: Update the URDF files in the `urdf/` directory.
- **Adding New Meshes**: Place new mesh files in the `meshes/` directory and update the URDF accordingly.

## Notes

- Follow the ROS2 URDF tutorials for modifying robot models.
- Ensure compatibility with the `ur_printer_control` package.
