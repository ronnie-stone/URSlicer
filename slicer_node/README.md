# Slicer Node Package

## Overview

The `slicer_node` package serves as the ROS2 wrapper for the slicing algorithms. It processes STL files and generates motion paths for the UR5e robotic arm to execute 3D printing tasks.

## Core Structure

- **slicing_scripts/**: Contains Python scripts implementing slicing algorithms.
  - `slicing_algo_integrate_v4.py`: Main slicing algorithm script.
  - `ur_slicer.py`: Utility script for STL to motion path conversion.
- **launch/**: Launch files for running the slicing node.
- **src/**: Source files for ROS2 node integration.
- **CMakeLists.txt**: Build configuration for the package.
- **package.xml**: ROS2 package metadata.

## Usage

1. Ensure the ROS2 workspace is sourced:
   ```
   source ~/ros2_ws/install/setup.bash
   ```

2. Run the slicing node:
   ```
   ros2 run slicer_node slicing_algo_integrate_v4.py
   ```

3. Provide the STL file path and slicing parameters as inputs.

## Development Instructions

- **Adding New Algorithms**: Place new slicing scripts in the `slicing_scripts/` directory and ensure they follow the existing interface for compatibility.
- **Testing**: Use the `main()` function in the scripts for standalone testing.
- **Dependencies**: Ensure all required Python libraries (e.g., `numpy`, `stl`) are installed.

## Notes

- Refer to the ROS2 tutorials for integrating new nodes or modifying existing ones.
- Ensure compatibility with the `ur_slicer_interfaces` package for message types.