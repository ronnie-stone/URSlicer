# Slicer Panel Package

## Overview

The `slicer_panel` package provides a custom RViz2 panel for user interaction. It allows users to configure slicing settings, select STL files, and visualize the slicing process.

## Core Structure

- **include/slicer_panel/**: Header files for the panel implementation.
- **src/**: Source files for the panel logic.
- **launch/**: Launch files for integrating the panel with RViz2.
- **CMakeLists.txt**: Build configuration for the package.
- **package.xml**: ROS2 package metadata.

## Usage

1. Launch RViz2 with the slicer panel:
   ```bash
   ros2 launch slicer_panel slicer_panel.launch.py
   ```

2. Use the panel to:
   - Select STL files.
   - Configure slicing parameters (e.g., layer height, infill density).
   - Visualize slicing results.

## Development Instructions

- **Adding New Widgets**: Modify `panel.hpp` and `panel.cpp` to include new UI elements.
- **Event Handling**: Use Qt signals and slots for handling user interactions.
- **Testing**: Launch the panel in RViz2 and verify functionality.

## Notes

- Ensure compatibility with the `ur_slicer_interfaces` package for message types.
- Follow the Qt and RViz2 documentation for UI development.
