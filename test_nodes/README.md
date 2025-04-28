# Test Nodes Package

## Overview

The `test_nodes` package contains test nodes and scripts for verifying the functionality of various components in the URSlicer project. It includes both automated tests and standalone nodes for manual testing.

## Core Structure

- **test/**: Contains automated test scripts.
  - `test_pep257.py`: Linter test for PEP 257 compliance.
  - `test_copyright.py`: Ensures copyright headers are present.
- **test_nodes/**: Contains standalone test nodes.
  - `bed_test.py`: Node for testing bed corner publishing.
- **CMakeLists.txt**: Build configuration for the package.
- **package.xml**: ROS2 package metadata.

## Usage

1. Run automated tests:
   ```bash
   colcon test --packages-select test_nodes
   ```

2. Launch standalone test nodes:
   ```bash
   ros2 run test_nodes bed_test.py
   ```

## Development Instructions

- **Adding New Tests**:
  1. Place new test scripts in the `test/` directory.
  2. Use `pytest` for Python tests or ROS2 testing utilities for C++ tests.
  3. Update the `CMakeLists.txt` to include the new tests.

- **Adding New Test Nodes**:
  1. Place new test nodes in the `test_nodes/` directory.
  2. Ensure they are executable and follow ROS2 node conventions.

## Notes

- Use this package for both unit testing and integration testing.
- Follow ROS2 testing best practices to ensure compatibility and maintainability.
