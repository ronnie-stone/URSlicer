# ur_slicer_interfaces

This is a package containing custom interfaces for ros2 in use with the URSlicer project. Refer to the ros2 custom interface tutorials for adding new types. Current types are detailed below, if changing current types ensure all collaborators are aware.

## Path.msg

A size unconstrained list of geometry_msgs/Point objects. Intended to provide a path of waypoints for printing.