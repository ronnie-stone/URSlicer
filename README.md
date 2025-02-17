# URSlicer

## Necessary Packages

Install UR ROS2 Drivers
```
sudo apt-get install ros-${ROS_DISTRO}-ur
```

## Packages for STL Movement Demo

cd to ros_ws 
ensure git repo has been cloned into the correct branch
ensure that workspace has been sourced
```
colcon build --packages-select slicer_panel
```
