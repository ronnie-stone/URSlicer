# URSlicer

Every time a new terminal is opened ensure the following line is entered unless it has been added to your bashrc.
```
source /opt/ros/humble/setup.bash
```
The following line can be entered once. This will add it to your .bashrc which is a file that runs everytime a new terminal is opened.
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

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
colcon build --packages-select slicer_panel test_nodes
```

After building any packages the following line must be entered while in the ros_ws directory.

```
source install/setup.bash
```

## How to run stl viz demo

Once the above packages are built the demo should be able to be run.

1. Run the following line to start the mock hardware simulation of the UR5e arm.

```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=true
```
2. In the top left click "Panels -> Add New Panel" and select "Slicer"

3. Click the "Add" button on the middle left. Select "Interactive Marker"

4. In the Interactive Marker display options set the "Interactive Marker Namespace" to /stl_markers.

5. The panel is now usable.

Optional bed placer:

In another sourced terminal run:
```
ros2 run test_nodes bed_corner_tester
``

