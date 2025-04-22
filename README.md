# URSlicer

This is the primary repo for the UR Slicer 3D printing project sponsored by the SiDi laboratory. This repo contains all packages and instructions for dependency installation necessary for running the slicer and printer. 

---

_**Warning:**_ Safe operation is critical to prevent injury and damage to people and objects surrounding the system. Before operating the robot, powering on the hotend, or running any prints ensure that everyone in the surrounding area is aware of the robot operation and that the robot is being monitored at all times.

---
## Table of Contents

* [Functionality Overview](#functionality-overview)
* [Key Dependencies](#key-dependencies)
* [Installation Instructions](#installation-instructions)
* [Operating Instructions](#operating-instructions)
* [Included Package Overview](#package-overview)
* [Continued Development](#continued-development)

---

## Functionality Overview

This system is intended to enable the UR5e arm and printing end effector to act as a 3D printer and enable planar printing of complex geometries. This system encompasses all aspects and steps to the 3D printing process to go from a desired STL file to a finished 3D print. While some testing has been performed it is highly possible that bugs still exist.

### Printing Process Steps

1. Open necessary terminal windows
2. Select desired print object
3. Place print object within scene and select slicing settings
4. Slice part and plan motion
5. Preheat hotend
6. Print object!

### User Interface

![User Interface](images/ui_overview.png)

### Robot Model

![URDF Based Model](images/urdf.png)

---

## Key Dependencies

**Preface:** This project and system are heavily dependent on a linux based operating system, the command line, and ROS2. It is highly recommended to have a basic familiarity with each of these concepts. Reference material for each of these topics has been included. 

### System Requirements/Recommendations

This project was intended to be ran and used on a computer running the Ubuntu 22.04 operating system. It is possible to run this system on other operating systems or inside of a Docker container or WSL2 although this would require a higher level of familiarity with containerization and networking principles.

The installation instructions assume that a computer with a fresh Ubuntu 22.04 distribution is being used. [Here](https://www.xda-developers.com/dual-boot-windows-11-linux/) is a link to a guide for dual-booting a Windows machine with an Ubuntu distribution.


### ROS2 Humble 

[This](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) is the link for installing the correct distribution of ROS2. It is recommended to go through both sets of beginner [tutorials](https://docs.ros.org/en/humble/Tutorials.html) if you are unfamiliar with ROS2.

### MoveIt2

The following command can be entered in the terminal to install MoveIt2. The tutorials are not necessary unless you are trying to add onto this code.

```sudo apt install ros-humble-moveit```

### UR ROS2 Drivers

The following command installs the necessary ROS2 drivers developed by Universal Robotics.

```sudo apt-get install ros-${ROS_DISTRO}-ur```

---

## Installation Instructions

### Networking Setup

In order to control the UR5e arm using another computer a few pieces of network setup are required. This only needs to be done once as long as it is not changed. These steps are not required for the simulated robot.

1. Turn on the UR5e tablet and control box

2. Ensure the network settings on the UR5e control tablet match the following image.

TODO:

3. Plug the ethernet cable from the UR5e box into your computer.

4. On the Ubuntu computer go to, **Settings -> Network -> Wired -> IPV4**

5. Apply the network settings shown in the image below.

![Network Settings](images/network_settings.png)

### ROS2 Workspace Setup 

The following command will create a ROS2 workspace if you do not already have one.

```mkdir -p ~/ros2_ws/src```

Every time a new terminal is opened ensure the following line is entered unless it has been added to your bashrc.
```
source /opt/ros/humble/setup.bash
```
The following line can be entered once. This will add it to your .bashrc which is a file that runs everytime a new terminal is opened.
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Package Installation

Enter the following commands to install and build the packages. (This assumes that your Github SSH Key has already been setup, tutorials for this can be found online.)

```cd ~/ros2_ws/src```

```git clone git@github.com:ronnie-stone/URSlicer.git```

```cd ~/ros2_ws```

```colcon build```

```source install/setup.bash```



## Packages for STL Movement Demo

cd to ros_ws 
ensure git repo has been cloned into the correct branch
ensure that workspace has been sourced
```
colcon build --packages-select ur_slicer_interfaces slicer_panel test_nodes
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
```

