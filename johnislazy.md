# Quick Commands for Various Terminal Configurations in Testing

## Running URSim and MoveIt 

This creates a simulated ur5e, launches moveit, and opens necessary controllers.

`ros2 run ur_client_library start_ursim.sh`

`ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101 launch_rviz:=false`

`ros2 launch printing_manager moveit.launch.py ur_type:=ur5e launch_rviz:=true`

## Running Real Robot and MoveIt

`ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=169.254.107.107`

`ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true`
