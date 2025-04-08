# Quick Commands for Various Terminal Configurations in Testing

## Running URSim and MoveIt 

This creates a simulated ur5e, launches moveit, and opens necessary controllers.

`ros2 run ur_client_library start_ursim.sh`

`ros2 launch ur_printer_control start_robot.launch.py launch_rviz:=false robot_ip:=192.168.56.101`

`ros2 launch ur_printer_moveit_config move_group.launch.py`

`ros2 launch ur_printer_moveit_config moveit_rviz.launch.py`

`ros2 launch printer_manager test_sim_testing.launch.py ur_type:=ur5e`

## Running Real Robot and MoveIt

`ros2 launch ur_printer_control start_robot.launch.py launch_rviz:=false robot_ip:=169.254.107.107`

`ros2 launch ur_printer_moveit_config move_group.launch.py`

`ros2 launch ur_printer_moveit_config moveit_rviz.launch.py`

`ros2 launch printer_manager TBD.launch.py ur_type:=ur5e`