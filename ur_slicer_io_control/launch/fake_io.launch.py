from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ur_slicer_io_control",
                executable="io_control_node",
                name="fake_io_node",
                output="screen",
                emulate_tty=True,
                parameters=[{"simulation": True}],
            )
        ]
    )
