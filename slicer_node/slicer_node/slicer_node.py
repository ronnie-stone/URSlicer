import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from std_msgs.msg import Bool
from ur_slicer_interfaces.msg import SlicerSettings, NozzleTemperature, Path
from ur_slicer_interfaces.action import PreparePrinter, Slicer
from ur_slicer_interfaces.srv import ExtruderControl, HeaterControl
from geometry_msgs.msg import Pose, Point

from slicing_scripts.slicing_algo_integrate_v4 import STL2Motion


class SlicerNode(Node):
    def __init__(self):
        super().__init__("slicer_node")

        # Action server for PreparePrinter
        self.prepare_printer_server = ActionServer(
            self,
            Slicer,
            "slicer_action",
            execute_callback=self.prepare_printer_execute_callback,
        )

        self.get_logger().info("Slicer initialized.")

    def prepare_printer_execute_callback(self, goal_handle):
        self.get_logger().info(
            f"Received PreparePrinter goal with filepath: {goal_handle.request.filepath}"
        )

        # TODO: Fill in Aditya code.
        slicer = STL2Motion(goal_handle.request.filepath, quaternion=None)
        robot_points = slicer.generate_robot_points()

        result = Slicer.Result()
        result.path_list = []

        for layer in robot_points:
            path_msg = Path()
            path_msg.path = [Point(x=pt[0], y=pt[1], z=pt[2]) for pt in layer]
            result.path_list.append(path_msg)

        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SlicerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
