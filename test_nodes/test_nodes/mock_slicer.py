import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from std_msgs.msg import Bool
from ur_slicer_interfaces.msg import SlicerSettings, NozzleTemperature, Path
from ur_slicer_interfaces.action import PreparePrinter, Slicer
from ur_slicer_interfaces.srv import ExtruderControl, HeaterControl
from geometry_msgs.msg import Pose, Point


class ManagerTestNode(Node):
    def __init__(self):
        super().__init__("mock_slicer_node")

        # Action server for PreparePrinter
        self.prepare_printer_server = ActionServer(
            self,
            Slicer,
            "slicer_action",
            execute_callback=self.prepare_printer_execute_callback,
        )

        self.get_logger().info("Mock Slicer initialized.")

    def prepare_printer_execute_callback(self, goal_handle):
        self.get_logger().info(
            f"Received PreparePrinter goal with filepath: {goal_handle.request.filepath}"
        )

        # Create a test square
        point_1 = Point(x=0.0, y=-0.0, z=0.0)
        point_2 = Point(x=0.25, y=-0.0, z=0.0)
        point_3 = Point(x=0.25, y=-0.05, z=0.0)
        point_4 = Point(x=-0.25, y=-0.05, z=0.0)
        point_5 = Point(x=-0.25, y=-0.0, z=0.0)
        point_6 = Point(x=0.0, y=-0.0, z=0.0)

        test_path = Path()
        test_path.path = [point_1, point_2, point_3, point_4, point_5, point_6]

        result = Slicer.Result()

        result.path_list = [test_path, test_path, test_path, test_path]
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ManagerTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
