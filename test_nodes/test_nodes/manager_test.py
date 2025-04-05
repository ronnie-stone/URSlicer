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
        super().__init__("manager_test_node")

        # Publisher for nozzle temperature
        self.nozzle_temp_pub = self.create_publisher(
            NozzleTemperature, "nozzle_temperature", 10
        )

        # Subscriber for slicer settings
        self.slicer_settings_sub = self.create_subscription(
            SlicerSettings, "slicer_settings", self.slicer_settings_callback, 10
        )

        # Subscriber for begin printing
        self.begin_printing_sub = self.create_subscription(
            Bool, "begin_printing", self.begin_printing_callback, 10
        )

        # Action server for PreparePrinter
        self.prepare_printer_server = ActionServer(
            self,
            Slicer,
            "slicer_action",
            execute_callback=self.prepare_printer_execute_callback,
        )

        # Service server for HeaterControl
        self.heater_control_server = self.create_service(
            HeaterControl, "heater_control", self.heater_control_callback
        )

        # Service server for ExtruderControl
        self.extruder_control_server = self.create_service(
            ExtruderControl, "extruder_control", self.extruder_control_callback
        )

        self.get_logger().info("ManagerTestNode initialized.")

    def slicer_settings_callback(self, msg):
        self.get_logger().info(
            f"Received slicer settings: Nozzle Temp: {msg.nozzle_temperature}, "
            f"Infill Density: {msg.infill_density}, Layer Height: {msg.layer_height}, "
            f"Print Speed: {msg.print_speed}"
        )

    def begin_printing_callback(self, msg):
        if msg.data:
            self.get_logger().info("Begin printing signal received.")

    def prepare_printer_execute_callback(self, goal_handle):
        self.get_logger().info(
            f"Received PreparePrinter goal with filepath: {goal_handle.request.filepath}"
        )

        # Create a test square
        point_1 = Point(x=0.0, y=0.05, z=0.0)
        point_2 = Point(x=0.05, y=0.05, z=0.0)
        point_3 = Point(x=0.05, y=0.10, z=0.0)
        point_4 = Point(x=0.10, y=0.10, z=0.0)
        point_5 = Point(x=0.10, y=0.05, z=0.0)
        point_6 = Point(x=0.05, y=0.0, z=0.0)

        test_path = Path()
        test_path.path = [point_1, point_2, point_3, point_4, point_5, point_6]

        result = Slicer.Result()

        result.path_list = [test_path]
        goal_handle.succeed()
        return result

    def heater_control_callback(self, request, response):
        if request.heater_on:
            self.get_logger().info("Heater turned on.")
            response.control_confirmed = True
        elif request.heater_off:
            self.get_logger().info("Heater turned off.")
            response.control_confirmed = True
        else:
            self.get_logger().info("Heater status checked.")
            response.control_confirmed = False
        return response

    def extruder_control_callback(self, request, response):
        if request.on_extruder:
            self.get_logger().info("Extruder turned on.")
            response.motion_allowed = True
        else:
            self.get_logger().info("Extruder turned off.")
            response.motion_allowed = False
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ManagerTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
