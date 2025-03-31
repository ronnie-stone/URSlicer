import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Temperature
from ur_slicer_interfaces.msg import SlicerCommand
from ur_slicer_interfaces.srv import GetDOCommands, GetHotendTemperature
from ur_msgs.msg import Analog

class URExtruderInterface(Node):
    """
    Service-based node that:
      - Receives updates via subscriptions (from slicer commands, joint states, and an Analog message).
      - Provides two services:
          1. get_do_commands: Returns DO commands based on robot motion and slicer commands.
          2. get_hotend_temperature: Returns the latest hotend temperature computed from an Analog input.
    """

    def __init__(self):
        super().__init__('ur5e_extruder_interface')

        # ------------------Subscriptions--------------------
        # Subscription for slicer commands (custom message SlicerCommand)
        self.create_subscription(
            SlicerCommand,
            '/slicer/command',
            self.slicer_command_callback,
            1
        )

        # Subscription for joint states (to determine if the robot is moving)
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            1
        )

        # Subscription for the analog input signal from the UR toolhead using ur_msgs/Analog
        self.create_subscription(
            Analog,
            '/ur_hardware_interface/tool_analog_input',
            self.analog_input_callback,
            1
        )

        # ------------------Service Servers--------------------
        # Service to get the current DO commands
        self.get_do_commands_srv = self.create_service(
            GetDOCommands,
            'get_do_commands',
            self.handle_get_do_commands
        )

        # Service to get the current hotend temperature
        self.get_hotend_temperature_srv = self.create_service(
            GetHotendTemperature,
            'get_hotend_temperature',
            self.handle_get_hotend_temperature
        )

        # ------------------Internal State--------------------
        self.slicer_monitor_heating = False
        self.slicer_extrude_filament = False
        self.robot_in_motion = False
        self.latest_temperature = 0.0  # Latest computed temperature from the analog input

        # If only one analog pin is used for temperature (e.g., pin 0)
        self.temperature_pin = 0

        # Threshold for joint velocities to consider the robot "in motion"
        self.motion_threshold = 0.01

        self.get_logger().info("UR5e Extruder Interface Node Initialized (Service-based)")

    # ------------------Subscription Callbacks--------------------
    def slicer_command_callback(self, msg: SlicerCommand):
        """
        Update internal flags based on the slicer command.
        Assumes SlicerCommand has:
          - monitor_heating (bool)
          - extrude_filament (bool)
        """
        self.slicer_monitor_heating = msg.monitor_heating
        self.slicer_extrude_filament = msg.extrude_filament
        self.get_logger().info(
            f"Received slicer command: monitor_heating={msg.monitor_heating}, extrude_filament={msg.extrude_filament}"
        )

    def joint_state_callback(self, msg: JointState):
        """
        Update the robot motion flag.
        If any joint's absolute velocity exceeds the motion_threshold,
        the robot is considered in motion.
        """
        self.robot_in_motion = any(abs(vel) > self.motion_threshold for vel in msg.velocity)
        self.get_logger().debug(f"Robot in motion: {self.robot_in_motion}")

    def analog_input_callback(self, msg: Analog):
        """
        Process the Analog message from the UR toolhead.
        Checks the pin number and converts the voltage reading (msg.state)
        to a temperature in °C.
        For example, here 0–10 V corresponds to 0–300 °C.
        Adjust the conversion factor based on your sensor calibration.
        """
        if msg.pin == self.temperature_pin:
            voltage = msg.state
            self.latest_temperature = (voltage / 10.0) * 300.0
            self.get_logger().info(
                f"Analog input (pin {msg.pin}): {voltage:.2f} V -> Temperature: {self.latest_temperature:.2f} °C"
            )

    # ------------------Service Handlers--------------------
    def handle_get_do_commands(self, request, response):
        """
        Service callback to compute and return the DO commands.
        - extruder_command: True if the robot is in motion OR the slicer command requests extrusion.
        - heating_command: True if the slicer command requests heating monitoring.
        """
        extruder_command = self.robot_in_motion or self.slicer_extrude_filament
        heating_command = self.slicer_monitor_heating

        response.extruder_command = extruder_command
        response.heating_command = heating_command

        self.get_logger().info(
            f"Service get_do_commands called: extruder={extruder_command}, heating={heating_command}"
        )
        return response

    def handle_get_hotend_temperature(self, request, response):
        """
        Service callback to return the latest hotend temperature.
        """
        response.temperature = self.latest_temperature
        self.get_logger().info(
            f"Service get_hotend_temperature called: temperature={self.latest_temperature:.2f} °C"
        )
        return response

def main(args=None):
    rclpy.init(args=args)
    node = URExtruderInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
