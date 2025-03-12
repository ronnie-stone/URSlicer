import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
# Import the custom slicer command message; make sure it is defined in your package.
from ur_slicer_interfaces.msg import SlicerCommand  

class UR5eExtruderHeatingInterface(Node):
    """Node that sends DO commands based on slicer commands and robot motion.

    It publishes:
      - A command to /ur_hardware_interface/tool_digital_output_extruder to turn on the filament extruder driver.
      - A command to /ur_hardware_interface/tool_digital_output_heating to instruct the Arduino to monitor the heating element.
    """

    def __init__(self):
        super().__init__('ur5e_extruder_heating_interface')

        # Publisher for the extruder DO command.
        self.extruder_dout_pub = self.create_publisher(
            Bool, '/ur_hardware_interface/tool_digital_output_extruder', 1
        )
        # Publisher for the heating element monitoring DO command.
        self.heating_dout_pub = self.create_publisher(
            Bool, '/ur_hardware_interface/tool_digital_output_heating', 1
        )

        # Subscription for slicer commands using a custom message.
        self.create_subscription(
            SlicerCommand, '/slicer/command', self.slicer_command_callback, 1
        )

        # Subscription for joint states to determine if the robot is moving.
        self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 1
        )

        # Timer to periodically check conditions and publish DO commands.
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Internal flags to hold the current command states.
        self.slicer_monitor_heating = False
        self.slicer_extrude_filament = False
        self.robot_in_motion = False

        # Threshold for joint velocities to consider the robot "in motion."
        self.motion_threshold = 0.01

        self.get_logger().info("UR5e Extruder & Heating Interface Node Initialized")

    def slicer_command_callback(self, msg: SlicerCommand):
        """
        Callback to update the command flags from the slicer.
        Assumes the custom message has:
          - monitor_heating (bool)
          - extrude_filament (bool)
        """
        self.slicer_monitor_heating = msg.monitor_heating
        self.slicer_extrude_filament = msg.extrude_filament
        self.get_logger().info(
            f"Received slicer command: monitor_heating={self.slicer_monitor_heating}, "
            f"extrude_filament={self.slicer_extrude_filament}"
        )

    def joint_state_callback(self, msg: JointState):
        """
        Callback to determine if the robot is in motion.
        If any joint's absolute velocity exceeds the motion_threshold,
        the robot is considered in motion.
        """
        self.robot_in_motion = any(abs(vel) > self.motion_threshold for vel in msg.velocity)
        self.get_logger().debug(f"Robot in motion: {self.robot_in_motion}")

    def timer_callback(self):
        """
        Timer callback that evaluates the conditions and publishes:
          - A DO command for the extruder (True if the robot is in motion OR the slicer command extrude_filament is True).
          - A DO command for the heating element (True if the slicer command monitor_heating is True).
        """
        # Extruder command: active if the robot is moving or the slicer instructs extrusion.
        extrude_command = self.robot_in_motion or self.slicer_extrude_filament

        # Heating command: active if the slicer instructs monitoring of the heating element.
        heating_command = self.slicer_monitor_heating

        extruder_msg = Bool()
        extruder_msg.data = extrude_command
        heating_msg = Bool()
        heating_msg.data = heating_command

        self.extruder_dout_pub.publish(extruder_msg)
        self.heating_dout_pub.publish(heating_msg)

        self.get_logger().info(
            f"Published DO commands: Extruder={extrude_command}, Heating={heating_command}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = UR5eExtruderHeatingInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
