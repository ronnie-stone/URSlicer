import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import JointState, Temperature
from ur_slicer_interfaces.msg import SlicerCommand  

class URExtruderInterface(Node):
    """Node that sends DO commands based on slicer commands and robot motion,
    and also processes an analog input to measure the hotend temperature.
    
    It publishes:
      - A command to /ur_hardware_interface/tool_digital_output_extruder to turn on the filament extruder driver.
      - A command to /ur_hardware_interface/tool_digital_output_heating to instruct the Arduino to monitor the heating element.
      - The hotend temperature as a sensor_msgs/Temperature message on /hotend_temperature.
    """

    def __init__(self):
        super().__init__('ur5e_extruder_interface')

        #-----------------Publishers-------------------
        # Publisher for the extruder DO command.
        self.extruder_dout_pub = self.create_publisher(Bool, '/ur_hardware_interface/tool_digital_output_extruder', 1)
        
        # Publisher for the heating element monitoring DO command.
        self.heating_dout_pub = self.create_publisher(Bool, '/ur_hardware_interface/tool_digital_output_heating', 1)
        
        # Publisher for hotend temperature.
        self.temperature_pub = self.create_publisher(Temperature, '/hotend_temperature', 1)

        #------------------Subscribers--------------------
        # Subscription for slicer commands using a custom message.
        self.create_subscription(SlicerCommand, '/slicer/command', self.slicer_command_callback, 1)

        # Subscription for joint states to determine if the robot is moving.
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1)

        # Subscription for the analog input signal from the UR toolhead.

        # Assumes a Float64 message representing the voltage reading.
        self.create_subscription(Float64, '/ur_hardware_interface/tool_analog_input', self.analog_input_callback, 1)

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

    def analog_input_callback(self, msg: Float64):
        """
        Callback to process the analog input from the UR toolhead.
        Converts the voltage reading to a temperature value and publishes it.
        
        In this example, we assume 0–10 V corresponds to 0–100°C.
        Adjust the conversion as per your sensor's calibration.
        """
        voltage = msg.data
        # Convert the analog voltage to temperature in Celsius.
        temperature_c = (voltage / 10.0) * 300.0 # 0-10v corresponds to the range given by the thermistor being used. Adjustment may be needed

        temp_msg = Temperature()
        temp_msg.header.stamp = self.get_clock().now().to_msg()
        temp_msg.temperature = temperature_c
        temp_msg.variance = 0.0  # Adjust if you have a variance value.

        self.temperature_pub.publish(temp_msg)
        self.get_logger().info(f"Published hotend temperature: {temperature_c:.2f} °C")

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
    node = URExtruderInterface()  # Ensure class name matches.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
