import rclpy
from rclpy.node import Node
from ur_slicer_interfaces.msg import NozzleTemperature
from ur_slicer_interfaces.srv import ExtruderControl, HeaterControl
from ur_msgs.msg import ToolDataMsg
from ur_msgs.srv import SetIO


class URExtruderInterface(Node):
    """
    Service-based node that:
      - Receives updates via subscriptions (from slicer commands, joint states, and an Analog message).
      - Provides two services:
          1. get_do_commands: Returns DO commands based on robot motion and slicer commands.
          2. get_hotend_temperature: Returns the latest hotend temperature computed from an Analog input.
    """

    def __init__(self):
        super().__init__("ur5e_extruder_interface")

        self.declare_parameter("simulation", True)

        self.simulation = self.get_parameter("simulation").value

        self.HEATER_DO_PIN = SetIO.Request.PIN_TOOL_DOUT0
        self.EXTRUDER_DO_PIN = SetIO.Request.PIN_TOOL_DOUT1

        # -------------------Publishers--------------------
        self.temp_pub = self.create_publisher(
            NozzleTemperature, "extruder_temperature", 1
        )
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.extruder_temp_pub)

        # ------------------Subscriptions--------------------
        # Subscription for the analog input signal from the UR toolhead using ur_msgs/Analog
        self.create_subscription(
            ToolDataMsg,
            "/io_and_status_controller/tool_data",
            self.tool_data_callback,
            1,
        )

        # ------------------Service Servers--------------------
        # Heater Control Service
        self.create_service(
            HeaterControl,
            "heater_control",
            self.handle_heater_control,
        )

        # Extruder Control Service
        self.create_service(
            ExtruderControl,
            "extruder_control",
            self.handle_extruder_control,
        )

        # ------------------Internal State--------------------
        self.latest_temperature = 0.0
        self.desired_temperature = 0.0
        self.maintain_heater = False

        # -----------------Service Clients----------------------
        self.io_set_client = self.create_client(
            SetIO, "/io_and_status_controller/set_io"
        )
        while not self.io_set_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for UR IO service to be available...")
        self.get_logger().info("UR IO service is available.")

        self.get_logger().info(
            "UR5e Extruder Interface Node Initialized (Service-based)"
        )

    # ------------------Publisher--------------------
    def extruder_temp_pub(self):
        """
        Publish the latest temperature to the topic "extruder_temperature".
        """
        msg = NozzleTemperature()
        msg.nozzle_temperature = self.latest_temperature
        self.temp_pub.publish(msg)
        self.get_logger().info(
            f"Published extruder temperature: {self.latest_temperature:.2f} °C"
        )

    # ------------------Subscription Callbacks--------------------

    def tool_data_callback(self, msg: ToolDataMsg):
        """
        Process the Analog message from the UR toolhead.
        Checks the pin number and converts the voltage reading (msg.state)
        to a temperature in °C.
        For example, here 0–10 V corresponds to 0–300 °C.
        Adjust the conversion factor based on your sensor calibration.
        """
        analog_signal = msg.analog_input2
        self.latest_temperature = int((analog_signal / 10.0) * 300.0)
        if self.simulation:
            self.latest_temperature = int(self.desired_temperature)
            return
        if self.maintain_heater:
            if self.latest_temperature < self.desired_temperature:
                self.set_heater(True)
            else:
                self.set_heater(False)

    # ------------------Service Handlers--------------------
    def handle_heater_control(
        self, request: HeaterControl.Request, response: HeaterControl.Response
    ):
        """
        Service callback to control the heater.
        """
        self.desired_temperature = request.desired_temperature

        if request.heater_on and request.heater_off:
            self.get_logger().error(
                "Service heater_control called: heater ON and OFF at the same time"
            )
            response.control_confirmed = False
            return response

        if request.heater_on:
            self.maintain_heater = True
            self.get_logger().info("Service heater_control called: heater ON")
        if request.heater_off:
            self.maintain_heater = False
            self.get_logger().info("Service heater_control called: heater OFF")

        self.get_logger().info(
            f"Service heater_control called: temperature={self.desired_temperature:.2f} °C"
        )
        response.control_confirmed = True
        return response

    def handle_extruder_control(
        self, request: ExtruderControl.Request, response: ExtruderControl.Response
    ):
        """
        Service callback to control the extruder.
        """
        if request.on_extruder and request.off_extruder:
            self.get_logger().error(
                "Service extruder_control called: extruder ON and OFF at the same time"
            )
            response.control_confirmed = False
            return response
        if request.on_extruder:
            self.get_logger().info("Service extruder_control called: extruder ON")
            response.control_confirmed = self.set_extruder(True)
        if request.off_extruder:
            self.get_logger().info("Service extruder_control called: extruder OFF")
            response.control_confirmed = self.set_extruder(False)
        return response

    # ------------------Service Clients--------------------
    def set_heater(self, state: bool):
        """
        Set the heater state using the UR IO service.
        """
        if self.simulation:
            return True
        request = SetIO.Request()
        request.fun = SetIO.Request.FUN_SET_DIGITAL_OUT
        request.pin = self.HEATER_DO_PIN
        if state:
            request.state = SetIO.Request.STATE_ON
        else:
            request.state = SetIO.Request.STATE_OFF
        future = self.io_set_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is not None:
            self.get_logger().info(f"Set heater state to {state}")
            return True
        else:
            self.get_logger().error("Failed to set heater state")
            return False

    def set_extruder(self, state: bool):
        """
        Set the extruder state using the UR IO service.
        """
        if self.simulation:
            return True
        request = SetIO.Request()
        request.fun = SetIO.Request.FUN_SET_DIGITAL_OUT
        request.pin = self.EXTRUDER_DO_PIN
        if state:
            request.state = SetIO.Request.STATE_ON
        else:
            request.state = SetIO.Request.STATE_OFF
        future = self.io_set_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.success:
            self.get_logger().info(f"Set extruder state to {state}")
            return True
        else:
            self.get_logger().error("Failed to set extruder state")
            return False


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


if __name__ == "__main__":
    main()
