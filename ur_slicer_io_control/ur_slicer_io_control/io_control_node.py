import rclpy
import threading
from rclpy.node import Node
from ur_slicer_interfaces.msg import NozzleTemperature
from ur_slicer_interfaces.srv import ExtruderControl, HeaterControl
from ur_msgs.msg import ToolDataMsg
from ur_msgs.srv import SetIO
from functools import partial


class URExtruderInterface(Node):
    def __init__(self):
        super().__init__("ur5e_extruder_interface")

        # Declare parameters first
        self.declare_parameter("simulation", True)
        self.simulation = self.get_parameter("simulation").value

        # Hardware configuration
        self.HEATER_DO_PIN = SetIO.Request.PIN_TOOL_DOUT1
        self.EXTRUDER_DO_PIN = SetIO.Request.PIN_TOOL_DOUT0

        # State management
        self.latest_temperature = 0.0
        self.desired_temperature = 0.0
        self.maintain_heater = False
        self.heater_on = False

        # Thread safety
        self.service_lock = threading.Lock()
        self.pending_requests = {"heater": None, "extruder": None}

        # Initialize components in correct order
        self._init_clients()
        self._init_publishers()
        self._init_subscriptions()
        self._init_services()

        self.get_logger().info("UR5e Extruder Interface Node Initialized")

    def _init_clients(self):
        """Initialize service clients first for early availability"""
        self.io_set_client = self.create_client(
            SetIO, "/io_and_status_controller/set_io"
        )
        while not self.io_set_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for UR IO service...")

    def _init_publishers(self):
        """Initialize publishers and timer with correct callback reference"""
        self.temp_pub = self.create_publisher(
            NozzleTemperature, "extruder_temperature", 1
        )
        self.timer = self.create_timer(5.0, self.extruder_temp_pub)  # Float period

    def _init_subscriptions(self):
        """Initialize subscriptions after publishers"""
        self.create_subscription(
            ToolDataMsg,
            "/io_and_status_controller/tool_data",
            self.tool_data_callback,
            1,
        )

    def _init_services(self):
        """Initialize services last to ensure dependencies are ready"""
        self.create_service(HeaterControl, "heater_control", self.handle_heater_control)
        self.create_service(
            ExtruderControl, "extruder_control", self.handle_extruder_control
        )

    # ------------------ Publisher Method --------------------
    def extruder_temp_pub(self):
        """Timer callback for temperature publishing"""
        msg = NozzleTemperature()
        msg.nozzle_temperature = self.latest_temperature
        self.temp_pub.publish(msg)
        self.get_logger().info(
            f"Published extruder temperature: {self.latest_temperature:.2f} °C"
        )
        if self.heater_on and not self.maintain_heater:
            self.set_heater(False)
            self.heater_on = False

    # ------------------ Subscription Callback --------------------
    def tool_data_callback(self, msg: ToolDataMsg):
        """Process tool data and manage heater state"""
        analog_signal = msg.analog_input3
        self.latest_temperature = int((analog_signal) * 100.0)

        if self.simulation:
            self.latest_temperature = int(self.desired_temperature)
            return

        if self.maintain_heater:
            target_state = self.latest_temperature < self.desired_temperature
            if target_state != self.heater_on:
                self.set_heater(target_state)
                self.heater_on = target_state
        return

    # ------------------ Service Handlers --------------------
    def handle_heater_control(self, request, response):
        """Handle heater control service requests"""
        self.desired_temperature = request.desired_temperature

        if request.heater_on and request.heater_off:
            self.get_logger().error("Invalid heater state request: both ON and OFF")
            response.control_confirmed = False
            return response

        if request.heater_on:
            self.maintain_heater = True
            self.get_logger().info("Heater control: ON")
        elif request.heater_off:
            self.maintain_heater = False
            self.get_logger().info("Heater control: OFF")

        response.control_confirmed = True
        return response

    def handle_extruder_control(self, request, response):
        """Handle extruder control service requests"""
        if request.on_extruder and request.off_extruder:
            self.get_logger().error("Invalid extruder state request: both ON and OFF")
            response.control_confirmed = False
            return response

        target_state = None
        if request.on_extruder:
            target_state = True
            self.get_logger().info("Extruder control: ON")
        elif request.off_extruder:
            target_state = False
            self.get_logger().info("Extruder control: OFF")

        if target_state is not None:
            response.control_confirmed = self.set_extruder(target_state)
        else:
            response.control_confirmed = False

        return response

    # ------------------ Service Clients --------------------
    def set_heater(self, state: bool) -> bool:
        """Thread-safe heater control with async calls"""
        with self.service_lock:
            if (
                self.pending_requests["heater"]
                and not self.pending_requests["heater"].done()
            ):
                self.get_logger().warning("Heater command already in progress")
                return False

            request = SetIO.Request(
                fun=SetIO.Request.FUN_SET_DIGITAL_OUT,
                pin=self.HEATER_DO_PIN,
                state=float(
                    SetIO.Request.STATE_ON if state else SetIO.Request.STATE_OFF
                ),
            )

            future = self.io_set_client.call_async(request)
            future.add_done_callback(
                partial(self._heater_response_cb, target_state=state)
            )
            self.pending_requests["heater"] = future
            return True

    def set_extruder(self, state: bool) -> bool:
        """Thread-safe extruder control with async calls"""
        with self.service_lock:
            if (
                self.pending_requests["extruder"]
                and not self.pending_requests["extruder"].done()
            ):
                self.get_logger().warning("Extruder command already in progress")
                return False

            request = SetIO.Request(
                fun=SetIO.Request.FUN_SET_DIGITAL_OUT,
                pin=self.EXTRUDER_DO_PIN,
                state=float(
                    SetIO.Request.STATE_ON if state else SetIO.Request.STATE_OFF
                ),
            )

            future = self.io_set_client.call_async(request)
            future.add_done_callback(
                partial(self._extruder_response_cb, target_state=state)
            )
            self.pending_requests["extruder"] = future
            return True

    # ------------------ Response Callbacks --------------------
    def _heater_response_cb(self, future, target_state):
        """Handle heater service response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"Heater {'ON' if target_state else 'OFF'} confirmed"
                )
                self.heater_on = target_state
            else:
                self.get_logger().error("Heater control failed")
        except Exception as e:
            self.get_logger().error(f"Heater service error: {str(e)}")

    def _extruder_response_cb(self, future, target_state):
        """Handle extruder service response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"Extruder {'ON' if target_state else 'OFF'} confirmed"
                )
            else:
                self.get_logger().error("Extruder control failed")
        except Exception as e:
            self.get_logger().error(f"Extruder service error: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = URExtruderInterface()

    try:
        # Use multi-threaded executor for concurrent processing
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
