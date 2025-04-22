"""Testing node for bed corner publishing"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from ur_slicer_interfaces.msg import BedCorners


class BedCornersPublisher(Node):
    """Publisher node for test bed corners message."""

    def __init__(self):
        super().__init__("bed_corners_publisher")
        self.publisher_ = self.create_publisher(BedCorners, "/bed_corners", 1)
        self.timer = self.create_timer(0.1, self.publish_and_shutdown)
        self.shutdown_flag = False

    def publish_and_shutdown(self):
        """Publishes a sample bed corners message and then sets the shutdown flag."""
        msg = BedCorners()
        msg.corners = [
            Point(x=0.25, y=0.25, z=-0.415),
            Point(x=0.25, y=0.75, z=-0.415),
            Point(x=-0.25, y=0.75, z=-0.415),
            Point(x=-0.25, y=0.25, z=0 - 0.415),
        ]
        self.publisher_.publish(msg)
        self.get_logger().info("Published sample bed corners")
        self.shutdown_flag = True

    def spin_once(self):
        """Spins the node once and checks for the shutdown flag."""
        rclpy.spin_once(self)
        if self.shutdown_flag:
            self.destroy_node()
            return False
        return True


def main(args=None):
    """Main function for the node."""
    rclpy.init(args=args)
    node = BedCornersPublisher()
    while node.spin_once():
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
