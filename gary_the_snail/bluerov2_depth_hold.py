import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from sensor_msgs.msg import FluidPressure    # the Vector3 message type definition

import numpy as np

ATMOSPHERIC_PRESSURE = 101325
GRAVITATIONAL_ACC = 9.81

class depth_hold(Node):
    def __init__(self):
        super().__init__("depth_hold")    # names the node when running

        self.sub = self.create_subscription(
            FluidPressure,        # the message type
            "/pressure",    # the topic name,
            self.depth_callback,  # the subscription's callback method
            10              # QOS (will be covered later)
        )

        self.get_logger().info("initialized subscriber node")

    def calculate_depth(pressure):
        return (pressure - ATMOSPHERIC_PRESSURE) / (1000 * GRAVITATIONAL_ACC)

    def depth_callback(self, msg):
        self.get_logger().info("depth: " + str(self.calculate_depth(msg.pressure)))  

def main(args=None):
    rclpy.init(args=args)
    node = depth_hold()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()