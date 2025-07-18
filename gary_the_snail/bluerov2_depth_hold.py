import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from sensor_msgs.msg import FluidPressure    # the Vector3 message type definition
from mavros_msgs.msg import ManualControl

import numpy as np
from time import time

ATMOSPHERIC_PRESSURE = 101325
GRAVITATIONAL_ACC = 9.81

class depth_hold(Node):
    def __init__(self):
        super().__init__("depth_hold")    # names the node when running
        
        self.target_depth = 5.0

        self.Kp = 70.0
        self.Ki = 0.0
        self.Kd = 0.0

        self.integral = 0.0
        self.last_error = 0.0
        

        self.sub = self.create_subscription(
            FluidPressure,        # the message type
            "/pressure",    # the topic name,
            self.depth_callback,  # the subscription's callback method
            10              # QOS (will be covered later)
        )

        self.pub = self.create_publisher(
            ManualControl,        # the message type
            "/manual_control",    # the topic name
            10              # QOS (will be covered later)
        )

        self.last_time = time()

        self.get_logger().info("initialized subscriber node")

    def calculate_depth(self, pressure):
        return (pressure - ATMOSPHERIC_PRESSURE) / (1000 * GRAVITATIONAL_ACC)

    def depth_callback(self, msg):
        depth = self.calculate_depth(msg.fluid_pressure)
        self.get_logger().info(f"depth: {depth}")

        error = self.target_depth - depth
        
        dt = time() - self.last_time
        self.integral += dt * error

        derivative = (error - self.last_error) / dt
        output = error * self.Kp + self.integral * self.Ki + derivative * self.Kd

        self.last_error = error
        self.last_time = time()

        self.publish_depth_move(-output)

    def publish_depth_move(self, z):
        msg = ManualControl()

        msg.z = z

        self.pub.publish(msg)

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