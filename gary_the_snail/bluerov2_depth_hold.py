import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Int16, Float32

import numpy as np
from time import time

class depth_hold(Node):
    def __init__(self):
        super().__init__("depth_hold")    # names the node when running
        
        self.Kp = 70.0
        self.Ki = 0.0
        self.Kd = 0.0

        self.integral = 0.0
        self.last_error = 0.0
        self.target_depth = 1.0

        self.pub = self.create_publisher(
            ManualControl,        # the message type
            "/manual_control",    # the topic name
            10              # QOS (will be covered later)
        )

        self.sub_target = self.create_subscription(
            Int16,
            "/target_depth",
            self.get_target_depth,
            10
        )

        self.sub_depth = self.create_subscription(
            Float32, 
            "/depth", 
            self.get_depth,
            10
        )

        self.depth = 0.0

        self.last_time = time()

        self.get_logger().info("initialized depth hold subscriber node")

    def get_depth(self, msg):
        self.depth = msg.data

        error = self.target_depth - self.depth
        self.get_logger().info("error: " + str(error))
        
        dt = time() - self.last_time
        self.integral += max(-20.0, min(20.0, dt*error))
        
        derivative = (error - self.last_error) / dt
        output = error * self.Kp + self.integral * self.Ki + derivative * self.Kd

        self.last_error = error
        self.last_time = time()

        self.publish_depth_move(-output)

    def get_target_depth(self, msg):
        self.target_depth = msg.data

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