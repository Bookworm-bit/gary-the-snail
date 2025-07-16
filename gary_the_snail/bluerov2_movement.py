import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from mavros_msgs.msg import ManualControl    # the Vector3 message type definition

class movement(Node):
    def __init__(self):
        super().__init__("movement")    # names the node when running

        self.pub = self.create_publisher(
            ManualControl,        # the message type
            "/manual_control",    # the topic name
            10              # QOS (will be covered later)
        )

        self.timer = self.create_timer(
            10.0,    # timer period (sec)
            self.publish_motor_control    # callback function
        )

        self.get_logger().info("initialized publisher node")

    def publish_motor_control(self):
        msg = ManualControl()

        msg.x = 100.0
        msg.y = 0.0
        msg.z = 0.0
        msg.r = 0.0
        msg.buttons = 0

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = movement()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

