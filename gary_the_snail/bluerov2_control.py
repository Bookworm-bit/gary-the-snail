import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from std_msgs.msg import Float32    # the Vector3 message type definition
from mavros_msgs.msg import ManualControl

class control(Node):
    def __init__(self):
        super().__init__("control_node")    # names the node when running

        self.pub = self.create_publisher(
            ManualControl,        # the message type
            "/manual_control",    # the topic name
            10              # QOS (will be covered later)
        )

        self.sub_depth = self.create_subscription(
            Float32,        # the message type
            "/depth_control",    # the topic name
            self.depth_control_callback,
            10              # QOS (will be covered later)
        )
        
        self.sub_heading = self.create_subscription(
            Float32,        # the message type
            "/heading_control",    # the topic name
            self.heading_control_callback,
            10              # QOS (will be covered later)
        )

        self.timer = self.create_timer(
            1.0,    # timer period (sec)
            self.publish_manual_control    # callback function
        )

        self.x = 0
        self.y = 0
        self.z = 0
        self.r = 0

        self.get_logger().info("initialized control node")

    def depth_control_callback(self, msg):
        self.z = msg.data

    def heading_control_callback(self, msg):
        self.r = msg.data

    def publish_manual_control(self):
        msg = ManualControl()

        msg.x = self.x
        msg.y = self.y
        msg.z = self.z
        msg.r = self.r

def main(args=None):
    rclpy.init(args=args)
    node = control()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

