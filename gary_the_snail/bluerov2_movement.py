import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from mavros_msgs.msg import ManualControl    # the Vector3 message type definition
from time import sleep

LIST_MOVES = [] # each move is a tuple(turn_status, time)

class movement(Node):
    def __init__(self):
        super().__init__("movement")    # names the node when running

        self.pub = self.create_publisher(
            ManualControl,        # the message type
            "/manual_control",    # the topic name
            10              # QOS (will be covered later)
        )

        self.play_moves()

        self.get_logger().info("initialized publisher node")

    def publish_move(self, turn):
        msg = ManualControl()

        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0
        msg.r = 0.0

        if turn:
            msg.x = 70.0
        else:
            msg.r = 70.0

        self.pub.publish(msg)

    def play_moves(self):
        for move in LIST_MOVES:
            self.publish_move(move[0], move[1])
        

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

