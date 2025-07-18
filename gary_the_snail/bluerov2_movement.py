import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from mavros_msgs.msg import ManualControl    # the Vector3 message type definition
from time import time, sleep

TURN_180_TIME = 0 # seconds
TURN_360_TIME = 0 # seconds

class movement(Node):
    def __init__(self):
        super().__init__("movement")    # names the node when running
        
        self.LIST_MOVES = [("forward", 2), ("counter", 2), ("left", 2), ("clock", 2), ("left", 2), ("counter", 2), ("right", 2), ("clock", 2), ("right", 1), ("left", 1), ("right", 1), ("forward", 1), ("backward", 1), ("forward", 1), ("clock", 2), ("counter", 2), ("down", 1), ("up", 1), ("clock", 2), ("counter", 2), ("down", 1), ("up", 1), ("down", 3), ("up", 1), ("up", 1), ("up", 1), ("counter", 2), ("clock", 2), ("right", 1),("down", 1), ("left", 1), ("right", 1), ("left", 1), ("right", 3), ("left", 2), ("right", 2), ("forward", 3), ("left", 3)] # each move is a tuple(move_type, time)
        #self.LIST_MOVES = [("forward", 2), ("left", 2), ("left", 2), ("right", 2), ("right", 1), ("left", 1), ("right", 1), ("forward", 1), ("backward", 2), ("down", 1), ("up", 1), ("down", 1), ("up", 1), ("down", 3), ("up", 1), ("up", 1), ("up", 1), ("right", 1),("down", 1), ("left", 1), ("right", 1), ("left", 1), ("right", 3), ("left", 2), ("right", 2), ("forward", 3), ("left", 3)] # each move is a tuple(move_type, time)
        self.LIST_MOVES = [("forward", 1),("left", 2)]
        self.pub = self.create_publisher(
            ManualControl,        # the message type
            "/manual_control",    # the topic name
            10              # QOS (will be covered later)
        )

        self.get_logger().info("initialized movement node")

        # self.play_moves()

    def publish_move(self, move, t, pct=0):
        """
        Does a move relative to the AUV

        Options:
        - up
        - down
        - left
        - right
        - forward
        - backward
        - turn (category)
            - clock
            - counter
        """
        msg = ManualControl()

        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0
        msg.r = 0.0

        if move == "forward":
            msg.x = pct
        elif move == "backward":
            msg.x = -pct
        elif move == "left":
            msg.y = -pct
        elif move == "right":
            msg.y = pct
        elif move == "up":
            msg.z = pct
        elif move == "down":
            msg.z = -pct
        else:
            msg.x = 0.0
            msg.y = 0.0
            msg.z = 0.0
            msg.r = 0.0
        
        self.pub.publish(msg)

    def time_move(self, move, t):
        start = time()

        self.move_timer = self.create_timer(
            0.1,
            self.publish_move(move, t, pct=30)
        )

        while time() - start < t:
            if time() - start >= 0.8 * t:
                self.publish_move(move, pct=30 * (time() - start) / t)
            else:
                self.publish_move(move, pct=30.0)
            sleep(0.1)


    def play_moves(self):
        self.publish_move("stop")

        for move in self.LIST_MOVES:
            self.get_logger().info("started " + move[0])

            self.time_move(move[0], move[1])
            self.time_move("stop", 1.0)
            self.get_logger().info("ended " + move[0])
        
        self.publish_move("stop")
        
        self.get_logger().info("stopped dance")

def main(args=None):
    rclpy.init(args=args)

    node = movement()

    node.play_moves()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

