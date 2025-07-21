import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from mavros_msgs.msg import ManualControl
from time import time

class heading_control(Node):
    def __init__(self):
        super().__init__("heading_control")
        
        self.target_heading = 120

        self.Kp = 6.0
        self.Ki = 0.3
        self.Kd = 0.075
        
        self.integral = 0.0
        self.last_error = 0.0

        self.sub = self.create_subscription(
            Int16,
            "/heading",
            self.heading_callback,
            10
        )

        self.pub = self.create_publisher(
            ManualControl,
            "/manual_control",
            10
        )

        self.sub_target = self.create_subscription(
            Int16,
            "/target_heading",
            self.get_target_heading,
            10
        )
        
        self.last_time = time()
        self.get_logger().info("initialized heading control subscriber node")

    def get_target_heading(self, msg):
        self.target_heading = msg.data

    def heading_callback(self, msg):
        heading = msg.data
        self.get_logger().info(f"heading: {heading}")

        if heading > 180.0:
            heading = heading - 360

        error = self.target_heading - heading
        
        dt = time() - self.last_time
        self.integral += max(-20.0, min(20.0, dt*error))

        derivative = (error - self.last_error) / dt
        output = error * self.Kp + self.integral * self.Ki + derivative * self.Kd

        self.last_error = error
        self.last_time = time()

        self.publish_rotation(output)
    
    def publish_rotation(self, r):
        r = max(-32767.0, min(32766.0, float(r)))
        msg = ManualControl()
        msg.r = r

        self.pub.publish(msg)

    
def main(args=None):
    rclpy.init(args=args)
    node = heading_control()    

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()