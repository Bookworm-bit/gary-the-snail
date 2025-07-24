import rclpy
from rclpy.node import Node

from gary_the_snail import lane_detection
from gary_the_snail import lane_following

from std_msgs.msg import Int16, Float32
from sensor_msgs.msg import Image

import numpy as np

class lane_following(Node):
    def __init__(self):
        super().__init__("lane_following")

        self.pub_lateral = self.create_publisher(
            Int16,
            "/lane_following",
            10
        )

        self.pub_relative_heading = self.create_publisher(
            Int16,
            "/relative_heading",
            10
        )

        self.sub_camera = self.create_subscription(
            Image,
            "/camera", # bluerov2/camera ?
            self.camera_callback,
            10
        )
        
    def camera_callback(self, msg):
        img = np.array(msg.data)
        img = lane_detection.crop_half(img)
        self.IMAGE_WIDTH = msg.width
        self.IMAGE_HEIGHT = msg.height

        self.lane_follow_publisher(img)

    def lane_follow_publisher(self, img):
        lines = lane_detection.detect_lines(img, threshold1=20, threshold2=60, aperture_size=3, minLineLength=25, maxLineGap=25)
        lanes = lane_detection.detect_lanes(lines)

        intercept, slope = lane_following.get_lane_center(lanes)
        recommended = lane_following.recommend_direction(intercept, slope)

        msg = Int16()
        if recommended == "counter":
            msg.data = -20.0
        elif recommended == "clock":
            msg.data = 20.0
        self.pub_relative_heading.publish(msg)

    