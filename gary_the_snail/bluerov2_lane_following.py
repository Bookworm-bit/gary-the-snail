import rclpy
from rclpy.node import Node

from gary_the_snail import lane_detection
from gary_the_snail import lane_following

from std_msgs.msg import Int16, Float32
from sensor_msgs.msg import Image

import numpy as np
from time import time

class lane_following(Node):
    def __init__(self):
        super().__init__("lane_following")

        self.pub_lateral = self.create_publisher(
            Float32,
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

        self.Kp = 60.0
        self.Ki = 8.0
        self.Kd = 50.0

        self.integral = 0.0
        self.last_error = 0.0
        
    def camera_callback(self, msg):
        img = np.array(msg.data)
        img = lane_detection.crop_half(img)
        self.IMAGE_WIDTH = msg.width
        self.IMAGE_HEIGHT = msg.height

        self.lane_follow_publisher(img)

    def lane_follow_publisher(self, img):
        lines = lane_detection.detect_lines(img, threshold1=20, threshold2=60, aperture_size=3, minLineLength=25, maxLineGap=25)
        lanes = lane_detection.detect_lanes(lines)

        intercept, slope = lane_following.get_lane_center(img, lanes)
        recommended = lane_following.recommend_direction(img, intercept, slope)

        msg = Int16()
        if recommended == "counter":
            msg.data = -20
        elif recommended == "clock":
            msg.data = 20
        self.pub_relative_heading.publish(msg)

        error = self.IMAGE_WIDTH // 2 - intercept

        if error < 25:
            msg = Int16()

            angle_line = np.arctan(slope)
            relative_angle = np.pi / 2 - angle_line

            msg.data = int(relative_angle)
            self.pub_relative_heading(msg)
        else:
            dt = time() - self.last_time
            self.integral += max(-20.0, min(20.0, dt*error))
            
            derivative = (error - self.last_error) / dt
            output = error * self.Kp + self.integral * self.Ki + derivative * self.Kd

            self.last_error = error
            self.last_time = time()

            msg = Float32()
            msg.data = output

            self.publish_lateral(msg)