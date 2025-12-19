#!/usr/bin/env python3
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2
import numpy as np


class CircleBallVisualizer(Node):
    def __init__(self):
        super().__init__("circle_ball_visualizer")

        # Topics
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("output_topic", "/camera/image_ball")

        # HoughCircles params (tune!)
        self.declare_parameter("hough_dp", 1.2)
        self.declare_parameter("hough_min_dist", 60.0)
        self.declare_parameter("hough_param1", 120.0)
        self.declare_parameter("hough_param2", 25.0)
        self.declare_parameter("hough_min_radius", 6)
        self.declare_parameter("hough_max_radius", 0)

        self.bridge = CvBridge()

        self.pub = self.create_publisher(
            Image,
            self.get_parameter("output_topic").get_parameter_value().string_value,
            10
        )

        self.sub = self.create_subscription(
            Image,
            self.get_parameter("image_topic").get_parameter_value().string_value,
            self.image_cb,
            10
        )

        self.get_logger().info("CircleBallVisualizer started.")

    def image_cb(self, msg: Image):
        try:
            img_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (9, 9), 2.0)

        circles = self.detect_circles(gray)
        if circles is None or len(circles) == 0:
            return

        best = max(circles, key=lambda c: c[2])
        u, v, r_px = int(best[0]), int(best[1]), int(best[2])

        # Draw red circle and center dot
        cv2.circle(img_bgr, (u, v), r_px, (0, 0, 255), 2)
        cv2.circle(img_bgr, (u, v), 2, (0, 0, 255), 3)

        out = self.bridge.cv2_to_imgmsg(img_bgr, encoding="bgr8")
        out.header = msg.header
        self.pub.publish(out)

    def detect_circles(self, gray: np.ndarray) -> Optional[np.ndarray]:
        dp = self.get_parameter("hough_dp").get_parameter_value().double_value
        min_dist = self.get_parameter("hough_min_dist").get_parameter_value().double_value
        p1 = self.get_parameter("hough_param1").get_parameter_value().double_value
        p2 = self.get_parameter("hough_param2").get_parameter_value().double_value
        min_r = self.get_parameter("hough_min_radius").get_parameter_value().integer_value
        max_r = self.get_parameter("hough_max_radius").get_parameter_value().integer_value

        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=dp,
            minDist=min_dist,
            param1=p1,
            param2=p2,
            minRadius=int(min_r),
            maxRadius=int(max_r) if max_r > 0 else 0
        )

        if circles is None:
            return None

        return np.squeeze(circles, axis=0)


def main():
    rclpy.init()
    node = CircleBallVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
