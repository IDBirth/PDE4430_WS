#!/usr/bin/env python3
from typing import Optional, Tuple
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class CircleBallVisualizer(Node):
    """Publishes /camera/image_ball with the detected ball outlined (same detector logic as circle_ball_detector)."""

    def __init__(self):
        super().__init__("circle_ball_visualizer")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("output_topic", "/camera/image_ball")

        self.declare_parameter("sat_min", 70)
        self.declare_parameter("val_min", 40)
        self.declare_parameter("morph_kernel", 5)
        self.declare_parameter("min_circularity", 0.65)

        self.declare_parameter("min_radius_px", 8.0)
        self.declare_parameter("max_radius_px", 200.0)

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, self.get_parameter("output_topic").value, 10)
        self.sub = self.create_subscription(Image, self.get_parameter("image_topic").value, self.image_cb, 10)

        self.get_logger().info("CircleBallVisualizer started.")

    def image_cb(self, msg: Image):
        try:
            img_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        found = self.find_ball_circle(img_bgr)
        if found is not None:
            u, v, r = found
            u_i, v_i, r_i = int(u), int(v), int(r)
            cv2.circle(img_bgr, (u_i, v_i), r_i, (0, 0, 255), 2)
            cv2.circle(img_bgr, (u_i, v_i), 2, (0, 0, 255), 3)

        out = self.bridge.cv2_to_imgmsg(img_bgr, encoding="bgr8")
        out.header = msg.header
        self.pub.publish(out)

    def find_ball_circle(self, img_bgr: np.ndarray) -> Optional[Tuple[float, float, float]]:
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        _, s, v = cv2.split(hsv)

        sat_min = int(self.get_parameter("sat_min").value)
        val_min = int(self.get_parameter("val_min").value)

        mask = cv2.inRange(s, sat_min, 255) & cv2.inRange(v, val_min, 255)

        k = int(self.get_parameter("morph_kernel").value)
        k = max(3, k | 1)
        kernel = np.ones((k, k), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        min_circ = float(self.get_parameter("min_circularity").value)
        min_r = float(self.get_parameter("min_radius_px").value)
        max_r = float(self.get_parameter("max_radius_px").value)

        best_score = -1.0
        best = None

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 30.0:
                continue
            per = cv2.arcLength(cnt, True)
            if per < 1e-6:
                continue

            circularity = 4.0 * math.pi * area / (per * per)
            if circularity < min_circ:
                continue

            (u, v0), r = cv2.minEnclosingCircle(cnt)
            if r < min_r or r > max_r:
                continue

            score = circularity * r
            if score > best_score:
                best_score = score
                best = (float(u), float(v0), float(r))

        return best


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
