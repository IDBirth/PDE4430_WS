#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

from cv_bridge import CvBridge
import cv2
import numpy as np


class CircleBallDetector(Node):
    def __init__(self):
        super().__init__("circle_ball_detector")

        # Topics
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("pose_topic", "/pose_ball")

        # Frame skipping
        self.declare_parameter("process_every_n_frames", 1)

        # Ball model (meters) - tune to your spheres
        self.declare_parameter("ball_radius_m", 0.005)

        # "In gripper" gating (image-space)
        self.declare_parameter("gripper_y_threshold_ratio", 0.80)  # bottom 20% => don't publish

        # Optional extra gating by size (close == likely in gripper)
        self.declare_parameter("min_radius_px", 6.0)
        self.declare_parameter("max_radius_px", 500.0)
        self.declare_parameter("gripper_max_radius_px", 180.0)  # if detected circle too big => don't publish

        # HoughCircles params (tune!)
        self.declare_parameter("hough_dp", 1.2)
        self.declare_parameter("hough_min_dist", 60.0)
        self.declare_parameter("hough_param1", 120.0)  # Canny high threshold
        self.declare_parameter("hough_param2", 25.0)   # accumulator threshold
        self.declare_parameter("hough_min_radius", 6)
        self.declare_parameter("hough_max_radius", 0)  # 0 => no limit

        self.bridge = CvBridge()
        self.frame_count = 0

        # Camera intrinsics (from CameraInfo)
        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.cx: Optional[float] = None
        self.cy: Optional[float] = None
        self.camera_frame: Optional[str] = None

        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter("pose_topic").get_parameter_value().string_value,
            10
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").get_parameter_value().string_value,
            self.camera_info_cb,
            10
        )

        self.img_sub = self.create_subscription(
            Image,
            self.get_parameter("image_topic").get_parameter_value().string_value,
            self.image_cb,
            10
        )

        self.get_logger().info("CircleBallDetector started.")

    def camera_info_cb(self, msg: CameraInfo):
        # K = [fx, 0, cx,
        #      0, fy, cy,
        #      0,  0,  1]
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.camera_frame = msg.header.frame_id if msg.header.frame_id else "camera"

    def image_cb(self, msg: Image):
        self.frame_count += 1
        n = self.get_parameter("process_every_n_frames").get_parameter_value().integer_value
        if n > 1 and (self.frame_count % n) != 0:
            return

        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            self.get_logger().warn("No /camera/camera_info yet; cannot estimate 3D pose.")
            return

        try:
            img_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        h, w = img_bgr.shape[:2]
        _ = w

        # If the ball is too low in the image => likely in gripper => don't publish
        y_ratio = self.get_parameter("gripper_y_threshold_ratio").get_parameter_value().double_value
        y_cut = int(y_ratio * h)

        # Preprocess
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (9, 9), 2.0)

        circles = self.detect_circles(gray)

        if circles is None or len(circles) == 0:
            return

        # Choose the "best" circle: largest radius (usually closest / most confident)
        # circles: Nx3 (x, y, r) float
        best = max(circles, key=lambda c: c[2])
        u, v, r_px = float(best[0]), float(best[1]), float(best[2])

        # Size gating
        min_r = self.get_parameter("min_radius_px").get_parameter_value().double_value
        max_r = self.get_parameter("max_radius_px").get_parameter_value().double_value
        if r_px < min_r or r_px > max_r:
            return

        # Gripper gating (by y position + optional radius)
        if int(v) >= y_cut:
            return

        gripper_max_r = self.get_parameter("gripper_max_radius_px").get_parameter_value().double_value
        if r_px >= gripper_max_r:
            return

        pose_cam = self.circle_to_pose(u, v, r_px)
        if pose_cam is None:
            return

        x, y, z = pose_cam

        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.camera_frame if self.camera_frame else msg.header.frame_id

        out.pose.position.x = float(x)
        out.pose.position.y = float(y)
        out.pose.position.z = float(z)

        # Orientation not estimated here
        out.pose.orientation.w = 1.0

        self.pose_pub.publish(out)

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

        circles = np.squeeze(circles, axis=0)  # shape (N,3)
        return circles

    def circle_to_pose(self, u: float, v: float, r_px: float) -> Optional[Tuple[float, float, float]]:
        # Estimate depth from apparent radius:
        # r_px ~= fx * R / z  => z ~= fx * R / r_px
        R = self.get_parameter("ball_radius_m").get_parameter_value().double_value

        if r_px <= 1e-6:
            return None

        z = (self.fx * R) / r_px

        # Back-project pixel to camera coordinates (pinhole)
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        # Basic sanity checks (optional)
        if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
            return None
        if z <= 0.0:
            return None

        return (x, y, z)


def main():
    rclpy.init()
    node = CircleBallDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
