#!/usr/bin/env python3
import math
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

from cv_bridge import CvBridge
import cv2
import numpy as np


class CircleBallDetector(Node):
    """
    Detects a colored spherical ball in /camera/image_raw and publishes /pose_ball (PoseStamped).
    - Processes every N frames (default 5).
    - Uses saturation/value segmentation + circularity gating (robust vs walls/columns).
    - Estimates depth from apparent radius using candidate real-world radii, auto-picking the best.
    - Suppresses publishing when the detection is low in the image (assume ball is in gripper).
    """

    def __init__(self):
        super().__init__("circle_ball_detector")

        # Topics
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("pose_topic", "/pose_ball")

        # Run every N frames
        self.declare_parameter("process_every_n_frames", 5)

        # Known ball radii (meters). From your spawner: [0.1, 0.2, 0.3]
        # If you only want one size, set this to [0.2] etc.
        self.declare_parameter("ball_radii_m", [0.1, 0.2, 0.3])

        # Depth plausibility window (meters) for selecting the best radius hypothesis
        self.declare_parameter("min_depth_m", 0.15)
        self.declare_parameter("max_depth_m", 8.0)

        # Simple smoothing on the published position (0=off, closer to 1 = smoother)
        self.declare_parameter("ema_alpha", 0.35)

        # "In gripper" gating (image-space)
        self.declare_parameter("gripper_y_threshold_ratio", 0.80)  # bottom 20% => don't publish

        # Circle size gating (pixels)
        self.declare_parameter("min_radius_px", 8.0)
        self.declare_parameter("max_radius_px", 200.0)
        self.declare_parameter("gripper_max_radius_px", 260.0)  # very large => probably in gripper/too close

        # Saturation-based segmentation (works well in your grey/colored Gazebo scene)
        self.declare_parameter("sat_min", 70)
        self.declare_parameter("val_min", 40)
        self.declare_parameter("morph_kernel", 5)

        # Circularity gating (1.0 = perfect circle)
        self.declare_parameter("min_circularity", 0.65)

        self.bridge = CvBridge()
        self.frame_count = 0

        # Camera intrinsics (from CameraInfo)
        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.cx: Optional[float] = None
        self.cy: Optional[float] = None
        self.camera_frame: Optional[str] = None

        # For smoothing / best-hypothesis selection
        self._prev_xyz: Optional[Tuple[float, float, float]] = None

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

        self.get_logger().info("CircleBallDetector started (segmentation + multi-radius depth).")

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

        h, _ = img_bgr.shape[:2]

        # In-gripper gating line
        y_ratio = self.get_parameter("gripper_y_threshold_ratio").get_parameter_value().double_value
        y_cut = int(y_ratio * h)

        found = self.find_ball_circle(img_bgr)
        if found is None:
            return

        u, v, r_px = found

        # Size gating
        min_r = self.get_parameter("min_radius_px").get_parameter_value().double_value
        max_r = self.get_parameter("max_radius_px").get_parameter_value().double_value
        if r_px < min_r or r_px > max_r:
            return

        # Gripper gating (by y and by huge radius)
        if int(v) >= y_cut:
            return
        gripper_max_r = self.get_parameter("gripper_max_radius_px").get_parameter_value().double_value
        if r_px >= gripper_max_r:
            return

        xyz = self.circle_to_pose_multi_radius(u, v, r_px)
        if xyz is None:
            return

        x, y, z = self.apply_ema(xyz)

        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.camera_frame if self.camera_frame else msg.header.frame_id
        out.pose.position.x = float(x)
        out.pose.position.y = float(y)
        out.pose.position.z = float(z)
        out.pose.orientation.w = 1.0  # orientation not estimated here

        self.pose_pub.publish(out)

    def find_ball_circle(self, img_bgr: np.ndarray) -> Optional[Tuple[float, float, float]]:
        """Detect a colored spherical ball by thresholding saturation/value and selecting the most circular blob."""
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        _, s, v = cv2.split(hsv)

        sat_min = int(self.get_parameter("sat_min").get_parameter_value().integer_value)
        val_min = int(self.get_parameter("val_min").get_parameter_value().integer_value)

        mask = cv2.inRange(s, sat_min, 255) & cv2.inRange(v, val_min, 255)

        k = int(self.get_parameter("morph_kernel").get_parameter_value().integer_value)
        k = max(3, k | 1)  # ensure odd >=3
        kernel = np.ones((k, k), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        min_circ = float(self.get_parameter("min_circularity").get_parameter_value().double_value)
        min_r = float(self.get_parameter("min_radius_px").get_parameter_value().double_value)
        max_r = float(self.get_parameter("max_radius_px").get_parameter_value().double_value)

        best_score = -1.0
        best_circle = None

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

            # Prefer larger + more circular blobs (usually the ball)
            score = circularity * r
            if score > best_score:
                best_score = score
                best_circle = (float(u), float(v0), float(r))

        return best_circle

    def circle_to_pose_multi_radius(self, u: float, v: float, r_px: float) -> Optional[Tuple[float, float, float]]:
        """
        Estimate 3D position in camera frame using z ≈ fx * R / r_px.
        Tries multiple candidate radii and chooses the best plausible depth.
        """
        if r_px <= 1e-6:
            return None

        # Read radii list
        radii_param = self.get_parameter("ball_radii_m").get_parameter_value().double_array_value
        radii: List[float] = list(radii_param) if len(radii_param) else []

        if not radii:
            # fallback: assume medium
            radii = [0.2]

        min_z = float(self.get_parameter("min_depth_m").get_parameter_value().double_value)
        max_z = float(self.get_parameter("max_depth_m").get_parameter_value().double_value)

        candidates: List[Tuple[float, float, float]] = []
        for R in radii:
            z = (self.fx * float(R)) / r_px
            if not (math.isfinite(z) and z > 0.0):
                continue
            if z < min_z or z > max_z:
                continue

            x = (u - self.cx) * z / self.fx
            y = (v - self.cy) * z / self.fy

            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            candidates.append((x, y, z))

        if not candidates:
            return None

        # Choose candidate closest to previous depth/position if available; else pick the middle-depth hypothesis
        if self._prev_xyz is not None:
            px, py, pz = self._prev_xyz
            candidates.sort(key=lambda c: (c[2]-pz)**2 + (c[0]-px)**2 + (c[1]-py)**2)
            return candidates[0]

        candidates.sort(key=lambda c: c[2])
        return candidates[len(candidates)//2]

    def apply_ema(self, xyz: Tuple[float, float, float]) -> Tuple[float, float, float]:
        alpha = float(self.get_parameter("ema_alpha").get_parameter_value().double_value)
        alpha = max(0.0, min(0.95, alpha))

        if self._prev_xyz is None or alpha <= 1e-6:
            self._prev_xyz = xyz
            return xyz

        px, py, pz = self._prev_xyz
        x, y, z = xyz
        sx = alpha * px + (1.0 - alpha) * x
        sy = alpha * py + (1.0 - alpha) * y
        sz = alpha * pz + (1.0 - alpha) * z
        self._prev_xyz = (sx, sy, sz)
        return self._prev_xyz


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
