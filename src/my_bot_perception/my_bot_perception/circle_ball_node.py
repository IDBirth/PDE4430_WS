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


class CircleBallNode(Node):
    """
    Combined detector + visualizer:
      - Subscribes: /camera/image_raw, /camera/camera_info
      - Publishes: /pose_ball (PoseStamped), /camera/image_ball (annotated image)
      - Detects every N frames (default 5) using saturation/value segmentation + circularity gating.
      - Estimates 3D pose in camera frame via z ~= fx * R / r_px, trying multiple candidate radii.
      - Suppresses publishing if detection is low in the image (assume ball is in gripper).
    """

    def __init__(self):
        super().__init__("circle_ball_node")

        # Topics
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("pose_topic", "/pose_ball")
        self.declare_parameter("debug_image_topic", "/camera/image_ball")

        # Run detection every N frames
        self.declare_parameter("process_every_n_frames", 5)

        # Known ball radii (meters). From your spawner: [0.1, 0.2, 0.3]
        self.declare_parameter("ball_radii_m", [0.1, 0.2, 0.3])

        # Depth plausibility window (meters) for selecting the best radius hypothesis
        self.declare_parameter("min_depth_m", 0.15)
        self.declare_parameter("max_depth_m", 8.0)

        # Simple smoothing on published pose (0=off, closer to 1=smoother)
        self.declare_parameter("ema_alpha", 0.35)

        # "In gripper" gating (image-space)
        self.declare_parameter("gripper_y_threshold_ratio", 0.80)  # bottom 20% => don't publish

        # Circle size gating (pixels)
        self.declare_parameter("min_radius_px", 8.0)
        self.declare_parameter("max_radius_px", 220.0)
        self.declare_parameter("gripper_max_radius_px", 300.0)  # too big => probably in gripper/too close

        # Saturation/value gating (colored balls vs grey pillars/walls)
        self.declare_parameter("sat_min", 70)
        self.declare_parameter("val_min", 40)
        self.declare_parameter("morph_kernel", 5)

        # Circularity gating (1.0 = perfect circle)
        self.declare_parameter("min_circularity", 0.65)

        # Debug image behavior
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_every_frame", True)  # if False, only publish on detection frames
        self.declare_parameter("draw_text", True)

        self.bridge = CvBridge()
        self.frame_count = 0

        # Camera intrinsics (from CameraInfo)
        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.cx: Optional[float] = None
        self.cy: Optional[float] = None
        self.camera_frame: Optional[str] = None

        # Last detection for drawing
        self._last_circle: Optional[Tuple[float, float, float]] = None  # (u, v, r_px)
        self._prev_xyz: Optional[Tuple[float, float, float]] = None

        # Publishers
        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter("pose_topic").get_parameter_value().string_value,
            10
        )

        self.debug_pub = self.create_publisher(
            Image,
            self.get_parameter("debug_image_topic").get_parameter_value().string_value,
            10
        )

        # Subscribers
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

        self.get_logger().info("CircleBallNode started (detector + visualizer).")

    def camera_info_cb(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.camera_frame = msg.header.frame_id if msg.header.frame_id else "camera"

    def image_cb(self, msg: Image):
        try:
            img_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        h, _ = img_bgr.shape[:2]
        publish_debug = bool(self.get_parameter("publish_debug_image").get_parameter_value().bool_value)
        debug_every_frame = bool(self.get_parameter("debug_every_frame").get_parameter_value().bool_value)

        # Detection cadence
        self.frame_count += 1
        n = int(self.get_parameter("process_every_n_frames").get_parameter_value().integer_value)
        do_detect = (n <= 1) or (self.frame_count % n == 0)

        circle = self._last_circle

        if do_detect:
            circle = self.find_ball_circle(img_bgr)
            self._last_circle = circle

            # Publish pose only if we can estimate 3D
            if circle is not None and self.fx is not None:
                u, v, r_px = circle

                # Size gating
                min_r = float(self.get_parameter("min_radius_px").get_parameter_value().double_value)
                max_r = float(self.get_parameter("max_radius_px").get_parameter_value().double_value)
                if not (min_r <= r_px <= max_r):
                    self._last_circle = None
                else:
                    # Gripper gating
                    y_ratio = float(self.get_parameter("gripper_y_threshold_ratio").get_parameter_value().double_value)
                    y_cut = int(y_ratio * h)
                    gripper_max_r = float(self.get_parameter("gripper_max_radius_px").get_parameter_value().double_value)

                    if int(v) < y_cut and r_px < gripper_max_r:
                        xyz = self.circle_to_pose_multi_radius(u, v, r_px)
                        if xyz is not None:
                            xyz = self.apply_ema(xyz)

                            out = PoseStamped()
                            out.header.stamp = msg.header.stamp
                            out.header.frame_id = self.camera_frame if self.camera_frame else msg.header.frame_id
                            out.pose.position.x = float(xyz[0])
                            out.pose.position.y = float(xyz[1])
                            out.pose.position.z = float(xyz[2])
                            out.pose.orientation.w = 1.0
                            self.pose_pub.publish(out)

        # Publish debug image
        if publish_debug and (debug_every_frame or do_detect):
            dbg = img_bgr.copy()

            if circle is not None:
                u, v, r = circle
                cv2.circle(dbg, (int(u), int(v)), int(r), (0, 0, 255), 2)
                cv2.circle(dbg, (int(u), int(v)), 2, (0, 0, 255), 3)

                if bool(self.get_parameter("draw_text").get_parameter_value().bool_value):
                    text = f"r_px={r:.1f}"
                    if self._prev_xyz is not None:
                        text += f" z={self._prev_xyz[2]:.2f}m"
                    cv2.putText(
                        dbg,
                        text,
                        (max(5, int(u) - 40), max(20, int(v) - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )

            out_img = self.bridge.cv2_to_imgmsg(dbg, encoding="bgr8")
            out_img.header = msg.header
            self.debug_pub.publish(out_img)

    def find_ball_circle(self, img_bgr: np.ndarray) -> Optional[Tuple[float, float, float]]:
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        _, s, v = cv2.split(hsv)

        sat_min = int(self.get_parameter("sat_min").get_parameter_value().integer_value)
        val_min = int(self.get_parameter("val_min").get_parameter_value().integer_value)

        mask = cv2.inRange(s, sat_min, 255) & cv2.inRange(v, val_min, 255)

        k = int(self.get_parameter("morph_kernel").get_parameter_value().integer_value)
        k = max(3, k | 1)
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

            score = circularity * r
            if score > best_score:
                best_score = score
                best_circle = (float(u), float(v0), float(r))

        return best_circle

    def circle_to_pose_multi_radius(
        self, u: float, v: float, r_px: float
    ) -> Optional[Tuple[float, float, float]]:
        if r_px <= 1e-6 or self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            return None

        radii_param = self.get_parameter("ball_radii_m").get_parameter_value().double_array_value
        radii: List[float] = list(radii_param) if len(radii_param) else [0.2]

        min_depth = float(self.get_parameter("min_depth_m").get_parameter_value().double_value)
        max_depth = float(self.get_parameter("max_depth_m").get_parameter_value().double_value)

        candidates: List[Tuple[float, float, float]] = []

        for R in radii:
            z = (self.fx * float(R)) / r_px
            if not (min_depth <= z <= max_depth):
                continue

            x = (u - self.cx) * z / self.fx
            y = (v - self.cy) * z / self.fy

            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z) and z > 0.0:
                candidates.append((x, y, z))

        if not candidates:
            return None

        if self._prev_xyz is not None:
            px, py, pz = self._prev_xyz
            candidates.sort(key=lambda c: (c[2] - pz) ** 2 + (c[0] - px) ** 2 + (c[1] - py) ** 2)
            return candidates[0]

        candidates.sort(key=lambda c: c[2])
        return candidates[len(candidates) // 2]

    def apply_ema(self, xyz: Tuple[float, float, float]) -> Tuple[float, float, float]:
        alpha = float(self.get_parameter("ema_alpha").get_parameter_value().double_value)
        alpha = max(0.0, min(1.0, alpha))

        if self._prev_xyz is None or alpha <= 0.0:
            self._prev_xyz = xyz
            return xyz

        px, py, pz = self._prev_xyz
        x, y, z = xyz
        smoothed = (
            px * alpha + x * (1.0 - alpha),
            py * alpha + y * (1.0 - alpha),
            pz * alpha + z * (1.0 - alpha),
        )
        self._prev_xyz = smoothed
        return smoothed


def main():
    rclpy.init()
    node = CircleBallNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
