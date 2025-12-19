#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


def yaw_to_quat(yaw: float) -> Quaternion:
    """Convert yaw (rad) to geometry_msgs/Quaternion."""
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q


class BallGoalTransformer(Node):
    """
    Subscribes:
      - /pose_ball (PoseStamped) in camera frame
    Uses TF2 to transform pose into target frame (default: map),
    then computes a standoff goal pose facing the ball.
    Publishes:
      - /goal_pose (PoseStamped) in target frame
    Optionally sends Nav2 NavigateToPose goals.
    """

    def __init__(self):
        super().__init__("ball_goal_transformer")

        # Topics
        self.declare_parameter("ball_pose_topic", "/pose_ball")
        self.declare_parameter("goal_pose_topic", "/goal_pose")

        # Frames
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")

        # Goal geometry
        self.declare_parameter("standoff_distance_m", 0.6)
        self.declare_parameter("min_goal_distance_m", 0.25)

        # Nav2 behavior
        self.declare_parameter("send_nav2_goal", False)
        self.declare_parameter("nav2_action_name", "navigate_to_pose")
        self.declare_parameter("cooldown_s", 2.0)

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Pub/sub
        self.goal_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter("goal_pose_topic").get_parameter_value().string_value,
            10
        )

        self.ball_sub = self.create_subscription(
            PoseStamped,
            self.get_parameter("ball_pose_topic").get_parameter_value().string_value,
            self.ball_cb,
            10
        )

        # Nav2 Action client (optional)
        self.nav2_client: Optional[ActionClient] = None
        if bool(self.get_parameter("send_nav2_goal").value):
            action_name = str(self.get_parameter("nav2_action_name").value)
            self.nav2_client = ActionClient(self, NavigateToPose, action_name)

        self._last_sent_time = self.get_clock().now()
        self._last_goal: Optional[PoseStamped] = None

        self.get_logger().info("BallGoalTransformer started.")

    def ball_cb(self, msg: PoseStamped):
        target_frame = str(self.get_parameter("target_frame").value)
        base_frame = str(self.get_parameter("robot_base_frame").value)

        # 1) Transform ball pose into target_frame
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame,
                msg.header.frame_id,
                rclpy.time.Time()
            )
            ball_in_target = do_transform_pose(msg, tf)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"TF transform failed ({msg.header.frame_id} -> {target_frame}): {e}"
            )
            return

        # 2) Get robot base position in target_frame
        try:
            tf_base = self.tf_buffer.lookup_transform(
                target_frame,
                base_frame,
                rclpy.time.Time()
            )
            robot_x = tf_base.transform.translation.x
            robot_y = tf_base.transform.translation.y
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"TF base lookup failed ({base_frame} in {target_frame}): {e}"
            )
            return

        bx = ball_in_target.pose.position.x
        by = ball_in_target.pose.position.y

        dx = bx - robot_x
        dy = by - robot_y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return

        min_goal_dist = float(self.get_parameter("min_goal_distance_m").value)
        if dist < min_goal_dist:
            return

        # 3) Compute standoff goal
        standoff = float(self.get_parameter("standoff_distance_m").value)
        ux = dx / dist
        uy = dy / dist

        goal_x = bx - ux * standoff
        goal_y = by - uy * standoff

        # 4) Face the ball
        yaw = math.atan2(by - goal_y, bx - goal_x)

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = target_frame
        goal.pose.position.x = float(goal_x)
        goal.pose.position.y = float(goal_y)
        goal.pose.position.z = 0.0
        goal.pose.orientation = yaw_to_quat(yaw)

        self.goal_pub.publish(goal)

        # 5) Optional Nav2 goal send
        if not bool(self.get_parameter("send_nav2_goal").value) or self.nav2_client is None:
            self._last_goal = goal
            return

        cooldown_s = float(self.get_parameter("cooldown_s").value)
        now = self.get_clock().now()
        if (now - self._last_sent_time).nanoseconds < cooldown_s * 1e9:
            return

        if not self.nav2_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("Nav2 action server not available yet.")
            return

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal

        self.nav2_client.send_goal_async(nav_goal)
        self._last_sent_time = now
        self._last_goal = goal
        self.get_logger().info(
            f"Sent Nav2 goal: ({goal_x:.2f}, {goal_y:.2f}) yaw={yaw:.2f} rad in {target_frame}"
        )


def main():
    rclpy.init()
    node = BallGoalTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
