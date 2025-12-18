import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


class ArmControl(Node):
    def __init__(self) -> None:
        super().__init__('arm_control')
        self._left_pub = self.create_publisher(Float64, '/left_arm/cmd_pos', 10)
        self._right_pub = self.create_publisher(Float64, '/right_arm/cmd_pos', 10)

        self._open_srv = self.create_service(Trigger, '/arms/open', self._handle_open)
        self._close_srv = self.create_service(Trigger, '/arms/close', self._handle_close)

        self._open_pos = 0.0
        self._close_pos = math.radians(30.0)
        self.get_logger().info('Arm control ready: /arms/open and /arms/close')

    def _publish_positions(self, left_pos: float, right_pos: float) -> None:
        left_msg = Float64()
        left_msg.data = left_pos
        right_msg = Float64()
        right_msg.data = right_pos

        self._left_pub.publish(left_msg)
        self._right_pub.publish(right_msg)

    def _handle_open(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._publish_positions(self._open_pos, self._open_pos)
        response.success = True
        response.message = 'Arms opening to 0 deg.'
        return response

    def _handle_close(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        # Mirror the arms: left positive, right negative.
        self._publish_positions(self._close_pos, -self._close_pos)
        response.success = True
        response.message = 'Arms closing to +/- 30 deg.'
        return response


def main() -> None:
    rclpy.init()
    node = ArmControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
