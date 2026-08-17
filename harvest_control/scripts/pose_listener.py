#!/usr/bin/env python3
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped, Point

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException, TimeoutException, TransformException

from harvest_interfaces.srv import GetGripperPose


def maybe_prefix(frame: str, tf_prefix: str) -> str:
    """Apply tf_prefix to a frame name if not empty and if frame doesn't already start with it."""
    if not tf_prefix:
        return frame
    return frame if frame.startswith(tf_prefix) else f'{tf_prefix}{frame}'


class TfListener(Node):
    def __init__(self):
        super().__init__('tf_listener')

        # ---- Parameters ----
        self.declare_parameter('source_frame', 'amiga__base')
        self.declare_parameter('tool_frame', 'tool0')
        self.declare_parameter('gripper_tip_frame', 'gripper_link')
        self.declare_parameter('tf_prefix', '')  # e.g., 'ur_'
        self.declare_parameter('lookup_timeout_sec', 0.5)
        self.declare_parameter('use_time_zero', True)  # better for fixed joints

        self.source = self.get_parameter('source_frame').get_parameter_value().string_value
        tool = self.get_parameter('tool_frame').get_parameter_value().string_value
        gripper = self.get_parameter('gripper_tip_frame').get_parameter_value().string_value
        tf_prefix = self.get_parameter('tf_prefix').get_parameter_value().string_value
        self.lookup_timeout = Duration(seconds=self.get_parameter('lookup_timeout_sec').get_parameter_value().double_value)
        self.use_time_zero = self.get_parameter('use_time_zero').get_parameter_value().bool_value

        # Apply tf_prefix consistently to UR-side frames
        self.tool_frame = maybe_prefix(tool, tf_prefix)
        self.gripper_tip_frame = gripper  # usually unprefixed; change if your gripper is in UR namespace

        # TF buffer/listener
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        # Pubs
        self.tool_pub = self.create_publisher(TransformStamped, '/tool_pose', 10)
        self.gripper_pub = self.create_publisher(TransformStamped, '/gripper_tip', 10)

        # Service
        self.srv = self.create_service(GetGripperPose, 'get_gripper_pose', self.handle_get_gripper_pose)
        self.get_logger().info('GetGripperPose service ready')

        # Delay start a bit to let TF populate, then run at 10 Hz
        self.delay_timer = self.create_timer(0.75, self._start_after_delay)
        self._timer = None

    # ---- Timers ----
    def _start_after_delay(self):
        # Cancel the delay timer so it only fires once
        if self.delay_timer is not None:
            self.delay_timer.cancel()
            self.delay_timer = None

        # Now start the real timer
        self._timer = self.create_timer(0.1, self.timer_callback)

    # ---- TF helpers ----
    def _safe_lookup(self, target: str, source: str) -> Optional[TransformStamped]:
        """
        Try Time(0) first (good for static/fixed edges). On Extrapolation, fall back to 'now'.
        Use can_transform to avoid throwing when not yet available.
        """
        # Try time zero
        query_time = Time() if self.use_time_zero else self.get_clock().now().to_msg()
        try:
            if self._tf_buffer.can_transform(target, source, query_time, timeout=self.lookup_timeout):
                return self._tf_buffer.lookup_transform(target, source, query_time, timeout=self.lookup_timeout)
        except (LookupException, ConnectivityException, TimeoutException) as e:
            self.get_logger().debug(f'can_transform (time0/now first pass) failed: {e}')

        # Fallback: use "now" if time-zero failed due to extrapolation or empty buffer
        try:
            now_time = self.get_clock().now().to_msg()
            if self._tf_buffer.can_transform(target, source, now_time, timeout=self.lookup_timeout):
                return self._tf_buffer.lookup_transform(target, source, now_time, timeout=self.lookup_timeout)
        except (LookupException, ConnectivityException, ExtrapolationException, TimeoutException) as e:
            self.get_logger().debug(f'lookup fallback (now) failed: {e}')

        return None

    # ---- Callbacks ----
    def timer_callback(self):
        trans = self._safe_lookup(self.source, self.tool_frame)
        if trans:
            self.tool_pub.publish(trans)
        else:
            # self.get_logger().throttle_warn(self.get_clock(), 2000,  # warn at most every 2s
            #                                 f'No TF {self.source} <- {self.tool_frame} yet')
            self.get_logger().warn(f'No TF {self.source} <- {self.tool_frame} yet')
            
        trans_g = self._safe_lookup(self.source, self.gripper_tip_frame)
        if trans_g:
            self.gripper_pub.publish(trans_g)
        else:
            # self.get_logger().throttle_warn(self.get_clock(), 2000,
            #                                 f'No TF {self.source} <- {self.gripper_tip_frame} yet')
            self.get_logger().warn(f'No TF {self.source} <- {self.gripper_tip_frame} yet')

    def handle_get_gripper_pose(self, request, response):
        try:
            trans = self._safe_lookup(self.source, self.gripper_tip_frame)
            if trans is None:
                raise TransformException(f'No TF available {self.source} <- {self.gripper_tip_frame}')
            response.point = Point(
                x=trans.transform.translation.x,
                y=trans.transform.translation.y,
                z=trans.transform.translation.z
            )
        except (LookupException, ConnectivityException, ExtrapolationException, TimeoutException, TransformException) as e:
            self.get_logger().warn(f'Failed service lookup: {e}')
            # response.point stays default (0,0,0)
        return response


def main(argv=None):
    rclpy.init(args=argv)
    node = TfListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
