#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R


class OdomReframer(Node):
    """
    Subscribe to odom messages from a bag and:
      - recenter around the first pose (optional)
      - rotate the odom pose for visualization (odom topic only)
      - publish a TF transform odom -> world that keeps your world / robot as-is
    """

    def __init__(self):
        super().__init__('odom_reframer')

        # Topics / frames
        self.declare_parameter('input_odom_topic', '/filter/state')
        self.declare_parameter('output_odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('child_frame', 'world')

        # Behavior parameters
        self.declare_parameter('recenter', True)
        self.declare_parameter('rotate_z_deg', 180.0)  # only applied to Odometry message
        self.declare_parameter('use_now_stamp', True)

        input_topic = self.get_parameter('input_odom_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_odom_topic').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value

        self.recenter = self.get_parameter('recenter').get_parameter_value().bool_value
        self.rotate_z_deg = self.get_parameter('rotate_z_deg').get_parameter_value().double_value
        self.use_now_stamp = self.get_parameter('use_now_stamp').get_parameter_value().bool_value

        # Precompute rotation for odom visualization (if nonzero)
        if abs(self.rotate_z_deg) > 1e-3:
            self.rot_z = R.from_euler("z", self.rotate_z_deg, degrees=True)
        else:
            self.rot_z = None

        # Offset captured from the first odom message
        self.initial_offset = None  # np.array([x, y, z])

        self.sub = self.create_subscription(Odometry, input_topic, self.odom_callback, 10)
        self.pub = self.create_publisher(Odometry, self.output_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f"Reframing odometry:\n"
            f"  input topic:  {input_topic}\n"
            f"  output topic: {self.output_topic}\n"
            f"  TF: {self.odom_frame} -> {self.child_frame}\n"
            f"  recenter: {self.recenter}, rotate_z_deg: {self.rotate_z_deg}, use_now_stamp: {self.use_now_stamp}"
        )

    def _get_stamp(self, msg: Odometry):
        if self.use_now_stamp:
            return self.get_clock().now().to_msg()
        return msg.header.stamp

    def _capture_initial_offset(self, pos_vec: np.ndarray):
        if self.initial_offset is None:
            self.initial_offset = pos_vec.copy()
            x, y, z = self.initial_offset
            self.get_logger().info(
                f"Captured initial odom offset: x={x:.3f}, y={y:.3f}, z={z:.3f}"
            )

    def _relative_position(self, pos_vec: np.ndarray) -> np.ndarray:
        if not self.recenter or self.initial_offset is None:
            return pos_vec
        return pos_vec - self.initial_offset

    def _rotate_for_odom(self, rel_pos: np.ndarray, q_in: Quaternion):
        """
        Apply configured z-rotation to the Odometry message only.
        TF uses the unrotated pose.
        """
        if self.rot_z is None:
            # No rotation requested
            return rel_pos, q_in

        # Rotate position
        pos_rot = self.rot_z.apply(rel_pos)

        # Rotate orientation
        R_in = R.from_quat([q_in.x, q_in.y, q_in.z, q_in.w])
        R_out = R_in * self.rot_z
        q_out = R_out.as_quat()  # [x, y, z, w]

        q_rot = Quaternion(x=q_out[0], y=q_out[1], z=q_out[2], w=q_out[3])
        return pos_rot, q_rot

    def odom_callback(self, msg: Odometry):
        stamp = self._get_stamp(msg)

        # Raw position as numpy vector
        pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])

        # Capture offset on first message (if recenter is on)
        if self.initial_offset is None and self.recenter:
            self._capture_initial_offset(pos)

        # Re-centered position (this is the "true" world pose)
        rel = self._relative_position(pos)

        # Odometry message uses rotated pose (for visualization)
        q_in = msg.pose.pose.orientation
        pos_odom, q_odom = self._rotate_for_odom(rel, q_in)

        # ---------------- publish Odometry ----------------
        new_msg = Odometry()
        new_msg.header = msg.header
        new_msg.header.stamp = stamp
        new_msg.header.frame_id = self.odom_frame
        new_msg.child_frame_id = self.child_frame

        new_msg.pose.pose.position.x = float(pos_odom[0])
        new_msg.pose.pose.position.y = float(pos_odom[1])
        new_msg.pose.pose.position.z = float(pos_odom[2])
        new_msg.pose.pose.orientation = q_odom
        new_msg.twist = msg.twist

        self.pub.publish(new_msg)

        # ---------------- publish TF ----------------
        # Keep using the unrotated (but recentered) pose so 'world' + robot stay the same.
        # The negative XY matches the convention found that makes the robot follow the arrows.
        # The recorded odom topic odom frame does not match hardware system frames, hence the negation.
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.child_frame

        t.transform.translation.x = float(-rel[0])
        t.transform.translation.y = float(-rel[1])
        t.transform.translation.z = float(rel[2])
        t.transform.rotation = q_in  # original orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomReframer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
