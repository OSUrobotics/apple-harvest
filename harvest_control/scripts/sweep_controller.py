#!/usr/bin/env python3
"""
Sweep controller (ROS2 / MoveIt Servo).

Rotates the end-effector along a vertical arc centered on a pivot that is a
fixed gripper-frame offset from tool0, by streaming feedforward + feedback
Cartesian velocity commands to MoveIt Servo's /servo_node/delta_twist_cmds.
Servo owns the differential-IK (Jacobian) step -- this node only computes the
arc geometry and a world-frame pose-tracking term.

Geometry:
    T_pivot = T_start * T_off     -- pivot is a fixed offset from tool0, built from
                                      a translation along the tool's own z-axis
                                      (pivot_tool_z) plus a translation straight up
                                      in world Z (pivot_world_z) -- see pivot_offset_vector()
    forward = R_start @ forward_axis_body   -- gripper's pointing axis, in world
    k_hat   = horizontal, perpendicular to forward, sign chosen so the arc
              starts by moving upward in world Z. A sweep of theta=90deg takes
              the tool's pointing direction from horizontal to straight down.
    T_d(s)  = Rot(k_hat, s*theta) about pivot c, applied to T_start

Commands are published with header.frame_id = base_frame (default base_link),
which Servo's robot_link_command_frame must be set to match (see
test_sweep.py). Feedback pose comes from a direct tf2 lookup of
gripper_tip_frame in base_frame, done in-loop at rate_hz -- NOT from
pose_listener.py's /gripper_tip topic, which only updates at 10Hz (too slow
for a 100Hz control loop).
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Empty, Trigger
from harvest_interfaces.srv import SetValue
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


# ----------------------------------------------------------------------
# Geometry
# ----------------------------------------------------------------------

def pivot_offset_vector(R0, tool_z, world_z):
    """
    Gripper-frame translation vector combining a translation `tool_z` along
    the end-effector's own z-axis with a translation `world_z` straight up
    in world Z -- meant to be passed as `offset` to pivot_and_axis (which
    applies it as p0 + R0 @ offset, i.e. T_start * T_off in SE3 terms).

    The world_z part is pre-rotated by R0.T so it turns back into pure
    world Z once R0 is reapplied there. NOTE: this depends on R0's
    *current* orientation -- recompute it if T_start's orientation changes.
    """
    return np.array([0.0, 0.0, tool_z]) + R0.T @ np.array([0.0, 0.0, world_z])


def pivot_and_axis(p0, R0, offset, theta, forward_axis_body, eps=1e-9):
    """
    World pivot position + world sweep axis.

    `offset` (gripper-frame) locates the pivot -- a pure translation, T_pivot =
    T_start * T_off. `forward_axis_body` (gripper-frame unit vector, the tool's
    pointing/approach axis) is rotated into world by R0 and crossed with world
    Z to get k_hat: horizontal, and perpendicular to wherever the tool
    currently points, so the sweep takes the pointing direction through the
    vertical plane containing it. `theta`'s sign only picks which way k_hat
    points (so the arc starts upward).
    """
    c = p0 + R0 @ offset
    r = p0 - c
    z_world = np.array([0.0, 0.0, 1.0])
    forward_world = R0 @ np.asarray(forward_axis_body, dtype=float)
    k_hat = np.cross(z_world, forward_world)
    norm = np.linalg.norm(k_hat)
    if norm < eps:
        raise ValueError(
            "forward_axis_body is currently vertical -- no horizontal sweep "
            "axis is defined for 'upward' relative to it."
        )
    k_hat /= norm

    if np.linalg.norm(r) > 1e-4:
        v0_z = np.cross(k_hat, r)[2]
        if theta * v0_z < 0:
            k_hat = -k_hat
    return c, k_hat


def arc_pose(p0, R0, c, k_hat, theta, s):
    """T_d(s): rigidly rotate (p0, R0) about the world axis (c, k_hat)."""
    R_s = Rotation.from_rotvec(s * theta * k_hat).as_matrix()
    p_d = c + R_s @ (p0 - c)
    R_d = R_s @ R0
    return p_d, R_d


# ----------------------------------------------------------------------
# Node
# ----------------------------------------------------------------------

class SweepController(Node):

    def __init__(self):
        super().__init__('sweep_controller')

        # ---- Parameters ----
        # Pivot location, built from a translation along the tool's own z-axis
        # plus a translation straight up in world Z -- see pivot_offset_vector().
        self.declare_parameter('pivot_tool_z', 0.04)   # m, along the tool's local z-axis
        self.declare_parameter('pivot_world_z', 0.08)  # m, straight up in world Z
        self.declare_parameter('forward_axis_body', [0.0, 0.0, 1.0])  # gripper's pointing axis
        self.declare_parameter('theta_deg', 90.0)
        self.declare_parameter('duration', 8.0)
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('kp_lin', 4.0)
        self.declare_parameter('kp_ang', 8.0)
        # If tracking falls this far behind (rad), pause advancing the reference
        # until the arm catches up, instead of letting the target keep moving on
        # a fixed schedule while Servo is decelerating/halting for its own
        # reasons -- without this a stall turns into an ever-growing error.
        self.declare_parameter('error_pause_threshold', 0.5)
        # Hard ceiling, independent of the pause above -- if the arm never
        # catches back up, stop and say why instead of hanging forever.
        self.declare_parameter('max_duration', 15.0)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('gripper_tip_frame', 'gripper_link')

        self.pivot_tool_z = self.get_parameter('pivot_tool_z').get_parameter_value().double_value
        self.pivot_world_z = self.get_parameter('pivot_world_z').get_parameter_value().double_value
        self.forward_axis_body = np.array(
            self.get_parameter('forward_axis_body').get_parameter_value().double_array_value)
        self.theta = np.deg2rad(self.get_parameter('theta_deg').get_parameter_value().double_value)
        self.duration = self.get_parameter('duration').get_parameter_value().double_value
        rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.kp_lin = self.get_parameter('kp_lin').get_parameter_value().double_value
        self.kp_ang = self.get_parameter('kp_ang').get_parameter_value().double_value
        self.error_pause_threshold = self.get_parameter(
            'error_pause_threshold').get_parameter_value().double_value
        self.max_duration = self.get_parameter('max_duration').get_parameter_value().double_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.gripper_tip_frame = self.get_parameter(
            'gripper_tip_frame').get_parameter_value().string_value
        self.dt = 1.0 / rate_hz

        # ---- State ----
        self.have_pose = False
        self.tool_p = np.zeros(3)
        self.tool_R = np.eye(3)

        self.running = False
        self.start_time = None
        self.p0 = None
        self.R0 = None
        self.c = None
        self.k_hat = None
        self.s = 0.0                # progress fraction, advanced by a virtual clock (see _step)
        self._last_rot_err = 0.0    # gates whether s is allowed to advance this tick

        # ---- TF ----
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        # ---- Pub/Sub ----
        self.cmd_publisher = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.status_publisher = self.create_publisher(Bool, '/sweep/status', 10)
        self.error_publisher = self.create_publisher(Float64, '/sweep/tracking_error', 10)

        self.timer = self.create_timer(self.dt, self.timer_callback)

        # ---- Services ----
        self.start_service = self.create_service(Trigger, 'sweep/start_controller', self.start)
        self.stop_service = self.create_service(Empty, 'sweep/stop_controller', self.stop)
        self.set_theta_service = self.create_service(SetValue, 'sweep/set_theta_deg', self.set_theta)

    # ---- Services ----

    def set_theta(self, request, response):
        self.theta = np.deg2rad(request.val)
        response.success = True
        return response

    def start(self, request, response):
        if not self.have_pose:
            response.success = False
            response.message = "No pose yet -- can't start sweep"
            self.get_logger().error(response.message)
            return response

        self.p0 = self.tool_p.copy()
        self.R0 = self.tool_R.copy()
        offset = pivot_offset_vector(self.R0, self.pivot_tool_z, self.pivot_world_z)
        try:
            self.c, self.k_hat = pivot_and_axis(
                self.p0, self.R0, offset, self.theta, self.forward_axis_body)
        except ValueError as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(response.message)
            return response

        self.start_time = self.get_clock().now()
        self.s = 0.0
        self._last_rot_err = 0.0
        self.running = True
        response.success = True
        response.message = "sweep started"
        self.get_logger().info(
            f"starting sweep: theta={np.rad2deg(self.theta):.1f} deg, "
            f"duration={self.duration:.2f}s, pivot={np.round(self.c, 4)}"
        )
        return response

    def stop(self, request, response):
        self.running = False
        self.get_logger().info("sweep stopped")
        return response

    # ---- Timer ----

    def timer_callback(self):
        self._update_pose()

        if self.running:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            if elapsed > self.max_duration:
                self.running = False
                self.get_logger().error(
                    f"sweep timed out after {elapsed:.1f}s, stuck at s={self.s:.3f} "
                    f"(rot err={self._last_rot_err:.4f} rad) -- the arm isn't converging "
                    f"on a held target; check /servo_node/status for what's blocking it."
                )
            else:
                self._step()

        status_msg = Bool()
        status_msg.data = self.running
        self.status_publisher.publish(status_msg)

    def _step(self):
        # Advance the reference only if the arm was keeping up as of last tick.
        # Otherwise hold the target (and drop feedforward) so a Servo-side
        # stall doesn't turn into a growing, unrecoverable gap.
        advancing = self._last_rot_err < self.error_pause_threshold
        if advancing:
            self.s = min(self.s + self.dt / self.duration, 1.0)

        p_d, R_d = arc_pose(self.p0, self.R0, self.c, self.k_hat, self.theta, self.s)

        if advancing:
            theta_rate = self.theta / self.duration
            w_ff = theta_rate * self.k_hat
            v_ff = np.cross(w_ff, p_d - self.c)
        else:
            w_ff = np.zeros(3)
            v_ff = np.zeros(3)

        # World-frame Cartesian feedback
        e_p = p_d - self.tool_p
        R_err = R_d @ self.tool_R.T
        e_rot = Rotation.from_matrix(R_err).as_rotvec()
        self._last_rot_err = float(np.linalg.norm(e_rot))

        v_cmd = self.kp_lin * e_p + v_ff
        w_cmd = self.kp_ang * e_rot + w_ff

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = v_cmd
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = w_cmd
        self.cmd_publisher.publish(msg)

        err_msg = Float64()
        err_msg.data = float(np.linalg.norm(np.concatenate([e_p, e_rot])))
        self.error_publisher.publish(err_msg)

        if self.s >= 1.0:
            self.running = False
            self.get_logger().info("sweep complete")

    # ---- TF ----

    def _update_pose(self):
        """Direct, in-loop TF lookup -- see module docstring. Returns True if refreshed."""
        try:
            trans = self._tf_buffer.lookup_transform(
                self.base_frame, self.gripper_tip_frame, Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup {self.base_frame} <- {self.gripper_tip_frame} '
                                    f'failed: {e}', throttle_duration_sec=1.0)
            return False

        t = trans.transform.translation
        q = trans.transform.rotation
        self.tool_p = np.array([t.x, t.y, t.z])
        self.tool_R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        self.have_pose = True
        return True


def main():
    rclpy.init()
    node = SweepController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
