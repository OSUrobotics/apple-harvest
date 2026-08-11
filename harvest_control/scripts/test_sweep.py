#!/usr/bin/env python3
"""
Standalone sweep_controller.py test harness.

Does just enough of what start_harvest.py's run_stage()/pick_controller()
do to exercise the sweep in isolation:
  1. switch ros2_control from joint_trajectory_controller to
     forward_position_controller (required for Servo to drive the arm)
  2. call /servo_node/start_servo
  3. set Servo's command frame to base_link (its default is tool0 --
     see ur_servo.yaml -- which is NOT what sweep_controller.py assumes)
  4. trigger sweep/start_controller and wait for /sweep/status to
     go True then False
  5. switch back to joint_trajectory_controller

Position the arm at your desired start pose (e.g. via RViz's
MotionPlanning panel, Plan & Execute) BEFORE running this script --
sweep_controller.py uses whatever /tool_pose reports at the moment
start_controller is called.

Usage:
    ros2 launch harvest_control arm_control.launch.py use_fake_hardware:=true view_rviz:=true
    # ...position the arm in RViz...
    python3 test_sweep.py
    # optionally tweak the sweep before running, e.g.:
    #   ros2 param set /sweep_controller theta_deg 45.0
    #   ros2 param set /sweep_controller duration 4.0
    #   ros2 param set /sweep_controller pivot_tool_z 0.08
    #   ros2 param set /sweep_controller pivot_world_z 0.04
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Trigger, Empty
from std_msgs.msg import Bool, Float64, Int8
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from controller_manager_msgs.srv import SwitchController


SERVO_STATUS_NAMES = {
    -1: 'INVALID',
    0: 'NO_WARNING',
    1: 'DECELERATE_FOR_SINGULARITY',
    2: 'HALT_FOR_SINGULARITY',
    3: 'DECELERATE_FOR_COLLISION',
    4: 'HALT_FOR_COLLISION',
    5: 'JOINT_BOUND',
}


class TestSweep(Node):
    def __init__(self):
        super().__init__('test_sweep')
        self.cb_group = MutuallyExclusiveCallbackGroup()

        self.switch_controller_client = self.make_client(
            SwitchController, '/controller_manager/switch_controller')
        self.start_servo_client = self.make_client(Trigger, '/servo_node/start_servo')
        self.configure_servo_cli = self.make_client(SetParameters, '/servo_node/set_parameters')
        self.sweep_start_cli = self.make_client(Trigger, 'sweep/start_controller')
        self.sweep_stop_cli = self.make_client(Empty, 'sweep/stop_controller')

        self.sweep_running = False
        self.create_subscription(Bool, '/sweep/status', self._status_cb, 10)
        self.create_subscription(Float64, '/sweep/tracking_error', self._error_cb, 10)
        self.create_subscription(Int8, '/servo_node/status', self._servo_status_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)
        self._last_err = None
        self._last_servo_status = None
        self._latest_joint_state = None

    def make_client(self, srv_type, name):
        client = self.create_client(srv_type, name, callback_group=self.cb_group)
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for service '{name}', retrying...")
        return client

    def _status_cb(self, msg):
        self.sweep_running = msg.data

    def _error_cb(self, msg):
        self._last_err = msg.data

    def _joint_state_cb(self, msg):
        self._latest_joint_state = msg

    def _servo_status_cb(self, msg):
        if msg.data != 0 and msg.data != self._last_servo_status:
            name = SERVO_STATUS_NAMES.get(msg.data, f'UNKNOWN({msg.data})')
            self.get_logger().warn(f'/servo_node/status -> {msg.data} ({name})')
            if self._latest_joint_state is not None:
                js = self._latest_joint_state
                joints = ', '.join(
                    f'{n}={p:.3f}' for n, p in zip(js.name, js.position))
                self.get_logger().warn(f'  joint_states at that instant: {joints}')
        self._last_servo_status = msg.data

    def switch_controller(self, to_servo):
        req = SwitchController.Request()
        if to_servo:
            req.activate_controllers = ['forward_position_controller']
            req.deactivate_controllers = ['joint_trajectory_controller']
        else:
            req.activate_controllers = ['joint_trajectory_controller']
            req.deactivate_controllers = ['forward_position_controller']
        req.timeout = rclpy.duration.Duration(seconds=5.0).to_msg()
        req.strictness = SwitchController.Request.BEST_EFFORT
        future = self.switch_controller_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def start_servo(self):
        future = self.start_servo_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def configure_servo_frame(self, frame):
        req = SetParameters.Request()
        val = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=frame)
        req.parameters = [Parameter(name='moveit_servo.robot_link_command_frame', value=val)]
        future = self.configure_servo_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def run(self):
        self.get_logger().info('Switching to forward_position_controller...')
        self.switch_controller(to_servo=True)

        self.get_logger().info('Starting servo...')
        self.start_servo()

        self.get_logger().info('Setting servo command frame to base_link...')
        set_result = self.configure_servo_frame('base_link')
        ok = bool(set_result and set_result.results and set_result.results[0].successful)
        if not ok:
            reason = set_result.results[0].reason if (set_result and set_result.results) else 'no response'
            self.get_logger().error(
                f'FAILED to set moveit_servo.robot_link_command_frame -- servo is still using '
                f'whatever ur_servo.yaml set (tool0). reason: {reason}')
        else:
            self.get_logger().info('robot_link_command_frame set to base_link (confirmed).')

        self.get_logger().info('Triggering sweep...')
        future = self.sweep_start_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        if not result.success:
            self.get_logger().error(f'Sweep failed to start: {result.message}')
        else:
            while not self.sweep_running:
                rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().info('Sweep running...')
            while self.sweep_running:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self._last_err is not None:
                    self.get_logger().info(
                        f'tracking error: {self._last_err:.4f}',
                        throttle_duration_sec=0.5)
            self.get_logger().info('Sweep complete.')

        self.sweep_stop_cli.call_async(Empty.Request())  # idempotent safety call

        self.get_logger().info('Switching back to joint_trajectory_controller...')
        self.switch_controller(to_servo=False)


def main():
    rclpy.init()
    node = TestSweep()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()