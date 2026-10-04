#!/usr/bin/env python3
"""
Merges start_harvest.py (scan -> predict -> multi-apple loop) and
start_harvest_free_drive.py (manual single-apple test flow) into one node,
selected at runtime via the 'freedrive' parameter, and adds the ability to
abort a stage mid-flight when moveit_servo reports it is near a singularity,
collision, or joint limit (/servo_node/status).

Visual servo, grasp/pressure-servo approach, and the legacy pick patterns
(force-heuristic, pull-twist, linear-pull, stiffness-seeking) are the
operations that actually move the arm for an extended, variable amount of
time, so they're the ones worth cancelling instead of just waiting out.
They're called through ActionClients (VisualServo, GraspControl, PickControl)
so the abort logic has a goal it can cancel:
  - VisualServo is served by visual_servo.py's 'visual_servo' action server.
  - GraspControl is served by grasp_controller.py's 'grasp_apple' action
    server (package gripper_msgs, from the apple_gripper repo) -- if that
    package isn't built into this workspace the import falls back to None
    and the grasp stage logs an error and is skipped.
  - PickControl is served by pick_controller.py's 'pick_controller' action
    server, which wraps the start_controller/stop_controller/pull_twist/
    linear_pull/stiffness service pairs -- goal.pattern selects which one.

'sweep' is NOT one of PickControl's patterns -- pick_controller.py has no
branch for it, so routing it through the pick_controller action would just
silently no-op. sweep_controller.py only exposes plain services/topics
(sweep/start_controller, sweep/stop_controller, sweep/set_theta_deg,
/sweep/status, /sweep/tracking_error), so pick_controller_action() talks to
those directly (see _run_sweep_pick) instead, polling abort_event itself and
calling sweep/stop_controller the moment an abort is requested.
"""

# ROS
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.action import ActionClient
# Interfaces
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import SetBool, Trigger, Empty
from std_msgs.msg import Int8, Bool, Float32, Float64, Int32MultiArray
from geometry_msgs.msg import Point, Pose, PoseArray
from harvest_interfaces.srv import ApplePrediction, CoordinateToTrajectory, SendTrajectory, RecordTopics, GetGripperPose, SetValue, MoveToPose, SetAppleTarget
from controller_manager_msgs.srv import SwitchController
from harvest_interfaces.action import EventDetection

# TEMPLATE: action interfaces that don't exist yet, see module docstring.
try:
    from harvest_interfaces.action import VisualServo
except ImportError:
    VisualServo = None
try:
    from gripper_msgs.action import GraspControl
except ImportError:
    GraspControl = None
try:
    from harvest_interfaces.action import PickControl
except ImportError:
    PickControl = None

# Python
import threading
import time
import numpy as np
import os
import signal
import subprocess
import yaml
import re
from threading import Condition
from contextlib import contextmanager
from enum import Enum
from pathlib import Path


class ControllerState(str, Enum):
    TRAJECTORY = "scaled_joint_trajectory_controller"
    SERVO = "forward_position_controller"
    FREEDRIVE = "freedrive_mode_controller"


# freedrive_mode_controller drops back out of freedrive if it doesn't see a
# new enable message within its inactive_timeout (defaults to 1s) -- 2 Hz
# has been confirmed to keep it engaged reliably.
FREEDRIVE_ENABLE_TOPIC = '/freedrive_mode_controller/enable_freedrive_mode'
FREEDRIVE_PUBLISH_RATE_HZ = 2.0


def get_data_storage_dir():
    current_dir = Path(__file__).resolve().parent
    workspace_root = current_dir.parents[3]
    data_dir = workspace_root / 'data'
    data_dir.mkdir(parents=True, exist_ok=True)
    data_dir = str(data_dir)
    print(f"Using data directory: {data_dir}")
    return data_dir


class ServoStatusCode:
    """Mirrors moveit_servo::StatusCode (moveit_servo/status_codes.h)."""
    INVALID = -1
    NO_WARNING = 0
    DECELERATE_FOR_APPROACHING_SINGULARITY = 1
    HALT_FOR_SINGULARITY = 2
    DECELERATE_FOR_COLLISION = 3
    HALT_FOR_COLLISION = 4
    JOINT_BOUND = 5
    DECELERATE_FOR_LEAVING_SINGULARITY = 6


class HarvestAborted(Exception):
    """Raised to unwind the current stage/apple loop after an abort."""
    pass


class AppleSelection:
    """One operator-selected ID at a time from the current predicted poses."""

    def __init__(self):
        self._condition = Condition()
        self._available: set[int] | None = None
        self._pending: int | None = None
        self._finished = False

    def set_available(self, count: int) -> None:
        with self._condition:
            self._available = set(range(count))
            self._pending = None
            self._finished = count == 0
            self._condition.notify_all()

    def selectable_ids(self) -> list[int]:
        with self._condition:
            if self._available is None or self._finished:
                return []
            return sorted(self._available)

    def request(self, apple_id: int) -> tuple[bool, str]:
        with self._condition:
            if self._available is None:
                return False, "Wait for apple prediction to finish"
            if self._finished:
                return False, "Apple selection has finished"
            if apple_id not in self._available:
                return False, f"Apple ID {apple_id} is not in the current prediction; valid IDs: {sorted(self._available)}"
            self._pending = apple_id
            self._condition.notify_all()
            return True, f"Apple ID {apple_id} selected for the next attempt"

    def take_next(self, timeout: float = 0.2) -> int | None:
        with self._condition:
            if self._pending is None and not self._finished:
                self._condition.wait(timeout)
            if self._finished or self._pending is None:
                return None
            apple_id = self._pending
            self._pending = None
            return apple_id

    def clear_pending(self) -> None:
        """Require a fresh operator choice after an aborted attempt."""
        with self._condition:
            self._pending = None

    def finish(self) -> tuple[bool, str]:
        with self._condition:
            if self._available is None:
                return False, "Wait for apple prediction to finish"
            self._pending = None
            self._finished = True
            self._condition.notify_all()
            return True, "No further apples will be selected for this batch"

    @property
    def done(self) -> bool:
        with self._condition:
            return self._finished


class StartHarvestAbort(Node):
    def __init__(self):
        super().__init__("start_harvest_abort_node")
        self.cb_group = MutuallyExclusiveCallbackGroup()
        # Dedicated group so the abort monitor is never queued behind a
        # pending service/action call on cb_group.
        self.status_cb_group = ReentrantCallbackGroup()

        # Track ros2 control controller type
        self.controller_state = ControllerState.TRAJECTORY
        self._controller_switch_lock = threading.RLock()
        self._motion_lock = threading.RLock()
        self._motion_depth = 0
        self._motion_label = None
        self.apple_selection = AppleSelection()
        apple_ids_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.available_apple_ids_pub = self.create_publisher(
            Int32MultiArray, '/harvest/available_apple_ids', apple_ids_qos)

        # Add publisher for freedrive mode in a timer callback. 
        self.freedrive_pub = self.create_publisher(Bool, FREEDRIVE_ENABLE_TOPIC, 10)
        self.freedrive_timer = None

        self.storage_directory = get_data_storage_dir()
        self.batch_dir = self.storage_directory + '/batch/'
        self.batch_number = 0

        # Declare parameters with defaults
        self.declare_parameter('pick_pattern', 'force-heuristic')
        self.declare_parameter('event_sensitivity', 0.43)
        self.declare_parameter('recording_startup_delay', 0.5)
        self.declare_parameter('prediction_throttle_hz', 0.1)
        self.declare_parameter('base_data_dir', self.storage_directory)
        self.declare_parameter('enable_recording', True)
        self.declare_parameter('enable_visual_servo', True)
        self.declare_parameter('enable_apple_prediction', True)
        self.declare_parameter('manual_apple_selection', False)
        self.declare_parameter('enable_pressure_servo', True)
        self.declare_parameter('enable_picking', True)
        self.declare_parameter('optimal_trajectory', True)
        self.declare_parameter('sweep_theta_deg', 90.0)
        # Start harvest mode (Freedrive or full auton)
        self.declare_parameter('freedrive', False)
        # Abort tuning
        self.declare_parameter('servo_status_topic', '/servo_node/status')
        self.declare_parameter('abort_on_decelerate', False)
        self.declare_parameter('abort_recovery', 'freedrive')  # 'freedrive' or 'home'

        self.org_pick_pattern = self.get_parameter('pick_pattern').get_parameter_value().string_value
        self.EVENT_SENSITIVITY = self.get_parameter('event_sensitivity').get_parameter_value().double_value
        self.recording_startup_delay = self.get_parameter('recording_startup_delay').get_parameter_value().double_value
        self.prediction_throttle_hz = self.get_parameter('prediction_throttle_hz').get_parameter_value().double_value
        self.base_data_dir = self.get_parameter('base_data_dir').get_parameter_value().string_value
        self.enable_recording = self.get_parameter('enable_recording').get_parameter_value().bool_value
        self.enable_visual_servo = self.get_parameter('enable_visual_servo').get_parameter_value().bool_value
        self.enable_apple_prediction = self.get_parameter('enable_apple_prediction').get_parameter_value().bool_value
        self.manual_apple_selection = self.get_parameter('manual_apple_selection').get_parameter_value().bool_value
        self.enable_pressure_servo = self.get_parameter('enable_pressure_servo').get_parameter_value().bool_value
        self.enable_picking = self.get_parameter('enable_picking').get_parameter_value().bool_value
        self.use_optimal_trajectory = self.get_parameter('optimal_trajectory').get_parameter_value().bool_value
        self.SWEEP_THETA_DEG = self.get_parameter('sweep_theta_deg').get_parameter_value().double_value
        self.freedrive_mode = self.get_parameter('freedrive').get_parameter_value().bool_value
        self.abort_on_decelerate = self.get_parameter('abort_on_decelerate').get_parameter_value().bool_value
        self.abort_recovery_mode = self.get_parameter('abort_recovery').get_parameter_value().string_value
        self.PICK_PATTERN = self.org_pick_pattern

        if not self.freedrive_mode and not self.enable_apple_prediction:
            apple_loc_path = "/home/jn2/college/data/apple_locations"
            self.pre_saved_apple_locations = self.read_apple_locations(apple_loc_path)


        # Helper clients
        self.switch_controller_client = self.make_client(SwitchController, '/controller_manager/switch_controller')
        self.start_servo_client = self.make_client(Trigger, '/servo_node/start_servo')
        self.configure_servo_cli = self.make_client(SetParameters, '/servo_node/set_parameters')
        self.start_move_arm_to_home_client = self.make_client(Trigger, '/move_arm_to_home')
        self.trigger_move_arm_to_config_client = self.make_client(Trigger, 'move_arm_to_config')
        self.coord_to_traj_client = self.make_client(CoordinateToTrajectory, 'coordinate_to_trajectory')
        self.trigger_arm_mover_client = self.make_client(SendTrajectory, 'send_arm_trajectory')
        self.trigger_move_arm_to_pose_client = self.make_client(MoveToPose, 'move_arm_to_pose')

        # Conditional clients
        if self.enable_recording:
            self.start_record_client = self.make_client(RecordTopics, 'record_topics')
            self.stop_record_client = self.make_client(Trigger, 'stop_recording')
        self.init_metadata_and_topics()

        if self.enable_visual_servo:
            # TEMPLATE action client -- see module docstring.
            self._visual_servo_client = ActionClient(self, VisualServo, 'visual_servo') if VisualServo else None
        if self.enable_apple_prediction:
            self.start_apple_prediction_client = self.make_client(ApplePrediction, '/apple_prediction')
        if self.enable_pressure_servo:
            # TEMPLATE action client -- see module docstring.
            self._grasp_client = ActionClient(self, GraspControl, 'grasp_apple') if GraspControl else None
            self.release_controller_client = self.make_client(Trigger, 'release_apple')
        if self.enable_picking:
            self.set_goal_cli = self.make_client(SetValue, 'set_goal')
            # Action client for the legacy patterns -- goal.pattern picks the
            # behavior. Not used for 'sweep', see module docstring.
            self._pick_controller_client = ActionClient(self, PickControl, 'pick_controller') if PickControl else None
            if self.PICK_PATTERN == 'sweep':
                self.sweep_start_cli = self.make_client(Trigger, 'sweep/start_controller')
                self.sweep_stop_cli = self.make_client(Empty, 'sweep/stop_controller')
                self.sweep_set_theta_cli = self.make_client(SetValue, 'sweep/set_theta_deg')
                self.sweep_running = False
                self.sweep_tracking_error = None
                self.sweep_subscription = self.create_subscription(
                    Bool, '/sweep/status', self.sweep_status_callback, 10)
                self.sweep_error_subscription = self.create_subscription(
                    Float64, '/sweep/tracking_error', self.sweep_error_callback, 10)
            self._event_client = ActionClient(self, EventDetection, 'event_detection')

        # Abort machinery
        self._goal_lock = threading.Lock()
        self.current_goal_handle = None
        self.abort_event = threading.Event()
        self.last_servo_status = ServoStatusCode.NO_WARNING
        servo_status_topic = self.get_parameter('servo_status_topic').get_parameter_value().string_value
        self.servo_status_sub = self.create_subscription(
            Int8, servo_status_topic, self._servo_status_cb, 10,
            callback_group=self.status_cb_group,
        )
        # Manual abort service 
        self.abort_service = self.create_service(
            Trigger, 'abort_harvest', self._abort_service_cb,
            callback_group=self.status_cb_group,
        )
        self.freedrive_control_service = self.create_service(
            SetBool, 'set_harvest_freedrive', self._freedrive_service_cb,
            callback_group=self.status_cb_group,
        )
        self.apple_target_service = self.create_service(
            SetAppleTarget, 'set_apple_target', self._apple_target_service_cb,
            callback_group=self.status_cb_group,
        )
        self.finish_apple_selection_service = self.create_service(
            Trigger, 'finish_apple_selection', self._finish_apple_selection_cb,
            callback_group=self.status_cb_group,
        )

    # Setup Functions
    def make_client(self, srv_type, name):
        client = self.create_client(srv_type, name, callback_group=self.cb_group)
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for service '{name}', retrying...")
        return client

    def init_metadata_and_topics(self):
        self.apple_coordinates = {}
        self.pick_pattern = {'pick controller': self.PICK_PATTERN}

        # /apple_markers and /apple_annotated are published once, when apple
        # prediction finishes -- record them as-is. /joint_states and the
        # camera feeds publish continuously for the whole ~20s prediction
        # stage; recording them at full rate is mostly wasted space since
        # prediction only needs a couple of frames, so those are recorded
        # through low-rate topic_tools throttle relays instead (see
        # start_prediction_throttles/stop_prediction_throttles).
        self.prediction_one_shot_topics = ['/apple_markers', '/apple_annotated']
        self.prediction_throttled_topics = [
            '/joint_states',
            '/camera/gripper_camera/color/image_raw',
            '/camera/gripper_camera/aligned_depth_to_color/image_raw',
            '/camera/gripper_camera/depth/image_rect_raw',
        ]
        self.prediction_topics = self.prediction_one_shot_topics + [
            self.throttled_topic_name(topic) for topic in self.prediction_throttled_topics
        ]
        self.approach_trajectory_topics = ['/apple_markers']
        self.visual_servo_topics = ['/gripper/rgb_palm_camera/image_raw', '/joint_states', '/servo_node/delta_twist_cmds']
        self.pressure_servo_topics = [
            '/microROS/sensor_data',
            '/microROS/can_status',
            '/microROS/imu1',
            '/camera/gripper_camera/color/image_raw',
            '/camera/gripper_camera/aligned_depth_to_color/image_raw',
            '/gripper/rgb_palm_camera/image_raw',
            '/joint_states',
            '/force_torque_sensor_broadcaster/wrench', '/servo_node/delta_twist_cmds',
            '/sweep/status','/sweep/tracking_error'
        ]
        self.pick_controller_topics = [
            '/microROS/sensor_data',
            '/microROS/can_status',
            '/microROS/imu1',
            '/camera/gripper_camera/color/image_raw',
            '/camera/gripper_camera/aligned_depth_to_color/image_raw',
            '/gripper/rgb_palm_camera/image_raw',
            '/joint_states',
            '/tool_pose', '/force_torque_sensor_broadcaster/wrench', '/servo_node/delta_twist_cmds',
            '/sweep/status','/sweep/tracking_error',
        ]
        self.pressure_servo_and_pick_controller_topics = list(set(self.pressure_servo_topics + self.pick_controller_topics))

        self.batch_dir, self.batch_number = self.create_new_batch_directory(self.base_data_dir)

        self.prediction_file_name_prefix = 'prediction'
        self.approach_trajectory_file_name_prefix = 'approach_trajectory'
        self.visual_servo_file_name_prefix = 'visual_servo'
        self.pressure_servo_file_name_prefix = 'pressure_servo'
        self.pick_controller_file_name_prefix = 'pick_controller'
        self.final_approach_and_pick_file_name_prefix = 'final_approach_and_pick'

    def create_new_batch_directory(self, base_directory):
        if not os.path.exists(base_directory):
            os.makedirs(base_directory)
        batch_number = 1
        while True:
            batch_directory = os.path.join(base_directory, f"batch_{batch_number}/")
            if not os.path.exists(batch_directory):
                os.makedirs(batch_directory)
                print(f"Created new directory: {batch_directory}")
                print(
                    f"HARVEST_CONTEXT batch={batch_number} batch_dir={batch_directory}",
                    flush=True,
                )
                break
            batch_number += 1
        return batch_directory, batch_number

    def read_apple_locations(self, directory):
        csv_file = Path(directory) / 'apple_locations.csv'
        data = np.loadtxt(str(csv_file), delimiter=',', skiprows=1)
        if data.ndim == 1:
            data = data[np.newaxis, :]
        return data

    def get_current_gripper_pose(self):
        request = GetGripperPose.Request()
        future = self.get_gripper_pose_client.call_async(request)
        self._wait_for_future(future)
        return future.result().point

    def save_metadata(self):
        coord_list = [[float(x), float(y), float(z)] for (x, y, z) in self.apple_coordinates.values()]
        data = {'apple_coordinates': coord_list, 'pick_controller': self.PICK_PATTERN}
        with open(self.batch_dir + f'batch_{self.batch_number}_metadata.yaml', 'w') as file:
            yaml.dump(data, file)
        self.get_logger().info("YAML file saved successfully.")

    # ------------------------------------------------------------------
    # Blocking-call helper. Waits on a future without spinning ourselves --
    # the executor is already spinning in a background thread (see main()),
    # which is what lets the servo-status subscriber fire while a stage runs.
    # ------------------------------------------------------------------
    def _wait_for_future(self, future, poll_period=0.02):
        while rclpy.ok() and not future.done():
            time.sleep(poll_period)
        return future.result()

    # Abort monitor
    def _servo_status_cb(self, msg):
        self.last_servo_status = msg.data
        abort_codes = {ServoStatusCode.HALT_FOR_SINGULARITY, ServoStatusCode.HALT_FOR_COLLISION, ServoStatusCode.JOINT_BOUND}
        if self.abort_on_decelerate:
            abort_codes |= {ServoStatusCode.DECELERATE_FOR_COLLISION, ServoStatusCode.DECELERATE_FOR_APPROACHING_SINGULARITY}
        if msg.data in abort_codes and not self.abort_event.is_set():
            self.get_logger().error(f"servo status={msg.data}: aborting current stage")
            self.abort_event.set()

    def _abort_service_cb(self, request, response):
        if self.abort_event.is_set():
            response.success = True
            response.message = "Abort already in progress"
            return response

        self.get_logger().error("Manual abort requested")
        self.abort_event.set()
        response.success = True
        response.message = "Abort requested"
        return response

    def _publish_available_apple_ids(self):
        message = Int32MultiArray()
        message.data = self.apple_selection.selectable_ids()
        self.available_apple_ids_pub.publish(message)

    def _apple_target_service_cb(self, request, response):
        if self.freedrive_mode or not self.manual_apple_selection:
            response.success = False
            response.message = "Operator apple selection is not enabled for this run"
            return response
        response.success, response.message = self.apple_selection.request(request.apple_id)
        return response

    def _finish_apple_selection_cb(self, request, response):
        if self.freedrive_mode or not self.manual_apple_selection:
            response.success = False
            response.message = "Operator apple selection is not enabled for this run"
            return response
        response.success, response.message = self.apple_selection.finish()
        if response.success:
            self._publish_available_apple_ids()
        return response

    def _wait_for_selected_apple(self):
        self.get_logger().info('Waiting for an apple ID from Controls; select an ID or finish the batch')
        while rclpy.ok() and not self.apple_selection.done:
            apple_id = self.apple_selection.take_next()
            if apple_id is not None:
                self.get_logger().info(f'Operator selected apple {apple_id}')
                return apple_id
        return None

    def _freedrive_service_cb(self, request, response):
        target = "freedrive" if request.data else "trajectory"
        with self._motion_lock:
            if request.data:
                motion_label = self._motion_label if self._motion_depth else None
                with self._goal_lock:
                    action_active = self.current_goal_handle is not None
                if motion_label or action_active or self.abort_event.is_set():
                    if motion_label:
                        activity = motion_label
                    elif action_active:
                        activity = "an action"
                    else:
                        activity = "abort recovery"
                    response.success = False
                    response.message = f"Freedrive rejected while {activity} is active"
                    return response

            # Keep the motion gate locked through the controller switch so a
            # new motion cannot start between the safety check and activation.
            result = self.switch_free_drive_controller(controller=target)
        already_active = (
            request.data and self.controller_state == ControllerState.FREEDRIVE
        ) or (
            not request.data and self.controller_state == ControllerState.TRAJECTORY
        )
        response.success = already_active or bool(result is not None and result.ok)
        response.message = (
            f"{self.controller_state.value} active"
            if response.success
            else f"Failed to activate {target} controller"
        )
        return response

    def handle_abort(self):
        self.get_logger().error("ABORT: stopping current stage and recovering")
        with self._goal_lock:
            goal_handle = self.current_goal_handle
        if goal_handle is not None:
            goal_handle.cancel_goal_async()

        if self.enable_recording:
            self.stop_recording()

        self.switch_controller(servo=False)

        if self.abort_recovery_mode == 'freedrive':
            self.get_logger().warn("Switching to freedrive_mode_controller for manual recovery")
            self.switch_free_drive_controller(controller="freedrive")
        else:
            self.get_logger().warn("Returning arm to home position")
            self.go_to_scan_position()
            # self.go_to_home()

        self.abort_event.clear()

    def _raise_if_aborted(self, stage_name):
        if self.abort_event.is_set():
            self.handle_abort()
            raise HarvestAborted(stage_name)

    # ------------------------------------------------------------------
    # Topic throttling (prediction stage only -- see prediction_topics)
    # ------------------------------------------------------------------
    def throttled_topic_name(self, topic):
        return f'{topic}/throttled'

    def start_prediction_throttles(self):
        self.prediction_throttle_processes = []
        for topic in self.prediction_throttled_topics:
            cmd = [
                'ros2', 'run', 'topic_tools', 'throttle', 'messages',
                topic, str(self.prediction_throttle_hz), self.throttled_topic_name(topic),
            ]
            self.prediction_throttle_processes.append(
                subprocess.Popen(cmd, preexec_fn=os.setsid)
            )

    def stop_prediction_throttles(self):
        for proc in getattr(self, 'prediction_throttle_processes', []):
            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        self.prediction_throttle_processes = []

    # ------------------------------------------------------------------
    # Recording
    # ------------------------------------------------------------------
    def start_recording(self, topics, file_name_prefix):
        request = RecordTopics.Request()
        request.topics = topics
        request.file_name_prefix = file_name_prefix
        self.get_logger().info(f'Calling /record_topics with topics: {topics}')
        future = self.start_record_client.call_async(request)
        self._wait_for_future(future)
        if future.result() is not None:
            self.get_logger().info(f'Recording started: {future.result().success}')
        else:
            self.get_logger().error('Failed to call /record_topics service.')

    def stop_recording(self):
        self.get_logger().info('Calling /stop_recording service...')
        future = self.stop_record_client.call_async(Trigger.Request())
        self._wait_for_future(future)
        if future.result() is not None:
            self.get_logger().info(f'Recording stopped: {future.result().success}')
        else:
            self.get_logger().error('Failed to call /stop_recording service.')

    # ------------------------------------------------------------------
    # Controller switching
    # ------------------------------------------------------------------
    def switch_controller(self, servo=False, sim=False):
        del sim  # Retained for compatibility with the older call signature.
        return self.switch_free_drive_controller(controller="servo" if servo else "trajectory")

    def _start_freedrive_heartbeat(self):
        if self.freedrive_timer is not None:
            return
        self.freedrive_timer = self.create_timer(
            1.0 / FREEDRIVE_PUBLISH_RATE_HZ, self._publish_freedrive_enable,
            callback_group=self.status_cb_group,
        )

    def _publish_freedrive_enable(self):
        self.freedrive_pub.publish(Bool(data=True))

    def _stop_freedrive_heartbeat(self):
        if self.freedrive_timer is not None:
            self.freedrive_timer.cancel()
            self.destroy_timer(self.freedrive_timer)
            self.freedrive_timer = None
        self.freedrive_pub.publish(Bool(data=False))

    def switch_free_drive_controller(self, controller="trajectory"):
        # controller: "trajectory", "servo", or "freedrive"
        target_state = {
            "trajectory": ControllerState.TRAJECTORY,
            "servo": ControllerState.SERVO,
            "freedrive": ControllerState.FREEDRIVE,
        }[controller]

        with self._controller_switch_lock:
            if target_state == self.controller_state:
                self.get_logger().info(f'{target_state.value} already active, skipping switch')
                return None

            # Deactivate whatever is actually tracked as active, including
            # freedrive, before activating the requested controller.
            previous_state = self.controller_state
            request = SwitchController.Request()
            request.deactivate_controllers = [previous_state.value]
            request.activate_controllers = [target_state.value]
            request.timeout = rclpy.duration.Duration(seconds=5.0).to_msg()
            request.strictness = SwitchController.Request.BEST_EFFORT
            future = self.switch_controller_client.call_async(request)
            self._wait_for_future(future)
            result = future.result()

            if result is not None and result.ok:
                self.controller_state = target_state
                if target_state == ControllerState.FREEDRIVE:
                    self._start_freedrive_heartbeat()
                elif previous_state == ControllerState.FREEDRIVE:
                    self._stop_freedrive_heartbeat()
            else:
                self.get_logger().error(f'Failed to switch from {previous_state.value} to {target_state.value}')

            return result

    @contextmanager
    def _motion_activity(self, label):
        with self._motion_lock:
            if self._motion_depth == 0:
                if self.controller_state == ControllerState.FREEDRIVE:
                    result = self.switch_free_drive_controller(controller="trajectory")
                    if result is not None and not result.ok:
                        raise RuntimeError("Cannot start arm motion: trajectory controller activation failed")
                self._motion_label = str(label)
            self._motion_depth += 1
        try:
            yield
        finally:
            with self._motion_lock:
                self._motion_depth = max(0, self._motion_depth - 1)
                if self._motion_depth == 0:
                    self._motion_label = None

    def start_servo(self):
        future = self.start_servo_client.call_async(Trigger.Request())
        self._wait_for_future(future)
        return future.result()

    def configure_servo(self, frame):
        # "base_link" for base frame, "tool0" for tool frame
        req = SetParameters.Request()
        new_param_value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=frame)
        req.parameters = [Parameter(name='moveit_servo.robot_link_command_frame', value=new_param_value)]
        future = self.configure_servo_cli.call_async(req)
        self._wait_for_future(future)

    # ------------------------------------------------------------------
    # Arm motion helpers
    # ------------------------------------------------------------------
    def go_to_home(self):
        with self._motion_activity("move arm home"):
            future = self.start_move_arm_to_home_client.call_async(Trigger.Request())
            self._wait_for_future(future)
            return future.result()

    def go_to_scan_position(self):
        with self._motion_activity("move arm to scan position"):
            future = self.trigger_move_arm_to_config_client.call_async(Trigger.Request())
            self._wait_for_future(future)
            return future.result()

    def start_apple_prediction(self):
        future = self.start_apple_prediction_client.call_async(ApplePrediction.Request())
        self._wait_for_future(future)
        return future.result().apple_poses

    def call_coord_to_traj(self, apple_pose):
        coord = Point()
        coord.x = apple_pose.position.x
        coord.y = apple_pose.position.y
        coord.z = apple_pose.position.z
        request = CoordinateToTrajectory.Request()
        request.coordinate = coord
        future = self.coord_to_traj_client.call_async(request)
        self._wait_for_future(future)
        return future.result().waypoints

    def trigger_arm_mover(self, trajectory):
        with self._motion_activity("execute arm trajectory"):
            request = SendTrajectory.Request()
            request.waypoints = trajectory
            future = self.trigger_arm_mover_client.call_async(request)
            self._wait_for_future(future)
            return future.result()

    def trigger_move_arm_to_pose(self, apple_pose):
        with self._motion_activity("move arm to apple pose"):
            request = MoveToPose.Request()
            request.orientation = apple_pose.orientation
            request.position = apple_pose.position
            request.position.y = request.position.y - 0.3
            future = self.trigger_move_arm_to_pose_client.call_async(request)
            self._wait_for_future(future)
            return future.result()

    def configure_controller(self):
        pick_force = 20.0
        set_goal_req = SetValue.Request()
        set_goal_req.val = pick_force
        future = self.set_goal_cli.call_async(set_goal_req)
        self._wait_for_future(future)

    def sweep_status_callback(self, msg):
        self.sweep_running = msg.data

    def sweep_error_callback(self, msg):
        self.sweep_tracking_error = msg.data

    def set_sweep_theta(self, theta_deg):
        request = SetValue.Request()
        request.val = theta_deg
        future = self.sweep_set_theta_cli.call_async(request)
        self._wait_for_future(future)
        return future.result()

    def release_controller(self):
        future = self.release_controller_client.call_async(Trigger.Request())
        self._wait_for_future(future)
        return future.result()

    # ------------------------------------------------------------------
    # Cancelable actions -- visual servo, grasp/pressure-servo approach, and
    # pick controller all run for a variable, potentially long time and are
    # the ones an abort needs to actually interrupt.
    # ------------------------------------------------------------------
    def _action_feedback_cb(self, feedback_msg):
        self.get_logger().info(f'action feedback: {feedback_msg.feedback}')

    def _run_cancelable_action(self, client, goal_msg, label):
        if client is None:
            self.get_logger().error(f"{label} action client unavailable (interface not implemented yet) -- skipping stage")
            return None

        client.wait_for_server()
        send_goal_future = client.send_goal_async(goal_msg, feedback_callback=self._action_feedback_cb)
        self._wait_for_future(send_goal_future)
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"{label} goal rejected")
            return None

        with self._goal_lock:
            self.current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            if self.abort_event.is_set():
                self.get_logger().warn(f"Abort requested -- cancelling {label}")
                cancel_future = goal_handle.cancel_goal_async()
                self._wait_for_future(cancel_future)
                self._wait_for_future(result_future)
                break
            time.sleep(0.02)

        with self._goal_lock:
            self.current_goal_handle = None

        return result_future.result().result if result_future.done() else None

    def visual_servo_action(self):
        goal = VisualServo.Goal() if VisualServo else None
        if goal is not None:
            goal.start = True
        return self._run_cancelable_action(self._visual_servo_client, goal, 'visual_servo')

    def grasp_controller_action(self):
        goal = GraspControl.Goal() if GraspControl else None
        if goal is not None:
            goal.start_grasp = True
        return self._run_cancelable_action(self._grasp_client, goal, 'grasp_controller')

    def pick_controller_action(self):
        if self.PICK_PATTERN == 'sweep':
            # pick_controller.py has no 'sweep' branch -- talk to
            # sweep_controller.py's own services directly instead of
            # routing through the pick_controller action/node.
            return self._run_sweep_pick()

        if PickControl is None:
            self.get_logger().error("PickController action client unavailable (interface not implemented yet) -- skipping stage")
            return None
        self.configure_controller()
        goal = PickControl.Goal()
        goal.pattern = self.PICK_PATTERN
        goal.stop_time = Float32(data=2.0 if self.PICK_PATTERN == 'stiffness-seeking' else 5.0)
        return self._run_cancelable_action(self._pick_controller_client, goal, 'pick_controller')

    def _run_sweep_pick(self):
        # Not cancelable through a goal handle like _run_cancelable_action --
        # there's no action server here, so this polls abort_event itself and
        # calls sweep/stop_controller directly the moment an abort fires.
        theta_result = self.set_sweep_theta(self.SWEEP_THETA_DEG)
        if not (theta_result and theta_result.success):
            self.get_logger().error(
                f"failed to set sweep theta to {self.SWEEP_THETA_DEG} deg -- "
                f"continuing with sweep_controller's current value")

        self.sweep_tracking_error = None
        start_future = self.sweep_start_cli.call_async(Trigger.Request())
        self._wait_for_future(start_future)

        if not start_future.result().success:
            self.get_logger().error(f"sweep failed to start: {start_future.result().message}")
        else:
            while not self.sweep_running and not self.abort_event.is_set():
                time.sleep(0.02)
            while self.sweep_running:
                if self.abort_event.is_set():
                    self.get_logger().warn("Abort requested -- stopping sweep")
                    break
                if self.sweep_tracking_error is not None:
                    self.get_logger().info(
                        f'sweep tracking error: {self.sweep_tracking_error:.4f}',
                        throttle_duration_sec=0.5)
                time.sleep(0.02)

        stop_future = self.sweep_stop_cli.call_async(Empty.Request())  # idempotent safety call
        self._wait_for_future(stop_future)
        return None

    # ------------------------------------------------------------------
    # EventDetection action (unchanged from start_harvest*.py)
    # ------------------------------------------------------------------
    def start_detection(self):
        goal_msg = EventDetection.Goal()
        goal_msg.failure_ratio = (1.0 - self.EVENT_SENSITIVITY)
        self._event_client.wait_for_server()
        send_goal_future = self._event_client.send_goal_async(goal_msg)
        return send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.finished))

    # ------------------------------------------------------------------
    # Stage runner
    # ------------------------------------------------------------------
    def run_stage(self, topics, prefix, servo_frame=None, use_servo=True, action_fn=None):
        stage_name = prefix if isinstance(prefix, str) else str(prefix)
        print(f"--- Running stage: {stage_name} ---")
        with self._motion_activity(stage_name):
            if self.abort_event.is_set():
                # Caught here rather than only after action_fn(), so a manual
                # abort requested while idle between stages (e.g. at an input()
                # prompt) is honored before spinning up recording/servo again.
                self.handle_abort()
                raise HarvestAborted(stage_name)

            if self.enable_recording:
                self.start_recording(topics, prefix)
                time.sleep(self.recording_startup_delay)

            self.switch_controller(servo=use_servo)
            if use_servo:
                self.start_servo()
            if servo_frame:
                self.configure_servo(servo_frame)
            if action_fn:
                action_fn()

            if self.abort_event.is_set():
                self.handle_abort()
                raise HarvestAborted(stage_name)

            self.switch_controller(servo=not use_servo)
            if self.enable_recording:
                self.stop_recording()

    # ------------------------------------------------------------------
    # Stage sets
    # ------------------------------------------------------------------
    def _run_freedrive_apple(self, base_dir):
        """One manual, free-driven apple cycle: user free-drives the arm to
        the apple, then the normal visual-servo / pressure-servo / pick /
        release stages run and record exactly like the automated flow."""
        input('Hit enter to start free drive controller and move the arm to the apple location manually, then hit enter to continue')
        self.switch_free_drive_controller(controller="freedrive")
        input('Hit enter to stop free drive controller and continue')
        self.switch_free_drive_controller(controller="trajectory")

        if self.enable_visual_servo:
            input('hit enter to start visual servoing')
            self.run_stage(
                self.visual_servo_topics,
                base_dir + self.visual_servo_file_name_prefix,
                use_servo=True,
                action_fn=self.visual_servo_action,
            )

        if self.enable_pressure_servo or self.enable_picking:
            input('Done with approach, hit enter to start pressure servoing and pick controller')

            def pick_action():
                self.configure_servo('tool0')
                if self.enable_pressure_servo:
                    self.grasp_controller_action()
                if self.enable_picking:
                    self.get_logger().info(f"Picking with: {self.PICK_PATTERN}")
                    self.pick_controller_action()
                
            self.PICK_PATTERN = self.org_pick_pattern
            self.run_stage(
                self.pressure_servo_and_pick_controller_topics,
                base_dir + self.final_approach_and_pick_file_name_prefix,
                servo_frame='base_link',
                use_servo=True,
                action_fn=pick_action,
            )

            original_pick_pattern = self.PICK_PATTERN
            self.configure_servo('base_link')
            def pull_back_action():
                self.configure_servo('base_link')
                if self.enable_picking:
                    self.PICK_PATTERN = 'linear-pull'
                    self.pick_controller_action()
                
            self.run_stage(
                self.pressure_servo_and_pick_controller_topics,
                base_dir + 'post_pick_pull',
                servo_frame='amiga__base',
                use_servo=True,
                action_fn=pull_back_action,
            )
            self.PICK_PATTERN = original_pick_pattern

        input('Done with pick, hit enter to release and continue')
        if self.enable_pressure_servo:
            self.release_controller()

    def _run_freedrive_loop(self, start_idx=1):
        """Repeats _run_freedrive_apple, incrementing the apple index each
        time since freedrive has no predicted locations to key off of.
        Keeps going until the user stops it."""
        idx = start_idx
        while True:
            base_dir = self.batch_dir + f'apple_{idx}/'
            print(
                f"HARVEST_CONTEXT batch={self.batch_number} apple={idx} "
                f"batch_dir={self.batch_dir}",
                flush=True,
            )
            try:
                self.get_logger().info(f'Freedrive apple {idx}')
                self._run_freedrive_apple(base_dir)
            except HarvestAborted:
                self.get_logger().error(f'Aborted during freedrive apple {idx} -- back in freedrive, ready to retry')
            idx += 1

            again = input('Hit enter to freedrive to another apple, or type q then enter to stop freedrive: ')
            if again.strip().lower() == 'q':
                break

    def _finish_batch(self):
        if self.enable_recording:
            self.save_metadata()
        self.get_logger().info('Batch Complete')

    def _run_full_batch_flow(self):
        """Predict apples, then run autonomous pick attempts."""
        self.get_logger().info("Moving to scan position")
        self.go_to_scan_position()

        if self.enable_apple_prediction:
            self.get_logger().info('Predicting apple locations')
            apple_poses = None
            while rclpy.ok():
                apple_poses = None

                def predict_action():
                    nonlocal apple_poses
                    apple_poses = self.start_apple_prediction()

                if self.enable_recording:
                    self.start_prediction_throttles()
                try:
                    self.run_stage(
                        self.prediction_topics,
                        self.batch_dir + self.prediction_file_name_prefix,
                        use_servo=False,
                        action_fn=predict_action,
                    )
                    self.switch_controller(servo=False)
                except HarvestAborted:
                    self.get_logger().error('Aborted during apple prediction')
                    if self.abort_recovery_mode != 'freedrive':
                        self.get_logger().error('Ending batch early')
                        return
                    input('Arm is in freedrive. Reposition it, then hit enter to repeat the scan and prediction: ')
                    self.go_to_scan_position()
                    continue
                finally:
                    if self.enable_recording:
                        self.stop_prediction_throttles()
                break
            if apple_poses is None:
                self.get_logger().error('No apple prediction available; ending batch')
                return
        else:
            self.get_logger().info('Skipping apple prediction, using pre-saved locations')
            apple_poses = PoseArray()
            apple_poses.poses = [
                Pose(position=Point(x=row[0], y=row[1], z=row[2]))
                for row in self.pre_saved_apple_locations
            ]
        self.apple_coordinates = {
            f'apple_{i + 1}': [p.position.x, p.position.y, p.position.z]
            for i, p in enumerate(apple_poses.poses)
        }
        self.get_logger().info(f'Found {len(apple_poses.poses)} apples!')
        if self.manual_apple_selection:
            self.apple_selection.set_available(len(apple_poses.poses))
            self._publish_available_apple_ids()

        self.get_logger().info('Resetting arm to home position')
        # self.go_to_home()

        automatic_ids = iter(range(len(apple_poses.poses)))
        while rclpy.ok():
            idx = self._wait_for_selected_apple() if self.manual_apple_selection else next(automatic_ids, None)
            if idx is None:
                break
            coord = apple_poses.poses[idx]
            base_dir = self.batch_dir + f'apple_{idx}/'
            print(
                f"HARVEST_CONTEXT batch={self.batch_number} apple={idx} "
                f"batch_dir={self.batch_dir}",
                flush=True,
            )
            try:
                input(f'Hit enter to start with apple {idx}')
                self._raise_if_aborted(f'apple {idx} before approach')
                self.get_logger().info(f'Approaching apple {idx}: Coord {coord}')
                if self.use_optimal_trajectory:
                    waypoints = self.call_coord_to_traj(coord)
                    self._raise_if_aborted(f'apple {idx} trajectory planning')
                    self.trigger_arm_mover(waypoints)
                else:
                    self.trigger_move_arm_to_pose(coord)
                self._raise_if_aborted(f'apple {idx} approach')

                if self.enable_visual_servo:
                    input('hit enter to start visual servoing')
                    self.run_stage(
                        self.visual_servo_topics,
                        base_dir + self.visual_servo_file_name_prefix,
                        use_servo=True,
                        action_fn=self.visual_servo_action,
                    )

                if self.enable_pressure_servo or self.enable_picking:
                    input('Done with approach, hit enter to start pressure servoing and pick controller')

                    def pick_action():
                        if self.enable_pressure_servo:
                            self.grasp_controller_action()
                        if self.enable_picking:
                            self.pick_controller_action()
                        self.configure_servo('tool0')

                    self.run_stage(
                        self.pressure_servo_and_pick_controller_topics,
                        base_dir + self.final_approach_and_pick_file_name_prefix,
                        servo_frame='base_link',
                        use_servo=True,
                        action_fn=pick_action,
                    )

                original_pick_pattern = self.PICK_PATTERN

                def pull_back_action():
                    if self.enable_picking:
                        self.PICK_PATTERN = 'linear-pull'
                        self.pick_controller_action()
                    self.configure_servo('base_link')

                try:
                    self.run_stage(
                        [],
                        base_dir + 'post_pick_pull',
                        servo_frame='amiga__base',
                        use_servo=True,
                        action_fn=pull_back_action,
                    )
                finally:
                    self.PICK_PATTERN = original_pick_pattern

                input('Done with pick, hit enter to return home')
                self._raise_if_aborted(f'apple {idx} before returning to scan')
                # self.go_to_home()
                self.go_to_scan_position()
                if self.enable_pressure_servo:
                    self.release_controller()

            except HarvestAborted:
                self.get_logger().error(f'Aborted while working on apple {idx}')
                if self.abort_recovery_mode == 'freedrive':
                    if not self.manual_apple_selection:
                        self.manual_apple_selection = True
                        self.apple_selection.set_available(len(apple_poses.poses))
                    self.apple_selection.clear_pending()
                    self._publish_available_apple_ids()
                    print('HARVEST_SELECTION enabled', flush=True)
                    self.get_logger().warn(
                        f'Arm is in freedrive. Reposition it, then select an apple ID '
                        f'for another autonomous attempt; apple {idx} may be selected again.'
                    )
                    continue
                else:
                    self.get_logger().error('Ending batch early')
                break

    def start(self):
        if self.freedrive_mode:
            self._run_freedrive_loop()
        else:
            self._run_full_batch_flow()
        self._finish_batch()


def main(args=None):
    rclpy.init(args=args)
    node = StartHarvestAbort()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Spin in the background so the abort/servo-status subscriber keeps
    # being serviced while start() blocks through its stages -- this is
    # what makes an abort possible mid-stage instead of only between stages.
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.start()
    except HarvestAborted as exc:
        node.get_logger().error(f"Harvest aborted during stage: {exc}")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
