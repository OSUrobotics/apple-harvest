#!/usr/bin/env python3
"""
orchard_templating.py

Reachability trial on presaved data: template collision geometry vs. voxelized point cloud.
harvest/launch/templating_trial.launch.py starts the other nodes.

For each tree in tree_ids:
   1. Load the tree (apple_prediction presaved_images.tree_id).
   2. Place the template (trellis_from_rgbd_extraction anchor_tree_id).
   3. Correct the template from the base-camera trunk detections (/correct_template_anchor).
   4. Detect the apples once (/apple_prediction). The same apple list and indices go to both methods.
   5. Mark the apples outside the arm workspace (arm_workspace_radius from the manipulator base).
   6. Template method: plan to each apple in the workspace.
   7. Remove the template from the planning scene.
   8. Voxelize the mast-camera point cloud (/voxel_grid).
   9. Voxel method: for each apple, the voxel object without the voxels within voxel_free_radius of that
      apple (moveit_msgs/ApplyPlanningScene), then plan.
  10. Remove the voxels.
  11. Save the results: results.yaml and the images (apples_annotated.png, trunk_seg_<n>.png) in
      data_save_dir/<dataset>/<YYYYmmdd_HHMMSS>/tree_<id>/.

results.yaml: dataset, tree id, git state of the two repos, settings, template registry before and after the
correction, correction result, and for each apple its position, in_workspace and the result of each method.

Planning: MoveIt /plan_kinematic_path (no motion), from the home joint state, with the same settings for both
methods. Goal: gripper_link at the apple moved apple_approach_offset toward the robot, with the move_arm_node default
orientation (turned 180 deg about z for apples on the right, -y).

Run:
    ros2 run harvest orchard_templating.py --ros-args -p tree_id:=3
    ros2 run harvest orchard_templating.py --ros-args -p "tree_ids:=[0, 1, 2]"
"""

# ROS
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import (
    BoundingVolume, Constraints, MoveItErrorCodes, OrientationConstraint, PlanningScene,
    PlanningSceneComponents, PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPlanningScene
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Image, JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
import tf2_ros

# Interfaces
from cv_bridge import CvBridge
from harvest_interfaces.srv import ApplePrediction, VoxelGrid
from harvest_vision.voxelize_scan import collision_object
from tree_template.trellis_geometry import load_meta_yamls
from tree_template_interfaces.msg import TrunkRegistry

# Python
import cv2
import datetime
import json
import math
import os
import subprocess
import threading
import time

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

# MoveItErrorCodes value -> name
_ERROR_NAMES = {getattr(MoveItErrorCodes, n): n for n in dir(MoveItErrorCodes)
                if n.isupper() and isinstance(getattr(MoveItErrorCodes, n), int)}


def git_info(repo_dir: str) -> dict:
    """Commit, branch and dirty flag of a git repo."""
    def run(*args):
        return subprocess.run(['git', '-C', repo_dir, *args], capture_output=True, text=True, timeout=10).stdout.strip()
    try:
        return {'commit': run('rev-parse', 'HEAD'), 'branch': run('rev-parse', '--abbrev-ref', 'HEAD'),
                'dirty': bool(run('status', '--porcelain', '--untracked-files=no'))}
    except Exception as e:
        return {'error': str(e)}


class OrchardTemplating(Node):

    def __init__(self):
        super().__init__("orchard_templating_node")
        m_callback_group = MutuallyExclusiveCallbackGroup()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Parameters ----------------------------------------------------
        # Trees to run, by SLAM map tree id. tree_ids (list) wins over tree_id. -1 = not set.
        self.declare_parameter('tree_id', -1)
        self.declare_parameter('tree_ids', [-1])
        self.declare_parameter('trial_dir', '')       # dataset folder; its name goes into the output path
        self.declare_parameter('data_save_dir', os.path.expanduser('~/apple_harvest_ws/test_templating_data'))
        self.declare_parameter('planning_frame', 'amiga__base')
        self.declare_parameter('manipulator_base_frame', 'base_link')
        self.declare_parameter('arm_workspace_radius', 1.0)          # [m] from the manipulator base (UR5e reach ~0.85 m)
        self.declare_parameter('apple_approach_offset', 0.04)        # [m] goal moved toward the robot
        # Planning (the same for both methods)
        self.declare_parameter('planning_group', 'ur5e_manipulator')
        self.declare_parameter('ee_link', 'gripper_link')
        self.declare_parameter('planner_id', 'RRTConnectkConfigDefault')
        self.declare_parameter('planning_time', 5.0)                 # [s] for each apple
        self.declare_parameter('num_planning_attempts', 10)
        self.declare_parameter('goal_position_tolerance', 1e-4)      # [m] (MoveGroupInterface default)
        self.declare_parameter('goal_orientation_tolerance', 1.05)   # [rad] (move_arm_node)
        self.declare_parameter('home_joint_names', [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'])
        self.declare_parameter('home_joint_positions', [               # move_arm_node home
            math.pi / 4, -math.pi / 2, 2 * math.pi / 3, 5 * math.pi / 6, -math.pi / 2, 0.0])
        # Scene
        self.declare_parameter('voxel_size', 0.05)                   # [m]
        self.declare_parameter('voxel_free_radius', 0.08)            # [m] free space around the target apple
        self.declare_parameter('voxel_object_id', 'voxels')          # voxelize_scan voxel_object_id
        self.declare_parameter('template_prefix', 'v_trellis_tree_') # tree_scene_node trellis_prefix
        self.declare_parameter('scene_wait_sec', 10.0)
        # Results
        self.declare_parameter('registry_topic', '/fastslam_registry')
        self.declare_parameter('apple_image_topic', '/apple_annotated')
        self.declare_parameter('trunk_image_topic', '/annotated_segmented_image')
        self.declare_parameter('repo_dirs', [
            os.path.expanduser('~/apple_harvest_ws/src/apple-harvest'),
            os.path.expanduser('~/apple_harvest_ws/src/orchard_tree_templating')])

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        tree_ids = [int(t) for t in gp('tree_ids') if int(t) >= 0]
        if not tree_ids and int(gp('tree_id')) >= 0:
            tree_ids = [int(gp('tree_id'))]
        self.tree_ids = tree_ids
        if not self.tree_ids:
            self.get_logger().error('No tree set. Set the tree_id or tree_ids parameter.')
        self.trial_dir = str(gp('trial_dir'))
        self.data_save_dir = str(gp('data_save_dir'))
        self.planning_frame = str(gp('planning_frame'))
        self.manipulator_base_frame = str(gp('manipulator_base_frame'))
        self.arm_workspace_radius = float(gp('arm_workspace_radius'))
        self.apple_approach_offset = float(gp('apple_approach_offset'))
        self.planning_group = str(gp('planning_group'))
        self.ee_link = str(gp('ee_link'))
        self.planner_id = str(gp('planner_id'))
        self.planning_time = float(gp('planning_time'))
        self.num_planning_attempts = int(gp('num_planning_attempts'))
        self.goal_position_tolerance = float(gp('goal_position_tolerance'))
        self.goal_orientation_tolerance = float(gp('goal_orientation_tolerance'))
        self.home_joint_names = list(gp('home_joint_names'))
        self.home_joint_positions = [float(v) for v in gp('home_joint_positions')]
        self.voxel_size = float(gp('voxel_size'))
        self.voxel_free_radius = float(gp('voxel_free_radius'))
        self.voxel_object_id = str(gp('voxel_object_id'))
        self.template_prefix = str(gp('template_prefix'))
        self.scene_wait_sec = float(gp('scene_wait_sec'))
        self.run_stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.git = {os.path.basename(d): git_info(d) for d in gp('repo_dirs')}

        # ---- Subscriptions for the results (they run while start() waits for service responses) ----
        sub_group = ReentrantCallbackGroup()
        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._registry = (None, 0.0)          # (TrunkRegistry, receive time)
        self._apple_image = (None, 0.0)       # (Image, receive time)
        self._trunk_images = None             # list while a correction runs
        self.create_subscription(
            TrunkRegistry, str(gp('registry_topic')), self._registry_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=sub_group)
        self.create_subscription(Image, str(gp('apple_image_topic')), self._apple_image_cb, 10,
                                 callback_group=sub_group)
        self.create_subscription(Image, str(gp('trunk_image_topic')), self._trunk_image_cb, 10,
                                 callback_group=sub_group)

        # ---- Service clients -------------------------------------------------
        def client(srv_type, name):
            c = self.create_client(srv_type, name, callback_group=m_callback_group)
            while not c.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {name}...')
            return c

        self.start_apple_prediction_client = client(ApplePrediction, '/apple_prediction')
        self.apple_prediction_set_params_client = client(SetParameters, '/apple_prediction/set_parameters')
        self.set_template_anchor_client = client(SetParameters, '/trellis_from_rgbd_extraction/set_parameters')
        self.correct_template_anchor_client = client(Trigger, '/correct_template_anchor')
        self.clear_template_collisions_client = client(Trigger, '/clear_trellis_trees')
        self.voxel_client = client(VoxelGrid, '/voxel_grid')
        self.clear_voxel_collisions_client = client(Trigger, '/clear_voxels')
        self.plan_client = client(GetMotionPlan, '/plan_kinematic_path')
        self.apply_scene_client = client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_planning_scene_client = client(GetPlanningScene, '/get_planning_scene')

    # =========================================================================
    #  Subscriptions (results)
    # =========================================================================

    def _registry_cb(self, msg: TrunkRegistry):
        with self._lock:
            self._registry = (msg, time.monotonic())

    def _apple_image_cb(self, msg: Image):
        with self._lock:
            self._apple_image = (msg, time.monotonic())

    def _trunk_image_cb(self, msg: Image):
        with self._lock:
            if self._trunk_images is not None:
                self._trunk_images.append(msg)

    def _wait_newer(self, attr: str, t0: float, timeout_sec: float = 5.0):
        """The message in self.<attr> received after t0 (monotonic), or None after timeout_sec."""
        t_end = time.monotonic() + timeout_sec
        while time.monotonic() < t_end:
            with self._lock:
                msg, t = getattr(self, attr)
            if msg is not None and t > t0:
                return msg
            time.sleep(0.05)
        return None

    def registry_dict(self, msg) -> list:
        """Trunks of a TrunkRegistry with their tree ids (the registry has the trees with a position, in id order)."""
        if msg is None:
            return []
        data = load_meta_yamls(self.trial_dir) if self.trial_dir else {}
        ids = [t for t in sorted(data) if 'tree_world_xy_m' in data[t]]
        if len(ids) != len(msg.trunks):
            ids = [None] * len(msg.trunks)
        out = []
        for tid, t in zip(ids, msg.trunks):
            o = t.pose.orientation
            out.append({'tree_id': tid, 'x': float(t.pose.position.x), 'y': float(t.pose.position.y),
                        'yaw': float(Rotation.from_quat([o.x, o.y, o.z, o.w]).as_euler('ZYX')[0]),
                        'side': t.side})
        return out

    # =========================================================================
    #  Service calls
    # =========================================================================

    def _call(self, client, request, timeout_sec: float = 30.0):
        """Service call with a timeout. Return the response, or None on timeout."""
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=timeout_sec):
            client.remove_pending_request(future)
            self.get_logger().error(f'{client.srv_name}: no response after {timeout_sec:.0f} s.')
            return None
        return future.result()

    # =========================================================================
    #  Parameter / vision helpers
    # =========================================================================

    def _set_int_param(self, client, name: str, value: int) -> bool:
        req = SetParameters.Request(parameters=[Parameter(
            name=name, value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(value)))])
        resp = self._call(client, req)
        return resp is not None and all(r.successful for r in resp.results)

    def set_template_anchor(self, tree_id: int) -> bool:
        return self._set_int_param(self.set_template_anchor_client, 'anchor_tree_id', tree_id)

    def set_apple_prediction_tree_id(self, tree_id: int) -> bool:
        return self._set_int_param(self.apple_prediction_set_params_client, 'presaved_images.tree_id', tree_id)

    def correct_template_anchor(self) -> dict:
        """/correct_template_anchor result: the JSON of Trigger.message plus 'success'."""
        resp = self._call(self.correct_template_anchor_client, Trigger.Request(), 300.0)
        result = json.loads(resp.message) if resp is not None else {'error': 'no response'}
        result['success'] = resp is not None and resp.success
        if result['success']:
            self.get_logger().info(f"Template corrected: {result['summary']}")
        else:
            self.get_logger().warn(f"Template correction failed: {result['error']}")
        return result

    def detect_apples(self) -> np.ndarray:
        """(N, 3) apple centers in the planning frame (empty if the prediction failed)."""
        resp = self._call(self.start_apple_prediction_client, ApplePrediction.Request(), 120.0)
        if resp is None:
            self.get_logger().error('Apple prediction service returned no response')
            return np.zeros((0, 3))
        if resp.apple_poses.poses and resp.apple_poses.header.frame_id not in ('', self.planning_frame):
            self.get_logger().error(
                f"Apple poses are in '{resp.apple_poses.header.frame_id}', not '{self.planning_frame}'.")
            return np.zeros((0, 3))
        return np.array([[p.position.x, p.position.y, p.position.z] for p in resp.apple_poses.poses]).reshape(-1, 3)

    # =========================================================================
    #  Planning scene
    # =========================================================================

    def scene_object_ids(self):
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        resp = self._call(self.get_planning_scene_client, req, 10.0)
        return None if resp is None else [co.id for co in resp.scene.world.collision_objects]

    def wait_for_scene(self, predicate, what: str) -> bool:
        """Wait (scene_wait_sec) until predicate(object ids) is true."""
        t_end = time.monotonic() + self.scene_wait_sec
        while True:
            ids = self.scene_object_ids()
            if ids is not None and predicate(ids):
                return True
            if time.monotonic() > t_end:
                self.get_logger().error(f'Planning scene: timeout waiting for {what}.')
                return False
            time.sleep(0.2)

    def n_template_objects(self, ids) -> int:
        return sum(i.startswith(self.template_prefix) for i in ids)

    def remove_template(self) -> bool:
        resp = self._call(self.clear_template_collisions_client, Trigger.Request())
        ok = resp is not None and resp.success
        return ok and self.wait_for_scene(lambda ids: self.n_template_objects(ids) == 0, 'template removal')

    def remove_voxels(self) -> bool:
        resp = self._call(self.clear_voxel_collisions_client, Trigger.Request())
        return resp is not None and resp.success

    def set_voxel_object(self, centers: np.ndarray) -> bool:
        """Put the voxel collision object in the scene with these voxel centers (ADD replaces the object)."""
        co = collision_object(self.voxel_object_id, self.planning_frame, centers, self.voxel_size)
        req = ApplyPlanningScene.Request(scene=PlanningScene(is_diff=True))
        req.scene.world.collision_objects = [co]
        resp = self._call(self.apply_scene_client, req)
        return resp is not None and resp.success

    # =========================================================================
    #  Arm workspace, goals and planning
    # =========================================================================

    def get_manipulator_base_position(self) -> np.ndarray:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.planning_frame, self.manipulator_base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=2.0))
            p = tf.transform.translation
            return np.array([p.x, p.y, p.z])
        except Exception as e:
            self.get_logger().error(f'Failed to get manipulator base position: {e}')
            return None

    def goal_for(self, apple: np.ndarray, base: np.ndarray):
        """
        Gripper goal (position, quaternion xyzw) for one apple: apple_approach_offset toward the robot (y), and the
        move_arm_node default orientation (RPY pi/2, pi, pi, gripper to +y) turned by pi about z for -y apples.
        """
        side = 1.0 if apple[1] >= base[1] else -1.0
        position = apple - np.array([0.0, side * self.apple_approach_offset, 0.0])
        rot = Rotation.from_euler('xyz', [math.pi / 2, math.pi, math.pi])
        if side < 0:
            rot = Rotation.from_euler('z', math.pi) * rot
        return position, rot.as_quat()

    def plan_to(self, position: np.ndarray, quat_xyzw: np.ndarray) -> dict:
        """Plan (no motion) from the home joint state to the goal. Return the result for results.yaml."""
        req = GetMotionPlan.Request()
        mp = req.motion_plan_request
        mp.group_name = self.planning_group
        mp.planner_id = self.planner_id
        mp.pipeline_id = 'ompl'
        mp.num_planning_attempts = self.num_planning_attempts
        mp.allowed_planning_time = self.planning_time
        mp.max_velocity_scaling_factor = 0.1
        mp.max_acceleration_scaling_factor = 0.1
        mp.start_state.is_diff = True
        mp.start_state.joint_state = JointState(name=self.home_joint_names, position=self.home_joint_positions)

        pc = PositionConstraint(link_name=self.ee_link, weight=1.0)
        pc.header.frame_id = self.planning_frame
        pc.constraint_region = BoundingVolume(
            primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[self.goal_position_tolerance])],
            primitive_poses=[Pose(position=Point(x=float(position[0]), y=float(position[1]),
                                                 z=float(position[2])), orientation=Quaternion(w=1.0))])
        oc = OrientationConstraint(
            link_name=self.ee_link, weight=1.0,
            orientation=Quaternion(x=float(quat_xyzw[0]), y=float(quat_xyzw[1]),
                                   z=float(quat_xyzw[2]), w=float(quat_xyzw[3])),
            absolute_x_axis_tolerance=self.goal_orientation_tolerance,
            absolute_y_axis_tolerance=self.goal_orientation_tolerance,
            absolute_z_axis_tolerance=self.goal_orientation_tolerance)
        oc.header.frame_id = self.planning_frame
        mp.goal_constraints = [Constraints(position_constraints=[pc], orientation_constraints=[oc])]

        t0 = time.monotonic()
        resp = self._call(self.plan_client, req, self.planning_time + 30.0)
        wall = time.monotonic() - t0
        if resp is None:
            return {'success': False, 'error_code': None, 'error': 'no response', 'planning_time_s': wall}
        r = resp.motion_plan_response
        code = int(r.error_code.val)
        return {
            'success': code == MoveItErrorCodes.SUCCESS,
            'error_code': code,
            'error': _ERROR_NAMES.get(code, str(code)),
            'planning_time_s': float(r.planning_time),
            'n_waypoints': len(r.trajectory.joint_trajectory.points),
        }

    # =========================================================================
    #  Main sequence
    # =========================================================================

    def start(self):
        self.get_logger().info(f'Trees to run: {self.tree_ids}')
        for tree_id in self.tree_ids:
            try:
                self.run_tree(tree_id)
            except Exception as e:  # the next trees still run
                self.get_logger().error(f'Tree {tree_id} failed: {type(e).__name__}: {e}')
        self.get_logger().info('Trial complete!')

    def run_tree(self, tree_id: int):
        log = self.get_logger()
        res = {'tree_id': tree_id, 'trial_dir': self.trial_dir, 'git': self.git, 'settings': self.settings(),
               'apples': []}
        images = {}

        # 1-3. Load, place and correct the template
        if not self.set_apple_prediction_tree_id(tree_id):
            log.error(f'apple_prediction could not load tree {tree_id}; skipping this tree.')
            return
        t0 = time.monotonic()
        if not self.set_template_anchor(tree_id):
            log.error(f'Could not set the template anchor to tree {tree_id}; skipping this tree.')
            return
        res['registry_before'] = self.registry_dict(self._wait_newer('_registry', t0))
        with self._lock:
            self._trunk_images = []
        t0 = time.monotonic()
        res['correction'] = self.correct_template_anchor()
        with self._lock:
            for k, m in enumerate(self._trunk_images):
                images[f'trunk_seg_{k}.png'] = m
            self._trunk_images = None
        if res['correction'].get('success'):
            res['registry_after'] = self.registry_dict(self._wait_newer('_registry', t0))
            ar = res['correction'].get('anchor_residual_m')
            res['correction']['anchor_check_ok'] = ar is not None and ar < 0.05

        # 4-5. Apples (one frozen list for both methods) and arm workspace
        t0 = time.monotonic()
        apples = self.detect_apples()
        img = self._wait_newer('_apple_image', t0, 2.0)
        if img is not None:
            images['apples_annotated.png'] = img
        base = self.get_manipulator_base_position()
        if base is None:
            log.error('No manipulator base TF; skipping this tree.')
            return
        log.info(f'Tree {tree_id}: {len(apples)} apple(s)')
        for i, a in enumerate(apples):
            goal, quat = self.goal_for(a, base)
            res['apples'].append({
                'index': i,
                'position': [float(v) for v in a],
                'goal': [float(v) for v in goal],
                'goal_orientation_xyzw': [float(v) for v in quat],
                'distance_from_arm_base_m': float(np.linalg.norm(a - base)),
                'in_workspace': bool(np.linalg.norm(a - base) <= self.arm_workspace_radius),
            })
        work = [r for r in res['apples'] if r['in_workspace']]
        log.info(f'{len(work)} of {len(apples)} apple(s) in the arm workspace ({self.arm_workspace_radius} m)')

        # 6. Template method
        if self.wait_for_scene(lambda ids: self.n_template_objects(ids) > 0, 'template collision objects'):
            res['n_template_objects'] = self.n_template_objects(self.scene_object_ids() or [])
            for r in work:
                r['template'] = self.plan_to(np.array(r['goal']), np.array(r['goal_orientation_xyzw']))
                log.info(f"[template] apple {r['index']}: {r['template']['error']} "
                         f"({r['template']['planning_time_s']:.2f} s)")
        else:
            res['template_error'] = 'template not in the planning scene'

        # 7-10. Voxel method
        if not self.remove_template():
            res['voxel_error'] = 'template could not be removed'
        else:
            vox = self._call(self.voxel_client, VoxelGrid.Request(voxel_size=self.voxel_size), 120.0)
            centers = np.array([[p.x, p.y, p.z] for p in (vox.voxel_centers if vox else [])]).reshape(-1, 3)
            res['n_voxels'] = len(centers)
            if len(centers) == 0:
                res['voxel_error'] = 'no voxels'
            else:
                for r in work:
                    near = np.linalg.norm(centers - np.array(r['position']), axis=1) <= self.voxel_free_radius
                    if not self.set_voxel_object(centers[~near]):
                        r['voxel'] = {'success': False, 'error': 'apply_planning_scene failed'}
                        continue
                    r['voxel'] = self.plan_to(np.array(r['goal']), np.array(r['goal_orientation_xyzw']))
                    r['voxel']['n_voxels_freed'] = int(near.sum())
                    log.info(f"[voxel] apple {r['index']}: {r['voxel']['error']} "
                             f"({r['voxel']['planning_time_s']:.2f} s, {int(near.sum())} voxel(s) freed)")
            self.remove_voxels()

        # Summary and save
        for m in ('template', 'voxel'):
            res[f'n_reached_{m}'] = sum(1 for r in work if r.get(m, {}).get('success'))
        res['n_apples'] = len(apples)
        res['n_in_workspace'] = len(work)
        log.info(f"Tree {tree_id}: reached template {res['n_reached_template']} / voxel {res['n_reached_voxel']} "
                 f"of {len(work)} apple(s) in the workspace")
        self.save(tree_id, res, images)

    def settings(self) -> dict:
        names = ['planning_frame', 'arm_workspace_radius', 'apple_approach_offset', 'planning_group', 'ee_link',
                 'planner_id', 'planning_time', 'num_planning_attempts', 'goal_position_tolerance',
                 'goal_orientation_tolerance', 'home_joint_positions', 'voxel_size', 'voxel_free_radius']
        return {n: getattr(self, n) for n in names}

    def save(self, tree_id: int, res: dict, images: dict):
        name = os.path.basename(os.path.normpath(self.trial_dir)) if self.trial_dir else 'unknown_dataset'
        out_dir = os.path.join(self.data_save_dir, name, self.run_stamp, f'tree_{tree_id:03d}')
        os.makedirs(out_dir, exist_ok=True)
        res['images'] = []
        for fname, msg in images.items():
            try:
                cv2.imwrite(os.path.join(out_dir, fname), self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
                res['images'].append(fname)
            except Exception as e:
                self.get_logger().warn(f'Could not save {fname}: {e}')
        with open(os.path.join(out_dir, 'results.yaml'), 'w') as f:
            yaml.safe_dump(res, f, sort_keys=False)
        self.get_logger().info(f'Saved {out_dir}/results.yaml and {len(res["images"])} image(s)')


def main(args=None):
    rclpy.init(args=args)
    node = OrchardTemplating()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    # start() waits for service responses, so it runs in a thread while the executor spins.
    # The executor stops when start() is done.
    def run():
        try:
            node.start()
        finally:
            executor.shutdown(timeout_sec=0)

    threading.Thread(target=run, daemon=True).start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
