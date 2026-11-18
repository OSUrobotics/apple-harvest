#!/usr/bin/env python3

# ROS
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray
import tf2_ros
from moveit_msgs.srv import GetPlanningScene
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

# Interfaces
from harvest_interfaces.srv import ApplePrediction, VoxelGrid, MoveToPose, SendTrajectory
from tree_template_interfaces.msg import TrunkInfo

# Python
import threading
import time
import numpy as np
import os
import yaml
import copy
from sklearn.cluster import DBSCAN


class OrchardTemplating(Node):

    def __init__(self):
        super().__init__("orchard_templating_node")
        m_callback_group = MutuallyExclusiveCallbackGroup()
        # ReentrantCallbackGroup for the trunk measurement subscriber so it
        # fires freely regardless of what other callbacks are executing.
        sub_callback_group = ReentrantCallbackGroup()

        # Initialize TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # TODO: Set as ros2 parameters
        self.data_save_dir = '/home/marcus/apple_harvest_ws/test_templating_data/'
        self.arm_workspace_radius = 1.5  # meters
        self.planning_frame = 'amiga__base'
        self.rgbd_dir_path = None
        self.tree_number = None
        self.yolo_model = None
        self.apple_coords = None
        self.voxel_size = 0.05
        self.apple_approach_offset = 0.04  # meters
        self.apples_found = 0
        self.apples_reached_templating = 0
        self.apples_reached_voxelization = 0
        self.unreached_idx_templating = []
        self.unreached_idx_voxelization = []
        self.unreachable_idx_arm_ws = []
        self.side_branch_locations = []

        # Trunk measurement state — populated by _trunk_measurement_callback,
        # used by trigger_trunk_estimation to block until a fresh measurement
        # arrives after the trigger fires.
        self._latest_trunk_pose = None
        self._trunk_detections: list = []    # all TrunkInfo msgs from current trigger
        self._trunk_trigger_active: bool = False  # True while a trigger is in flight
        self._trunk_received_event: threading.Event | None = None

        # Publishers
        self.voxel_collision_pub = self.create_publisher(CollisionObject, "/collision_object", 10)
        self.voxel_marker_publisher = self.create_publisher(MarkerArray, 'voxel_markers', 10)
        self.voxel_marker_removed_publisher = self.create_publisher(MarkerArray, 'removed_voxel_markers', 10)

        # Trunk measurement subscriber — ReentrantCallbackGroup so it is
        # never blocked by service calls happening on m_callback_group.
        self.trunk_meas_sub = self.create_subscription(
            TrunkInfo,
            '/trunk_measurements_raw',
            self._trunk_measurement_callback,
            10,
            callback_group=sub_callback_group,
        )

        # Service clients
        self.start_apple_prediction_client = self.create_client(
            ApplePrediction, "/apple_prediction", callback_group=m_callback_group)
        while not self.start_apple_prediction_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Apple prediction service not available, waiting...")

        self.apple_prediction_set_params_client = self.create_client(
            SetParameters, '/apple_prediction/set_parameters', callback_group=m_callback_group)
        while not self.apple_prediction_set_params_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for apple prediction set_parameters service...')

        self.set_template_anchor_client = self.create_client(
            SetParameters, '/trellis_from_rgbd_extraction/set_parameters', callback_group=m_callback_group)
        while not self.set_template_anchor_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for template anchor parameter service...')

        self.trigger_estimation_client = self.create_client(
            Trigger, '/trigger_estimation', callback_group=m_callback_group)
        while not self.trigger_estimation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /trigger_estimation service...')

        self.correct_template_anchor_client = self.create_client(
            Trigger, '/correct_template_anchor', callback_group=m_callback_group)
        while not self.correct_template_anchor_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /correct_template_anchor service...')

        self.clear_template_collisions_client = self.create_client(
            Trigger, '/clear_trellis_trees', callback_group=m_callback_group)
        while not self.clear_template_collisions_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for clear template collisions service...')

        self.voxel_client = self.create_client(
            VoxelGrid, "voxel_grid", callback_group=m_callback_group)
        while not self.voxel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Voxel grid service not available, waiting...")

        self.clear_voxel_collisions_client = self.create_client(
            Trigger, '/clear_voxels', callback_group=m_callback_group)
        while not self.clear_voxel_collisions_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for clear voxel collisions service...')

        self.move_arm_to_pose_client = self.create_client(
            MoveToPose, "/move_arm_to_pose", callback_group=m_callback_group)
        while not self.move_arm_to_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Move arm to pose service not available, waiting...")

        self.start_move_arm_to_home_client = self.create_client(
            Trigger, "/move_arm_to_home", callback_group=m_callback_group)
        while not self.start_move_arm_to_home_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Start move arm to home service not available, waiting...")

        self.trigger_arm_mover_client = self.create_client(
            SendTrajectory, 'send_arm_trajectory', callback_group=m_callback_group)
        while not self.trigger_arm_mover_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for send_arm_trajectory to be available...')

        self.get_planning_scene_client = self.create_client(
            GetPlanningScene, 'get_planning_scene', callback_group=m_callback_group)
        while not self.get_planning_scene_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for get_planning_scene_client to be available...')

        # Trees to run, by SLAM map tree id. tree_ids (list) wins over tree_id. -1 = not set.
        #   ros2 run harvest orchard_templating.py --ros-args -p tree_id:=3
        #   ros2 run harvest orchard_templating.py --ros-args -p "tree_ids:=[0, 1, 2]"
        self.declare_parameter('tree_id', -1)
        self.declare_parameter('tree_ids', [-1])
        self.declare_parameter('data_save_dir', self.data_save_dir)
        self.data_save_dir = os.path.join(str(self.get_parameter('data_save_dir').value), '')

        tree_ids = [int(t) for t in self.get_parameter('tree_ids').value if int(t) >= 0]
        if not tree_ids and int(self.get_parameter('tree_id').value) >= 0:
            tree_ids = [int(self.get_parameter('tree_id').value)]
        self.tree_ids = tree_ids
        if not self.tree_ids:
            self.get_logger().error('No tree set. Set the tree_id or tree_ids parameter.')

    # =========================================================================
    #  Callbacks
    # =========================================================================

    def _trunk_measurement_callback(self, msg: TrunkInfo):
        """Accumulate trunk detections while a trigger is in flight.

        All TrunkInfo messages that arrive while _trunk_trigger_active is True
        are collected into _trunk_detections.  The threading event is set on
        the first message so trigger_trunk_estimation can unblock; the 50 ms
        sleep that follows collects any remaining detections from the same
        frame before _select_most_central_detection picks the winner.

        _trunk_trigger_active provides the temporal gate: messages outside a
        trigger window are dropped immediately, so no additional stamp-based
        stale guard is needed.
        """
        if not self._trunk_trigger_active:
            return

        self._trunk_detections.append(msg)

        if self._trunk_received_event is not None:
            self._trunk_received_event.set()

    # =========================================================================
    #  Parameter / vision helpers
    # =========================================================================

    def set_template_anchor(self, anchor_position):
        param_value = ParameterValue(
            type=ParameterType.PARAMETER_INTEGER, integer_value=anchor_position)
        param = Parameter(name='anchor_tree_id', value=param_value)
        req = SetParameters.Request()
        req.parameters = [param]
        return self.set_template_anchor_client.call(req)

    def set_apple_prediction_tree_id(self, tree_id):
        """Set the presaved_images.tree_id parameter on the apple_prediction node.
        Accepts an int, numeric string, or 'tree_###' format.
        """
        if tree_id is None:
            self.get_logger().warn('Requested to set empty tree_id; skipping')
            return None

        try:
            if isinstance(tree_id, str) and tree_id.startswith('tree_'):
                tid_val = int(tree_id.split('_')[-1])
            else:
                tid_val = int(tree_id)
        except Exception as e:
            self.get_logger().error(f"Invalid tree_id '{tree_id}': {e}")
            return None

        pv = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=tid_val)
        p = Parameter(name='presaved_images.tree_id', value=pv)
        req = SetParameters.Request()
        req.parameters = [p]

        resp = self.apple_prediction_set_params_client.call(req)
        if resp is not None:
            self.get_logger().info(f"Set apple_prediction presaved_images.tree_id -> {tid_val}")
        else:
            self.get_logger().error("Failed to set apple_prediction tree id: no response")
        return resp

    def set_base_cam_frame_index(self, offset: int):
        """Tell apple_prediction to switch the canonical base_cam topic to *offset*.

        *offset* must be one of -2, -1, 0, 1, 2 (signed frame offset relative to
        the broadside timestamp).  The latched republish happens immediately on the
        apple_prediction side, so callers should allow a brief propagation delay
        (~100 ms) before triggering trunk estimation.
        """
        pv  = ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                             integer_value=int(offset))
        p   = Parameter(name='presaved_images.base_cam_frame_index', value=pv)
        req = SetParameters.Request()
        req.parameters = [p]
        resp = self.apple_prediction_set_params_client.call(req)
        if resp is None:
            self.get_logger().warn(
                f"set_base_cam_frame_index({offset}): no response from apple_prediction")
        return resp

    def trigger_trunk_estimation(self, timeout_sec: float = 3.0):
        """Fire /trigger_estimation and block until a fresh measurement arrives
        on /trunk_measurements_raw or the timeout expires.

        Design notes
        ------------
        * Trigger state (_trunk_trigger_active, event) is armed BEFORE the
          trigger fires so that even a very fast response is never missed.
        * We intentionally keep waiting for the measurement even when the
          service call itself times out — the trigger may still be processing
          (ML inference can take >5 s) and the measurement will arrive shortly.
        """
        # Arm trigger state BEFORE firing so fast replies are never missed.
        self._latest_trunk_pose = None
        self._trunk_detections = []
        self._trunk_trigger_active = True
        self._trunk_received_event = threading.Event()

        # Allow plenty of time for ML inference; do NOT exit early if this
        # call times out — the trigger may still be running on the remote node.
        result = self.trigger_estimation_client.call(Trigger.Request())

        if result is not None and not result.success:
            # Service explicitly reported failure — no measurement will arrive.
            self.get_logger().warn(
                f'Trunk estimation trigger reported failure: {result.message}')
            self._trunk_received_event = None
            self._trunk_trigger_active = False
            return result

        if result is None:
            self.get_logger().warn(
                'Trunk estimation service call timed out (30 s). '
                'The remote node may still be processing — continuing to '
                f'wait up to {timeout_sec} s for the measurement.'
            )
        else:
            self.get_logger().info(
                f'Estimation triggered — waiting up to {timeout_sec}s '
                'for measurement on /trunk_measurements_raw...'
            )

        # Block this thread; _trunk_measurement_callback sets the event
        # from the executor's ReentrantCallbackGroup thread.
        received = self._trunk_received_event.wait(timeout=timeout_sec)
        self._trunk_received_event = None

        if received:
            # Brief pause to let any additional detections from the same frame
            # arrive before we close the accumulation window.
            time.sleep(0.05)

        self._trunk_trigger_active = False

        if self._trunk_detections:
            best = self._select_most_central_detection(self._trunk_detections)
            self._latest_trunk_pose = best.pose
            self.get_logger().info(
                f'Trunk measurement received — '
                f'{len(self._trunk_detections)} detection(s), '
                f'selected most central '
                f'(cam_lateral={best.cam_lateral:.3f} m, cam_depth={best.cam_depth:.3f} m).'
            )
        else:
            self._latest_trunk_pose = None
            self.get_logger().warn(
                f'Timed out after {timeout_sec}s waiting for trunk measurement. '
                'Check that the camera has a clear view of the trunk and '
                'trunk_detection_relay is running.'
            )

        return result

    def trigger_trunk_estimation_buffer(
        self,
        offsets: tuple = (-2, -1, 0, 1, 2),
        timeout_sec: float = 3.0,
    ) -> list:
        """Trigger trunk estimation for every frame in the base_cam buffer.

        For each *offset* in *offsets*:
          1. Switch apple_prediction's canonical base_cam topic to that frame.
          2. Wait briefly for the latched republish to reach the trunk estimator.
          3. Call trigger_trunk_estimation() and record the result.

        Returns a flat list of ``(offset, TrunkInfo)`` tuples — one entry per
        detection across all frames.  Frames with multiple detections each
        contribute multiple entries; frames with no detection contribute none.
        """
        all_detections = []
        for offset in offsets:
            self.get_logger().info(
                f"[buffer] Switching base_cam active frame → offset {offset:+d}")
            self.set_base_cam_frame_index(offset)
            time.sleep(0.1)  # allow latched publish to reach the trunk estimator

            self.trigger_trunk_estimation(timeout_sec=timeout_sec)

            # Snapshot every TrunkInfo from this frame (trigger_trunk_estimation
            # freezes _trunk_detections before returning).
            frame_detections = list(self._trunk_detections)
            for trunk_info in frame_detections:
                all_detections.append((offset, trunk_info))

            if frame_detections:
                self.get_logger().info(
                    f"[buffer] offset={offset:+d}  {len(frame_detections)} detection(s): "
                    + ", ".join(
                        f"(x={ti.pose.position.x:.3f}, y={ti.pose.position.y:.3f}, "
                        f"cam_depth={ti.cam_depth:.3f})"
                        for ti in frame_detections
                    ))
            else:
                self.get_logger().warn(f"[buffer] offset={offset:+d}  no detection")

        self.get_logger().info(
            f"[buffer] {len(all_detections)} total detection(s) across {len(offsets)} frames")
        return all_detections

    def _select_most_central_detection(self, detections: list):
        """Return the TrunkInfo from *detections* whose angular offset from the
        camera optical axis is smallest (|cam_lateral / cam_depth|).

        When the trunk estimator finds multiple trunks in a single frame, the
        one closest to the image centre is most likely to be the target tree
        rather than a neighbouring trunk or artefact near the frame edge.
        """
        best = min(detections, key=lambda m: m.cam_lateral ** 2 + m.cam_depth ** 2)

        if len(detections) > 1:
            self.get_logger().info(
                f'[centrality] {len(detections)} detections; '
                + ', '.join(
                    f'(lat={m.cam_lateral:.3f}, dep={m.cam_depth:.3f}){"*" if m is best else ""}'
                    for m in detections
                )
            )
        return best

    def _select_best_trunk_detection(self, detections: list) -> int:
        """Cluster all detections from the buffer sweep and return the offset
        of the best detection in the cluster nearest to the camera.

        detections: flat list of (offset, TrunkInfo) as returned by
                    trigger_trunk_estimation_buffer().
        Returns: the signed frame offset to re-trigger on.
        """
        if len(detections) == 1:
            return detections[0][0]

        positions = np.array(
            [[d[1].pose.position.x, d[1].pose.position.y] for d in detections]
        )

        # DBSCAN: eps=0.5 m in base-frame XY; min_samples=1 so every point
        # belongs to a cluster (no noise points).
        labels = DBSCAN(eps=0.5, min_samples=1).fit_predict(positions)
        unique_labels = np.unique(labels)

        cluster_avg_depths = {}
        for label in unique_labels:
            idxs = [i for i, l in enumerate(labels) if l == label]
            avg_depth = float(np.mean([detections[i][1].cam_depth for i in idxs]))
            cluster_avg_depths[label] = avg_depth
            centroid = positions[idxs].mean(axis=0)
            self.get_logger().info(
                f"[cluster {label}] {len(idxs)} detection(s), "
                f"centroid=({centroid[0]:.3f}, {centroid[1]:.3f}), "
                f"avg_cam_depth={avg_depth:.3f} m")

        # Select cluster with smallest average cam_depth (nearest to camera).
        best_label = min(unique_labels, key=lambda l: cluster_avg_depths[l])
        best_cluster_idxs = [i for i, l in enumerate(labels) if l == best_label]

        # Within that cluster pick the detection with the smallest individual cam_depth.
        best_idx = min(best_cluster_idxs, key=lambda i: detections[i][1].cam_depth)
        best_offset = detections[best_idx][0]

        self.get_logger().info(
            f"[buffer] {len(unique_labels)} cluster(s) → selected cluster {best_label} "
            f"(avg_cam_depth={cluster_avg_depths[best_label]:.3f} m), "
            f"best offset={best_offset:+d}")
        return best_offset

    def correct_template_anchor(self):
        """Delegate anchor correction to trellis_from_rgbd_extraction node."""
        result = self.correct_template_anchor_client.call(Trigger.Request())
        if result is None:
            self.get_logger().warn('Template anchor correction failed: no response')
        elif result.success:
            self.get_logger().info(f'Template anchor corrected: {result.message}')
        else:
            self.get_logger().warn(f'Template anchor correction failed: {result.message}')
        return result

    # =========================================================================
    #  Scene management
    # =========================================================================

    def remove_tree_from_scene(self):
        result = self.clear_template_collisions_client.call(Trigger.Request())
        if result and result.success:
            self.get_logger().info('Successfully cleared template collisions from scene')
        else:
            self.get_logger().warn('Failed to clear template collisions from scene')
        return result

    def remove_voxels_from_scene(self):
        result = self.clear_voxel_collisions_client.call(Trigger.Request())
        if result and result.success:
            self.get_logger().info('Successfully cleared voxel collisions from scene')
        else:
            self.get_logger().warn('Failed to clear voxel collisions from scene')
        return result

    # =========================================================================
    #  Apple prediction / motion
    # =========================================================================

    def start_apple_prediction(self):
        result = self.start_apple_prediction_client.call(ApplePrediction.Request())
        if result is None:
            self.get_logger().error('Apple prediction service returned no response')
            return None
        return result.apple_poses

    def sort_coordinates(self, target_coord, coords):
        distances = np.linalg.norm(coords - target_coord, axis=1)
        sorted_indices = np.argsort(distances)
        return coords[sorted_indices]

    def get_manipulator_base_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'amiga__base', 'base_link', rclpy.time.Time())
            position = transform.transform.translation
            return np.array([position.x, position.y, position.z])
        except Exception as e:
            self.get_logger().error(f"Failed to get manipulator base position: {e}")
            return np.array([0.0, 0.0, 0.0])

    def filter_reachable_coords_from_arm_ws(self, coordinates):
        manipulator_base_position = self.get_manipulator_base_position()
        reachable_coords = []
        reachable_count = 0
        for i, coord in enumerate(coordinates):
            distance = np.linalg.norm(coord - manipulator_base_position)
            if distance <= self.arm_workspace_radius:
                reachable_coords.append(coord)
                reachable_count += 1
            else:
                self.unreachable_idx_arm_ws.append(i)
        self.get_logger().info(
            f'# of reachable apple coordinates within arm workspace: '
            f'{reachable_count} out of {len(coordinates)}'
        )
        return np.array(reachable_coords)

    def call_voxel_grid_service(self):
        request = VoxelGrid.Request()
        request.voxel_size = self.voxel_size
        return self.voxel_client.call(request)

    def send_pose_goal(self, coordinate):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.planning_frame
        pose_stamped.pose.position.x = coordinate[0]
        pose_stamped.pose.position.y = coordinate[1]
        pose_stamped.pose.position.z = coordinate[2]
        request = MoveToPose.Request()
        request.pose_stamped = pose_stamped
        return self.move_arm_to_pose_client.call(request)

    def send_trajectory(self, trajectory):
        request = SendTrajectory.Request()
        request.waypoints = trajectory
        return self.trigger_arm_mover_client.call(request)

    def go_to_home(self):
        result = self.start_move_arm_to_home_client.call(Trigger.Request())
        if result and result.success:
            self.get_logger().info('Successfully moved home')
        else:
            self.get_logger().warn('Failed to move home')
        return result

    # =========================================================================
    #  Data saving
    # =========================================================================

    def save_metadata(self):
        data = {
            'rgbd_dir_path':                self.rgbd_dir_path,
            'tree_number':                  self.tree_number,
            'YOLO_model':                   self.yolo_model,
            'apple_coordinates':            self.apple_coords.tolist(),
            'voxel_size':                   self.voxel_size,
            'apple_approach_offset':        self.apple_approach_offset,
            'apples_found':                 self.apples_found,
            'apples_reached_templating':    self.apples_reached_templating,
            'apples_reached_voxelization':  self.apples_reached_voxelization,
            'unreached_idx_templating':     self.unreached_idx_templating,
            'unreached_idx_voxelization':   self.unreached_idx_voxelization,
            'unreachable_idx_via_arm_ws':   self.unreachable_idx_arm_ws,
        }

        os.makedirs(self.data_save_dir, exist_ok=True)
        with open(self.data_save_dir + f'tree_{self.tree_number}_results.yaml', 'w') as file:
            yaml.dump(data, file)

        self.get_logger().info("YAML file saved successfully.")

    # =========================================================================
    #  Main sequence
    # =========================================================================

    def start(self):
        self.get_logger().info(f'Trees to run: {self.tree_ids}')
        for tree_id in self.tree_ids:
            self.run_tree(tree_id)
        self.get_logger().info('Trial complete!')

    def run_tree(self, tree_id: int):
        self.tree_number = tree_id

        ### Set presaved_images.tree_id on apple_prediction node
        self.get_logger().info(f"Setting apple_prediction tree id to {self.tree_number}")
        resp = self.set_apple_prediction_tree_id(self.tree_number)
        if resp is None or not all(r.successful for r in resp.results):
            self.get_logger().error(
                f'apple_prediction could not load tree {self.tree_number}; skipping this tree.')
            return

        ### STAGE 0 - INITIALIZE ARM POSITION AND LOCATE APPLES
        # self.get_logger().info('Moving arm to home')
        # self.go_to_home()

        # self.get_logger().info('Sending request to predict apple centerpoint locations in scene.')
        # apple_poses = self.start_apple_prediction()
        # self.apples_found = len(apple_poses.poses)
        # self.apple_coords = np.array([
        #     [pose.position.x, pose.position.y, pose.position.z]
        #     for pose in apple_poses.poses
        # ])
        # self.get_logger().info(f'# of apples found: {len(apple_poses.poses)}')
        # apple_coords = self.filter_reachable_coords_from_arm_ws(self.apple_coords)

        ### STAGE 1 - TEMPLATING
        self.get_logger().info(f'Starting templating method at tree number: {self.tree_number}')

        # Place trellis template at the stored anchor position.
        # Blocks until trellis_from_rgbd_extraction has published the registry.
        self.get_logger().info('Setting template anchor...')
        resp = self.set_template_anchor(self.tree_number)
        if resp is None or not all(r.successful for r in resp.results):
            self.get_logger().error(
                f'Could not set the template anchor to tree {self.tree_number}; skipping this tree.')
            return
        self.get_logger().info('Template anchor set.')

        # Sweep all 5 base_cam buffer frames and collect trunk detections.
        self.get_logger().info('Triggering trunk estimation across buffer frames...')
        detections = self.trigger_trunk_estimation_buffer()

        if not detections:
            self.get_logger().warn(
                'No trunk detections across any buffer frame — skipping '
                'anchor correction. Check camera view and relay node.'
            )
        else:
            # Pick the frame whose detection is closest to the median position
            # (robust against frames blocked by foliage).
            best_offset = self._select_best_trunk_detection(detections)
            self.get_logger().info(
                f'Re-triggering estimation on best frame (offset={best_offset:+d})...')
            self.set_base_cam_frame_index(best_offset)
            time.sleep(0.1)  # allow latched republish to reach trunk estimator
            self.trigger_trunk_estimation()

            if self._latest_trunk_pose is None:
                self.get_logger().warn(
                    f'Final estimation on offset={best_offset:+d} produced no '
                    'detection — skipping anchor correction.')
            else:
                # Brief propagation delay so trellis_from_rgbd_extraction has
                # dispatched its own callback before we ask it to correct.
                time.sleep(0.3)
                self.get_logger().info('Requesting template anchor correction...')
                self.correct_template_anchor()

        # # Apply gripper approach offset to each apple location
        # apple_coords = copy.deepcopy(apple_coords)
        # apple_coords[:, 1] -= self.apple_approach_offset

        # # Move arm to each apple position
        # for i, apple in enumerate(self.apple_coords):
        #     if i in self.unreachable_idx_arm_ws:
        #         self.get_logger().warn(
        #             f'Apple ID: {i} not reachable via arm workspace, '
        #             'skipping templating for this apple'
        #         )
        #         self.unreached_idx_templating.append(i)
        #         continue
        #     self.get_logger().info(f'Moving arm to apple ID: {i}')
        #     result = self.send_pose_goal(apple)
        #     if result.result:
        #         self.get_logger().info(f'Apple ID: {i} reached')
        #         self.apples_reached_templating += 1
        #         self.get_logger().info('Moving arm to home')
        #         self.send_trajectory(result.reverse_traj)
        #     else:
        #         self.get_logger().warn(f'Apple ID: {i} not reachable')
        #         self.unreached_idx_templating.append(i)

        # self.get_logger().info(
        #     f'Number of apples reached via templating: {self.apples_reached_templating}')

        # Remove template
        # self.get_logger().info('Removing template from the planning scene')
        # self.remove_tree_from_scene()

        ### STAGE 2 - VOXELIZATION
        # self.get_logger().info('Starting voxelization method')
        # self.get_logger().info('Sending request to extract voxels from point cloud.')
        # voxel_data = self.call_voxel_grid_service()
        # self.get_logger().info(f"# of voxels generated: {len(voxel_data.voxel_centers)}")

        # for i, apple in enumerate(self.apple_coords):
        #     if i in self.unreachable_idx_arm_ws:
        #         self.get_logger().warn(
        #             f'Apple ID: {i} not reachable via arm workspace, '
        #             'skipping voxelization for this apple'
        #         )
        #         self.unreached_idx_voxelization.append(i)
        #         continue
        #     self.get_logger().info(f'Moving arm to apple ID: {i}')
        #     result = self.send_pose_goal(apple)
        #     if result.result:
        #         self.get_logger().info(f'Apple ID: {i} reached')
        #         self.apples_reached_voxelization += 1
        #         self.get_logger().info('Moving arm to home')
        #         self.send_trajectory(result.reverse_traj)
        #     else:
        #         self.get_logger().warn(f'Apple ID: {i} not reachable')
        #         self.unreached_idx_voxelization.append(i)

        # self.get_logger().info(
        #     f'Number of apples reached via voxelization: {self.apples_reached_voxelization}')

        # Remove voxels
        # self.get_logger().info('Removing voxels from the planning scene')
        # self.remove_voxels_from_scene()

        ### STAGE 3 - SAVE DATA
        # self.save_metadata()

        self.get_logger().info(f'Tree {self.tree_number} complete.')


def main(args=None):
    rclpy.init(args=args)
    node = OrchardTemplating()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    # Run start() in a daemon thread so executor.spin() can run concurrently.
    # This is required because trigger_trunk_estimation uses threading.Event.wait()
    # — the executor must be free to dispatch _trunk_measurement_callback while
    # start() is blocked waiting. Without this, the subscriber callback can never
    # fire and the wait always times out.
    t = threading.Thread(target=node.start, daemon=True)
    t.start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()