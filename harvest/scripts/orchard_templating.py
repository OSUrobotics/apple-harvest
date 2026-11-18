#!/usr/bin/env python3

# ROS
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
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

# Python
import json
import threading
import numpy as np
import os
import yaml


class OrchardTemplating(Node):

    def __init__(self):
        super().__init__("orchard_templating_node")
        m_callback_group = MutuallyExclusiveCallbackGroup()

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

        # Result of the last /correct_template_anchor call: dict from the JSON in Trigger.message
        # (see trellis_from_rgbd_extraction.py), plus "success"
        self.correction = None

        # Publishers
        self.voxel_collision_pub = self.create_publisher(CollisionObject, "/collision_object", 10)
        self.voxel_marker_publisher = self.create_publisher(MarkerArray, 'voxel_markers', 10)
        self.voxel_marker_removed_publisher = self.create_publisher(MarkerArray, 'removed_voxel_markers', 10)

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

    def correct_template_anchor(self):
        """
        Ask trellis_from_rgbd_extraction to correct the template map from the base-camera trunk detections
        (it selects the frames, triggers the detector, matches and fits). Keep the response in self.correction.
        """
        resp = self.correct_template_anchor_client.call(Trigger.Request())
        if resp is None:
            self.correction = {'success': False, 'error': 'no response'}
            self.get_logger().warn('Template anchor correction failed: no response')
            return self.correction
        try:
            self.correction = json.loads(resp.message)
        except ValueError:
            self.correction = {'error' if not resp.success else 'summary': resp.message}
        self.correction['success'] = bool(resp.success)
        if resp.success:
            self.get_logger().info(
                f"Template corrected: {self.correction.get('summary')} "
                f"(matched trees {self.correction.get('matched_tree_ids')}, "
                f"frames {self.correction.get('frames_used')})")
        else:
            self.get_logger().warn(f"Template anchor correction failed: {self.correction.get('error')}")
        return self.correction

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

        # Correct the whole template map from the base-camera trunk detections.
        self.get_logger().info('Requesting template correction...')
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
    # Run start() in a daemon thread so executor.spin() can run concurrently: start() makes blocking
    # service calls, and the executor must be free to handle their responses.
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