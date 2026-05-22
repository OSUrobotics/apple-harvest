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
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

# Interfaces
from harvest_interfaces.srv import ApplePrediction, VoxelGrid, MoveToPose, SendTrajectory

# Python
import numpy as np
import os
import yaml
import copy


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

        # Publishers
        self.voxel_collision_pub = self.create_publisher(CollisionObject, "/collision_object", 10)
        self.voxel_marker_publisher = self.create_publisher(MarkerArray, 'voxel_markers', 10)
        self.voxel_marker_removed_publisher = self.create_publisher(MarkerArray, 'removed_voxel_markers', 10)

        # Services
        self.start_apple_prediction_client = self.create_client(ApplePrediction, "/apple_prediction", callback_group=m_callback_group)
        while not self.start_apple_prediction_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Apple prediction service not available, waiting...")   

        self.apple_prediction_params_client = self.create_client(GetParameters, '/apple_prediction/get_parameters', callback_group=m_callback_group)
        while not self.apple_prediction_params_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for apple prediction parameters service...')

        self.set_template_anchor_client = self.create_client(SetParameters, '/trellis_from_rgbd_extraction/set_parameters', callback_group=m_callback_group)
        while not self.set_template_anchor_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for template anchor parameter service...')

        self.trigger_estimation_client = self.create_client(Trigger, '/trigger_estimation', callback_group=m_callback_group)
        while not self.trigger_estimation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /trigger_estimation service...')

        self.correct_template_anchor_client = self.create_client(Trigger, '/correct_template_anchor', callback_group=m_callback_group)
        while not self.correct_template_anchor_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /correct_template_anchor service...')

        self.clear_template_collisions_client = self.create_client(Trigger, '/clear_trellis_trees', callback_group=m_callback_group)
        while not self.clear_template_collisions_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for clear template collisions service...')        

        self.voxel_client = self.create_client(VoxelGrid, "voxel_grid", callback_group=m_callback_group)    
        while not self.voxel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Voxel grid service not available, waiting...")   

        self.clear_voxel_collisions_client = self.create_client(Trigger, '/clear_voxels', callback_group=m_callback_group)
        while not self.clear_voxel_collisions_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for clear voxel collisions service...')   

        self.move_arm_to_pose_client = self.create_client(MoveToPose, "/move_arm_to_pose", callback_group=m_callback_group)
        while not self.move_arm_to_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Move arm to pose service not available, waiting...")

        self.start_move_arm_to_home_client = self.create_client(Trigger, "/move_arm_to_home", callback_group=m_callback_group)
        while not self.start_move_arm_to_home_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Start move arm to home service not available, waiting...")

        self.trigger_arm_mover_client = self.create_client(SendTrajectory, 'send_arm_trajectory', callback_group=m_callback_group)
        while not self.trigger_arm_mover_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for send_arm_trajectory to be available...')

        self.get_planning_scene_client = self.create_client(GetPlanningScene, 'get_planning_scene', callback_group=m_callback_group)
        while not self.get_planning_scene_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for get_planning_scene_client to be available...')

    # =========================================================================
    #  Parameter / vision helpers
    # =========================================================================

    def get_apple_prediction_params(self):
        req = GetParameters.Request()
        req.names = ['prediction_model_path', 'presaved_images.rgbd_dir_path']
        future = self.apple_prediction_params_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
            self.yolo_model = response.values[0].string_value
            self.rgbd_dir_path = response.values[1].string_value
            self.get_logger().info(f"Retrieved parameter: {self.rgbd_dir_path}")
            self.tree_number = int(self.rgbd_dir_path[-3:])
            self.get_logger().info(f"Tree number for templating: {self.tree_number}")
        except Exception as e:
            self.get_logger().error(f"Failed to get parameter: {e}")

    def set_template_anchor(self, anchor_position):
        param_value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=anchor_position)
        param = Parameter(name='anchor_tree_id', value=param_value)

        req = SetParameters.Request()
        req.parameters = [param]

        future = self.set_template_anchor_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def trigger_trunk_estimation(self):
        future = self.trigger_estimation_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def correct_template_anchor(self):
        """Delegate anchor correction to the TemplateAnchorCorrector node."""
        future = self.correct_template_anchor_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        if result.success:
            self.get_logger().info(f"Template anchor corrected: {result.message}")
        else:
            self.get_logger().warn(f"Template anchor correction failed: {result.message}")

        return result

    # =========================================================================
    #  Scene management
    # =========================================================================

    def remove_tree_from_scene(self):
        request = Trigger.Request()
        future = self.clear_template_collisions_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('Successfully cleared template collisions from scene')
        else:
            self.get_logger().warn('Failed to clear template collisions from scene')

        return future.result()

    def remove_voxels_from_scene(self):
        request = Trigger.Request()
        future = self.clear_voxel_collisions_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('Successfully cleared voxel collisions from scene')
        else:
            self.get_logger().warn('Failed to clear voxel collisions from scene')

        return future.result()

    # =========================================================================
    #  Apple prediction / motion
    # =========================================================================

    def start_apple_prediction(self):
        self.request = ApplePrediction.Request()
        self.future = self.start_apple_prediction_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().apple_poses

    def sort_coordinates(self, target_coord, coords):
        distances = np.linalg.norm(coords - target_coord, axis=1)
        sorted_indices = np.argsort(distances)
        return coords[sorted_indices]

    def get_manipulator_base_position(self):
        try:
            transform = self.tf_buffer.lookup_transform('amiga__base', 'base_link', rclpy.time.Time())
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
        self.get_logger().info(f'# of reachable apple coordinates within arm workspace: {reachable_count} out of {len(coordinates)}')
        return np.array(reachable_coords)

    def call_voxel_grid_service(self):
        request = VoxelGrid.Request()
        request.voxel_size = self.voxel_size

        self.future = self.voxel_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_pose_goal(self, coordinate):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.planning_frame
        pose_stamped.pose.position.x = coordinate[0]
        pose_stamped.pose.position.y = coordinate[1]
        pose_stamped.pose.position.z = coordinate[2]

        self.request = MoveToPose.Request()
        self.request.pose_stamped = pose_stamped
        self.future = self.move_arm_to_pose_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_trajectory(self, trajectory):
        request = SendTrajectory.Request()
        request.waypoints = trajectory

        future = self.trigger_arm_mover_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def go_to_home(self):
        self.request = Trigger.Request()
        self.future = self.start_move_arm_to_home_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)

        if self.future.result().success:
            self.get_logger().info('Successfully moved home')
        else:
            self.get_logger().warn('Failed to move home')

        return self.future.result()

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
        ### GET VISION EXPERIMENT PARAMETER
        self.get_logger().info('Getting vision experiment parameter')
        self.get_apple_prediction_params()

        ### STAGE 0 - INITIALIZE ARM POSITION AND LOCATE APPLES
        self.get_logger().info('Moving arm to home')
        self.go_to_home()

        self.get_logger().info('Sending request to predict apple centerpoint locations in scene.')
        apple_poses = self.start_apple_prediction()
        self.apples_found = len(apple_poses.poses)
        self.apple_coords = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in apple_poses.poses])
        self.get_logger().info(f'# of apples found: {len(apple_poses.poses)}')
        apple_coords = self.filter_reachable_coords_from_arm_ws(self.apple_coords)

        ### STAGE 1 - TEMPLATING
        self.get_logger().info(f'Starting templating method at tree number: {self.tree_number}')

        # Place trellis template at the stored anchor position
        self.set_template_anchor(self.tree_number)

        self.get_logger().info('Triggering trunk estimation...')
        self.trigger_trunk_estimation()

        # Refine anchor trunk position using the live depth sensor
        self.get_logger().info('Requesting template anchor correction...')
        self.correct_template_anchor()

        # Apply gripper approach offset to each apple location
        apple_coords = copy.deepcopy(apple_coords)
        apple_coords[:, 1] -= self.apple_approach_offset

        # Move arm to each apple position
        for i, apple in enumerate(self.apple_coords):
            if i in self.unreachable_idx_arm_ws:
                self.get_logger().warn(f'Apple ID: {i} not reachable via arm workspace, skipping templating for this apple')
                self.unreached_idx_templating.append(i)
                continue
            self.get_logger().info(f'Moving arm to apple ID: {i}')
            result = self.send_pose_goal(apple)
            if result.result:
                self.get_logger().info(f'Apple ID: {i} reached')
                self.apples_reached_templating += 1
                self.get_logger().info('Moving arm to home')
                self.send_trajectory(result.reverse_traj)
            else:
                self.get_logger().warn(f'Apple ID: {i} not reachable')
                self.unreached_idx_templating.append(i)

        self.get_logger().info(f'Number of apples reached via templating: {self.apples_reached_templating}')

        # Remove template
        self.get_logger().info('Removing template from the planning scene')
        self.remove_tree_from_scene()

        ### STAGE 2 - VOXELIZATION
        self.get_logger().info('Starting voxelization method')
        self.get_logger().info('Sending request to extract voxels from point cloud.')
        voxel_data = self.call_voxel_grid_service()
        self.get_logger().info(f"# of voxels generated: {len(voxel_data.voxel_centers)}")

        for i, apple in enumerate(self.apple_coords):
            if i in self.unreachable_idx_arm_ws:
                self.get_logger().warn(f'Apple ID: {i} not reachable via arm workspace, skipping voxelization for this apple')
                self.unreached_idx_voxelization.append(i)
                continue
            self.get_logger().info(f'Moving arm to apple ID: {i}')
            result = self.send_pose_goal(apple)
            if result.result:
                self.get_logger().info(f'Apple ID: {i} reached')
                self.apples_reached_voxelization += 1
                self.get_logger().info('Moving arm to home')
                self.send_trajectory(result.reverse_traj)
            else:
                self.get_logger().warn(f'Apple ID: {i} not reachable')
                self.unreached_idx_voxelization.append(i)

        self.get_logger().info(f'Number of apples reached via voxelization: {self.apples_reached_voxelization}')

        # Remove voxels
        self.get_logger().info('Removing voxels from the planning scene')
        self.remove_voxels_from_scene()

        ### STAGE 3 - SAVE DATA
        self.save_metadata()

        ### COMPLETE
        self.get_logger().info('Trial complete!')


def main(args=None):
    rclpy.init(args=args)
    node = OrchardTemplating()
    node.start()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()