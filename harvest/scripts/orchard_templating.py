#!/usr/bin/env python3

# ROS
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from geometry_msgs.msg import TransformStamped
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from rcl_interfaces.srv import GetParameters

# Interfaces
from harvest_interfaces.srv import ApplePrediction, VoxelGrid, MoveToPose, SendTrajectory
from tree_template_interfaces.srv import UpdateTrellisPosition

# Python 
import numpy as np
import os
import yaml
import copy
from scipy.spatial.transform import Rotation as R

class OrchardTemplating(Node):

    def __init__(self):
        super().__init__("orchard_templating_node")
        m_callback_group = MutuallyExclusiveCallbackGroup()

        # Initialize TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Data saving directory
        self.data_save_dir = '/home/marcus/orchard_template_ws/results_data/v5/'

        # TODO: Set as a ros2 parameters
        self.vision_experiment = None
        self.yolo_model = 'v9e.pt'
        self.apple_coords = None
        self.voxel_size = 0.01
        self.voxel_neighbor_radii = 0.08 # 2cm larger than the max apple radii threshold
        self.apple_approach_offset = 0.04 # meters
        self.apples_found = 0
        self.apples_reached_templating = 0
        self.apples_reached_voxelization = 0
        self.unreached_idx_templating = []
        self.unreached_idx_voxelization = []
        self.side_branch_locations = []

        # Publishers
        self.voxel_collision_pub = self.create_publisher(CollisionObject, "/collision_object", 10)
        self.voxel_marker_publisher = self.create_publisher(MarkerArray, 'voxel_markers', 10)
        self.voxel_marker_removed_publisher = self.create_publisher(MarkerArray, 'removed_voxel_markers', 10)

        # Services
        self.start_apple_prediction_client = self.create_client(ApplePrediction, "/apple_prediction_presaved_images", callback_group=m_callback_group)
        while not self.start_apple_prediction_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Apple prediction service not available, waiting...")   

        self.vision_experiment_param_client = self.create_client(GetParameters, '/apple_prediction_presaved_images/get_parameters', callback_group=m_callback_group)
        while not self.vision_experiment_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for vision experiment parameter service...')

        self.voxel_client = self.create_client(VoxelGrid, "voxel_grid", callback_group=m_callback_group)    
        while not self.voxel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Voxel grid service not available, waiting...")   

        self.move_arm_to_pose_client = self.create_client(MoveToPose, "/move_arm_to_pose",callback_group=m_callback_group)
        while not self.move_arm_to_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Move arm to pose service not available, waiting...")

        self.start_move_arm_to_home_client = self.create_client(Trigger, "/move_arm_to_home", callback_group=m_callback_group)
        while not self.start_move_arm_to_home_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Start move arm to home service not available, waiting...")

        self.trellis_template_client = self.create_client(UpdateTrellisPosition, "/update_trellis_position", callback_group=m_callback_group)
        while not self.trellis_template_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Trellis template placement service not available, waiting...")

        self.trigger_arm_mover_client = self.create_client(SendTrajectory, 'send_arm_trajectory', callback_group=m_callback_group)
        while not self.trigger_arm_mover_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for send_arm_trajectory to be available...')

        self.get_planning_scene_client = self.create_client(GetPlanningScene, 'get_planning_scene', callback_group=m_callback_group)
        while not self.get_planning_scene_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for get_planning_scene_client to be available...')

    def param_callback(self, future):
        try:
            response = future.result()
            self.vision_experiment = response.values[0].string_value
            self.get_logger().info(f"Retrieved parameter: {self.vision_experiment}")

            # Update trellis_base_position after retrieving vision_experiment
            offset = 0.1  # Example offset value
            trellis_tempate_params = {
                'a': [-0.07, 1.105 - offset, -0.19], 'b': [-0.07, 1.05 - offset, -0.14],
                'c': [-0.1, 1.0 - offset, -0.17], 'd': [-0.04, 1.21 - offset, -0.17],
                'e': [-0.15, 1.02 - offset, -0.22], 'f': [-0.05, 1.05 - offset, -0.15],
                'g': [-0.09, 1.16 - offset, -0.16], 'h': [-0.08, 1.11 - offset, -0.18],
                'i': [-0.085, 1.03 - offset, -0.15], 'j': [-0.05, 1.12 - offset, -0.19]
            }
            self.trellis_base_position = trellis_tempate_params.get(self.vision_experiment)

            if self.trellis_base_position:
                self.get_logger().info(f"Trellis base position set to: {self.trellis_base_position}")
            else:
                self.get_logger().warn(f"Invalid vision_experiment value: {self.vision_experiment}")
        except Exception as e:
            self.get_logger().error(f"Failed to get parameter: {e}")

    def get_vision_experiment_param(self):
        req = GetParameters.Request()
        req.names = ['vision_experiment']
        # TODO: Can I get away with not using call_async?
        future = self.vision_experiment_param_client.call_async(req)
        future.add_done_callback(self.param_callback)
    
    def get_gripper_position(self, target_frame="world", source_frame="gripper_link"):
        try:
            # Query the transform
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )
            position = transform.transform.translation
            # self.get_logger().info(f"Gripper position: x={position.x}, y={position.y}, z={position.z}")
            return np.array((position.x, position.y, position.z))
        except tf2_ros.LookupException:
            self.get_logger().error("Transform not available!")
        except tf2_ros.ExtrapolationException:
            self.get_logger().error("Extrapolation error!")
        except tf2_ros.TransformException as e:
            self.get_logger().error(f"Failed to get transform: {str(e)}")
        return None
        
    def start_apple_prediction(self):
        # Starts servo node
        self.request = ApplePrediction.Request()
        self.future = self.start_apple_prediction_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future) 
        return self.future.result().apple_poses 
    
    def sort_coordinates(self, target_coord, coords):
        # Calculate distances from each point to the target
        distances = np.linalg.norm(coords - target_coord, axis=1)

        # Sort indices based on distances
        sorted_indices = np.argsort(distances)

        # Get sorted coordinates
        return coords[sorted_indices]
    
    def call_voxel_grid_service(self):
        request = VoxelGrid.Request()
        request.voxel_size = self.voxel_size 

        self.future = self.voxel_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future) 
        return self.future.result()
    
    def update_trellis_template_pos(self, target_base_position):
        request = UpdateTrellisPosition.Request()
        request.x = target_base_position[0]
        request.y = target_base_position[1]
        request.z = target_base_position[2]

        self.future = self.trellis_template_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future) 
        return self.future.result().success
    
    def get_neighbors(self, coordinates, target_coordinates, radius=0.25):
        # Find coordinates within the sphere
        points = []
        idx = []
        for target in target_coordinates:
            # Compute Euclidean distances from the target to all original coordinates
            distances = np.linalg.norm(coordinates - target, axis=1)
            # Get indices of coordinates within the radius
            indices = np.where(distances <= radius)[0]
            # Collect results
            points.append(coordinates[indices])
            idx.append(indices)
        
        # Flatten the array and remove duplicates
        idx_flattened = np.unique(np.hstack(idx))

        return np.array(points, dtype=object), idx_flattened
        
    def add_collision_objects(self, voxel_centers):      
        for i, voxel_center in enumerate(voxel_centers):
            collision_object = CollisionObject()
            collision_object.id = f"voxel_{i}"
            collision_object.header.frame_id = "world"

            # Define the shape and size
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [self.voxel_size] * 3

            # Define the pose
            box_pose = Pose()
            box_pose.position = voxel_center
            box_pose.orientation.w = 1.0

            collision_object.primitives.append(primitive)
            collision_object.primitive_poses.append(box_pose)
            collision_object.operation = CollisionObject.ADD

            self.voxel_collision_pub.publish(collision_object)
    
    def add_voxels(self, voxel_centers, colors=None):
        removed = False
        if colors == None:
            removed = True
            colors = [[0.8, 0.0, 0.0, 0.5] for _ in range(len(voxel_centers))]

        # # Clear existing markers
        # clear_markers = MarkerArray()
        # marker = Marker()
        # marker.action = Marker.DELETEALL
        # clear_markers.markers.append(marker)
        # self.voxel_marker_publisher.publish(clear_markers)

        marker_array = MarkerArray()
        for i, (center, color) in enumerate(zip(voxel_centers, colors)):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "voxels"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = center[0]
            marker.pose.position.y = center[1]
            marker.pose.position.z = center[2]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.voxel_size  # Cube size matches voxel size
            marker.scale.y = self.voxel_size
            marker.scale.z = self.voxel_size
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]  # Ensure alpha is set
            marker_array.markers.append(marker)

        # Publish the markers
        self.get_logger().info(f"Publishing {len(marker_array.markers)} markers to RViz.")
        if removed:
            self.voxel_marker_removed_publisher.publish(marker_array)
        else:
            self.voxel_marker_publisher.publish(marker_array)
    
    def remove_tree_from_scene(self):
        tree_object = CollisionObject()
        tree_object.id = 'v_trellis_tree'
        tree_object.operation = CollisionObject.REMOVE
        self.voxel_collision_pub.publish(tree_object)

    def get_side_branch_locations(self):
        # Wait for the service to be available
        self.get_planning_scene_client.wait_for_service()

        # Create a request to get the full planning scene
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY

        # Call the service
        future = self.get_planning_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        # Find the v_trellis_tree object in the planning scene
        tree_object = None
        for obj in response.scene.world.collision_objects:
            if obj.id == 'v_trellis_tree':
                tree_object = obj
                break

        if tree_object is None:
            print("v_trellis_tree object not found in the planning scene.")
            return

        # Extract and print horizontal branch locations with applied trellis_position and canopy_orientation
        canopy_orientation = [tree_object.pose.orientation.x,
                              tree_object.pose.orientation.y,
                              tree_object.pose.orientation.z,
                              tree_object.pose.orientation.w]
        canopy_rotation = R.from_quat(canopy_orientation)
        for pose in tree_object.primitive_poses[1:]:  # Skip the base horizontal branch
            branch_position = np.array([pose.position.x, pose.position.y, pose.position.z])
            rotated_position = canopy_rotation.apply(branch_position)
            transformed_position = rotated_position + self.trellis_base_position
            self.side_branch_locations.append(transformed_position)
        
        self.side_branch_locations = np.vstack(self.side_branch_locations).tolist()

    def send_pose_goal(self, coordinate):
        point = Point()
        point.x = coordinate[0]
        point.y = coordinate[1]
        point.z = coordinate[2]

        # Sends x,y,z to C++ moveit node to execute pose goal since python moveit not available for humble
        self.request = MoveToPose.Request()
        self.request.position = point
        self.future = self.move_arm_to_pose_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def send_trajectory(self, trajectory):
        request = SendTrajectory.Request()
        request.waypoints = trajectory  # Pass the entire Float32MultiArray message

        # Use async call
        future = self.trigger_arm_mover_client.call_async(request)
        rclpy.spin_until_future_complete(self, future) 
        return future.result()
    
    def go_to_home(self):
        # Starts go to home
        self.request = Trigger.Request()
        self.future = self.start_move_arm_to_home_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future) 

        if self.future.result().success:
            self.get_logger().info(f'Successfully moved home')
        else:
            self.get_logger().warn(f'Failed to moved home')

        return self.future.result()

    def save_metadata(self):
        # Combine the dictionaries into a list or another structure if necessary
        data = {
            'vision_experiment':    self.vision_experiment,
            'YOLO_model':           self.yolo_model,
            'apple_coordinates':    self.apple_coords.tolist(),
            'voxel_size':           self.voxel_size,
            'voxel_neighbor_radii': self.voxel_neighbor_radii,
            'apple_approach_offset':self.apple_approach_offset,
            'apples_found':         self.apples_found,
            'template_base_position': self.trellis_base_position,
            'apples_reached_templating': self.apples_reached_templating,
            'apples_reached_voxelization': self.apples_reached_voxelization,
            'unreached_idx_templating': self.unreached_idx_templating,
            'unreached_idx_voxelization': self.unreached_idx_voxelization,
            'side_branch_locations': self.side_branch_locations,
        }

        # Save to a YAML file
        with open(self.data_save_dir + f'experiment_{self.vision_experiment}_results.yaml', 'w') as file:
            yaml.dump(data, file)

        self.get_logger().info("YAML file saved successfully.")   

    def start(self): 
        ### GET VISION EXPERIMENT PARAMETER
        self.get_logger().info(f'Getting vision experiment parameter')
        self.get_vision_experiment_param()

        # ### STAGE 0 - INITIALIZE ARM POSITION AND LOCATE APPLES
        # # Ensure arm is in home position
        # self.get_logger().info(f'Moving arm to home')
        # self.go_to_home()

        # Request apple location prediction
        self.get_logger().info(f'Sending request to predict apple centerpoint locations in scene.')
        apple_poses = self.start_apple_prediction()
        self.apples_found = len(apple_poses.poses)
        self.apple_coords = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in apple_poses.poses])
        self.get_logger().info(f'# of apples found: {len(apple_poses.poses)}')

        ### STAGE 1 - TEMPLATING
        self.get_logger().info(f'Starting templating method')
        # Place trellis template
        self.get_logger().info(f'Placing trellis template at x={np.round(self.trellis_base_position[0], 3)}, y={np.round(self.trellis_base_position[1], 3)}, z={np.round(self.trellis_base_position[2], 3)} in the planning scene')
        self.update_trellis_template_pos(self.trellis_base_position)
        self.get_side_branch_locations()

        # Set a gripper approach offset to each apple location
        apple_coords = copy.deepcopy(self.apple_coords)
        apple_coords[:, 1] -= (self.apple_approach_offset) # apple radius offset

        # Move arm to apple position
        for i, apple in enumerate(apple_coords):
            self.get_logger().info(f'Moving arm to apple ID: {i}')
            result = self.send_pose_goal(apple)
            if result.result:
                self.get_logger().info(f'Apple ID: {i} reached')
                self.apples_reached_templating += 1

                # self.get_logger().info(f'Moving arm to home')
                # trajectory = result.reverse_traj
                # self.send_trajectory(trajectory)
            else:
                self.get_logger().warn(f'Apple ID: {i} not reachable')
                self.unreached_idx_templating.append(i)
        
        self.get_logger().info(f'Number of apples reached via templating: {self.apples_reached_templating}')

        # Remove template
        self.get_logger().info(f'Removing template from the planning scene')
        self.remove_tree_from_scene()

        ### STAGE 2 - VOXELIZATION
        self.get_logger().info(f'Starting voxelization method')
        # Request voxel data from point cloud
        self.get_logger().info(f'Sending request to extract voxels from point cloud.')
        voxel_data = self.call_voxel_grid_service()
        voxel_centers = voxel_data.voxel_centers
        voxel_colors = voxel_data.voxel_colors
        voxel_centers = np.array([[point.x, point.y, point.z] for point in voxel_centers])
        voxel_colors = np.array([[color.r, color.g, color.b, color.a] for color in voxel_colors])
        self.get_logger().info(f"# of voxels generated: {len(voxel_centers)}")

        # Find all neighboring point within a sphere around the apple locations
        neighbor_coords, neighbor_idx = self.get_neighbors(voxel_centers, self.apple_coords, radius=self.voxel_neighbor_radii)
        self.get_logger().info(f'# of voxels to remove based on apple locations: {len(neighbor_idx)}')

        voxel_centers_apple_masked = [voxel_centers[i] for i in range(len(voxel_centers)) if i not in neighbor_idx]
        voxel_colors_apple_masked = [voxel_colors[i] for i in range(len(voxel_colors)) if i not in neighbor_idx]
        voxel_centers_removed = [voxel_centers[i] for i in neighbor_idx]
        voxel_colors_removed = [voxel_colors[i] for i in neighbor_idx]
        self.get_logger().info(f'# of voxels after apple location removal: {len(voxel_centers_apple_masked)}')
        self.get_logger().info(f'# of voxels removed: {len(voxel_centers_removed)}')

        self.add_voxels(voxel_centers_apple_masked, voxel_colors_apple_masked)
        self.add_voxels(voxel_centers_removed)

        # Add voxels as moveit2 collision objects - first convert back to pose message
        voxel_centers_apple_masked_poses = [Point(x=coord[0], y=coord[1], z=coord[2]) for coord in voxel_centers_apple_masked]
        self.get_logger().info(f"Publishing {len(voxel_centers_apple_masked_poses)} collision objects to planning scene")
        self.add_collision_objects(voxel_centers_apple_masked_poses)

        # Move arm to apple position
        for i, apple in enumerate(apple_coords):
            self.get_logger().info(f'Moving arm to apple ID: {i}')
            result = self.send_pose_goal(apple)
            if result.result:
                self.get_logger().info(f'Apple ID: {i} reached')
                self.apples_reached_voxelization += 1

                # self.get_logger().info(f'Moving arm to home')
                # trajectory = result.reverse_traj
                # self.send_trajectory(trajectory)
            else:
                self.get_logger().warn(f'Apple ID: {i} not reachable')
                self.unreached_idx_voxelization.append(i)

        self.get_logger().info(f'Number of apples reached via voxelization: {self.apples_reached_voxelization}')

        ### STAGE 3 - SAVE DATA
        self.save_metadata()

        ### COMPLETE
        self.get_logger().info(f'Trial complete!')


def main(args=None):
    rclpy.init(args=args)
    node = OrchardTemplating()
    node.start()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
