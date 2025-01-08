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
import tf2_ros
from geometry_msgs.msg import TransformStamped

# Interfaces
from harvest_interfaces.srv import ApplePrediction, VoxelGrid, MoveToPose, UpdateTrellisPosition, SendTrajectory

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

        # Data saving directory
        self.data_save_dir = '/home/marcus/orchard_template_ws/results_data/'

        # TODO: Set as a ros2 parameters
        self.vision_experiment = 'j'
        self.yolo_model = 'v9e.pt'
        self.apple_coords_sorted = None
        self.voxel_size = 0.01
        self.voxel_neighbor_radii = 0.08 # 2cm larger than the max apple radii threshold
        self.apple_approach_offset = 0.1 # meters
        self.apples_found = 0
        self.apples_reached_templating = 0
        self.apples_reached_voxelization = 0
        self.unreached_idx_templating = []
        self.unreached_idx_voxelization = []

        # Vision experiment parameters
        trellis_base_positions = [[-0.07, 1.105, -0.19], [-0.07, 1.05, -0.14], [-0.1, 1.0, -0.17], [-0.04, 1.21, -0.17], [-0.15, 1.02, -0.22], [-0.05, 1.05, -0.15], [-0.09, 1.16, -0.16], [-0.08, 1.11, -0.18], [-0.085, 1.03, -0.15], [-0.05, 1.12, -0.19]] # a through e (e has the most uncertainty on the bottom side branch)
        vision_experiments = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j']
        trellis_tempate_params = {key: value for key, value in zip(vision_experiments, trellis_base_positions)}
        self.trellis_base_position = trellis_tempate_params.get(self.vision_experiment)

        # Publishers
        self.voxel_collision_pub = self.create_publisher(CollisionObject, "/collision_object", 10)

        # Services
        self.start_apple_prediction_client = self.create_client(ApplePrediction, "/apple_prediction_presaved_images", callback_group=m_callback_group)
        while not self.start_apple_prediction_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Apple prediction service not available, waiting...")    

        self.voxel_client = self.create_client(VoxelGrid, "voxel_grid")    
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
        return self.future.result().voxel_centers
    
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
    
    def remove_tree_from_scene(self):
        tree_object = CollisionObject()
        tree_object.id = 'v_trellis_tree'
        tree_object.operation = CollisionObject.REMOVE
        self.voxel_collision_pub.publish(tree_object)

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
            'apple_coordinates':    self.apple_coords_sorted.tolist(),
            'voxel_size':           self.voxel_size,
            'voxel_neighbor_radii': self.voxel_neighbor_radii,
            'apple_approach_offset':self.apple_approach_offset,
            'apples_found':         self.apples_found,
            'template_base_position': self.trellis_base_position,
            'apples_reached_templating': self.apples_reached_templating,
            'apples_reached_voxelization': self.apples_reached_voxelization,
            'unreached_idx_templating': self.unreached_idx_templating,
            'unreached_idx_voxelization': self.unreached_idx_voxelization,
        }

        # Save to a YAML file
        with open(self.data_save_dir + f'experiment_{self.vision_experiment}_results.yaml', 'w') as file:
            yaml.dump(data, file)

        self.get_logger().info("YAML file saved successfully.")   

    def start(self): 
        ### STAGE 0 - INITIALIZE ARM POSITION AND LOCATE APPLES
        # Ensure arm is in home position
        self.get_logger().info(f'Moving arm to home')
        self.go_to_home()

        # Request apple location prediction
        self.get_logger().info(f'Sending request to predict apple centerpoint locations in scene.')
        apple_poses = self.start_apple_prediction()
        self.apples_found = len(apple_poses.poses)
        apple_coords = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in apple_poses.poses])
        self.get_logger().info(f'# of apples found: {len(apple_poses.poses)}')

        # Sort apple locations based on closeness to gripper_link
        self.get_logger().info(f'Sorting apple locations')
        gripper_position = self.get_gripper_position()
        self.apple_coords_sorted = self.sort_coordinates(gripper_position, apple_coords)

        ### STAGE 1 - TEMPLATING
        self.get_logger().info(f'Starting templating method')
        # Place trellis template
        self.get_logger().info(f'Placing trellis template at x={np.round(self.trellis_base_position[0], 3)}, y={np.round(self.trellis_base_position[1], 3)}, z={np.round(self.trellis_base_position[2], 3)} in the planning scene')
        self.update_trellis_template_pos(self.trellis_base_position)

        # Set a gripper approach offset to each apple location
        apple_coords[:, 1] -= self.apple_approach_offset

        # Move arm to apple position
        for i, apple in enumerate(apple_coords):
            self.get_logger().info(f'Moving arm to apple ID: {i}')
            result = self.send_pose_goal(apple)
            if result.result:
                self.get_logger().info(f'Apple ID: {i} reached')
                self.apples_reached_templating += 1

                self.get_logger().info(f'Moving arm to home')
                trajectory = result.reverse_traj
                self.send_trajectory(trajectory)
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
        voxel_centers = self.call_voxel_grid_service()
        voxel_centers = np.array([[point.x, point.y, point.z] for point in voxel_centers])
        self.get_logger().info(f"# of voxels generated: {len(voxel_centers)}")

        # Find all neighboring point within a sphere around the apple locations
        neighbor_coords, neighbor_idx = self.get_neighbors(voxel_centers, self.apple_coords_sorted, radius=self.voxel_neighbor_radii)
        self.get_logger().info(f'# of voxels to remove based on apple locations: {len(neighbor_idx)}')

        voxel_centers_apple_masked = [idx for i, idx in enumerate(voxel_centers) if i not in neighbor_idx]
        self.get_logger().info(f'# of voxels after apple location removal: {len(voxel_centers_apple_masked)}')

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

                self.get_logger().info(f'Moving arm to home')
                trajectory = result.reverse_traj
                self.send_trajectory(trajectory)
            else:
                self.get_logger().warn(f'Apple ID: {i} not reachable')
                self.unreached_idx_voxelization.append(i)

        self.get_logger().info(f'Number of apples reached via voxelizatoin: {self.apples_reached_voxelization}')

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
