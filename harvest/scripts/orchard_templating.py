#!/usr/bin/env python3

# ROS
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point
# Interfaces
from harvest_interfaces.srv import ApplePrediction, VoxelGrid

# Python 
import numpy as np

class OrchardTemplating(Node):

    def __init__(self):
        super().__init__("orchard_templating_node")
        m_callback_group = MutuallyExclusiveCallbackGroup()

        self.voxel_size = 0.05

        # Publishers
        self.voxel_collision_pub = self.create_publisher(CollisionObject, "/collision_object", 10)

        # Services
        self.start_apple_prediction_client = self.create_client(ApplePrediction, "/apple_prediction_presaved_images", callback_group=m_callback_group)
        while not self.start_apple_prediction_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Start visual servo service not available, waiting...")    

        self.voxel_client = self.create_client(VoxelGrid, "voxel_grid")    
        while not self.voxel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Start voxel grid service not available, waiting...")    
        
    def start_apple_prediction(self):
        # Starts servo node
        self.request = ApplePrediction.Request()
        self.future = self.start_apple_prediction_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future) 
        return self.future.result().apple_poses 
    
    def call_voxel_grid_service(self):
        request = VoxelGrid.Request()
        request.voxel_size = self.voxel_size  # TODO: Set voxel size as a ros2 parameter

        self.future = self.voxel_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future) 
        return self.future.result().voxel_centers
    
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

    def start(self): 
        # Request apple location prediction
        self.get_logger().info(f'Sending request to predict apple centerpoint locations in scene.')
        apple_poses = self.start_apple_prediction()
        apple_coords = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in apple_poses.poses])
        self.get_logger().info(f'# of apples found: {len(apple_poses.poses)}')

        # Request voxel data from point cloud
        self.get_logger().info(f'Sending request to extract voxels from point cloud.')
        voxel_centers = self.call_voxel_grid_service()
        voxel_centers = np.array([[point.x, point.y, point.z] for point in voxel_centers])
        self.get_logger().info(f"# of voxels generated: {len(voxel_centers)}")

        # Find all neighboring point within a sphere around the apple locations
        neighbor_coords, neighbor_idx = self.get_neighbors(voxel_centers, apple_coords, radius=0.1)
        self.get_logger().info(f'# of voxels to remove based on apple locations: {len(neighbor_idx)}')

        voxel_centers_apple_masked = [idx for i, idx in enumerate(voxel_centers) if i not in neighbor_idx]
        self.get_logger().info(f'# of voxels after apple location removal: {len(voxel_centers_apple_masked)}')

        # Add voxels as moveit2 collision objects - first convert back to pose message
        voxel_centers_apple_masked_poses = [Point(x=coord[0], y=coord[1], z=coord[2]) for coord in voxel_centers_apple_masked]
        self.get_logger().info(f"Publishing {len(voxel_centers_apple_masked_poses)} collision objects to planning scene")
        self.add_collision_objects(voxel_centers_apple_masked_poses)


def main(args=None):
    rclpy.init(args=args)
    node = OrchardTemplating()
    node.start()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
