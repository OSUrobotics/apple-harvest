#!/usr/bin/env python3

# ROS
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
# Interfaces
from harvest_interfaces.srv import ApplePrediction, VoxelGrid

# Python 
import numpy as np

class OrchardTemplating(Node):

    def __init__(self):
        super().__init__("orchard_templating_node")
        m_callback_group = MutuallyExclusiveCallbackGroup()

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
        request.voxel_size = 0.1  # Adjust voxel size

        self.future = self.voxel_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future) 
        return self.future.result().voxel_centers
    
    def add_collision_objects(self, voxel_centers):
        for i, voxel_center in enumerate(voxel_centers):
            collision_object = CollisionObject()
            collision_object.id = f"voxel_{i}"
            collision_object.header.frame_id = "world"

            # Define the shape and size
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [0.1, 0.1, 0.1]  # Voxel size

            # Define the pose
            box_pose = Pose()
            box_pose.position = voxel_center
            box_pose.orientation.w = 1.0

            collision_object.primitives.append(primitive)
            collision_object.primitive_poses.append(box_pose)
            collision_object.operation = CollisionObject.ADD

            self.voxel_collision_pub.publish(collision_object)
        self.get_logger().info(f"Published {len(voxel_centers)} collision objects to planning scene")

    def start(self): 
        # Stage 2: Request apple location prediction
        self.get_logger().info(f'Sending request to predict apple centerpoint locations in scene.')
        apple_poses = self.start_apple_prediction()
        point_cloud_voxels = self.call_voxel_grid_service()

        # Loop over apple locations
        # for i in apple_poses.poses:
        self.get_logger().info(f'# Apples found: {len(apple_poses.poses)}')
        self.get_logger().info(f"# Voxels found: {len(point_cloud_voxels)}")

        self.add_collision_objects(point_cloud_voxels)


def main(args=None):
    rclpy.init(args=args)
    node = OrchardTemplating()
    node.start()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
