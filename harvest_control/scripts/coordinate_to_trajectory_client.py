#!/usr/bin/env python3

import numpy as np
import time

import rclpy
import rclpy.logging
from rclpy.node import Node

from geometry_msgs.msg import Point, Pose, PoseArray
from std_srvs.srv import Empty
from harvest_interfaces.srv import CoordinateToTrajectory, ApplePrediction


class ApplePredictionListener(Node):
    def __init__(self):
        super().__init__('apple_prediction_listener')
        
        # First service client (request ApplePrediction)
        self.apple_pred_client = self.create_client(ApplePrediction, 'apple_prediction')
        # while not self.apple_pred_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for apple_prediction to be available...')
        
        # Second service client (triggered after ApplePrediction)
        self.coord_to_traj_client = self.create_client(CoordinateToTrajectory, 'coordinate_to_trajectory')
        while not self.coord_to_traj_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for coordinate_to_trajectory to be available...')

        # Third service client (triggered after CoordinateToTrajectory)
        self.return_home_traj_client = self.create_client(Empty, 'return_home_trajectory')
        while not self.return_home_traj_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for return_home_trajectory to be available...')

        self.end_effector_start_position = np.array([-0.13400002, 0.29166607, 0.84070509])

        self.time_between_arm_mover_call = 4 # seconds
        
        # Call the first service
        self.call_apple_prediction_service()

    def sort_nearest_coords(self, current_position, pose_array):
        # Convert PoseArray into an array of coordinates
        coordinates = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in pose_array.poses])
        
        # Calculate distances from the current position to each apple location
        distances = np.linalg.norm(coordinates - current_position, axis=1)
        
        # Sort coordinates by distance
        sorted_indices = np.argsort(distances)
        
        # Sort the PoseArray based on the sorted indices
        sorted_pose_array = PoseArray()
        sorted_pose_array.header = pose_array.header  # Copy the header
        sorted_pose_array.poses = [pose_array.poses[i] for i in sorted_indices]
        
        return sorted_pose_array

    def call_apple_prediction_service(self):
        # request = ApplePrediction.Request()

        # # Add request details here
        # future = self.apple_pred_client.call_async(request)
        # future.add_done_callback(self.handle_apple_prediction_response)

        # Create a dummy PoseArray
        dummy_apple_poses = self.create_dummy_pose_array()
        sorted_apple_poses = self.sort_nearest_coords(self.end_effector_start_position, dummy_apple_poses)

        # Call the coordinate to trajectory service using the dummy data
        self.call_coord_to_traj(sorted_apple_poses)

    def create_dummy_pose_array(self):
        pose_array = PoseArray()

        # Add the first Pose with the desired position (0.2, 0.2, 0.2)
        first_pose = Pose()
        first_pose.position.x = 0.2
        first_pose.position.y = 0.2
        first_pose.position.z = 0.2
        # You can also set orientation if needed, e.g., no rotation
        first_pose.orientation.x = 0.0
        first_pose.orientation.y = 0.0
        first_pose.orientation.z = 0.0
        first_pose.orientation.w = 1.0

        # Append the first pose to the PoseArray
        pose_array.poses.append(first_pose)

        # Now add subsequent poses with incrementing positions
        increment = 0.1  # Define the increment step for each subsequent pose
        num_poses = 5    # Define how many more poses to add

        for i in range(1, num_poses + 1):  # Start from 1 since the first pose is already set
            pose = Pose()
            pose.position.x = first_pose.position.x + i * increment
            pose.position.y = first_pose.position.y + i * increment
            pose.position.z = first_pose.position.z + i * increment
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        return pose_array

    def handle_apple_prediction_response(self, future):
        try:
            response = future.result()
            if isinstance(response.poses, PoseArray) and len(response.poses.poses) > 0:
                self.get_logger().info('Received valid apple PoseArray. Now calling the coordinate to trajectory service...')
                
                # Now trigger the second service
                self.call_coord_to_traj(response.apple_poses)
            else:
                self.get_logger().error('Invalid apple pose response received')

        except Exception as e:
            self.get_logger().error(f'ApplePrediction service call failed: {str(e)}')

    def call_coord_to_traj(self, apple_poses):
        # apple_coords = []
        for pose in apple_poses.poses:
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z

            # Create a Point message
            coord = Point()
            coord.x = x
            coord.y = y
            coord.z = z

            # coord = [x, y, z]
            # apple_coords.append(coord)
            self.get_logger().info(f"Calling trajectory service for coordinate: [x: {np.round(x, 2)}, y: {np.round(y, 2)}, z: {np.round(z, 2)}]")
        
            request = CoordinateToTrajectory.Request()
            request.coordinate = coord        

            future = self.coord_to_traj_client.call_async(request)
            future.add_done_callback(self.handle_coord_to_traj_response)

            # Add a slight delay
            time.sleep(self.time_between_arm_mover_call)

            self.call_return_home_traj()

    def handle_coord_to_traj_response(self, future):
        try:
            response = future.result()
            self.get_logger().info('Coordinate to trajectory service completed successfully')
        except Exception as e:
            self.get_logger().error(f'Coordinate to trajectory service call failed: {str(e)}')

    def call_return_home_traj(self):
        request = Empty.Request()

        future = self.return_home_traj_client.call_async(request)
        future.add_done_callback(self.handle_return_home_traj_response)

        time.sleep(self.time_between_arm_mover_call)

    def handle_return_home_traj_response(self, future):
        try:
            response = future.result()
            self.get_logger().info('Return home trajectory service completed successfully')

        except Exception as e:
            self.get_logger().error(f'Return home trajectory service call failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ApplePredictionListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()