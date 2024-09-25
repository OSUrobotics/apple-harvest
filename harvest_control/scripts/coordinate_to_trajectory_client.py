#!/usr/bin/env python3

import numpy as np
import time
import networkx as nx
from networkx.algorithms import approximation

import rclpy
import rclpy.logging
from rclpy.node import Node

from geometry_msgs.msg import Point, Pose, PoseArray
from std_srvs.srv import Empty
from harvest_interfaces.srv import CoordinateToTrajectory, ApplePrediction, TrajectoryBetweenPoints


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
        
        self.traj_between_points_client = self.create_client(TrajectoryBetweenPoints, 'trajectory_between_points')
        while not self.traj_between_points_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for trajectory_between_points to be available...')

        # Third service client (triggered after CoordinateToTrajectory)
        self.return_home_traj_client = self.create_client(Empty, 'return_home_trajectory')
        while not self.return_home_traj_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for return_home_trajectory to be available...')

        self.end_effector_start_position = np.array([-0.13400002, 0.29166607, 0.84070509])

        self.time_between_arm_mover_call = 5 # seconds
        
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
    
    def tsp_coords(self, coordinates):
        # Convert PoseArray into an array of coordinates
        # coordinates = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in pose_array.poses])
        # coordinates = np.array([[-0.25, 0.6, 0.5], [0, 0.5, 0.6], [0.15, 0.5, 0.7], [0.15, 0.5, 0.8], [0, 0.4, 0.9], [-0.15, 0.4, 0.9]])

        # Create a graph and add nodes
        G = nx.complete_graph(len(coordinates))

        # Assign Euclidean distances as edge weights
        for i in range(len(coordinates)):
            for j in range(i + 1, len(coordinates)):
                dist = np.linalg.norm(coordinates[i] - coordinates[j])
                G[i][j]['weight'] = dist

        # Solve the TSP
        tsp_solution = approximation.traveling_salesman_problem(G, weight='weight')

        # Sorted coordinates
        sorted_coords = [coordinates[idx] for idx in tsp_solution]

        # # Sort the PoseArray based on the sorted indices
        # sorted_pose_array = PoseArray()
        # sorted_pose_array.header = pose_array.header  # Copy the header
        # sorted_pose_array.poses = [pose_array.poses[i] for i in tsp_solution]
        
        return np.array(sorted_coords)

    def call_apple_prediction_service(self):
        # request = ApplePrediction.Request()

        # # Add request details here
        # future = self.apple_pred_client.call_async(request)
        # future.add_done_callback(self.handle_apple_prediction_response)

        # Create a dummy PoseArray
        # dummy_apple_poses = self.create_dummy_pose_array()
        apple_loc = np.array([[-0.25, 0.6, 0.5], [0, 0.5, 0.6], [0.15, 0.5, 0.7], [0.15, 0.5, 0.8], [0, 0.4, 0.9], [-0.15, 0.4, 0.9]])
        # sorted_apple_poses = self.sort_nearest_coords(self.end_effector_start_position, dummy_apple_poses)
        sorted_apple_poses = self.tsp_coords(apple_loc)

        # Call the coordinate to trajectory service using the dummy data
        self.call_coord_to_traj(sorted_apple_poses)

    def create_dummy_pose_array(self):
        pose_array = PoseArray()

        # Add the first Pose with the desired position (0.2, 0.2, 0.2)
        first_pose = Pose()
        first_pose.position.x = 0.0
        first_pose.position.y = 0.5
        first_pose.position.z = 0.5
        # You can also set orientation if needed, e.g., no rotation
        first_pose.orientation.x = 0.0
        first_pose.orientation.y = 0.0
        first_pose.orientation.z = 0.0
        first_pose.orientation.w = 1.0

        # Append the first pose to the PoseArray
        pose_array.poses.append(first_pose)

        # Now add subsequent poses with incrementing positions
        increment = 0.4  # Define the increment step for each subsequent pose
        num_poses = 1    # Define how many more poses to add

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
        for i in range(len(apple_poses)):
            if i == 0:
                x = apple_poses[i, 0]
                y = apple_poses[i, 1]
                z = apple_poses[i, 2]

                # Create a Point message
                coord = Point()
                coord.x = x
                coord.y = y
                coord.z = z

                self.get_logger().info(f"Calling trajectory service for coordinate: [x: {np.round(x, 2)}, y: {np.round(y, 2)}, z: {np.round(z, 2)}]")

                request = CoordinateToTrajectory.Request()
                request.coordinate = coord        

                future = self.coord_to_traj_client.call_async(request)
                future.add_done_callback(self.handle_coord_to_traj_response)

                # Add a slight delay
                time.sleep(self.time_between_arm_mover_call)

            elif i == len(apple_poses) - 1:
                self.get_logger().info('Calling return home trajectory')
                self.call_return_home_traj()
                time.sleep(self.time_between_arm_mover_call)

            else:
                x_start = apple_poses[i - 1, 0]
                y_start = apple_poses[i - 1, 1]
                z_start = apple_poses[i - 1, 2]
                
                # Create a Point message
                start_coord = Point()
                start_coord.x = x_start     
                start_coord.y = y_start
                start_coord.z = z_start

                x_end = apple_poses[i, 0]
                y_end = apple_poses[i, 1]
                z_end = apple_poses[i, 2]

                # Create a Point message
                end_coord = Point()
                end_coord.x = x_end
                end_coord.y = y_end
                end_coord.z = z_end

                self.get_logger().info(f"Calling trajectory service for coordinate: [x: {np.round(x_end, 2)}, y: {np.round(y_end, 2)}, z: {np.round(z_end, 2)}]")

                request = TrajectoryBetweenPoints.Request()
                request.start_coordinate = start_coord
                request.end_coordinate = end_coord 

                future = self.traj_between_points_client.call_async(request)
                future.add_done_callback(self.handle_traj_between_points_response)

                # Add a slight delay
                time.sleep(self.time_between_arm_mover_call)


    def handle_coord_to_traj_response(self, future):
        try:
            response = future.result()
            self.get_logger().info('Coordinate to trajectory service completed successfully')
        except Exception as e:
            self.get_logger().error(f'Coordinate to trajectory service call failed: {str(e)}')

    def handle_traj_between_points_response(self, future):
        try:
            response = future.result()
            self.get_logger().info('Trajectory between points service completed successfully')
        except Exception as e:
            self.get_logger().error(f'Trajectory between points  service call failed: {str(e)}')

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