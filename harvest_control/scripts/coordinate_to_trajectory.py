#!/usr/bin/env python3

import numpy as np
import os
import json

import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from std_srvs.srv import Empty
from harvest_interfaces.srv import CoordinateToTrajectory, SendTrajectory, VoxelMask, ApplePrediction


class CoordinateToTrajectoryService(Node):
    def __init__(self):
        super().__init__('voxel_search_service')
        
        # Create the service
        self.coord_to_traj_srv = self.create_service(CoordinateToTrajectory, 'coordinate_to_trajectory', self.coord_to_traj_callback)
        self.return_home_traj_srv = self.create_service(Empty, 'return_home_trajectory', self.return_home_traj_callback)
        self.voxel_mask = self.create_service(VoxelMask, 'voxel_mask', self.voxel_mask_callback)

        # Create the service client
        self.client = self.create_client(SendTrajectory, 'execute_arm_trajectory')
        self.return_home_client = self.create_client(Empty, 'return_home_trajectory')
        self.apple_pred_client = self.create_client(ApplePrediction, 'sort_apple_predictions')

        # Create a publisher for MarkerArray
        self.marker_publisher = self.create_publisher(MarkerArray, 'voxel_markers', 10)

        # Set the timer to publish markers periodically
        self.timer = self.create_timer(1.0, self.publish_markers)

        # Get the package share directory
        package_share_directory = get_package_share_directory('harvest_control')

        # Retrieve precomputed tree locations
        tree_wire_filter_file = os.path.join(package_share_directory, 'resource', 'tree_wire_mask.json')
        self.load_tree_wire_filter_ranges(tree_wire_filter_file)

        self.reversed_path = None

        # Define maximum distance tolerance between target location and precomputed voxel
        self.distance_tol = 0.5

        # Load voxel data
        # y_trans = 0
        self.voxel_data = np.loadtxt(os.path.join(package_share_directory, 'resource', 'reachable_voxel_centers.csv'))
        self.paths = np.load(os.path.join(package_share_directory, 'resource', 'reachable_paths.npy'))
        self.paths_orig = np.copy(self.paths)

        self.voxel_centers = self.voxel_data[:, :3]
        self.voxel_indices = self.voxel_data[:, 3:]
        self.voxel_centers_orig = np.copy(self.voxel_centers)

        self.get_logger().info('Coordinate to trajectory service up and running')

    def load_tree_wire_filter_ranges(self, filename):
        # Load JSON data from the file
        with open(filename, 'r') as file:
            data = json.load(file)

        # Extract the min and max pairs from the data
        x_filter_ranges = np.array([(entry['min'], entry['max']) for entry in data['tree_x_coordinate_ranges']])

        # Add an empty list x-ranges to act as "no filter needed" or "no tree"
        self.x_filter_ranges = np.insert(x_filter_ranges, 0, [None, None]).reshape(6, 2)
        
        self.z_filter_ranges = np.array([(entry['min'], entry['max']) for entry in data['z_wire_heights']])

    def publish_markers(self):
        apple_loc = [[-0.25, 0.6, 0.5], [0, 0.5, 0.6], [0.15, 0.5, 0.7], [0.15, 0.5, 0.8], [0, 0.4, 0.9], [-0.15, 0.4, 0.9]]
        # apple_loc = [[0, 0.5, 0.8]]

        apple_voxel_idxs = []
        for i, apple_pos in enumerate(apple_loc):
            _, _, closest_voxel_idx = self.path_to_closest_voxel(apple_pos)
            apple_voxel_idxs.append(closest_voxel_idx)

        marker_array = MarkerArray()

        for i, center in enumerate(self.voxel_centers):
            marker = Marker()
            marker.header.frame_id = 'base_link'  # Change this to your fixed frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'voxel'
            marker.id = i
            marker.action = Marker.ADD
            
            # Create and set the Point object
            point = Point()
            point.x = center[0]
            point.y = center[1]
            point.z = center[2]
            marker.pose.position = point
            
            if i in apple_voxel_idxs:
                marker.type = Marker.SPHERE
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.1  # Radius of the sphere
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.r = 1.0  # Red color 
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0  # Fully opaque

            else:
                marker.type = Marker.CUBE
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.09  # Size of the cube
                marker.scale.y = 0.09
                marker.scale.z = 0.09
                marker.color.r = 0.0 
                marker.color.g = 0.0
                marker.color.b = 1.0  # Blue color
                marker.color.a = 0.6  # Fully opaque

            marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)

    def coord_to_traj_callback(self, request, response):
        # Extract the requested coordinate
        point_msg = request.coordinate
        x = point_msg.x
        y = point_msg.y
        z = point_msg.z
        point = np.array([x, y, z])

        # Find the closest path associated with the target point
        path, distance_to_voxel, closest_voxel_index = self.path_to_closest_voxel(point)
        self.get_logger().info(f'Distance to nearest voxel: {distance_to_voxel}')

        if distance_to_voxel > self.distance_tol:
            self.get_logger().error("Distance to nearest voxel exceeded the distance threshold. Cancelling trajectory execution...")
            response.success = False
        
        else:
            # Package path as a float multiarray
            float32_array = Float32MultiArray()

            # Set the layout (optional, but helps with multi-dimensional arrays)
            float32_array.layout.dim.append(MultiArrayDimension())
            float32_array.layout.dim[0].label = "rows"
            float32_array.layout.dim[0].size = path.shape[0]
            float32_array.layout.dim[0].stride = path.size

            float32_array.layout.dim.append(MultiArrayDimension())
            float32_array.layout.dim[1].label = "columns"
            float32_array.layout.dim[1].size = path.shape[1]
            float32_array.layout.dim[1].stride = path.shape[1]

            # Flatten the NumPy array and assign it to the data field
            float32_array.data = path.flatten().tolist()

            # Assign to the response
            response.success = True

        if response.success:
            self.waypoint_msg = float32_array

            # Reverse the path and save it
            self.reverse_path(path)

            # Send trajectory to MoveIt
            self.trigger_arm_mover(self.waypoint_msg)

        return response
    
    def return_home_traj_callback(self, request, response):
        if self.reversed_path == None:
            self.get_logger().warn('No current return path. Not moving the arm')
        else:
            self.get_logger().info('Returning home by reversing previous trajectory')
            self.trigger_arm_mover(self.reversed_path)

            # Reset the reversed path
            self.reversed_path = None

        return response
    
    def reverse_path(self, path):
        # Reverse the path
        reversed_path = np.flip(path, axis=0)

        # Convert the reversed path to Float32MultiArray format
        float32_array = Float32MultiArray()

        # Set the layout (same as in coord_to_traj_callback)
        float32_array.layout.dim.append(MultiArrayDimension())
        float32_array.layout.dim[0].label = "rows"
        float32_array.layout.dim[0].size = reversed_path.shape[0]
        float32_array.layout.dim[0].stride = reversed_path.size

        float32_array.layout.dim.append(MultiArrayDimension())
        float32_array.layout.dim[1].label = "columns"
        float32_array.layout.dim[1].size = reversed_path.shape[1]
        float32_array.layout.dim[1].stride = reversed_path.shape[1]

        # Flatten the reversed path and assign it to the data field
        float32_array.data = reversed_path.flatten().tolist()

        # Save the reversed path as self.reversed_path
        self.reversed_path = float32_array

    def trigger_arm_mover(self, trajectory):
        if not self.client.service_is_ready():
            self.get_logger().info('Waiting for execute_arm_trajectory service to be available...')
            self.client.wait_for_service()

        request = SendTrajectory.Request()
        request.waypoints = trajectory  # Pass the entire Float32MultiArray message

        # Use async call
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_trajectory_response)

    def handle_trajectory_response(self, future):
        try:
            response = future.result()
            if response is not None:
                self.get_logger().info('Waypoint path service call succeeded')
            else:
                self.get_logger().error('Waypoint path service call failed')
        except Exception as e:
            self.get_logger().error(f'Exception occurred: {e}')

        # Reset state after the service call
        self.waypoint_msg = None

    def voxel_mask_callback(self, request, response):
        tree_pos = request.tree_pos
        self.get_logger().info(f'Received voxel mask request: {tree_pos}')
        
        # Reset voxel centers and paths to the originals
        voxel_centers_copy = np.copy(self.voxel_centers_orig)
        self.paths = np.copy(self.paths_orig)

        if tree_pos not in range(len(self.x_filter_ranges)):
            self.get_logger().warn('Voxel mask positions out of range')
            self.get_logger().info('Resetting to unfiltered voxels...')
            self.voxel_centers = voxel_centers_copy

        # No tree in range (just wires)
        elif tree_pos == 0:
            self.get_logger().info('Voxel mask set to just wire locations')

            # Get a mask of the z coords
            combined_z_mask = self.mask_z(voxel_centers_copy)

            # Apply the combined mask to filter out coordinates and paths
            self.voxel_centers = voxel_centers_copy[combined_z_mask]
            self.paths = self.paths[:, :, combined_z_mask]

        # Set the mask of the tree pos and wires
        elif tree_pos in range(len(self.x_filter_ranges)):
            self.get_logger().info(f'Voxel mask set to tree position {tree_pos} and wire locations')

            # Retrieve x-ranges
            x_min, x_max = self.x_filter_ranges[tree_pos]

            # Create a mask where x-values are *not* between the x-ranges and z-ranges
            x_mask = (voxel_centers_copy[:, 0] < x_min) | (voxel_centers_copy[:, 0] > x_max)

            # Get a mask of the z coords
            combined_z_mask = self.mask_z(voxel_centers_copy)

            # Combine the x mask with the z mask
            combined_mask = x_mask & combined_z_mask

            # Apply the combined mask to filter out coordinates and paths
            self.voxel_centers = voxel_centers_copy[combined_mask]
            self.paths = self.paths[:, :, combined_mask]

        self.get_logger().info(f'Length of voxel centers list: {len(self.voxel_centers)}')

        response.success = True 

        return response
    
    def mask_z(self, voxel_coords):
        # Initialize combined_z_mask to all True
        combined_z_mask = np.ones(voxel_coords.shape[0], dtype=bool)

        # Loop through each z range and update the combined_z_mask
        for z_min, z_max in self.z_filter_ranges:
            z_mask = (voxel_coords[:, 2] < z_min) | (voxel_coords[:, 2] > z_max)
            combined_z_mask &= z_mask
        
        return combined_z_mask

    def path_to_closest_voxel(self, target_point):
        """ Find the path to a voxel that the target point is closest to 

        Args:
            target_point (float list): target 3D coordinate

        Returns:
            path: the path to the voxel the target point is closest to
            distance_error: error between target point and closest voxel center
        """
        # Calculate distances
        distances = np.linalg.norm(self.voxel_centers - target_point, axis=1)
        
        # Find the index of the closest voxel
        closest_voxel_index = np.argmin(distances)

        distance_error = distances[closest_voxel_index]

        # Get the associated path to closest voxel
        return self.paths[:, :, closest_voxel_index], distance_error, closest_voxel_index
        
    def sort_nearest_coords(self, current_position, coordinates):
        # Calculate distances from current position to each apple location
        distances = np.linalg.norm(coordinates - current_position, axis=1)

        # Sort coordinates by distance
        sorted_indices = np.argsort(distances)
        sorted_coordinates = coordinates[sorted_indices]

        return sorted_coordinates

def main():
    rclpy.init()

    coord_to_traj = CoordinateToTrajectoryService()

    # Use a SingleThreadedExecutor to handle the callbacks
    executor = SingleThreadedExecutor()
    executor.add_node(coord_to_traj)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        coord_to_traj.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()