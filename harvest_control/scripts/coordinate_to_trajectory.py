#!/usr/bin/env python3

import numpy as np
import os
import json

import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from std_srvs.srv import Empty, Trigger
from harvest_interfaces.srv import CoordinateToTrajectory, VoxelMask, ApplePrediction
from harvest_interfaces.action import SendTrajectory


class CoordinateToTrajectoryService(Node):
    def __init__(self):
        super().__init__('trajectory_query_node')
        
        # Create the service
        self.coord_to_traj_srv = self.create_service(CoordinateToTrajectory, 'coordinate_to_trajectory', self.coord_to_traj_callback)
        self.return_home_traj_srv = self.create_service(Trigger, 'return_home_trajectory', self.return_home_traj_callback)
        self.voxel_mask = self.create_service(VoxelMask, 'voxel_mask', self.voxel_mask_callback)

        # Create the action client
        self.send_traj_client = ActionClient(self, SendTrajectory, 'send_arm_trajectory')

        # Create the service client
        self.apple_pred_client = self.create_client(ApplePrediction, 'sort_apple_predictions')

        # Create a publisher for MarkerArray
        self.voxel_marker_publisher = self.create_publisher(MarkerArray, 'voxel_markers', 10)
        self.apple_marker_publisher = self.create_publisher(MarkerArray, 'apple_markers_OLD', 10)

        # Set the timer to publish markers periodically
        self.voxel_timer = self.create_timer(1.0, self.publish_voxel_markers)
        self.apple_timer = self.create_timer(1.0, self.publish_apple_markers)

        # Get the package share directory
        package_share_directory = get_package_share_directory('harvest_control')

        # Load data
        tree_wire_filter_file = os.path.join(package_share_directory, 'resource', 'tree_wire_mask.json')
        self.load_tree_wire_filter_ranges(tree_wire_filter_file)
        self.voxel_data = np.loadtxt(os.path.join(package_share_directory, 'resource', 'reachable_voxel_centers_fine.csv'))
        self.trajectories = np.load(os.path.join(package_share_directory, 'resource', 'reachable_paths_fine.npy'))
        self.ik_data = np.loadtxt(os.path.join(package_share_directory, 'resource', 'voxel_ik_data_fine.csv'), delimiter=',', skiprows=1)

        # Extract the data
        self.trajectories_orig = np.copy(self.trajectories)
        self.voxel_centers = self.voxel_data[:, :3]
        self.voxel_centers_orig = np.copy(self.voxel_centers)
        self.target_configurations = self.ik_data[:, :6]

        # Parameters
        # self.num_configs_in_traj = len(self.trajectories)
        self.num_configs_in_traj = 100
        self.y_peck_distance = 0.3
        self.traj_msg = None

        self.declare_parameter("sim", False)
        self.sim = self.get_parameter("sim").get_parameter_value().bool_value
        self.get_logger().info(f'Simulation mode: {self.sim}')
        self.declare_parameter("voxel_distance_tol", 0.5)
        self.voxel_distance_tol = self.get_parameter("voxel_distance_tol").get_parameter_value().double_value

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

    def publish_apple_markers(self):
        apple_loc = [[-0.25, 0.6, 0.5], [0.0, 0.5, 0.6], [0.15, 0.5, 0.7], [0.15, 0.5, 0.8], [0.0, 0.4, 0.9], [-0.15, 0.4, 0.9]]
        
        marker_array = MarkerArray()
        for i, apple in enumerate(apple_loc):
            marker = Marker()
            marker.header.frame_id = 'base_link'  # Change this to your fixed frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'apple'
            marker.id = i
            marker.action = Marker.ADD
            
            # Create and set the Point object
            point = Point()
            point.x = apple[0]
            point.y = apple[1]
            point.z = apple[2]
            marker.pose.position = point

            marker.type = Marker.SPHERE
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1  # Radius of the sphere
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0  # Red color 
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Fully opaque

            marker_array.markers.append(marker)

        self.apple_marker_publisher.publish(marker_array)

    def publish_voxel_markers(self):
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

        self.voxel_marker_publisher.publish(marker_array)

    def coord_to_traj_callback(self, request, response):
        # Extract the requested coordinate
        point_msg = request.coordinate
        x = point_msg.x
        y = point_msg.y
        z = point_msg.z
        point = np.array([x, y, z])

        # Find the trajectory to a voxel that the target point is closest to
        traj, distance_to_voxel, _ = self.trajectory_to_closest_voxel(point)
        self.get_logger().info(f'Distance to nearest voxel: {distance_to_voxel}')

        if distance_to_voxel > self.voxel_distance_tol:
            self.get_logger().error(f"Distance to nearest voxel exceeded the distance threshold: {self.voxel_distance_tol}m. Cancelling trajectory execution...")
            response.success = False
        
        else:
            # Package the trajectory as a message
            self.package_traj_msg(traj)

            # Assign the response
            response.success = True
            response.waypoints = self.traj_msg

        if response.success:
            if self.sim:
                # Send trajectory message to MoveIt
                self.trigger_arm_mover(self.traj_msg)

            self.current_joint_config = traj[-1]

            # Stage the reverse the traj
            self.package_traj_msg(traj, reverse_traj=True)

        else:
            self.get_logger().warn('Failed to trigger arm mover. Not moving the arm...')

        return response
    
    def return_home_traj_callback(self, request, response):
        if self.traj_msg == None:
            self.get_logger().warn('No current return trajectory. Not moving the arm')
            response.success = False
        else:
            self.get_logger().info('Returning home by reversing previous trajectory')
            response.success = True
            self.trigger_arm_mover(self.traj_msg)

            # Reset the traj
            self.traj_msg = None

        return response
    
    def package_traj_msg(self, traj, reverse_traj=False):
        if reverse_traj:
            # Reverse the traj
            traj = np.flip(traj, axis=0)

        # Convert traj Float32MultiArray format
        float32_array = Float32MultiArray()

        # Set the layout
        float32_array.layout.dim.append(MultiArrayDimension())
        float32_array.layout.dim[0].label = "rows"
        float32_array.layout.dim[0].size = traj.shape[0]
        float32_array.layout.dim[0].stride = traj.size

        float32_array.layout.dim.append(MultiArrayDimension())
        float32_array.layout.dim[1].label = "columns"
        float32_array.layout.dim[1].size = traj.shape[1]
        float32_array.layout.dim[1].stride = traj.shape[1]

        # Flatten the traj and assign it to the data field
        float32_array.data = traj.flatten().tolist()

        # Save the traj message
        self.traj_msg = float32_array

    def trigger_arm_mover(self, trajectory):
        # Wait for the action server to be available
        self.send_traj_client.wait_for_server()

        goal_msg = SendTrajectory.Goal()
        goal_msg.waypoints = trajectory  # Pass the entire Float32MultiArray message

         # Send the goal to the action server
        self._send_goal_future = self.send_traj_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback  # Define feedback handling
        )
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Trajectory execution succeeded')
        else:
            self.get_logger().info('Trajectory execution failed')

    def feedback_callback(self, feedback):
        # Handle the feedback (e.g., print the progress)
        self.get_logger().info(f"Progress: {feedback.feedback.progress}")

    def handle_trajectory_response(self, future):
        try:
            response = future.result()
            if response is not None:
                self.get_logger().info('Send trajectory service call succeeded')
            else:
                self.get_logger().error('Send trajectory service call failed')
        except Exception as e:
            self.get_logger().error(f'Exception occurred: {e}')

    def voxel_mask_callback(self, request, response):
        tree_pos = request.tree_pos
        self.get_logger().info(f'Received voxel mask request: {tree_pos}')
        
        # Reset voxel centers and trajectories to the originals
        voxel_centers_copy = np.copy(self.voxel_centers_orig)
        self.trajectories = np.copy(self.trajectories_orig)

        if tree_pos not in range(len(self.x_filter_ranges)):
            self.get_logger().warn('Voxel mask positions out of range')
            self.get_logger().info('Resetting to unfiltered voxels...')
            self.voxel_centers = voxel_centers_copy

        # No tree in range (just wires)
        elif tree_pos == 0:
            self.get_logger().info('Voxel mask set to just wire locations')

            # Get a mask of the z coords
            combined_z_mask = self.mask_z(voxel_centers_copy)

            # Apply the combined mask to filter out coordinates and trajectories
            self.voxel_centers = voxel_centers_copy[combined_z_mask]
            self.trajectories = self.trajectories[:, :, combined_z_mask]

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

            # Apply the combined mask to filter out coordinates and trajectories
            self.voxel_centers = voxel_centers_copy[combined_mask]
            self.trajectories = self.trajectories[:, :, combined_mask]

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

    def trajectory_to_closest_voxel(self, target_point):
        """ Find the trajectory to a voxel that the target point is closest to 

        Args:
            target_point (float list): target 3D coordinate

        Returns:
            trajectory: the trajectory to the voxel the target point is closest to
            distance_error: error between target point and closest voxel center
        """
        # Calculate distances
        distances = np.linalg.norm(self.voxel_centers - target_point, axis=1)
        
        # Find the index of the closest voxel
        closest_voxel_index = np.argmin(distances)

        distance_error = distances[closest_voxel_index]

        # Get the associated trajectory to closest voxel
        return self.trajectories[:, :, closest_voxel_index], distance_error, closest_voxel_index

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