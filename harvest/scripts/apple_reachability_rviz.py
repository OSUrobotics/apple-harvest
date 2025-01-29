#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import yaml
import numpy as np

class CoordinateMarkerPublisher(Node):
    def __init__(self):
        super().__init__('coordinate_marker_publisher')

        self.apple_radii = 0.1

        self.publisher = self.create_publisher(MarkerArray, 'coordinate_markers', 10)

        self.declare_parameter('vision_experiment', 'NA')
        self.declare_parameter('pub_template_reachability', False)
        self.declare_parameter('pub_voxel_reachability', False)
        self.vision_experiment = self.get_parameter('vision_experiment').get_parameter_value().string_value
        self.pub_template_reachability = self.get_parameter('pub_template_reachability').get_parameter_value().bool_value
        self.pub_voxel_reachability = self.get_parameter('pub_voxel_reachability').get_parameter_value().bool_value
        self.get_logger().info(f'Vision experiment parameter: {self.vision_experiment}')
        self.get_logger().info(f'Publishing apple template reachability: {self.pub_template_reachability}')
        self.get_logger().info(f'Publishing apple voxel reachability: {self.pub_voxel_reachability}')

        self.apple_data = self.extract_reachable_apple_data(f'v3/experiment_{self.vision_experiment}_results.yaml')

        self.apple_coordinates = self.apple_data['apple_coords']
        self.unreached_idx_templating = self.apple_data['unreached_idx_templating']
        self.unreached_idx_voxelization = self.apple_data['unreached_idx_voxelization']

        # self.reachability_template, self.reachability_voxel = self.sort_reachable_apples()

        # Timer to publish markers at regular intervals
        self.timer = self.create_timer(0.5, self.marker_timer_callback)

    def extract_reachable_apple_data(self, file_path, dir="/home/marcus/orchard_template_ws/results_data/"):
        # Function to extract the required data
        with open(dir + file_path, "r") as file:
            data = yaml.safe_load(file)
        
        apples_found = data.get("apples_found", None)
        apples_reached_templating = data.get("apples_reached_templating", None)
        apples_reached_voxelization = data.get("apples_reached_voxelization", None)
        unreached_idx_voxelization = data.get("unreached_idx_voxelization", None)
        unreached_idx_templating = data.get("unreached_idx_templating", None)
        apple_coordinates = data.get("apple_coordinates", None)
        side_branch_locations = data.get("side_branch_locations", None)
        
        return {
            "apples_found": apples_found,
            "apples_reached_templating": apples_reached_templating,
            "apples_reached_voxelization": apples_reached_voxelization,
            "unreached_idx_voxelization": unreached_idx_voxelization,
            "unreached_idx_templating": unreached_idx_templating,
            "apple_coords": apple_coordinates,
            "side_branch_locations": side_branch_locations
        }
    
    def sort_reachable_apples(self):
        # Templating reachability
        unreached_template_coords = self.apple_coordinates[np.isin(range(len(self.apple_coordinates)), self.unreached_idx_templating)] # "is unreached"
        reached_template_coords = self.apple_coordinates[~np.isin(range(len(self.apple_coordinates)), self.unreached_idx_templating)] # "is reached"

        # Voxelization reachability
        unreached_voxel_coords = self.apple_coordinates[np.isin(range(len(self.apple_coordinates)), self.unreached_idx_voxelization)] # "is unreached"
        reached_voxel_coords = self.apple_coordinates[~np.isin(range(len(self.apple_coordinates)), self.unreached_idx_voxelization)] # "is reached"
        return (reached_template_coords, unreached_template_coords), (reached_voxel_coords, unreached_voxel_coords)

    def publish_coordinates(self):
        if self.pub_template_reachability or self.pub_voxel_reachability:
            pub_reachability = True
        else:
            pub_reachability = False
        if self.pub_template_reachability:
            unreached_idx = self.unreached_idx_templating
        elif self.pub_voxel_reachability:
            unreached_idx = self.unreached_idx_voxelization
        else:
            unreached_idx = []

        # Clear existing markers
        clear_markers = MarkerArray()
        for i in range(len(self.apple_coordinates)):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.id = i
            marker.action = Marker.DELETE
            clear_markers.markers.append(marker)
        self.publisher.publish(clear_markers)
            
        marker_array = MarkerArray()
        for idx, coord in enumerate(self.apple_coordinates):
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'coordinate_markers'
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = coord[0]
            marker.pose.position.y = coord[1]
            marker.pose.position.z = coord[2]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.apple_radii
            marker.scale.y = self.apple_radii
            marker.scale.z = self.apple_radii

            # Set color based on status
            if pub_reachability and idx not in unreached_idx:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            elif pub_reachability and idx in unreached_idx:
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:  # Default to red
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0

            marker_array.markers.append(marker)
        self.publisher.publish(marker_array)
        # self.get_logger().info('Published coordinate markers.')

    def marker_timer_callback(self):
        """Periodically publish markers using the latest apple data."""
        self.publish_coordinates()

def main(args=None):
    rclpy.init(args=args)
    publisher = CoordinateMarkerPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
