#!/usr/bin/env python3

# ROS
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
# Interfaces
from std_srvs.srv import Trigger, Empty
from geometry_msgs.msg import Point
from harvest_interfaces.srv import GetGripperPose

# Python 
import numpy as np
import time
import os
import yaml
import copy
# TODO: need to update this (used to rely on trellis position script)
class ManualAppleLocations(Node):
    def __init__(self):
        super().__init__("manual_apple_locations_node")

        self.get_gripper_pose_client = self.create_client(GetGripperPose, 'get_gripper_pose')
        while not self.get_gripper_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Get gripper pose service not available, waiting...')

        self.apple_locations = []
        self.base_dir = '/home/imml/demoData/'

        self.get_logger().info('Manual apple locations node running')

    def get_current_gripper_pose(self):
        request = GetGripperPose.Request()

        future = self.get_gripper_pose_client.call_async(request)
        rclpy.spin_until_future_complete(self, future) 
        return future.result().point
    
    def create_directory(self, base_name='farmng_position'):
        dir_name = self.base_dir + f"{base_name}_1"
        i = 1
        while os.path.exists(dir_name):
            i += 1
            dir_name = f"{base_name}_{i}"
        os.makedirs(dir_name)
        return dir_name

    def save_to_csv(self, directory):
        csv_file_path = os.path.join(directory, 'apple_locations.csv')
        apple_locations_array = np.array(self.apple_locations)
        header = np.array([['X', 'Y', 'Z']])  # Header
        np.savetxt(csv_file_path, np.vstack((header, apple_locations_array)), delimiter=',', fmt='%s')

    def start(self):
        self.get_logger().info('Beginning apple location search')

        num_apples = int(input('Enter the number of apples you wish to record: '))

        for i in range(num_apples):
            input(f'Free drive UR5 to apple_{i + 1}, hit ENTER when ready.')
            point_msg = self.get_current_gripper_pose()
            x = point_msg.x
            y = point_msg.y
            z = point_msg.z
            self.apple_locations.append([x, y, z])

        # Create directory and save to CSV
        directory = self.create_directory()
        self.save_to_csv(directory)
        self.get_logger().info(f'Saved apple locations to {directory}/apple_locations.csv')


def main(args=None):
    rclpy.init(args=args)
    get_apple_locations = ManualAppleLocations()
    get_apple_locations.start()
    rclpy.spin(get_apple_locations)
    rclpy.shutdown()


if __name__ == "__main__":
    main()