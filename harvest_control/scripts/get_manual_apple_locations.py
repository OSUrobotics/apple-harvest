#!/usr/bin/env python3

# ROS
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
# Interfaces
from harvest_interfaces.srv import GetGripperPose
# Python 
import numpy as np
import os
from datetime import datetime

class ManualAppleLocations(Node):
    def __init__(self):
        super().__init__("manual_apple_locations_node")

        # --- Declare & read new parameter for output dir ---
        self.declare_parameter('output_directory', '/home/imml/demoData/')
        self.output_directory = self.get_parameter('output_directory') \
                                    .get_parameter_value().string_value

        # GRIPPER‐POSE SERVICE
        self.get_gripper_pose_client = self.create_client(GetGripperPose, 'get_gripper_pose')
        while not self.get_gripper_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Get gripper pose service not available, waiting...')

        self.apple_locations = []
        self.get_logger().info(f'Manual apple locations node running; saving into: {self.output_directory}')

    def get_current_gripper_pose(self):
        request = GetGripperPose.Request()
        future = self.get_gripper_pose_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().point
    
    def create_directory(self, base_name='farmng_position'):
        # Use the parameterized base directory
        base = self.output_directory.rstrip('/') + '/'
        i = 1
        dir_name = f"{base}{base_name}_{i}"
        while os.path.exists(dir_name):
            i += 1
            dir_name = f"{base}{base_name}_{i}"
        os.makedirs(dir_name)
        return dir_name

    def save_to_csv(self, directory):
        # build timestamped filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"apple_locations_{timestamp}.csv"
        csv_path = os.path.join(directory, filename)

        arr = np.array(self.apple_locations)
        header = np.array([['X', 'Y', 'Z']])
        np.savetxt(csv_path, np.vstack((header, arr)), delimiter=',', fmt='%s')

        return csv_path

    def start(self):
        self.get_logger().info('Beginning apple location search')
        num_apples = int(input('Enter the number of apples you wish to record: '))

        for i in range(num_apples):
            input(f'Free drive UR5 to apple_{i + 1}, hit ENTER when ready.')
            p = self.get_current_gripper_pose()
            self.apple_locations.append([p.x, p.y, p.z])

        # Create directory and save to CSV
        directory = self.create_directory()
        csv_file = self.save_to_csv(directory)
        self.get_logger().info(f'Saved apple locations to {csv_file}')


def main(args=None):
    rclpy.init(args=args)
    node = ManualAppleLocations()
    node.start()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()