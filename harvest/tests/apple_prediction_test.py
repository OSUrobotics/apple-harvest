#!/usr/bin/env python3

# ROS
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
# Interfaces
from harvest_interfaces.srv import ApplePrediction

# Python 
import numpy as np

class StartHarvest(Node):

    def __init__(self):
        super().__init__("start_harvest_node")
        m_callback_group = MutuallyExclusiveCallbackGroup()

        # Service to start visual servo
        self.start_apple_prediction_client = self.create_client(ApplePrediction, "/apple_prediction_presaved_images", callback_group=m_callback_group)
        while not self.start_apple_prediction_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Start visual servo service not available, waiting...")        
       
    def start_apple_prediction(self):
        # Starts servo node
        self.request = ApplePrediction.Request()
        self.future = self.start_apple_prediction_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future) 
        return self.future.result().apple_poses 

    def start(self): 
        # Stage 2: Request apple location prediction
        self.get_logger().info(f'Sending request to predict apple centerpoint locations in scene.')
        apple_poses = self.start_apple_prediction()

        # Loop over apple locations
        # for i in apple_poses.poses:
        self.get_logger().info(f'# Apples found: {len(apple_poses.poses)}')


def main(args=None):
    rclpy.init(args=args)
    harvest_control = StartHarvest()
    harvest_control.start()
    executor = MultiThreadedExecutor()
    rclpy.spin(harvest_control, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
