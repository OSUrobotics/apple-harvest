#!/usr/bin/env python3

import numpy
import os
import json

# ROS2 imports
import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Point

class AlejoTrial(Node):
    def __init__(self):
        super().__init__('alejo_trial')


        # Create services
        # air_pressure_servoing


        # Create service client
        self.client = self.create_client()







def main():
    rclpy.init()

    alejo_trial = AlejoTrial()

    # Use a SingleThreadedExecutor to handle the callbacks
    executor = SingleThreadedExecutor()
    executor.add_node(alejo_trial)


    try:
        executor.spin()
    finally:
        executor.shutdown()
        alejo_trial.destroy_node()

        rclpy.shutdown()

if __name__ == '__main__':
    main()