#!/usr/bin/env python3

# ROS
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
# Interfaces
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController
# Python 
import numpy as np

class VisualServoTest(Node):

    def __init__(self):
        super().__init__("launch_param_test")
        m_callback_group = MutuallyExclusiveCallbackGroup()
        j = self.declare_parameter('prediction_model', 0.0)
        my_param = self.get_parameter('prediction_model').get_parameter_value().double_value
        self.get_logger().info('Hello %s!' % my_param)


def main(args=None):
    rclpy.init(args=args)
    servo_control = VisualServoTest()
    executor = MultiThreadedExecutor()
    rclpy.spin(servo_control, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
