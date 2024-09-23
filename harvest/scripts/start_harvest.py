#!/usr/bin/env python3

# ROS
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
# Interfaces
from std_srvs.srv import Trigger
from harvest_interfaces.srv import ApplePrediction
from controller_manager_msgs.srv import SwitchController
# Python 
import numpy as np

class StartHarvest(Node):

    def __init__(self):
        super().__init__("start_harvest_node")
        m_callback_group = MutuallyExclusiveCallbackGroup()

        # Client for switching controller to joint_trajcectory_controller
        self.switch_controller_client = self.create_client(SwitchController, "/controller_manager/switch_controller", callback_group=m_callback_group)
        while not self.switch_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Switch controller service not available, waiting...")

        # Service to activate servo mode on arm
        self.start_servo_client = self.create_client(Trigger, "/servo_node/start_servo", callback_group=m_callback_group)
        while not self.start_servo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Start moveit servo service not available, waiting...")

        # Service to start visual servo
        self.start_vservo_client = self.create_client(Trigger, "/start_visual_servo", callback_group=m_callback_group)
        while not self.start_vservo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Start visual servo service not available, waiting...")

        # Service to start visual servo
        self.start_apple_prediction_client = self.create_client(ApplePrediction, "/apple_prediction", callback_group=m_callback_group)
        while not self.start_apple_prediction_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Start visual servo service not available, waiting...")

        # TODO: ADD MARCUS AND ALEJO SERVICE CLIENTS


    def start_visual_servo(self):
        # Starts global planning sequence
        self.request = Trigger.Request()
        self.future = self.start_vservo_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def switch_controller(self, servo=False, sim=False):
        # Switches controller from forward position controller to joint_trajectory controller
        self.request = SwitchController.Request()
        if servo:
            if not sim:
                self.request.activate_controllers = ["forward_position_controller"] 
                self.request.deactivate_controllers = ["scaled_joint_trajectory_controller"]
            else:
                self.request.activate_controllers = ["forward_position_controller"] 
                self.request.deactivate_controllers = ["joint_trajectory_controller"]
        else:
            if not sim:
                self.request.activate_controllers = ["scaled_joint_trajectory_controller"]
                self.request.deactivate_controllers = ["forward_position_controller"]
            else:
                self.request.activate_controllers = ["joint_trajectory_controller"]
                self.request.deactivate_controllers = ["forward_position_controller"]
        self.request.timeout = rclpy.duration.Duration(seconds=5.0).to_msg()
        self.future = self.switch_controller_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def start_servo(self):
        # Starts servo node
        self.request = Trigger.Request()
        self.future = self.start_servo_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future) 
        return self.future.result()
    
    def start_apple_prediction(self):
        # Starts servo node
        self.request = ApplePrediction.Request()
        self.future = self.start_apple_prediction_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future) 
        return self.future.result()
    
    def start(self): 
        # TODO: CENTER ARM IN HOME POSITION
        self.get_logger().info(f'Resetting arm to home position')
        self.get_logger().info(f'Sending request to predict apple centerpoint locations in scene.')
        apple_poses = self.start_apple_prediction()
        for i in apple_poses:
            # TODO: CENTER ARM IN HOME POSITION
            self.get_logger().info(f'Resetting arm to home position')
            self.get_logger().info(f'Starting initial apple approach.')
            # TODO: MARCUS SERVICE CALL
            # TODO: APPROACH
            self.get_logger().info(f'Apple approach complete')
            self.get_logger().info(f'Switching controller to forward_position_controller.')
            self.switch_controller(servo=True, sim=False)
            self.get_logger().info(f'Starting servo node.')
            self.start_servo()
            self.get_logger().info(f'Starting visual servoing to center of apple')
            self.start_visual_servo()
            self.get_logger().info(f'Switching controller back to scaled_joint_trajectory_controller.')
            self.switch_controller(servo=False, sim=False)
            self.get_logger().info(f'Starting final apple approach and suction cup servoing.')
            # TODO: ALEJO SERVICE CALL
            # TODO: FINAL APPROACH
            # TODO: FINAL RETREAT AND PLACEMENt OF APPLE
            self.get_logger().info(f'Starting retreat sequence')
        self.get_logger().info(f'Test Complete.')

def main(args=None):
    rclpy.init(args=args)
    harvest_control = StartHarvest()
    harvest_control.start()
    executor = MultiThreadedExecutor()
    rclpy.spin(harvest_control, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
