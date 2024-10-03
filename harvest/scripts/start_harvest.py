#!/usr/bin/env python3

# ROS
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
# Interfaces
from std_srvs.srv import Trigger, Empty
from geometry_msgs.msg import Point
from harvest_interfaces.srv import ApplePrediction, CoordinateToTrajectory, SendTrajectory
from controller_manager_msgs.srv import SwitchController
# Python 
import numpy as np
import time

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

        # Service to change planning frame of servo mode
        self.configure_servo_cli = self.create_client(SetParameters, '/servo_node/set_parameters')
        while not self.configure_servo_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # Service to start visual servo
        self.start_vservo_client = self.create_client(Trigger, "/start_visual_servo", callback_group=m_callback_group)
        while not self.start_vservo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Start visual servo service not available, waiting...")

        # Service to start visual servo
        self.start_apple_prediction_client = self.create_client(ApplePrediction, "/apple_prediction", callback_group=m_callback_group)
        while not self.start_apple_prediction_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Start visual servo service not available, waiting...")        

        # TODO: ADD MARCUS AND ALEJO SERVICE CLIENTS
        # Service to move arm to home
        self.start_move_arm_to_home_client = self.create_client(Empty, "/move_arm_to_home", callback_group=m_callback_group)
        while not self.start_move_arm_to_home_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Start move arm to home service not available, waiting...")

        self.coord_to_traj_client = self.create_client(CoordinateToTrajectory, 'coordinate_to_trajectory', callback_group=m_callback_group)
        while not self.coord_to_traj_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for coordinate_to_trajectory to be available...')

        self.trigger_arm_mover_client = self.create_client(SendTrajectory, 'execute_arm_trajectory', callback_group=m_callback_group)
        while not self.trigger_arm_mover_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for execute_arm_trajectory to be available...')

        #Services for Miranda's pick controllers 

        self.start_controller_cli = self.create_client(Empty, 'start_controller')
        self.wait_for_srv(self.set_ee_z_cli)

        self.stop_controller_cli = self.create_client(Empty, 'stop_controller')
        self.wait_for_srv(self.stop_controller_cli)

        self.pull_twist_start_cli = self.create_client(Empty, 'pull_twist/start_controller')
        self.wait_for_srv(self.pull_twist_start_cli)

        self.pull_twist_stop_cli = self.create_client(Empty, 'pull_twist/stop_controller')
        self.wait_for_srv(self.pull_twist_stop_cli)

        self.linear_pull_start_cli = self.create_client(Empty, 'linear/start_controller')
        self.wait_for_srv(self.linear_pull_start_cli)

        self.linear_pull_stop_cli = self.create_client(Empty, 'linear/stop_controller')
        self.wait_for_srv(self.linear_pull_stop_cli)

        self.event_detection_start_cli = self.create_client(Empty, 'start_detection')
        self.wait_for_srv(self.event_detection_start_cli)

        self.event_detection_stop_cli = self.create_client(Empty, 'stop_detection')
        self.wait_for_srv(self.event_detection_stop_cli)

        #Parameters
        PICK_PATTERN = 'force-heuristic' 

    def wait_for_srv(self, srv):
        #service waiter because Miranda is lazy :3
        while not srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
            
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

    def configure_servo(self, frame):
        # Changes planning frame of the servo node
        #arguments: "base_link" for base frame, "tool0" for tool frame
        new_param_value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=frame)
        self.req.parameters = [Parameter(name='moveit_servo.robot_link_command_frame', value=new_param_value)]
        self.future = self.configure_servo_cli.call_async(self.req)
    
    def start_apple_prediction(self):
        # Starts servo node
        self.request = ApplePrediction.Request()
        self.future = self.start_apple_prediction_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future) 
        return self.future.result().apple_poses
    
    def go_to_home(self):
        # Starts go to home
        self.request = Empty.Request()
        self.future = self.start_move_arm_to_home_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future) 
        return self.future.result()
    
    def call_coord_to_traj(self, apple_pose):
        x = apple_pose.position.x
        y = apple_pose.position.y
        z = apple_pose.position.z

        # Create a Point message
        coord = Point()
        coord.x = x
        coord.y = y
        coord.z = z

        self.request = CoordinateToTrajectory.Request()
        self.request.coordinate = coord        

        self.future = self.coord_to_traj_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future) 
        return self.future.result().waypoints

    def trigger_arm_mover(self, trajectory):
        request = SendTrajectory.Request()
        request.waypoints = trajectory  # Pass the entire Float32MultiArray message

        # Use async call
        future = self.trigger_arm_mover_client.call_async(request)
        rclpy.spin_until_future_complete(self, future) 
        return future.result()
    
    def start(self): 
        # TODO: CENTER ARM IN HOME POSITION
        self.get_logger().info(f'Resetting arm to home position')
        self.go_to_home()
        self.get_logger().info(f'Sending request to predict apple centerpoint locations in scene.')
        apple_poses = self.start_apple_prediction()
        for i in apple_poses.poses:
            # TODO: CENTER ARM IN HOME POSITION
            self.get_logger().info(f'Starting initial apple approach.')
            # TODO: MARCUS SERVICE CALL
            waypoints = self.call_coord_to_traj(i)
            self.trigger_arm_mover(waypoints)
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

            #start event detection
            req = Empty.Request()
            self.future = self.event_detection_start_cli.call_async(req)
            rclpy.spin_until_future_complete(self, self.future)

            #activate pick controller
            if PICK_PATTERN == 'force-heuristic':
                
                self.future = self.start_controller_cli.call_async(req)
                rclpy.spin_until_future_complete(self, self.future)
                time.sleep(5)
                self.future = self.stop_controller_cli.call_async(req)
                rclpy.spin_until_future_complete(self, self.future)
                
            elif PICK_PATTERN == 'pull-twist':
                
                self.future = self.pull_twist_start_cli.call_async(req)
                rclpy.spin_until_future_complete(self, self.future)
                time.sleep(5)
                self.future = self.pull_twist_stop_cli.call_async(req)
                rclpy.spin_until_future_complete(self, self.future)
                
                
            elif PICK_PATTERN == 'linear-pull':
                
                self.future = self.linear_pull_start_cli.call_async(req)
                rclpy.spin_until_future_complete(self, self.future)
                time.sleep(5)
                self.future = self.linear_pull_stop_cli.call_async(req)
                rclpy.spin_until_future_complete(self, self.future)
                
                
            else:
                self.get_logger().info(f'No valid control scheme set')

            #todo: restart when event is detected (currently is on a timer)
            
            self.get_logger().info(f'Resetting arm to home position')
            self.go_to_home()
            
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
