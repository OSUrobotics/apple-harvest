#!/usr/bin/env python3

# ROS
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
# Interfaces
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters
from std_srvs.srv import Trigger, Empty
from geometry_msgs.msg import Point
from harvest_interfaces.srv import ApplePrediction, CoordinateToTrajectory, SendTrajectory, RecordTopics, GetGripperPose, SetValue
from controller_manager_msgs.srv import SwitchController
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from harvest_interfaces.action import EventDetection

# Python 
import numpy as np
import time
import os
import yaml
import copy
import re

class StartHarvest(Node):

    def __init__(self):
        super().__init__("start_harvest_node")
        m_callback_group = MutuallyExclusiveCallbackGroup()

        # Parameters
        self.PICK_PATTERN = 'force-heuristic' 
        self.EVENT_SENSITIVITY = 0.43 # a sensitivity of 1.0 will detect any deviation from perfection as failure

        self.apple_coorindates = {}
        self.pick_pattern = {'pick controller': self.PICK_PATTERN}
        
        # Base directory for presaved apple locations
        self.base_data_dir = '/home/imml/demoData/'
        pre_saved_apple_locations_dir = self.base_data_dir + 'farmng_position_1/'
        self.pre_saved_apple_locations = self.read_apple_locations(pre_saved_apple_locations_dir)

        self.get_gripper_pose_client = self.create_client(GetGripperPose, 'get_gripper_pose', callback_group=m_callback_group)
        while not self.get_gripper_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Get gripper pose service not available, waiting...')

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

        # Service to move arm to home
        self.start_move_arm_to_home_client = self.create_client(Trigger, "/move_arm_to_home", callback_group=m_callback_group)
        while not self.start_move_arm_to_home_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Start move arm to home service not available, waiting...")

        self.coord_to_traj_client = self.create_client(CoordinateToTrajectory, 'coordinate_to_trajectory', callback_group=m_callback_group)
        while not self.coord_to_traj_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for coordinate_to_trajectory to be available...')

        self.trigger_arm_mover_client = self.create_client(SendTrajectory, 'execute_arm_trajectory', callback_group=m_callback_group)
        while not self.trigger_arm_mover_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for execute_arm_trajectory to be available...')

        # Services provided by "grasp_controller.py"
        self.grasp_controller_client = self.create_client(Trigger, 'grasp_apple', callback_group=m_callback_group)
        while not self.grasp_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for grasp_apple service to be available...')
        
        self.release_controller_client = self.create_client(Trigger, 'release_apple', callback_group=m_callback_group)
        while not self.release_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for release_apple service to be available...')

        
        # Interfaces for Miranda's pick controllers 
        self.start_controller_cli = self.create_client(Empty, 'start_controller')
        self.wait_for_srv(self.start_controller_cli)

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

        self.set_goal_cli = self.create_client(SetValue, 'set_goal')
        self.wait_for_srv(self.set_goal_cli)

        self._event_client = ActionClient(self, EventDetection, 'event_detection')


        self.status = GoalStatus.STATUS_EXECUTING

    def create_new_batch_directory(self, base_directory):
        # Ensure the base directory exists
        if not os.path.exists(base_directory):
            os.makedirs(base_directory)
        
        # Find the next available batch directory number
        batch_number = 1
        while True:
            batch_directory = os.path.join(base_directory, f"batch_{batch_number}/")
            if not os.path.exists(batch_directory):
                os.makedirs(batch_directory)
                print(f"Created new directory: {batch_directory}")
                break
            batch_number += 1
        
        return batch_directory, batch_number

    def start_recording(self, topics, file_name_prefix):
        # Prepare the request
        request = RecordTopics.Request()
        request.topics = topics
        request.file_name_prefix = file_name_prefix
        
        # Call the service
        self.get_logger().info(f'Calling /record_topics with topics: {topics}')
        future = self.start_record_client.call_async(request)
        
        # Wait for the result
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Recording started: {future.result().success}')
        else:
            self.get_logger().error('Failed to call /record_topics service.')

    def stop_recording(self):
        # Call the stop service
        self.get_logger().info('Calling /stop_recording service...')
        future = self.stop_record_client.call_async(Trigger.Request())
        
        # Wait for the result
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Recording stopped: {future.result().success}')
        else:
            self.get_logger().error('Failed to call /stop_recording service.')

    def get_latest_directory(self, base_name='farmng_position', search_path='.'):
        # Get the list of directories in the specified search path
        dirs = [d for d in os.listdir(search_path) if os.path.isdir(os.path.join(search_path, d)) and re.match(rf'{base_name}_\d+', d)]
        
        # Extract the numeric part and convert to integers
        numbers = [int(re.search(r'\d+', d).group()) for d in dirs]
        
        # Get the directory with the highest number
        if numbers:
            max_index = numbers.index(max(numbers))
            latest_dir = dirs[max_index]
            return os.path.join(search_path, latest_dir)
        else:
            return None

    def read_apple_locations(self, directory):
        csv_file_path = os.path.join(directory, 'apple_locations.csv')
        
        # Load the data using NumPy, skipping the header
        try:
            data = np.loadtxt(csv_file_path, delimiter=',', skiprows=1)
            return data
        except OSError as e:
            print(f"Error reading {csv_file_path}: {e}")
            return None

    def get_current_gripper_pose(self):
        request = GetGripperPose.Request()

        future = self.get_gripper_pose_client.call_async(request)
        rclpy.spin_until_future_complete(self, future) 
        return future.result().point
    
    def scan_trellis_pts(self):
        for key in self.trellis_wire_positions:
            input(f"--- Place probe at wire {key} location, hit ENTER when ready.")
            point_msg = self.get_current_gripper_pose()
            print(point_msg)
            x = point_msg.x
            y = point_msg.y
            z = point_msg.z
            self.trellis_wire_positions[key] = [x, y, z]

    def wait_for_srv(self, srv):
        #service waiter because Miranda is lazy :3
        while not srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
    
    def start_detection(self):

        self.status = GoalStatus.STATUS_EXECUTING
        self.get_logger().info(f'Event Detection status {self.status}')

        goal_msg = EventDetection.Goal()
        goal_msg.failure_ratio = (1.0 - self.EVENT_SENSITIVITY)

        self._event_client.wait_for_server()
        self.get_logger().info(f'Server started')

        self._send_goal_future = self._event_client.send_goal_async(goal_msg)

        return self._send_goal_future.add_done_callback(self.goal_response_callback)

        # return self._send_goal_future

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.status = GoalStatus.STATUS_ABORTED
            return

        self.get_logger().info('Goal accepted :)')
        self.status = GoalStatus.STATUS_EXECUTING 

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.finished))

        if result.finished:
            self.status = GoalStatus.STATUS_SUCCEEDED 
        else:
            self.status = GoalStatus.STATUS_ABORTED

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.listening))

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
        req = SetParameters.Request()

        new_param_value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=frame)
        req.parameters = [Parameter(name='moveit_servo.robot_link_command_frame', value=new_param_value)]
        self.future = self.configure_servo_cli.call_async(req)
    
    def start_apple_prediction(self):
        # Starts servo node
        self.request = ApplePrediction.Request()
        self.future = self.start_apple_prediction_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future) 
        return self.future.result().apple_poses
    
    def go_to_home(self):
        # Starts go to home
        self.request = Trigger.Request()
        self.future = self.start_move_arm_to_home_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future) 
        return self.future.result()
    
    def call_coord_to_traj(self, apple_pose):
        x = apple_pose[0]
        y = apple_pose[1]
        z = apple_pose[2]

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
    
    def event_detection(self):
        # Start event detection
        req = Empty.Request()
        self.future = self.event_detection_start_cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
    
    def configure_controller(self):
        
        pick_force = 15.0
        
        set_goal_req = SetValue.Request()
        set_goal_req.val = pick_force
        self.future = self.set_goal_cli.call_async(set_goal_req)
        rclpy.spin_until_future_complete(self, self.future)

    
    def pick_controller(self):
        req = Empty.Request()
        stop_time = 10
        if self.PICK_PATTERN == 'force-heuristic':
            self.configure_controller()
            self.future = self.start_controller_cli.call_async(req)
            rclpy.spin_until_future_complete(self, self.future)
            #while self.status != GoalStatus.STATUS_SUCCEEDED: #full disclosure, no idea if this works or if it gums up ROS
            #   pass
            time.sleep(stop_time)
            
            self.future = self.stop_controller_cli.call_async(req)
            rclpy.spin_until_future_complete(self, self.future)
            
        elif self.PICK_PATTERN == 'pull-twist':
            self.future = self.pull_twist_start_cli.call_async(req)
            rclpy.spin_until_future_complete(self, self.future)
            while self.status != GoalStatus.STATUS_SUCCEEDED:
               pass
            #self.future = self.pull_twist_stop_cli.call_async(req)
            #rclpy.spin_until_future_complete(self, self.future)
            
        elif self.PICK_PATTERN == 'linear-pull':
            self.future = self.linear_pull_start_cli.call_async(req)
            rclpy.spin_until_future_complete(self, self.future)
            while self.status != GoalStatus.STATUS_SUCCEEDED:
               pass
            #self.future = self.linear_pull_stop_cli.call_async(req)
            #rclpy.spin_until_future_complete(self, self.future)
            
        else:
            self.get_logger().info(f'No valid control scheme set')
    
    def grasp_controller(self):
        # build request
        request = Trigger.Request()
        self.future = self.grasp_controller_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def release_controller(self):
        # build request
        request = Trigger.Request()
        self.future = self.release_controller_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def save_metadata(self):
        # Combine the dictionaries into a list or another structure if necessary
        data = {
            'trellis_wire_positions': copy.deepcopy(self.trellis_wire_positions),
            'apple_coordinates': copy.deepcopy(self.apple_coorindates),
            'pick_controller': self.PICK_PATTERN
        }

        # Save to a YAML file
        with open(self.batch_dir + f'batch_{self.batch_number}_metadata.yaml', 'w') as file:
            yaml.dump(data, file)

        self.get_logger().info("YAML file saved successfully.")    

    def start(self): 
        # Stage 1: Reset arm to home position
        self.get_logger().info(f'Resetting arm to home position')
        self.go_to_home()

        # Stage 3: Approach apple
        self.get_logger().info(f'Starting initial apple approach.')
        waypoints = self.call_coord_to_traj(self.pre_saved_apple_locations)
        self.trigger_arm_mover(waypoints)
        self.get_logger().info(f'Initial apple approach complete')

        # Stage 4: Start visual servo
        self.get_logger().info(f'Switching controller to forward_position_controller.')
        self.switch_controller(servo=True, sim=False)
        self.get_logger().info(f'Starting servo node.')
        self.start_servo()
        self.get_logger().info(f'Starting visual servoing to center of apple')
        self.start_visual_servo()
        self.get_logger().info(f'Switching controller back to scaled_joint_trajectory_controller.')
        self.switch_controller(servo=False, sim=False)

        # Stage 5: Start final approach and pressure servo
        self.get_logger().info(f'Switching controller to forward_position_controller.')
        self.switch_controller(servo=True, sim=False)
        self.get_logger().info(f'Starting servo node.')
        self.start_servo()
        self.get_logger().info(f'Configuring servo planning in base_link frame')
        self.configure_servo('base_link')
        self.get_logger().info(f'Starting apple grasp.')
        self.grasp_controller()
        self.get_logger().info(f'Switching controller back to scaled_joint_trajectory_controller.')
        self.switch_controller(servo=False, sim=False)
        
        # Stage 6: Picking
        self.get_logger().info(f'Starting pick controller')
        self.get_logger().info(f'Switching controller to forward_position_controller.')
        self.switch_controller(servo=True, sim=False)
        self.get_logger().info(f'Starting servo node.')
        self.start_servo()
        self.get_logger().info(f'Configuring servo planning in base_link frame')
        self.configure_servo('base_link')
        self.get_logger().info(f'Activating {self.PICK_PATTERN} controller')
        self.pick_controller()

        # self.get_logger().info('Starting event detection.')
        # self.start_detection()

        self.get_logger().info(f'Configuring servo planning in tool0 frame')
        self.configure_servo('tool0')
        self.get_logger().info(f'Switching controller back to scaled_joint_trajectory_controller.')
        self.switch_controller(servo=False, sim=False)

        # Stage 7: Return arm to home position
        self.get_logger().info(f'Resetting arm to home position')
        self.go_to_home()

        input("Do you want me to release the apple? Press Enter!")

        # Stage 8: Release apple
        self.get_logger().info(f'Releasing apple.')
        self.release_controller()

        input("Are all humans clear of robot path? Press Enter!")

        self.get_logger().info(f'Demo Complete!.')

def main(args=None):
    rclpy.init(args=args)
    harvest_control = StartHarvest()
    harvest_control.start()
    executor = MultiThreadedExecutor()
    rclpy.spin(harvest_control, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
