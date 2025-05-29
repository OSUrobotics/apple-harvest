#!/usr/bin/env python3

# ROS
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
# Interfaces
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger, Empty
from geometry_msgs.msg import Point, Pose, PoseArray
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
from pathlib import Path

def get_data_storage_dir():
    # 1. Find this file’s folder:
    current_dir = Path(__file__).resolve().parent
    # 2. Climb up to the workspace root (ros2_ws)
    #    scripts → harvest → apple-harvest → src → ros2_ws
    #    so we need to go up 4 levels:
    workspace_root = current_dir.parents[3]  
    # 3. Define your data directory alongside src/
    data_dir = workspace_root / 'data'

    # 4. Create it if it doesn’t already exist
    data_dir.mkdir(parents=True, exist_ok=True)

    data_dir = str(data_dir)

    print(f"Using data directory: {data_dir}")

    return data_dir

class StartHarvest(Node):
    def __init__(self):
        super().__init__("start_harvest_node")
        self.cb_group = MutuallyExclusiveCallbackGroup()

        # Get storage directory
        self.storage_directory = get_data_storage_dir()

        # Declare parameters with defaults
        self.declare_parameter('pick_pattern', 'force-heuristic')
        self.declare_parameter('event_sensitivity', 0.43)
        self.declare_parameter('recording_startup_delay', 0.5)
        self.declare_parameter('base_data_dir', self.storage_directory)
        self.declare_parameter('enable_recording', True)
        self.declare_parameter('enable_visual_servo', True)
        self.declare_parameter('enable_apple_prediction', True)

        # Retrieve parameter values
        self.PICK_PATTERN = self.get_parameter('pick_pattern').get_parameter_value().string_value
        self.EVENT_SENSITIVITY = self.get_parameter('event_sensitivity').get_parameter_value().double_value
        self.recording_startup_delay = self.get_parameter('recording_startup_delay').get_parameter_value().double_value
        self.base_data_dir = self.get_parameter('base_data_dir').get_parameter_value().string_value
        self.enable_recording = self.get_parameter('enable_recording').get_parameter_value().bool_value
        self.enable_visual_servo = self.get_parameter('enable_visual_servo').get_parameter_value().bool_value
        self.enable_apple_prediction = self.get_parameter('enable_apple_prediction').get_parameter_value().bool_value

        # Helper clients
        self.start_record_client = self.make_client(RecordTopics, 'record_topics')
        self.stop_record_client = self.make_client(Trigger, 'stop_recording')
        # self.get_gripper_pose_client = self.make_client(GetGripperPose, 'get_gripper_pose')
        self.switch_controller_client = self.make_client(SwitchController, '/controller_manager/switch_controller')
        self.start_servo_client = self.make_client(Trigger, '/servo_node/start_servo')
        self.configure_servo_cli = self.make_client(SetParameters, '/servo_node/set_parameters')
        self.start_vservo_client = self.make_client(Trigger, '/start_visual_servo')
        self.start_apple_prediction_client = self.make_client(ApplePrediction, '/apple_prediction')
        self.start_move_arm_to_home_client = self.make_client(Trigger, '/move_arm_to_home')
        self.coord_to_traj_client = self.make_client(CoordinateToTrajectory, 'coordinate_to_trajectory')
        self.trigger_arm_mover_client = self.make_client(SendTrajectory, 'send_arm_trajectory')
        self.grasp_controller_client = self.make_client(Trigger, 'grasp_apple')
        self.release_controller_client = self.make_client(Trigger, 'release_apple')
        self.start_controller_cli = self.make_client(Empty, 'start_controller')
        self.stop_controller_cli = self.make_client(Empty, 'stop_controller')
        self.pull_twist_start_cli = self.make_client(Empty, 'pull_twist/start_controller')
        self.pull_twist_stop_cli = self.make_client(Empty, 'pull_twist/stop_controller')
        self.linear_pull_start_cli = self.make_client(Empty, 'linear/start_controller')
        self.linear_pull_stop_cli = self.make_client(Empty, 'linear/stop_controller')
        self.set_goal_cli = self.make_client(SetValue, 'set_goal')
        self._event_client = ActionClient(self, EventDetection, 'event_detection')

        self.status = GoalStatus.STATUS_EXECUTING

        # Initialize metadata and topics
        self.init_metadata_and_topics()

    def make_client(self, srv_type, name):
        client = self.create_client(srv_type, name, callback_group=self.cb_group)
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for service '{name}', retrying...")
        return client

    def init_metadata_and_topics(self):
        # METADATA
        self.trellis_wire_positions = {k: None for k in [
            'bottom_left_coord','bottom_right_coord','top_right_coord','top_left_coord'
        ]}
        self.apple_coordinates = {}
        self.pick_pattern = {'pick controller': self.PICK_PATTERN}

        # Recording topics
        self.prediction_topics = ['/apple_markers']
        self.approach_trajectory_topics = ['/apple_markers']
        self.visual_servo_topics = ['/gripper/rgb_palm_camera/image_raw','/joint_states','/servo_node/delta_twist_cmds']
        self.pressure_servo_topics = [
            '/gripper/pressure','/gripper/distance','/gripper/motor/current',
            '/gripper/motor/position','/gripper/motor/velocity','/joint_states',
            '/force_torque_sensor_broadcaster/wrench','/servo_node/delta_twist_cmds'
        ]
        self.pick_controller_topics = [
            '/gripper/pressure','/gripper/distance','/joint_states',
            '/tool_pose','/force_torque_sensor_broadcaster/wrench','/servo_node/delta_twist_cmds'
        ]

        # Batch directories
        self.batch_dir, self.batch_number = self.create_new_batch_directory(self.base_data_dir)

        # File prefixes
        self.prediction_file_name_prefix = 'prediction'
        self.approach_trajectory_file_name_prefix = 'approach_trajectory'
        self.visual_servo_file_name_prefix = 'visual_servo'
        self.pressure_servo_file_name_prefix = 'pressure_servo'
        self.pick_controller_file_name_prefix = 'pick_controller'

        # Load pre-saved apple locations
        apple_loc_path = os.path.join(self.base_data_dir, 'apple_locations/')
        self.pre_saved_apple_locations = self.read_apple_locations(apple_loc_path)

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
        csv_file = Path(directory) / 'apple_locations.csv'
        data = np.loadtxt(str(csv_file), delimiter=',', skiprows=0)
        if data.ndim == 1:
            data = data[np.newaxis, :]
        return data  # shape is now (N, 3)

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
                self.request.deactivate_controllers = ["joint_trajectory_controller"]
            else:
                self.request.activate_controllers = ["forward_position_controller"] 
                self.request.deactivate_controllers = ["joint_trajectory_controller"]
        else:
            if not sim:
                self.request.activate_controllers = ["joint_trajectory_controller"]
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
            # while self.status != GoalStatus.STATUS_SUCCEEDED:
            #    pass
            time.sleep(stop_time)
            self.future = self.pull_twist_stop_cli.call_async(req)
            rclpy.spin_until_future_complete(self, self.future)
            
        elif self.PICK_PATTERN == 'linear-pull':
            self.future = self.linear_pull_start_cli.call_async(req)
            rclpy.spin_until_future_complete(self, self.future)
            # while self.status != GoalStatus.STATUS_SUCCEEDED:
            #    pass
            time.sleep(stop_time)
            self.future = self.linear_pull_stop_cli.call_async(req)
            rclpy.spin_until_future_complete(self, self.future)
            
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
            'apple_coordinates': copy.deepcopy(self.apple_coordinates),
            'pick_controller': self.PICK_PATTERN
        }

        # Save to a YAML file
        with open(self.batch_dir + f'batch_{self.batch_number}_metadata.yaml', 'w') as file:
            yaml.dump(data, file)

        self.get_logger().info("YAML file saved successfully.")    

    def run_stage(self, topics, prefix, servo_frame=None, use_servo=True, action_fn=None):
        stage_name = prefix if isinstance(prefix, str) else str(prefix)
        print(f"--- Running stage: {stage_name} ---")
        if self.enable_recording:
            self.start_recording(topics, self.base_data_dir + prefix)
            time.sleep(self.recording_startup_delay)
        # Engage servo or trajectory
        self.switch_controller(servo=use_servo)
        if use_servo:
            self.start_servo()
        if servo_frame:
            self.configure_servo(servo_frame)
        if action_fn:
            action_fn()
        # Return to trajectory and stop recording
        self.switch_controller(servo=not use_servo)
        if self.enable_recording:
            self.stop_recording()

    def start(self): 
        # Stage 1: Reset arm to home position
        self.get_logger().info(f'Resetting arm to home position')
        self.go_to_home()

        # Stage 2: Request apple location prediction
        if self.enable_apple_prediction:
            self.get_logger().info('Predicting apple locations')
            apple_poses = self.start_apple_prediction()
        else:
            self.get_logger().info('Skipping apple prediction, using pre-saved locations')
            apple_poses = PoseArray()
            apple_poses.poses = [
                Pose(position=Point(x=row[0], y=row[1], z=row[2]))
                for row in self.pre_saved_apple_locations
            ]
        self.apple_coordinates = {f'apple_{i+1}': [p.position.x,p.position.y,p.position.z]
                                    for i,p in enumerate(apple_poses.poses)}
        self.get_logger().info(f'Found {len(apple_poses.poses)} apples!')

        # Loop over apple locations
        for idx, coord in enumerate(apple_poses.poses):
            # Update base directory for new apple location
            base_dir = self.batch_dir + f'apple_{idx}/'

            # Stage 3: Approach apple
            self.get_logger().info(f'Approaching apple {idx}')
            waypoints = self.call_coord_to_traj(coord)
            self.trigger_arm_mover(waypoints)

            # Stage 4: visual servo
            if self.enable_visual_servo:
                self.run_stage(self.visual_servo_topics, self.visual_servo_file_name_prefix,
                               use_servo=True, action_fn=self.start_visual_servo)

            # Stage 5: pressure servo + grasp
            self.run_stage(
                self.pressure_servo_topics,
                self.pressure_servo_file_name_prefix,
                servo_frame='base_link',
                use_servo=True,
                action_fn=self.grasp_controller
            )           

            # Stage 6: pick controller
            def pick_action():
                # self.start_detection()
                self.pick_controller()
                # self.start_detection()
                self.configure_servo('tool0')

            self.run_stage(
                self.pick_controller_topics,
                self.pick_controller_file_name_prefix,
                servo_frame='base_link',
                use_servo=True,
                action_fn=pick_action
            )

            # Stage 7: home & release & save
            self.go_to_home()
            self.release_controller()
            self.save_metadata()

        self.get_logger().info('Batch Complete')

def main(args=None):
    rclpy.init(args=args)
    harvest_control = StartHarvest()
    harvest_control.start()
    executor = MultiThreadedExecutor()
    rclpy.spin(harvest_control, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
