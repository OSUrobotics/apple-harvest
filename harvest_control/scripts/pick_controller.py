#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


from std_msgs.msg import String
from std_srvs.srv import Empty
from harvest_interfaces.action import PickControl
from harvest_interfaces.srv import SetValue

import time

class PickController(Node):

    def __init__(self):
        super().__init__('pick_controller')

        self.cb_group = MutuallyExclusiveCallbackGroup()
        self.r_group = ReentrantCallbackGroup()


        self.pick_controller_action_server = ActionServer(self, PickControl, 'pick_controller', execute_callback=self.execute_pick_callback, cancel_callback=self.cancel_pick_callback)
        
        self.timer = self.create_timer(0.3, self.pick_controller, callback_group=self.r_group)

        #setup pick controller clients
        self.start_controller_cli = self.make_client(Empty, 'start_controller')
        self.start_stiffness_controller_cli = self.make_client(Empty, 'start_stiffness_controller')
        self.stop_controller_cli = self.make_client(Empty, 'stop_controller')
        self.stop_stiffness_controller_cli = self.make_client(Empty, 'stop_stiffness_controller')
        self.pull_twist_start_cli = self.make_client(Empty, 'pull_twist/start_controller')
        self.pull_twist_stop_cli = self.make_client(Empty, 'pull_twist/stop_controller')
        self.linear_pull_start_cli = self.make_client(Empty, 'linear/start_controller')
        self.linear_pull_stop_cli = self.make_client(Empty, 'linear/stop_controller')
        self.set_goal_cli = self.make_client(SetValue, 'set_goal')


        self.start_pick = False
        self.started = False
        self.PICK_PATTERN = "linear-pull"
        self.stop_time = 10
        self.rate = self.create_rate(1)
        self.start_time = self.get_clock().now()

    # Setup Functions
    def make_client(self, srv_type, name):
        client = self.create_client(srv_type, name, callback_group=self.cb_group)
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for service '{name}', retrying...")
        return client
    def execute_pick_callback(self, goal_handle):
        self.get_logger().info('Executing pick controller action...')
        self.start_pick = True
        self.started = False
        self.PICK_PATTERN = goal_handle.request.pattern
        self.stop_time = goal_handle.request.stop_time.data

        try:
            while rclpy.ok() and self.start_pick:
                self.rate.sleep()
        except KeyboardInterrupt:
            pass

        goal_handle.succeed()
        result = PickControl.Result()
        result.success = True
        return result
    
    def cancel_pick_callback(self, goal_handle):
        self.get_logger().info('Pick controller action canceled.')
        self.start_pick = False
        self.started = False
        goal_handle.canceled()
        result = PickControl.Result()
        result.success = False
        return result
    
    def _wait_for_future(self, future, poll_period=0.02):
        #Wait for service call to return
        while rclpy.ok() and not future.done():
            time.sleep(poll_period)
        return future.result()

    def configure_controller(self):

        pick_force = 20.0

        set_goal_req = SetValue.Request()
        set_goal_req.val = pick_force
        self.future = self.set_goal_cli.call_async(set_goal_req)
        self._wait_for_future(self.future)


    def cancel_pick(self):
        self.get_logger().info("cancelling pick controller")
        if self.PICK_PATTERN == 'force-heuristic':
            self.future = self.stop_controller_cli.call_async(Empty.Request())
            self._wait_for_future(self.future)
        elif self.PICK_PATTERN == 'pull-twist':
            self.future = self.pull_twist_stop_cli.call_async(Empty.Request())
            self._wait_for_future(self.future)
        elif self.PICK_PATTERN == 'linear-pull':
            self.future = self.linear_pull_stop_cli.call_async(Empty.Request())
            self._wait_for_future(self.future)
        elif self.PICK_PATTERN == 'stiffness-seeking':
            self.future = self.stop_stiffness_controller_cli.call_async(Empty.Request())
            self._wait_for_future(self.future)
        else:
            self.get_logger().info(f'No valid control scheme set')

    def start_pick_call(self):
        self.get_logger().info("starting pick controller")
        if self.PICK_PATTERN == 'force-heuristic':
            self.configure_controller()
            self.future = self.start_controller_cli.call_async(Empty.Request())
            self._wait_for_future(self.future)
        elif self.PICK_PATTERN == 'pull-twist':
            self.future = self.pull_twist_start_cli.call_async(Empty.Request())
            self._wait_for_future(self.future)
        elif self.PICK_PATTERN == 'linear-pull':
            self.future = self.linear_pull_start_cli.call_async(Empty.Request())
            self._wait_for_future(self.future)
        elif self.PICK_PATTERN == 'stiffness-seeking':
            self.future = self.start_stiffness_controller_cli.call_async(Empty.Request())
            self._wait_for_future(self.future)
        else:
            self.get_logger().info(f'No valid control scheme set')
            self.start_pick = False
        self.started = True
        self.start_time = self.get_clock().now()

    def pick_controller(self):
        if self.start_pick:
            if not self.started:
                self.start_pick_call()
            if self.get_clock().now() - self.start_time > rclpy.duration.Duration(seconds=self.stop_time):
                self.cancel_pick()
                self.start_pick = False
                self.get_logger().info("Pick controller action completed.")
            
        


def main(args=None):
    rclpy.init(args=args)

    pick_controller = PickController()
    executor = MultiThreadedExecutor()


    rclpy.spin(pick_controller, executor=executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pick_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()