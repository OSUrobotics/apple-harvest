#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, TwistStamped, TransformStamped
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from scipy.spatial.transform import Rotation
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters

class PTController(Node):
    
    def __init__(self):
        
        super().__init__('pull_twist_controller')
        
        self.max_velocity = 0.1 # * 0.6 m/s
        self.vel_cmd = Vector3() # * 0.6 m/s

        self.pose_subscription = self.create_subscription(TransformStamped,'/tool_pose', self.configure_self, 10)
        self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.status_publisher = self.create_publisher(Bool, '/pull_twist/status', 10)

        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.running = False

        self.start_service = self.create_service(Empty, 'pull_twist/start_controller', self.start)
        self.stop_service = self.create_service(Empty, 'pull_twist/stop_controller', self.stop)

        self.preferred_pull = np.array([0.0, 0.0, -1.0])

        self.R = np.identity(4)

        self.cli = self.create_client(SetParameters, '/servo_node/set_parameters')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetParameters.Request()

    ## SERVICES

    def start(self, request, response):

        self.running = True
        return response

    def stop(self, request, response):

        self.running = False
        self.iter = 0
        return response

    ## SUBSCRIBERS & PUBLISHERS


    def timer_callback(self):

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        if self.running:

            msg.twist.linear.x = self.max_velocity * self.preferred_pull[0]
            msg.twist.linear.y = self.max_velocity * self.preferred_pull[1]
            msg.twist.linear.z = self.max_velocity * self.preferred_pull[2]
            
            msg.twist.angular.x = self.preffered_twist[0]
            msg.twist.angular.y = self.preffered_twist[1]
            msg.twist.angular.z = self.preffered_twist[2]

            self.publisher.publish(msg)

       
    def configure_self(self, pose_msg):

        quat_msg = pose_msg.transform.rotation
        quat_vec = [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
        
        position_msg = pose_msg.transform.translation
        position_vec = [position_msg.x, position_msg.y, position_msg.z]

        r = Rotation.from_quat(quat_vec)
        self.R = r.as_matrix()

        self.preffered_twist = -1 * self.R[0:3, 2] 

        # self.preffered_twist = -1 * quat_vec

        self.preferred_pull = -1 * np.array(position_vec) / np.linalg.norm(position_vec)

    

def main():

    rclpy.init()

    node = PTController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()
