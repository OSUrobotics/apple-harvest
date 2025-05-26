#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, WrenchStamped, TwistStamped, TransformStamped
from std_msgs.msg import Float64, Bool
from harvest_interfaces.srv import SetValue
from std_srvs.srv import Empty
from scipy.spatial.transform import Rotation
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters

class PickController(Node):
    
    def __init__(self):
        
        super().__init__('pick_controller')
        
        self.goal= 0.0 #N
        self.max_velocity = 0.2 # * 0.6 m/s
        self.vel_cmd = Vector3() # * 0.6 m/s
        self.min_tension = 10.0

        self.wrench_subscription = self.create_subscription(WrenchStamped, '/filtered_wrench', self.process_force_meas, 10)
        self.pose_subscription = self.create_subscription(TransformStamped,'/tool_pose', self.configure_self, 10)
        
        self.cmd_publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.goal_publisher = self.create_publisher(Float64, '/hc_force_goal', 10)
        self.tangent_publisher = self.create_publisher(Vector3, '/hc_tangent', 10)
        self.status_publisher = self.create_publisher(Bool, '/force_heuristic/status', 10)

        self.timer = self.create_timer(0.01, self.timer_callback)
        
        # self.ee_weight = 2.09 # WUR to set (or can uncomment and use set_ee_weight)
        self.ee_weight = 0.0
        self.force_from_gravity = np.array([0.0, 0.0, 0.0])
        self.preferred_pull = np.array([0.0, -0.7, -0.7])
        self.last_t = np.array([0.0,0.0,0.0])

        self.running = False

        self.goal_service = self.create_service(SetValue, 'set_goal', self.change_goal)
        self.start_service = self.create_service(Empty, 'start_controller', self.start)
        self.stop_service = self.create_service(Empty, 'stop_controller', self.stop)
        
        self.R = np.identity(3)

        self.cli = self.create_client(SetParameters, '/servo_node/set_parameters')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetParameters.Request()        

        self.e_f_prev = 0


    ## SERVICES

    def change_goal(self, request, response):
        self.get_logger().info("setting goal...")
        self.goal = request.val
        response.success = True
        return response

    def start(self, request, response):

        self.get_logger().info("starting controller...")
        self.running = True
        return response

    def stop(self, request, response):

        self.running = False
        self.get_logger().info("finished")
        return response
     
    ## SUBSCRIBERS & PUBLISHERS

    def process_force_meas(self, msg):
        
        wrench = msg.wrench 
        
        current_force = np.array([wrench.force.x, wrench.force.y,
                                  wrench.force.z]) - self.force_from_gravity

        rotated_force = np.transpose(np.matmul(self.R, np.transpose(current_force)))

        # self.get_logger().info("I think the base frame force is {}".format(rotated_force))
        
        if self.running:
            self.update_velocity(rotated_force)
        else:
            self.set_initial_tangent(rotated_force)


    def timer_callback(self):

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        if self.running:

            msg.twist.linear = self.vel_cmd        


            self.cmd_publisher.publish(msg)

            msg2 = Float64()
            msg2.data = self.goal
            self.goal_publisher.publish(msg2)

            msg3 = Vector3()
            msg3.x = self.last_t[0]
            msg3.y = self.last_t[1]
            msg3.z = self.last_t[2]
            self.tangent_publisher.publish(msg3)

        msg4 = Bool()
        msg4.data = self.running
        self.status_publisher.publish(msg4)

    ## HELPERS

    def update_velocity(self, force):

        f = np.linalg.norm(force)
        e_f = f-self.goal

        n_hat = force/ f
        
        t = self.choose_tangent(n_hat)
        t_hat = t/np.linalg.norm(t)
        
        # self.get_logger().info("I think the force is {} N".format(f))
        
        if f >= self.min_tension:
            u = e_f / self.goal

            # self.get_logger().info("I think the force error is {}".format(u))
            new_dir = np.tanh(u)**3 * n_hat + (1-np.tanh(np.abs(u))**3) * t_hat
            new = self.max_velocity*new_dir
        else:
            # self.get_logger().info("Trying to tension...")
            new = self.max_velocity*self.preferred_pull 
                
        self.vel_cmd.x = new[0]
        self.vel_cmd.y = new[1]
        self.vel_cmd.z = new[2]
        self.e_f_prev = e_f
        
        
    def set_initial_tangent(self, force):

        new_tangent = np.cross(force, np.cross(self.preferred_pull, force))
        self.last_t = new_tangent/np.linalg.norm(new_tangent)
        
      
    def choose_tangent(self, force):

        new_tangent = np.cross(force, np.cross(self.last_t, force))
        self.last_t = new_tangent

        return new_tangent
    
    def configure_self(self, pose_msg):

        quat_msg = pose_msg.transform.rotation
        quat_vec = [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
        
        position_msg = pose_msg.transform.translation
        position_vec = [position_msg.x, position_msg.y, position_msg.z]

        r = Rotation.from_quat(quat_vec)
        self.R = r.as_matrix()
        
        #rotate force into ee frame
        self.force_from_gravity = np.transpose(np.matmul(self.R, np.transpose([0,0,-1*self.ee_weight])))

        self.preferred_pull = -1 * np.array(position_vec) / np.linalg.norm(position_vec)
        # self.get_logger().info("preferred direction: {}".format(self.preferred_pull))
        #option later to use unrotated lever arm and rotated force to get torque
        

def main():

    rclpy.init()

    node = PickController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()
