#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, TwistStamped
from std_msgs.msg import Bool
from std_srvs.srv import Empty


class LinController(Node):
    
    def __init__(self):
        
        super().__init__('linear_controller')
        
        self.max_velocity = 0.1 # * 0.6 m/s
        self.vel_cmd = Vector3() # * 0.6 m/s

        self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.status_publisher = self.create_publisher(Bool, '/linear/status', 10)

        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.running = False
        self.iter = 0
        self.max_iter = 1500

        self.approach_service = self.create_service(Empty, 'linear/approach', self.approach)
        self.pull_service = self.create_service(Empty, 'linear/start_controller', self.pull)
        self.stop_service = self.create_service(Empty, 'linear/stop_controller', self.stop)

        self.dir = -1.0 #negative = pull, positive = approach

    ## SERVICES

    def set_timer(self, request, response):

        self.max_iter = int(request.val)
        response.success = True
        return response

    def approach(self, request, response):

        self.dir = 1.0
        self.max_iter = 1250
        self.running = True
        return response

    def pull(self, request, response):

        self.dir = -1.0
        self.max_iter = 1000
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
        msg.header.frame_id = "tool0"

        if self.running:

            if self.iter == self.max_iter:
                self.running = False
                self.iter = 0
                self.get_logger().info("finished")

            else:
                msg.twist.linear.z = self.dir*self.max_velocity
                self.iter = self.iter + 1
            
            self.publisher.publish(msg)

def main():

    rclpy.init()

    node = LinController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()
