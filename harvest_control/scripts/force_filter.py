#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import WrenchStamped
import numpy as np
from scipy.signal import firwin

class ForceFilter(Node):

    def __init__(self):

        super().__init__('forcefilter')

        self.timer = self.create_timer(0.01, self.timer_callback)
        self.publisher = self.create_publisher(WrenchStamped, '/filtered_wrench' ,10)
        self.subscriber = self.create_subscription(WrenchStamped, '/force_torque_sensor_broadcaster/wrench', self.subscriber_callback, 10)

        self.memory = []
        self.window = 150

        self.header = None

        self.b = firwin(self.window, 5, fs=500)

    def add_value(self,value):
        self.memory.append(value)
        if len(self.memory) > self.window:
            self.memory = self.memory[-1*self.window:]

    def filter_force(self):

        f_hist = np.array(self.memory)
        fx = f_hist[:,0].flatten()
        #self.get_logger().info("vx is  size {}".format(np.size(vx)))
        fy = f_hist[:,1].flatten()
        fz = f_hist[:,2].flatten()

        fx_filt = np.dot(fx, self.b)
        fy_filt = np.dot(fy, self.b)
        fz_filt = np.dot(fz, self.b)

        return [fx_filt, fy_filt, fz_filt]

    def timer_callback(self):
        
        if len(self.memory) >= self.window:

            #v = np.mean(self.memory, axis=0)
            f = self.filter_force()

            filtered = WrenchStamped()
            filtered.header = self.header
            filtered.wrench.force.x = f[0]
            filtered.wrench.force.y = f[1]
            filtered.wrench.force.z = f[2]

            self.publisher.publish(filtered)

    def subscriber_callback(self, msg):

        self.header = msg.header
        self.add_value([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])


def main(args=None):
    rclpy.init(args=args)

    force_filter = ForceFilter()

    rclpy.spin(force_filter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    force_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
