#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16, Float32MultiArray


class PressureAverager(Node):

    def __init__(self):

        super().__init__('pressure_averager')

        # self.s1_subscriber = self.create_subscription(UInt16, '/gripper/pressure/sc1', self.log_sensor1, 10)
        # self.s2_subscriber = self.create_subscription(UInt16, '/gripper/pressure/sc2', self.log_sensor2, 10)
        # self.s3_subscriber = self.create_subscription(UInt16, '/gripper/pressure/sc3', self.log_sensor3, 10)

        self.pressure_sub = self.create_subscription(Float32MultiArray, '/gripper/pressure', self.pressure_log, 10)
        
        self.publisher = self.create_publisher(UInt16, '/pressure', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.pressures = [0, 0, 0]

    def pressure_log(self, msg):
        self.pressures = msg.data

    # def log_sensor1(self, msg):
    #     self.pressures[0] = msg.data

    # def log_sensor2(self, msg):
    #     self.pressures[1] = msg.data

    # def log_sensor3(self, msg):
    #     self.pressures[2] = msg.data
        
    def timer_callback(self):
        out_pressure = UInt16()
        out = int(np.round(np.linalg.norm(self.pressures)))
        # self.get_logger().info("{}".format(out))
        out_pressure.data = out
        self.publisher.publish(out_pressure)


def main():

    rclpy.init()

    node = PressureAverager()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()
