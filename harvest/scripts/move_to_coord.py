#!/usr/bin/env python3
"""
Move the arm to the precomputed voxel trajectory nearest an xyz coordinate.

Usage:
    ros2 run harvest move_to_coord.py X Y Z
    ros2 run harvest move_to_coord.py --home     # plan back to move_arm's home configuration
"""
import argparse

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from harvest_interfaces.srv import CoordinateToTrajectory, SendTrajectory


class MoveToCoord(Node):
    def __init__(self):
        super().__init__('move_to_coord')

    def call(self, srv_type, name, request):
        client = self.create_client(srv_type, name)
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {name}...')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def move_to(self, x, y, z):
        request = CoordinateToTrajectory.Request()
        request.coordinate.x, request.coordinate.y, request.coordinate.z = x, y, z
        result = self.call(CoordinateToTrajectory, 'coordinate_to_trajectory', request)
        if not result.success:
            self.get_logger().error(f'No trajectory found near ({x}, {y}, {z})')
            return False

        request = SendTrajectory.Request()
        request.waypoints = result.waypoints
        return self.call(SendTrajectory, 'send_arm_trajectory', request).success

    def move_home(self):
        result = self.call(Trigger, 'move_arm_to_home', Trigger.Request())
        if not result.success:
            self.get_logger().error(result.message)
        return result.success


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('coord', nargs='*', type=float, metavar='XYZ')
    parser.add_argument('--home', action='store_true', help='move to the home configuration')
    args, ros_args = parser.parse_known_args()
    if not args.home and len(args.coord) != 3:
        parser.error('provide X Y Z, or --home')

    rclpy.init(args=ros_args)
    node = MoveToCoord()
    try:
        success = node.move_home() if args.home else node.move_to(*args.coord)
        node.get_logger().info('Done' if success else 'Failed')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
