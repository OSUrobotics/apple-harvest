#!/usr/bin/env python3

import sys

from geometry_msgs.msg import TransformStamped, Point

import rclpy
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException

from harvest_interfaces.srv import GetGripperPose 

class TfListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        # TF frames
        self.source = 'base_link'
        self.tool_frame = 'tool0'
        self.gripper_tip_frame = 'gripper_link'

        # Initialize TF listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Publishers for visualizing transforms
        self.tool_pub = self.create_publisher(TransformStamped, '/tool_pose', 10)
        self.gripper_pub = self.create_publisher(TransformStamped, '/gripper_tip', 10)

        # Service server for GetGripperPose
        self.srv = self.create_service(
            GetGripperPose,
            'get_gripper_pose',
            self.handle_get_gripper_pose
        )
        self.get_logger().info('GetGripperPose service ready')

        # Timer to periodically publish TF
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Publish tool0 transform
        try:
            trans = self._tf_buffer.lookup_transform(
                self.source, self.tool_frame, rclpy.time.Time())
            self.tool_pub.publish(trans)
        except LookupException as e:
           self.get_logger().error(f'Failed to get transform {self.gripper_tip_frame}: {e}')
        except Exception as e:
            self.get_logger().error(f'urdf not processed yet, failing {e}')

    def handle_get_gripper_pose(self, request, response):
        # Lookup the gripper tip transform
        try:
            trans: TransformStamped = self._tf_buffer.lookup_transform(
                self.source, self.gripper_tip_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            # Fill response point
            response.point = Point(
                x=trans.transform.translation.x,
                y=trans.transform.translation.y,
                z=trans.transform.translation.z
            )
        except LookupException as e:
            self.get_logger().error(f'Failed service lookup: {e}')
            # leave response.point at default (0,0,0)
        return response


def main(argv=sys.argv):
    rclpy.init(args=argv)
    node = TfListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
