#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, TransformStamped
from harvest_interfaces.srv import GetGripperPose 


class GripperPoseService(Node):
    def __init__(self):
        super().__init__('gripper_pose_service')

        self.get_logger().info("Started gripper pose service")

        # Initialize the variable to hold the current gripper pose
        self.current_gripper_pose = Point()

        # Subscriber to the gripper_link pose topic
        self.pose_subscriber = self.create_subscription(
            TransformStamped,
            'gripper_tip',
            self.pose_callback,
            10
        )

        # Service to get the current gripper pose
        self.service = self.create_service(
            GetGripperPose,
            'get_gripper_pose',
            self.handle_get_gripper_pose
        )

    def pose_callback(self, msg):
        # Update the current gripper pose when a new message is received
        self.current_gripper_pose = msg

    def handle_get_gripper_pose(self, request, response):
        # Extract the translation components from the TransformStamped
        x = self.current_gripper_pose.transform.translation.x
        y = self.current_gripper_pose.transform.translation.y
        z = self.current_gripper_pose.transform.translation.z

        # Create a Point message
        point = Point()
        point.x = x
        point.y = y
        point.z = z

        response.point = point

        return response

def main(args=None):
    rclpy.init(args=args)
    gripper_pose_service = GripperPoseService()

    rclpy.spin(gripper_pose_service)

    # Cleanup and shutdown
    gripper_pose_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
