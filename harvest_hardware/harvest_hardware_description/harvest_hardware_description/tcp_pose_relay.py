#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TcpPoseRelay(Node):

    def __init__(self):
        super().__init__('tcp_pose_relay')

        # Declare parameters for flexible frame configuration
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tcp_frame', 'tool0')
        self.declare_parameter('publish_rate', 50.0) # Hz
        self.declare_parameter('use_fake_hardware', True)

        if not self.get_parameter('use_fake_hardware'):
            self.get_logger().info("Do not need fake hardware, bailing")
        else:
            self.get_logger().info("Generating end effector location from joint angles")

            self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
            self.tcp_frame = self.get_parameter('tcp_frame').get_parameter_value().string_value
            rate = self.get_parameter('publish_rate').get_parameter_value().double_value

            # Publisher for the target PoseStamped topic
            self.publisher_ = self.create_publisher(PoseStamped, '~/robot_tcp_pose', 10)

            # Set up TF2 listener buffers
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            # Timer to poll TF at the specified rate
            self.timer = self.create_timer(1.0 / rate, self.timer_callback)
            self.get_logger().info(f"Relay started: Tracking {self.base_frame} -> {self.tcp_frame}")

    def timer_callback(self):
        try:
            # Look up the latest available transform
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tcp_frame,
                now
            )

            # Construct and fill the PoseStamped message
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.base_frame

            # Map translations
            msg.pose.position.x = trans.transform.translation.x
            msg.pose.position.y = trans.transform.translation.y
            msg.pose.position.z = trans.transform.translation.z

            # Map rotations
            msg.pose.orientation.x = trans.transform.rotation.x
            msg.pose.orientation.y = trans.transform.rotation.y
            msg.pose.orientation.z = trans.transform.rotation.z
            msg.pose.orientation.w = trans.transform.rotation.w

            self.publisher_.publish(msg)

        except TransformException as ex:
            # Suppress excessive logging during initial startup/warmup
            self.get_logger().debug(f'Could not transform {self.base_frame} to {self.tcp_frame}: {ex}')

def main(args=None):
    rclpy.init(args=args)
    node = TcpPoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
