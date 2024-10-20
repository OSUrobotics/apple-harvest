#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from rclpy.time import Time

# UNTESTED NODE!!!!!!!!!!

class TrellisWireScanner(Node):
    def __init__(self):
        super().__init__('trellis_wire_scanner')

        self.get_logger().info("Started trellis wire scanning")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.SCAN_PROBE_LENGTH = 0.1    # Length of scanning probe in [m]

        # TODO: might need to increase this if we have the new plate from Ryan
        self.SCAN_PROBE_BASE_WIDTH = 1.3 * 0.0254 # Width of the base [m]. CAUTION: It includes the plate's width

        # Publisher for the scanned points
        self.publisher = self.create_publisher(PoseArray, 'trellis_wire_coordinates', 10)

    def scan_point(self):
        """Transforms the position of the probe's tip into base_link frame."""
        probe_tool = PoseStamped()
        # Probe's (x, y, z) coordinate at 'tool0' frame
        probe_tool.pose.position.x = 0.0
        probe_tool.pose.position.y = 0.0
        probe_tool.pose.position.z = (self.SCAN_PROBE_LENGTH + 
                                      self.SCAN_PROBE_BASE_WIDTH - 
                                      (0.162 + 0.02))  # Adjust according to your urdf.xacro
        probe_tool.header.frame_id = 'tool0'
        probe_tool.header.stamp = self.get_clock().now().to_msg()

        # Transform the position of the tip of the probe to the desired frame
        try:
            transform = self.tf_buffer.transform(probe_tool, 'base_link', timeout=rclpy.duration.Duration(seconds=2))
            probe_base = transform.pose.position
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"Transform error: {str(e)}")
            return None

        # Just pass the x, y, z coordinates
        point_coordinates = [float(probe_base.x), float(probe_base.y), float(probe_base.z)]
        self.get_logger().info(f"Probe coordinates at 'base_link' frame: {point_coordinates}")

        return point_coordinates

    def scan_trellis_wire_loc(self):
        self.get_logger().info("--- Scanning Trellis Wires ---")
        self.get_logger().info("Probing four points...")

        # Define the probe positions and their corresponding attribute names
        probe_positions = [
            ("bottom left", "bottom_left_coord"),
            ("bottom right", "bottom_right_coord"),
            ("top right", "top_right_coord"),
            ("top left", "top_left_coord")
        ]

        poses = []
        for position_name, attr_name in probe_positions:
            input(f"--- Place probe at wire {position_name} location, hit ENTER when ready.")
            coord = self.scan_point()
            setattr(self, attr_name, coord)
            if coord:
                pose = Pose()
                pose.position.x = coord[0]
                pose.position.y = coord[1]
                pose.position.z = coord[2]
                poses.append(pose)

        self.get_logger().info("The points are:")
        for position_name, attr_name in probe_positions:
            coord = getattr(self, attr_name)
            self.get_logger().info(f"{position_name.title()}: {coord}")

        # Publish the scanned points as a PoseArray
        pose_array = PoseArray()
        pose_array.header.frame_id = 'base_link'
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.poses = poses
        self.publisher.publish(pose_array)
        self.get_logger().info("Published trellis wire coordinates to 'trellis_wire_coordinates' topic.")

def main(args=None):
    rclpy.init(args=args)
    node = TrellisWireScanner()
    
    try:
        node.scan_trellis_wire_loc()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
