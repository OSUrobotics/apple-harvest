#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation as R
import numpy as np
from harvest_interfaces.srv import UpdateTrellisPosition

class TreeSceneNode(Node):
    def __init__(self):
        super().__init__('tree_scene_node')
        # Create the service
        self.update_position_service = self.create_service(
            UpdateTrellisPosition,
            'update_trellis_position',
            self.update_trellis_position_callback
        )

        # Publisher for collision objects
        self.collision_object_publisher = self.create_publisher(CollisionObject, 'collision_object', 10)

        # # Timer for periodic publishing
        # self.timer = self.create_timer(2.0, self.add_tree_to_scene)

        # Parameters 
        self.declare_parameter("leader_branch_radii", 0.08)
        self.declare_parameter("leader_branch_len", 2.0)
        self.declare_parameter("num_side_branches", 4.0)
        self.declare_parameter("side_branch_radii", 0.04)
        self.declare_parameter("side_branch_len", 2.0)
        self.leader_branch_radii = self.get_parameter("leader_branch_radii").get_parameter_value().double_value
        self.leader_branch_len = self.get_parameter("leader_branch_len").get_parameter_value().double_value
        self.num_side_branches = self.get_parameter("num_side_branches").get_parameter_value().double_value
        self.side_branch_radii = self.get_parameter("side_branch_radii").get_parameter_value().double_value
        self.side_branch_len = self.get_parameter("side_branch_len").get_parameter_value().double_value

        self.trellis_position = [0.0, 2.0, 0.0]
        self.trellis_angle = np.deg2rad(-18.435) # Angle provided by Martin (WSU) 9/11/2024
        self.branch_spacing = self.leader_branch_len / self.num_side_branches
        
        # # Add the tree to the scene
        # self.add_tree_to_scene()

        # self.get_logger().info(f'Trellis template node running')
        # self.get_logger().info(f'Added template to base location x={self.trellis_position[0]}, y={self.trellis_position[1]}, z={self.trellis_position[2]}, '
        #                        f'with tilt angle={np.rad2deg(self.trellis_angle)} deg')

    def update_trellis_position_callback(self, request, response):
        # Update the position
        self.trellis_position[0] = request.x
        self.trellis_position[1] = request.y
        self.trellis_position[2] = request.z

        # Re-add the tree to the planning scene with the new position
        self.add_tree_to_scene()

        response.success = True
        self.get_logger().info(f"Tree position updated to: x={self.trellis_position[0]}, "
                                f"y={self.trellis_position[1]}, z={self.trellis_position[2]}")
        return response

    def add_tree_to_scene(self):
        # Create a CollisionObject for the tree structure
        tree_object = CollisionObject()
        tree_object.id = 'v_trellis_tree'  # Unique identifier
        tree_object.header = Header()
        tree_object.header.frame_id = 'world'  # Root frame

        # Define the leader branch (cylinder)
        leader_branch = SolidPrimitive()
        leader_branch.type = SolidPrimitive.CYLINDER
        leader_branch.dimensions = [self.leader_branch_len, self.leader_branch_radii]  # Height, radius

        leader_pose = Pose()
        leader_pose.position.x = 0.0
        leader_pose.position.y = 0.0
        leader_pose.position.z = self.leader_branch_len / 2  # Centered vertically
        leader_pose.orientation.w = 1.0

        # Add the leader branch to the tree object
        tree_object.primitives.append(leader_branch)
        tree_object.primitive_poses.append(leader_pose)

        # Define and add horizontal branches
        for i in range(1, int(self.num_side_branches) + 1): # Skips putting a side branch at the base of the tree
            side_branch = SolidPrimitive()
            side_branch.type = SolidPrimitive.CYLINDER
            side_branch.dimensions = [self.side_branch_len, self.side_branch_radii]  # length, radius

            branch_pose = Pose()
            branch_pose.position.x = 0.0
            branch_pose.position.y = 0.0
            branch_pose.position.z = i * self.branch_spacing  # Spaced vertically

            # 90-degree rotation around X-axis
            branch_orientation = R.from_euler('xyz', [np.pi/2, 0.0, 0.0]).as_quat()
            branch_pose.orientation.x = branch_orientation[0]
            branch_pose.orientation.y = branch_orientation[1]
            branch_pose.orientation.z = branch_orientation[2]
            branch_pose.orientation.w = branch_orientation[3]

            tree_object.primitives.append(side_branch)
            tree_object.primitive_poses.append(branch_pose)

        # Set the pose of the tree object using the updated position
        tree_object.pose.position.x = self.trellis_position[0]
        tree_object.pose.position.y = self.trellis_position[1]
        tree_object.pose.position.z = self.trellis_position[2]

        canopy_orientation = R.from_euler('xyz', [0, self.trellis_angle, np.pi/2]).as_quat()
        tree_object.pose.orientation.x = canopy_orientation[0]
        tree_object.pose.orientation.y = canopy_orientation[1]
        tree_object.pose.orientation.z = canopy_orientation[2]
        tree_object.pose.orientation.w = canopy_orientation[3]

        # Set the operation to ADD
        tree_object.operation = CollisionObject.ADD

        # Publish the collision object
        self.collision_object_publisher.publish(tree_object)


def main(args=None):
    rclpy.init(args=args)
    node = TreeSceneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down TreeSceneNode...')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
