#!/usr/bin/env python3

# ROS
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point
from std_srvs.srv import Trigger
import tf2_ros
from geometry_msgs.msg import TransformStamped
# Interfaces
from harvest_interfaces.srv import ApplePrediction, VoxelGrid, MoveToPose, UpdateTrellisPosition

# Python 
import numpy as np

class OrchardTemplating(Node):

    def __init__(self):
        super().__init__("orchard_templating_node")
        m_callback_group = MutuallyExclusiveCallbackGroup()

        # Initialize TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # TODO: Set as a ros2 parameters
        self.voxel_size = 0.02
        self.neighbor_radii = 0.06

        # Publishers
        self.voxel_collision_pub = self.create_publisher(CollisionObject, "/collision_object", 10)

        # Services
        self.start_apple_prediction_client = self.create_client(ApplePrediction, "/apple_prediction_presaved_images", callback_group=m_callback_group)
        while not self.start_apple_prediction_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Apple prediction service not available, waiting...")    

        self.voxel_client = self.create_client(VoxelGrid, "voxel_grid")    
        while not self.voxel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Voxel grid service not available, waiting...")   

        self.move_arm_to_pose_client = self.create_client(MoveToPose, "/move_arm_to_pose",callback_group=m_callback_group)
        while not self.move_arm_to_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Move arm to pose service not available, waiting...")

        self.start_move_arm_to_home_client = self.create_client(Trigger, "/move_arm_to_home", callback_group=m_callback_group)
        while not self.start_move_arm_to_home_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Start move arm to home service not available, waiting...")

        self.trellis_template_client = self.create_client(UpdateTrellisPosition, "/update_trellis_position", callback_group=m_callback_group)
        while not self.trellis_template_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Trellis template placement service not available, waiting...")
    
    def get_gripper_position(self, target_frame="world", source_frame="gripper_link"):
        try:
            # Query the transform
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )
            position = transform.transform.translation
            # self.get_logger().info(f"Gripper position: x={position.x}, y={position.y}, z={position.z}")
            return np.array((position.x, position.y, position.z))
        except tf2_ros.LookupException:
            self.get_logger().error("Transform not available!")
        except tf2_ros.ExtrapolationException:
            self.get_logger().error("Extrapolation error!")
        except tf2_ros.TransformException as e:
            self.get_logger().error(f"Failed to get transform: {str(e)}")
        return None
        
    def start_apple_prediction(self):
        # Starts servo node
        self.request = ApplePrediction.Request()
        self.future = self.start_apple_prediction_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future) 
        return self.future.result().apple_poses 
    
    def sort_coordinates(self, target_coord, coords):
        # Calculate distances from each point to the target
        distances = np.linalg.norm(coords - target_coord, axis=1)

        # Sort indices based on distances
        sorted_indices = np.argsort(distances)

        # Get sorted coordinates
        return coords[sorted_indices]
    
    def call_voxel_grid_service(self):
        request = VoxelGrid.Request()
        request.voxel_size = self.voxel_size 

        self.future = self.voxel_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future) 
        return self.future.result().voxel_centers
    
    def update_trellis_template_pos(self, target_base_position):
        request = UpdateTrellisPosition.Request()
        request.x = target_base_position[0]
        request.y = target_base_position[1]
        request.z = target_base_position[2]

        self.future = self.trellis_template_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future) 
        return self.future.result().success
    
    def get_neighbors(self, coordinates, target_coordinates, radius=0.25):
        # Find coordinates within the sphere
        points = []
        idx = []
        for target in target_coordinates:
            # Compute Euclidean distances from the target to all original coordinates
            distances = np.linalg.norm(coordinates - target, axis=1)
            # Get indices of coordinates within the radius
            indices = np.where(distances <= radius)[0]
            # Collect results
            points.append(coordinates[indices])
            idx.append(indices)
        
        # Flatten the array and remove duplicates
        idx_flattened = np.unique(np.hstack(idx))

        return np.array(points, dtype=object), idx_flattened
        
    def add_collision_objects(self, voxel_centers):      
        for i, voxel_center in enumerate(voxel_centers):
            collision_object = CollisionObject()
            collision_object.id = f"voxel_{i}"
            collision_object.header.frame_id = "world"

            # Define the shape and size
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [self.voxel_size] * 3

            # Define the pose
            box_pose = Pose()
            box_pose.position = voxel_center
            box_pose.orientation.w = 1.0

            collision_object.primitives.append(primitive)
            collision_object.primitive_poses.append(box_pose)
            collision_object.operation = CollisionObject.ADD

            self.voxel_collision_pub.publish(collision_object)

    def send_pose_goal(self, coordinate):
        point = Point()
        point.x = coordinate[0]
        point.y = coordinate[1]
        point.z = coordinate[2]

        # Sends x,y,z to C++ moveit node to execute pose goal since python moveit not available for humble
        self.request = MoveToPose.Request()
        self.request.position = point
        self.future = self.move_arm_to_pose_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def go_to_home(self):
        # Starts go to home
        self.request = Trigger.Request()
        self.future = self.start_move_arm_to_home_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future) 
        return self.future.result()

    def start(self): 
        # Request apple location prediction
        self.get_logger().info(f'Sending request to predict apple centerpoint locations in scene.')
        apple_poses = self.start_apple_prediction()
        apple_coords = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in apple_poses.poses])
        self.get_logger().info(f'# of apples found: {len(apple_poses.poses)}')

        # Sort apple locations based on closeness to gripper_link
        self.get_logger().info(f'Sorting apple locations')
        gripper_position = self.get_gripper_position()
        apple_coords = self.sort_coordinates(gripper_position, apple_coords)

        # Request voxel data from point cloud
        self.get_logger().info(f'Sending request to extract voxels from point cloud.')
        voxel_centers = self.call_voxel_grid_service()
        voxel_centers = np.array([[point.x, point.y, point.z] for point in voxel_centers])
        self.get_logger().info(f"# of voxels generated: {len(voxel_centers)}")

        # Find all neighboring point within a sphere around the apple locations
        neighbor_coords, neighbor_idx = self.get_neighbors(voxel_centers, apple_coords, radius=self.neighbor_radii)
        self.get_logger().info(f'# of voxels to remove based on apple locations: {len(neighbor_idx)}')

        voxel_centers_apple_masked = [idx for i, idx in enumerate(voxel_centers) if i not in neighbor_idx]
        self.get_logger().info(f'# of voxels after apple location removal: {len(voxel_centers_apple_masked)}')

        # # Add voxels as moveit2 collision objects - first convert back to pose message
        # voxel_centers_apple_masked_poses = [Point(x=coord[0], y=coord[1], z=coord[2]) for coord in voxel_centers_apple_masked]
        # self.get_logger().info(f"Publishing {len(voxel_centers_apple_masked_poses)} collision objects to planning scene")
        # self.add_collision_objects(voxel_centers_apple_masked_poses)

        # Move arm to apple position
        apple_coords[:, 1] -= 0.1
        for i, apple in enumerate(apple_coords):
            self.get_logger().info(f'Moving arm to apple ID: {i}')
            self.send_pose_goal(apple)
            
            self.get_logger().info(f'Moving arm to home')
            self.go_to_home()
        
        self.get_logger().info(f'Searched through all apples!')

        # Place trellis template
        target_base_pos = np.array([-0.07, 1.105, -0.19])
        self.get_logger().info(f'Placing trellis template at x={np.round(target_base_pos[0], 3)}, y={np.round(target_base_pos[1], 3)}, z={np.round(target_base_pos[2], 3)} in the planning scene')
        self.update_trellis_template_pos(target_base_pos)


def main(args=None):
    rclpy.init(args=args)
    node = OrchardTemplating()
    node.start()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
