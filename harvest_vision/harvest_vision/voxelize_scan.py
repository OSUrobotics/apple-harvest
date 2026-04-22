#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from harvest_interfaces.srv import VoxelGrid
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseArray
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from std_srvs.srv import Trigger

import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as R


class VoxelGridService(Node):
    def __init__(self):
        super().__init__('voxel_grid_node')

        # Parameters
        self.declare_parameter("pointcloud_topic", "rgbd_pointcloud")
        self.declare_parameter("source_frame", "mast_camera_color_optical_frame")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("y_lower_threshold", 0.1)
        self.declare_parameter("y_upper_threshold", 1.5)
        self.declare_parameter("apple_neighbor_radius", 0.08)
        self.declare_parameter("voxel_obj_collision_id_prefix", "voxel_")

        self.pointcloud_topic = self.get_parameter("pointcloud_topic").get_parameter_value().string_value
        self.source_frame = self.get_parameter("source_frame").get_parameter_value().string_value
        self.target_frame = self.get_parameter("target_frame").get_parameter_value().string_value
        self.y_lower_threshold = self.get_parameter("y_lower_threshold").get_parameter_value().double_value
        self.y_upper_threshold = self.get_parameter("y_upper_threshold").get_parameter_value().double_value
        self.apple_neighbor_radius = self.get_parameter("apple_neighbor_radius").get_parameter_value().double_value
        self.voxel_obj_collision_id_prefix = self.get_parameter("voxel_obj_collision_id_prefix").get_parameter_value().string_value

        self.use_topic = True
        self.pointcloud_sub = self.create_subscription(PointCloud2, self.pointcloud_topic, self.pointcloud_callback, 10)
        self.latest_pointcloud = None

        # Track active counts for clean removal
        self.active_voxel_count = 0         # collision objects added to planning scene
        self.active_scene_marker_count = 0  # scene (natural colour) RViz markers
        self.active_apple_marker_count = 0  # apple-region (red) RViz markers

        # Publishers
        self.publisher = self.create_publisher(Point, 'voxel_centers', 10)
        self.voxel_collision_pub = self.create_publisher(CollisionObject, "/collision_object", 10)
        self.voxel_marker_publisher = self.create_publisher(MarkerArray, 'voxel_markers', 10)
        self.voxel_marker_removed_publisher = self.create_publisher(MarkerArray, 'removed_voxel_markers', 10)

        # Services
        self.srv = self.create_service(VoxelGrid, 'voxel_grid', self.voxel_grid_callback)
        self.clear_voxels_srv = self.create_service(Trigger, 'clear_voxels', self.clear_voxels_callback)

        # Client
        self.apply_planning_scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        while not self.apply_planning_scene_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /apply_planning_scene service...')

        # Subscribe to apple poses published by apple_prediction node.
        self.apple_coords = None
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.apple_poses_sub = self.create_subscription(
            PoseArray, 'apple_poses', self.apple_poses_callback, latched_qos
        )

        # TF2 Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('VoxelGrid Service ready.')

    def pointcloud_callback(self, msg):
        self.latest_pointcloud = msg

    def apple_poses_callback(self, msg: PoseArray):
        """Store the latest apple locations from the prediction node."""
        self.apple_coords = np.array([
            [p.position.x, p.position.y, p.position.z] for p in msg.poses
        ])
        self.get_logger().info(
            f"Received {len(self.apple_coords)} apple pose(s) on 'apple_poses' topic."
        )

    def get_neighbors(self, coordinates, target_coordinates, radius=0.25):
        """Find all voxel indices within `radius` of any target coordinate.

        Returns an (object array of per-target neighbour coords, flat unique index array).
        Guards against the empty-result case that would cause np.hstack to raise.
        """
        points = []
        idx = []
        for target in target_coordinates:
            distances = np.linalg.norm(coordinates - target, axis=1)
            indices = np.where(distances <= radius)[0]
            points.append(coordinates[indices])
            idx.append(indices)

        # Guard: if every apple returned zero neighbours, hstack would raise
        non_empty = [i for i in idx if len(i) > 0]
        if not non_empty:
            self.get_logger().warn("get_neighbors: no voxels found within radius of any apple coordinate.")
            return np.array([], dtype=object), np.array([], dtype=int)

        idx_flattened = np.unique(np.hstack(non_empty))
        return np.array(points, dtype=object), idx_flattened

    def add_collision_objects(self, voxel_centers):
        self.active_voxel_count = len(voxel_centers)
        for i, voxel_center in enumerate(voxel_centers):
            collision_object = CollisionObject()
            collision_object.id = f"{self.voxel_obj_collision_id_prefix}{i}"
            collision_object.header.frame_id = self.target_frame
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [self.voxel_size] * 3
            box_pose = Pose()
            box_pose.position = voxel_center
            box_pose.orientation.w = 1.0
            collision_object.primitives.append(primitive)
            collision_object.primitive_poses.append(box_pose)
            collision_object.operation = CollisionObject.ADD
            self.voxel_collision_pub.publish(collision_object)

    def add_voxels(self, voxel_centers, colors=None, is_apple_region=False):
        """Publish voxel markers to RViz.

        Args:
            voxel_centers: list/array of (x, y, z) positions.
            colors:        list/array of (r, g, b, a) values in [0, 1].
                           If None, voxels are rendered in solid red (apple region default).
            is_apple_region: when True, publishes to the 'removed_voxel_markers' topic
                             and overrides colors to red regardless of what is passed.
        """
        if is_apple_region:
            self.active_apple_marker_count = len(voxel_centers)
        else:
            self.active_scene_marker_count = len(voxel_centers)

        if is_apple_region or colors is None:
            is_apple_region = True
            colors = [[1.0, 0.0, 0.0, 0.8] for _ in range(len(voxel_centers))]

        marker_array = MarkerArray()
        for i, (center, color) in enumerate(zip(voxel_centers, colors)):
            marker = Marker()
            marker.header.frame_id = "amiga__base"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "voxels"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2])
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.voxel_size
            marker.scale.y = self.voxel_size
            marker.scale.z = self.voxel_size
            marker.color.r = float(color[0])
            marker.color.g = float(color[1])
            marker.color.b = float(color[2])
            marker.color.a = float(color[3]) if len(color) > 3 else 1.0
            marker_array.markers.append(marker)

        self.get_logger().info(f"Publishing {len(marker_array.markers)} {'apple-region (red)' if is_apple_region else 'scene'} markers to RViz.")
        if is_apple_region:
            self.voxel_marker_removed_publisher.publish(marker_array)
        else:
            self.voxel_marker_publisher.publish(marker_array)

    def clear_voxels_callback(self, request, response):
        # Build a planning scene diff removing all known collision objects atomically
        planning_scene = PlanningScene()
        planning_scene.is_diff = True

        for i in range(self.active_voxel_count):
            co = CollisionObject()
            co.id = f"{self.voxel_obj_collision_id_prefix}{i}"
            co.header.frame_id = self.target_frame
            co.operation = CollisionObject.REMOVE
            planning_scene.world.collision_objects.append(co)

        req = ApplyPlanningScene.Request()
        req.scene = planning_scene
        future = self.apply_planning_scene_client.call_async(req)

        # Spin a temporary separate executor to process the future
        # without deadlocking the node's own executor
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self)
        try:
            executor.spin_until_future_complete(future, timeout_sec=5.0)
        finally:
            executor.remove_node(self)

        if not future.done() or future.result() is None:
            self.get_logger().warn("apply_planning_scene did not complete within timeout.")

        # Delete scene markers by ID
        scene_delete = MarkerArray()
        for i in range(self.active_scene_marker_count):
            m = Marker()
            m.header.frame_id = self.target_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "voxels"
            m.id = i
            m.action = Marker.DELETE
            scene_delete.markers.append(m)
        self.voxel_marker_publisher.publish(scene_delete)

        # Delete apple-region markers by ID
        apple_delete = MarkerArray()
        for i in range(self.active_apple_marker_count):
            m = Marker()
            m.header.frame_id = self.target_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "voxels"
            m.id = i
            m.action = Marker.DELETE
            apple_delete.markers.append(m)
        self.voxel_marker_removed_publisher.publish(apple_delete)

        self.get_logger().info(
            f"Cleared {self.active_voxel_count} collision objects, "
            f"{self.active_scene_marker_count} scene markers, "
            f"{self.active_apple_marker_count} apple-region markers."
        )

        self.active_voxel_count = 0
        self.active_scene_marker_count = 0
        self.active_apple_marker_count = 0

        response.success = True
        response.message = "Voxels cleared."
        return response

    def voxelize_point_cloud(self, voxel_size, transform):
        if self.latest_pointcloud is None:
            self.get_logger().error("No pointcloud received yet")
            return np.array([]), np.array([])

        # --- XYZ ----------------------------------------------------------------
        xyz_struct = np.array(list(
            point_cloud2.read_points(
                self.latest_pointcloud, field_names=("x", "y", "z"), skip_nans=True
            )
        ))
        if xyz_struct.size == 0:
            self.get_logger().warn("Point cloud is empty after reading.")
            return np.array([]), np.array([])

        points = np.column_stack([
            xyz_struct["x"].astype(np.float64),
            xyz_struct["y"].astype(np.float64),
            xyz_struct["z"].astype(np.float64),
        ])

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # --- RGB ----------------------------------------------------------------
        if 'rgb' in [f.name for f in self.latest_pointcloud.fields]:
            rgb_struct = np.array(list(
                point_cloud2.read_points(
                    self.latest_pointcloud, field_names=("rgb",), skip_nans=True
                )
            ))
            # RGB field is packed as UINT32 (r<<16 | g<<8 | b).
            # Cast to uint32 first so bitwise ops work correctly.
            rgb_vals = rgb_struct["rgb"].view(np.uint32) \
                if rgb_struct["rgb"].dtype == np.float32 \
                else rgb_struct["rgb"].astype(np.uint32)
            colors = np.column_stack([
                (rgb_vals >> 16) & 0xFF,
                (rgb_vals >> 8)  & 0xFF,
                rgb_vals         & 0xFF,
            ]).astype(np.float64) / 255.0
            pcd.colors = o3d.utility.Vector3dVector(colors)

        # Filter out invalid points (NaN, Inf, or z <= 0)
        point_coords = np.asarray(pcd.points)
        valid_mask = np.isfinite(point_coords).all(axis=1) & (point_coords[:, 2] > 0.01)
        if not valid_mask.any():
            self.get_logger().warn("No valid points in point cloud after filtering.")
            return np.array([]), np.array([])
        pcd = pcd.select_by_index(np.where(valid_mask)[0])

        # Perform voxelization
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)

        # Extract voxel centers and build blended per-voxel colors
        point_colors = np.asarray(pcd.colors)
        point_coords = np.asarray(pcd.points)
        voxel_centers = []
        voxel_colors = []

        # Map each point to its voxel index
        voxel_map = {}
        for idx, point in enumerate(point_coords):
            voxel_index = tuple(voxel_grid.get_voxel(point))
            if voxel_index not in voxel_map:
                voxel_map[voxel_index] = []
            voxel_map[voxel_index].append(idx)

        # Calculate blended color for each occupied voxel.
        has_colors = point_colors.shape[0] > 0
        for voxel_index, point_indices in voxel_map.items():
            voxel_center = voxel_grid.get_voxel_center_coordinate(voxel_index)
            voxel_centers.append(voxel_center)

            if has_colors:
                voxel_color = np.mean(point_colors[point_indices], axis=0)
            else:
                voxel_color = [0.5, 0.5, 0.5]
            voxel_colors.append(voxel_color)

        voxel_centers = np.array(voxel_centers)
        voxel_colors = np.array(voxel_colors)

        quaternion = np.array([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ])
        rot_matrix = R.from_quat(quaternion).as_matrix()

        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])

        transformed_voxel_centers = (rot_matrix @ voxel_centers.T).T + translation

        y_mask = np.array([
            self.y_lower_threshold <= point[1] <= self.y_upper_threshold
            for point in transformed_voxel_centers
        ])
        transformed_voxel_centers = transformed_voxel_centers[y_mask]
        voxel_colors = voxel_colors[y_mask]

        return transformed_voxel_centers, voxel_colors

    def voxel_grid_callback(self, request, response):
        try:
            self.voxel_size = request.voxel_size

            # Clear any previously published voxels before generating new ones
            if self.active_voxel_count > 0:
                dummy_req = Trigger.Request()
                dummy_res = Trigger.Response()
                self.clear_voxels_callback(dummy_req, dummy_res)

            try:
                transform = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame, rclpy.time.Time())
            except Exception as e:
                self.get_logger().error(f"Transform lookup failed: {e}")
                return response

            # Voxelize the point cloud
            voxel_centers, voxel_colors = self.voxelize_point_cloud(self.voxel_size, transform)

            if len(voxel_centers) == 0:
                self.get_logger().warn("Voxelization returned no voxels.")
                return response

            # Apple-region masking: use poses received from the apple_poses topic.
            # Falls back gracefully to no-masking if prediction hasn't run yet.
            if self.apple_coords is not None and len(self.apple_coords) > 0:
                apple_coords = self.apple_coords
                radius = self.apple_neighbor_radius
                self.get_logger().info(f"Apple neighbor radius: {radius} m, {len(apple_coords)} apple(s) from topic.")

                neighbor_coords, neighbor_idx = self.get_neighbors(voxel_centers, apple_coords, radius=radius)

                neighbor_idx_set = set(neighbor_idx.tolist())
                all_indices = np.arange(len(voxel_centers))

                voxel_centers_scene = [voxel_centers[i] for i in all_indices if i not in neighbor_idx_set]
                voxel_colors_scene  = [voxel_colors[i]  for i in all_indices if i not in neighbor_idx_set]
                voxel_centers_apple = [voxel_centers[i] for i in neighbor_idx]

                self.get_logger().info(
                    f"Voxels total: {len(voxel_centers)} | "
                    f"scene (normal): {len(voxel_centers_scene)} | "
                    f"apple region (red): {len(voxel_centers_apple)}"
                )

                # Publish scene voxels with their blended natural colors
                self.add_voxels(voxel_centers_scene, voxel_colors_scene, is_apple_region=False)

                # Publish apple-region voxels as red — do NOT pass colors
                self.add_voxels(voxel_centers_apple, is_apple_region=True)

                # Only scene voxels become collision objects (apple space is intentionally free)
                voxel_centers_scene_poses = [Point(x=float(c[0]), y=float(c[1]), z=float(c[2])) for c in voxel_centers_scene]
                self.add_collision_objects(voxel_centers_scene_poses)

                # Response carries only the non-apple (obstacle) voxels
                response.voxel_centers = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in voxel_centers_scene]
                response.voxel_colors  = [ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=1.0) for c in voxel_colors_scene]

            else:
                # No apple poses received yet — publish everything with natural colors
                self.get_logger().warn("No apple poses received on 'apple_poses' topic yet — skipping apple-region masking.")
                self.add_voxels(voxel_centers, voxel_colors, is_apple_region=False)
                voxel_center_poses = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in voxel_centers]
                self.add_collision_objects(voxel_center_poses)
                response.voxel_centers = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in voxel_centers]
                response.voxel_colors  = [ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=1.0) for c in voxel_colors]

            # Publish raw voxel centers on the point topic (all voxels, pre-masking)
            for center in voxel_centers:
                point_msg = Point(x=float(center[0]), y=float(center[1]), z=float(center[2]))
                self.publisher.publish(point_msg)

            self.get_logger().info(f"Voxel grid generated with voxel size {self.voxel_size} m.")

        except Exception as e:
            self.get_logger().error(f"Error during voxel grid generation: {e}")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = VoxelGridService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()