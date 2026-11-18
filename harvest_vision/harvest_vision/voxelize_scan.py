#!/usr/bin/env python3
"""
voxelize_scan.py

Voxelizes the RGB-D point cloud (rgbd_pointcloud, from apple_prediction) and puts the voxels in the MoveIt planning
scene as one collision object (voxel_object_id) with one box for each voxel.

Services
--------
voxel_grid   (harvest_interfaces/VoxelGrid)  make the voxels from the latest cloud and add the collision object
             (it replaces the previous one). The response has all the voxel centers and colors.
clear_voxels (std_srvs/Trigger)              remove the collision object and the RViz markers.

The voxel grid has no free space around the apples. A planner client that needs free space around a target apple
adds the same object id again without the voxels near that apple (moveit_msgs/ApplyPlanningScene, operation ADD).

Parameters
----------
pointcloud_topic   (str)   – default rgbd_pointcloud (latched; apple_prediction publishes it when a tree loads, and an
                             empty cloud for a tree without mast-camera images).
target_frame       (str)   – planning frame of the voxels (default amiga__base).
y_abs_min / y_abs_max (float) – keep voxels with y_abs_min <= |y| <= y_abs_max in target_frame [m]
                               (removes the robot and the next rows; default 0.1 / 2.5).
voxel_object_id    (str)   – collision object id (default voxels).
"""

import threading

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Point, Pose
from harvest_interfaces.srv import VoxelGrid
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def voxelize(points: np.ndarray, colors: np.ndarray, voxel_size: float):
    """
    Voxel centers and mean colors of the occupied voxels of a regular grid (cell index = floor(p / voxel_size)).
    Return (centers (K, 3), colors (K, 3)).
    """
    idx = np.floor(points / voxel_size).astype(np.int64)
    cells, inverse, counts = np.unique(idx, axis=0, return_inverse=True, return_counts=True)
    inverse = inverse.ravel()
    mean_colors = np.stack([np.bincount(inverse, weights=colors[:, k], minlength=len(cells)) for k in range(3)],
                           axis=1) / counts[:, None]
    return (cells + 0.5) * voxel_size, mean_colors


def collision_object(object_id: str, frame_id: str, centers: np.ndarray, voxel_size: float,
                     operation=CollisionObject.ADD) -> CollisionObject:
    """One collision object with a box for each voxel center (ADD replaces an object with the same id)."""
    co = CollisionObject(id=object_id, operation=operation)
    co.header.frame_id = frame_id
    if operation == CollisionObject.ADD:
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(voxel_size)] * 3)
        co.primitives = [box] * len(centers)
        co.primitive_poses = [
            Pose(position=Point(x=float(c[0]), y=float(c[1]), z=float(c[2]))) for c in centers]
        for p in co.primitive_poses:
            p.orientation.w = 1.0
    return co


class VoxelGridService(Node):
    def __init__(self):
        super().__init__('voxel_grid_node')

        self.declare_parameter("pointcloud_topic", "rgbd_pointcloud")
        self.declare_parameter("target_frame", "amiga__base")
        self.declare_parameter("y_abs_min", 0.1)
        self.declare_parameter("y_abs_max", 2.5)
        self.declare_parameter("voxel_object_id", "voxels")
        gp = self.get_parameter
        self.target_frame = str(gp("target_frame").value)
        self.y_abs_min = float(gp("y_abs_min").value)
        self.y_abs_max = float(gp("y_abs_max").value)
        self.voxel_object_id = str(gp("voxel_object_id").value)

        self._srv_cbg = MutuallyExclusiveCallbackGroup()
        self._client_cbg = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self.latest_pointcloud = None
        self.has_voxels = False

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PointCloud2, str(gp("pointcloud_topic").value), self._cloud_cb, latched)
        self.marker_pub = self.create_publisher(MarkerArray, 'voxel_markers', latched)
        self.create_service(VoxelGrid, 'voxel_grid', self.voxel_grid_callback, callback_group=self._srv_cbg)
        self.create_service(Trigger, 'clear_voxels', self.clear_voxels_callback, callback_group=self._srv_cbg)
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene', callback_group=self._client_cbg)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info('VoxelGrid service ready.')

    def _cloud_cb(self, msg: PointCloud2):
        with self._lock:
            self.latest_pointcloud = msg

    # ------------------------------------------------------------------ planning scene / markers

    def _apply(self, co: CollisionObject) -> bool:
        """Apply one collision object change to the planning scene (waits for the response)."""
        if not self.apply_scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/apply_planning_scene is not available.")
            return False
        req = ApplyPlanningScene.Request(scene=PlanningScene(is_diff=True))
        req.scene.world.collision_objects = [co]
        done = threading.Event()
        future = self.apply_scene_client.call_async(req)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=30.0) or future.result() is None or not future.result().success:
            self.get_logger().error("apply_planning_scene failed.")
            return False
        return True

    def _publish_markers(self, centers, colors, voxel_size):
        m = Marker(ns="voxels", id=0)
        m.header.frame_id = self.target_frame
        m.header.stamp = self.get_clock().now().to_msg()
        if len(centers) == 0:
            m.action = Marker.DELETEALL
        else:
            m.type, m.action = Marker.CUBE_LIST, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = float(voxel_size)
            m.points = [Point(x=float(c[0]), y=float(c[1]), z=float(c[2])) for c in centers]
            m.colors = [ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=1.0) for c in colors]
        self.marker_pub.publish(MarkerArray(markers=[m]))

    # ------------------------------------------------------------------ services

    def clear_voxels_callback(self, request, response):
        ok = not self.has_voxels or self._apply(collision_object(self.voxel_object_id, self.target_frame, [], 0.0,
                                          operation=CollisionObject.REMOVE))
        self._publish_markers([], [], 0.0)
        self.has_voxels = False
        response.success = ok
        response.message = "Voxels cleared." if ok else "Could not remove the voxel collision object."
        return response

    def voxel_grid_callback(self, request, response):
        try:
            return self._voxel_grid(request, response)
        except Exception as e:  # log and answer (an exception here would leave the client without a response)
            self.get_logger().error(f"voxel_grid failed: {type(e).__name__}: {e}")
            return response

    def _voxel_grid(self, request, response):
        with self._lock:
            cloud = self.latest_pointcloud
        if cloud is None:
            self.get_logger().error("No point cloud received yet.")
            return response
        try:
            tf = self.tf_buffer.lookup_transform(self.target_frame, cloud.header.frame_id, Time())
        except Exception as e:
            self.get_logger().error(f"Transform lookup {cloud.header.frame_id} -> {self.target_frame} failed: {e}")
            return response

        fields = [f.name for f in cloud.fields]
        # read_points: structured array (the cloud mixes float32 xyz and uint32 rgb)
        data = point_cloud2.read_points(cloud, field_names=[n for n in ("x", "y", "z", "rgb") if n in fields],
                                        skip_nans=True)
        points = np.stack([data["x"], data["y"], data["z"]], axis=1).astype(np.float64)
        if "rgb" in fields:
            rgb = data["rgb"]
            rgb = rgb.view(np.uint32) if rgb.dtype == np.float32 else rgb.astype(np.uint32)
            colors = np.stack([(rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF], axis=1) / 255.0
        else:
            colors = np.full((len(points), 3), 0.5)
        keep = np.isfinite(points).all(axis=1) & (points[:, 2] > 0.01)
        points, colors = points[keep], colors[keep]
        if len(points) == 0:
            self.get_logger().warn("No valid points in the point cloud.")
            return response

        # Into the planning frame, then voxelize there (the boxes are axis-aligned in the planning frame)
        q, t = tf.transform.rotation, tf.transform.translation
        points = Rotation.from_quat([q.x, q.y, q.z, q.w]).apply(points) + [t.x, t.y, t.z]
        centers, vcolors = voxelize(points, colors, request.voxel_size)
        lat = np.abs(centers[:, 1])
        keep = (lat >= self.y_abs_min) & (lat <= self.y_abs_max)
        centers, vcolors = centers[keep], vcolors[keep]
        if len(centers) == 0:
            self.get_logger().warn("Voxelization returned no voxels.")
            return response

        if not self._apply(collision_object(self.voxel_object_id, self.target_frame, centers, request.voxel_size)):
            return response
        self.has_voxels = True
        self._publish_markers(centers, vcolors, request.voxel_size)
        response.voxel_centers = [Point(x=float(c[0]), y=float(c[1]), z=float(c[2])) for c in centers]
        response.voxel_colors = [ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=1.0) for c in vcolors]
        self.get_logger().info(
            f"{len(centers)} voxels ({request.voxel_size} m) from {len(points)} points in collision object "
            f"'{self.voxel_object_id}'.")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VoxelGridService()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
