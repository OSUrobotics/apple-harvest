#!/usr/bin/env python3
"""
apple_prediction.py

/apple_prediction (harvest_interfaces/ApplePrediction): apple centers in target_frame. YOLO masks -> depth
back-projection -> sphere RANSAC.

presaved_images = False: color + depth + camera_info from live topics (camera_ns); each call uses a new synced pair.
presaved_images = True:  the images of one tree (presaved_images.tree_id) of a fast_slam_rgbd_extraction folder
    (presaved_images.rgbd_dir_path): mast_cam for the apples, the base_cam buffer frames for the trunk detector.
    Intrinsics from <cam>/camera_info.yaml; without it, the images are resized to the camera_type intrinsics.
    Latched topics when a tree loads: rgb_image, depth_image, rgbd_pointcloud (mast_cam; empty without mast_cam),
    base_cam/frame_<n>/{rgb,depth}_image, and base_cam/{rgb_image,depth_image,pointcloud} for the frame
    presaved_images.base_cam_frame_index.
"""

import os
from threading import Event, Lock

import cv2
import numpy as np
import open3d as o3d
import rclpy
import torch
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray, PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformListener
from ultralytics import YOLO
from visualization_msgs.msg import Marker, MarkerArray

from harvest_interfaces.srv import ApplePrediction

from .sphere_ransac import Sphere

BASE_CAM_OFFSETS = (-2, -1, 0, 1, 2)       # base_cam buffer frames frame_-2 ... frame_2
BASE_CAM_FRAME = "base_camera_color_optical_frame"
# Intrinsics (target size, fx, fy, cx, cy) for presaved images without camera_info.yaml
FALLBACK_INTRINSICS = {
    "azure": ((1920, 1080), 902.98, 902.77, 956.55, 547.68),
    "realsense": ((848, 480), 609.6989, 609.8549, 420.2079, 235.2782),
}
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
CLOUD_FIELDS = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate("xyz")] + \
               [PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1)]


def load_color_camera_info(cam_dir: str):
    """(fx, fy, cx, cy, width, height) of the color camera in cam_dir/camera_info.yaml, or None."""
    path = os.path.join(cam_dir, "camera_info.yaml")
    if not os.path.exists(path):
        return None
    with open(path, "r") as f:
        info = (yaml.safe_load(f) or {}).get("color")
    if not info:
        return None
    k = info["k"]
    return float(k[0]), float(k[4]), float(k[2]), float(k[5]), int(info["width"]), int(info["height"])


def read_rgbd(img_dir: str):
    """(color BGR, depth, depth is aligned) of img_dir; depth_aligned.png before depth.png. None if missing."""
    color = cv2.imread(os.path.join(img_dir, "color.png"))
    for name, aligned in (("depth_aligned.png", True), ("depth.png", False)):
        path = os.path.join(img_dir, name)
        if color is not None and os.path.exists(path):
            return color, cv2.imread(path, cv2.IMREAD_UNCHANGED), aligned
    return None


def buffer_topic_stem(off: int) -> str:
    """Topic name of a buffer frame: frame_n2 ... frame_2."""
    return f"frame_n{abs(off)}" if off < 0 else f"frame_{off}"


class ApplePredictionNode(Node):

    def __init__(self):
        super().__init__("apple_prediction")
        for name, default in [
            ("presaved_images", False), ("prediction_model_path", "NA"), ("prediction_yolo_conf", 0.85),
            ("prediction_radius_min", 0.03), ("prediction_radius_max", 0.06), ("prediction_distance_max", 1.0),
            ("source_frame", "mast_camera_color_optical_frame"), ("target_frame", "world"),
            ("camera_type", "realsense"), ("ransac_iters", 1000), ("ransac_thresh", 0.005), ("depth_scale", 1000.0),
        ]:
            self.declare_parameter(name, default)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.presaved_images = bool(gp("presaved_images"))
        self.conf_thr = float(gp("prediction_yolo_conf"))
        self.rad_min = float(gp("prediction_radius_min"))
        self.rad_max = float(gp("prediction_radius_max"))
        self.dist_max = float(gp("prediction_distance_max"))
        self.source_frame = str(gp("source_frame"))
        self.target_frame = str(gp("target_frame"))
        self.camera_type = str(gp("camera_type"))
        self.ransac_iters = int(gp("ransac_iters"))
        self.ransac_thresh = float(gp("ransac_thresh"))
        self.depth_scale = float(gp("depth_scale"))

        self.model = YOLO(str(gp("prediction_model_path")))
        self.bridge = CvBridge()
        self._predict_lock = Lock()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(MarkerArray, "apple_markers", 10)
        self.annotated_pub = self.create_publisher(Image, "apple_annotated", 10)
        self.pc_pub = self.create_publisher(PointCloud2, "rgbd_pointcloud", LATCHED)

        if self.presaved_images:
            self._init_presaved()
        else:
            self._init_live()
        self.create_service(ApplePrediction, "apple_prediction", self.on_predict,
                            callback_group=MutuallyExclusiveCallbackGroup())

    # ================================================================== live

    def _init_live(self):
        for name, default in [("camera_ns", "camera/mast_camera"), ("use_aligned_depth", True),
                              ("allow_reuse_latest_frame", False), ("x_tolerance", 0.5)]:
            self.declare_parameter(name, default)
        ns = self.get_parameter("camera_ns").value
        self.use_aligned = bool(self.get_parameter("use_aligned_depth").value)
        self.allow_reuse_latest = bool(self.get_parameter("allow_reuse_latest_frame").value)
        self.x_tol = float(self.get_parameter("x_tolerance").value)
        color_topic = f"/{ns}/color/image_raw"
        depth_topic = f"/{ns}/aligned_depth_to_color/image_raw" if self.use_aligned else f"/{ns}/depth/image_rect_raw"
        cinfo_topic = f"/{ns}/color/camera_info" if self.use_aligned else f"/{ns}/depth/camera_info"

        self._last_cinfo = None
        self._last_pair = None
        self._last_used_stamp = None
        self._new_pair = Event()
        self.create_subscription(CameraInfo, cinfo_topic, self._cinfo_cb, qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer(
            [Subscriber(self, Image, color_topic, qos_profile=qos_profile_sensor_data),
             Subscriber(self, Image, depth_topic, qos_profile=qos_profile_sensor_data)], queue_size=10, slop=0.5)
        self.sync.registerCallback(self._sync_cb)
        self.get_logger().info(f"Live mode: {color_topic}, {depth_topic}, {cinfo_topic}")

    def _cinfo_cb(self, msg: CameraInfo):
        self._last_cinfo = msg

    def _sync_cb(self, color_msg: Image, depth_msg: Image):
        self._last_pair = (color_msg, depth_msg)
        self._new_pair.set()

    def _wait_for_new_pair(self, timeout_sec: float):
        """A synced (color, depth) pair newer than the last used one (or the last one, allow_reuse_latest_frame)."""
        def is_new(pair):
            return pair is not None and (self.allow_reuse_latest or pair[0].header.stamp != self._last_used_stamp)
        if not is_new(self._last_pair):
            self._new_pair.clear()
            if not self._new_pair.wait(timeout=timeout_sec) or not is_new(self._last_pair):
                return None
        self._last_used_stamp = self._last_pair[0].header.stamp
        return self._last_pair

    def _predict_live(self) -> PoseArray:
        if self._last_cinfo is None:
            self.get_logger().warn("No CameraInfo yet.")
            return PoseArray()
        pair = self._wait_for_new_pair(3.0)
        if pair is None:
            self.get_logger().warn("No new synced color/depth pair.")
            return PoseArray()
        color_msg, depth_msg = pair
        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        k = self._last_cinfo.k
        intr = (k[0], k[4], k[2], k[5])
        self._publish_pointcloud(color, depth, intr, self.pc_pub, self.source_frame, color_msg.header.stamp)
        poses, radii, boxes, labels = self._predict(color, depth, intr)
        keep = [i for i, p in enumerate(poses.poses) if abs(p.position.x) <= self.x_tol]   # |x| in target_frame
        poses.poses = [poses.poses[i] for i in keep]
        self._publish_results(color, poses, [radii[i] for i in keep], [boxes[i] for i in keep],
                              [labels[i] for i in keep], color_msg.header)
        return poses

    # ================================================================== presaved

    def _init_presaved(self):
        for name, default in [("presaved_images.rgbd_dir_path", ""), ("presaved_images.tree_id", -1),
                              ("presaved_images.base_cam_frame_index", 0)]:
            self.declare_parameter(name, default)
        if self.camera_type not in FALLBACK_INTRINSICS:
            raise ValueError(f"camera_type must be one of {list(FALLBACK_INTRINSICS)}")
        self._rgbd_dir = str(self.get_parameter("presaved_images.rgbd_dir_path").value)
        self.tree_id = -1
        self.mast = None                   # (color, depth, (fx, fy, cx, cy))
        self.base_frames = {}              # offset -> (color, depth)
        self.base_intrinsics = None        # (fx, fy, cx, cy, W, H)
        self.base_index = 0

        self.rgb_pub = self.create_publisher(Image, "rgb_image", LATCHED)
        self.depth_pub = self.create_publisher(Image, "depth_image", LATCHED)
        self.apple_poses_pub = self.create_publisher(PoseArray, "apple_poses", LATCHED)
        self.base_buffer_pubs = {off: (self.create_publisher(Image, f"base_cam/{buffer_topic_stem(off)}/rgb_image", LATCHED),
                                       self.create_publisher(Image, f"base_cam/{buffer_topic_stem(off)}/depth_image", LATCHED))
                                 for off in BASE_CAM_OFFSETS}
        self.base_rgb_pub = self.create_publisher(Image, "base_cam/rgb_image", LATCHED)
        self.base_depth_pub = self.create_publisher(Image, "base_cam/depth_image", LATCHED)
        self.base_pc_pub = self.create_publisher(PointCloud2, "base_cam/pointcloud", LATCHED)

        tree_id = int(self.get_parameter("presaved_images.tree_id").value)
        if tree_id >= 0:
            self.load_tree(tree_id)
        self.add_on_set_parameters_callback(self._on_params_set)

    def _on_params_set(self, params) -> SetParametersResult:
        try:
            for p in params:
                if p.name == "presaved_images.rgbd_dir_path" and p.value != self._rgbd_dir:
                    self._rgbd_dir = str(p.value)
                    if self.tree_id >= 0:
                        self.load_tree(self.tree_id)
                elif p.name == "presaved_images.tree_id" and int(p.value) != self.tree_id:
                    self.load_tree(int(p.value))
                elif p.name == "presaved_images.base_cam_frame_index":
                    if int(p.value) not in BASE_CAM_OFFSETS:
                        raise ValueError(f"base_cam_frame_index must be one of {BASE_CAM_OFFSETS}")
                    self.base_index = int(p.value)
                    self._publish_base_frame()
        except Exception as e:
            self.get_logger().error(f"{type(e).__name__}: {e}")
            return SetParametersResult(successful=False, reason=str(e))
        return SetParametersResult(successful=True)

    def load_tree(self, tree_id: int):
        """Load and publish the images of one tree (tree_id < 0: no tree). mast_cam is optional."""
        self.tree_id, self.mast, self.base_frames, self.base_index = tree_id, None, {}, 0
        if tree_id < 0:
            return
        if not self._rgbd_dir:
            raise FileNotFoundError("presaved_images.rgbd_dir_path is empty")
        tree_dir = os.path.join(self._rgbd_dir, f"tree_{tree_id:03d}")

        mast_dir = os.path.join(tree_dir, "mast_cam")
        rgbd = read_rgbd(mast_dir)
        if rgbd is None:
            self.get_logger().warn(f"No mast_cam images in {tree_dir}: no apple prediction for this tree.")
        else:
            color, depth, aligned = rgbd
            cinfo = load_color_camera_info(mast_dir)
            if cinfo is not None and (cinfo[4], cinfo[5]) == (color.shape[1], color.shape[0]):
                if depth.shape[:2] != color.shape[:2]:
                    raise ValueError(f"mast_cam depth {depth.shape[:2]} is not the color size {color.shape[:2]}")
                if not aligned:
                    self.get_logger().warn("mast_cam: no depth_aligned.png; depth.png (color size) is used.")
                self.mast = (color, depth, cinfo[:4])
            else:
                size, fx, fy, cx, cy = FALLBACK_INTRINSICS[self.camera_type]
                self.get_logger().warn(f"mast_cam: no camera_info.yaml; images resized to {size}, "
                                       f"'{self.camera_type}' intrinsics.")
                self.mast = (cv2.resize(color, size, interpolation=cv2.INTER_LINEAR),
                             cv2.resize(depth, size, interpolation=cv2.INTER_NEAREST), (fx, fy, cx, cy))

        base_dir = os.path.join(tree_dir, "base_cam")
        self.base_intrinsics = load_color_camera_info(base_dir)
        for off in BASE_CAM_OFFSETS:
            rgbd = read_rgbd(os.path.join(base_dir, f"frame_{off}"))
            if rgbd is None:
                self.get_logger().warn(f"base_cam frame_{off}: missing")
                continue
            if not rgbd[2]:
                self.get_logger().warn(f"base_cam frame_{off}: no depth_aligned.png; depth is not registered to color.")
            self.base_frames[off] = rgbd[:2]
        self.get_logger().info(f"Tree {tree_id}: mast_cam {'yes' if self.mast else 'no'}, "
                               f"base_cam frames {sorted(self.base_frames)}")
        self._publish_tree()

    def _image_msgs(self, color, depth, frame_id, stamp):
        rgb_msg = self.bridge.cv2_to_imgmsg(color, encoding="bgr8")
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="mono16")
        rgb_msg.header = depth_msg.header = Header(stamp=stamp, frame_id=frame_id)
        return rgb_msg, depth_msg

    def _publish_tree(self):
        stamp = self.get_clock().now().to_msg()
        if self.mast is not None:
            color, depth, intr = self.mast
            for pub, msg in zip((self.rgb_pub, self.depth_pub), self._image_msgs(color, depth, self.source_frame, stamp)):
                pub.publish(msg)
            self._publish_pointcloud(color, depth, intr, self.pc_pub, self.source_frame, stamp)
        else:
            # Empty cloud: voxelize_scan does not use the cloud of a different tree
            self.pc_pub.publish(point_cloud2.create_cloud(Header(stamp=stamp, frame_id=self.source_frame),
                                                          CLOUD_FIELDS, []))
        for off, (color, depth) in self.base_frames.items():
            for pub, msg in zip(self.base_buffer_pubs[off], self._image_msgs(color, depth, BASE_CAM_FRAME, stamp)):
                pub.publish(msg)
        self._publish_base_frame()

    def _publish_base_frame(self):
        """Publish the base_cam frame base_index on base_cam/rgb_image, depth_image and pointcloud."""
        if self.base_index not in self.base_frames:
            self.get_logger().warn(f"base_cam frame_{self.base_index} not loaded")
            return
        color, depth = self.base_frames[self.base_index]
        stamp = self.get_clock().now().to_msg()
        rgb_msg, depth_msg = self._image_msgs(color, depth, BASE_CAM_FRAME, stamp)
        self.base_rgb_pub.publish(rgb_msg)
        self.base_depth_pub.publish(depth_msg)
        ci = self.base_intrinsics
        if ci is not None and depth.shape[:2] == color.shape[:2] == (ci[5], ci[4]):
            self._publish_pointcloud(color, depth, ci[:4], self.base_pc_pub, BASE_CAM_FRAME, stamp)

    def _predict_presaved(self) -> PoseArray:
        if self.mast is None:
            self.get_logger().error("No mast_cam images for this tree.")
            return PoseArray()
        color, depth, intr = self.mast
        poses, radii, boxes, labels = self._predict(color, depth, intr)
        self._publish_results(color, poses, radii, boxes, labels, None)
        self.apple_poses_pub.publish(poses)
        return poses

    # ================================================================== prediction

    def on_predict(self, req, res):
        with self._predict_lock:
            res.apple_poses = self._predict_presaved() if self.presaved_images else self._predict_live()
        self.get_logger().info(f"{len(res.apple_poses.poses)} apple(s)")
        return res

    def _predict(self, color, depth, intr):
        """YOLO + sphere fit. Returns (PoseArray in target_frame, radii, boxes, labels)."""
        with torch.inference_mode():
            results = self.model(color, conf=self.conf_thr, verbose=False)[0]
        masks, boxes, confs = self._instance_masks(results, *color.shape[:2])
        centers, radii, kept = self._fit_spheres(depth, masks, intr)
        poses = self._to_pose_array(centers)
        if not poses.header.frame_id:          # TF failure: no poses
            return poses, [], [], []
        return poses, radii, [boxes[i] for i in kept], [f"apple {confs[i]:.2f}" for i in kept]

    @staticmethod
    def _instance_masks(results, H, W):
        """(masks, boxes xyxy, confidences) of the YOLO instances; box masks when the model has no masks."""
        if results.boxes is None or len(results.boxes) == 0:
            return [], [], []
        boxes = results.boxes.xyxy.cpu().numpy().astype(int)
        confs = results.boxes.conf.cpu().numpy().astype(float)
        polys = results.masks.xy if getattr(results, "masks", None) is not None else None
        masks = []
        for i, (x1, y1, x2, y2) in enumerate(boxes):
            m = np.zeros((H, W), dtype=np.uint8)
            if polys is not None:
                cv2.fillPoly(m, [polys[i].astype(np.int32)], 255)
            else:
                cv2.rectangle(m, (max(0, x1), max(0, y1)), (min(W - 1, x2), min(H - 1, y2)), 255, -1)
            masks.append(m)
        keep = [i for i, m in enumerate(masks) if m.any()]
        return [masks[i] for i in keep], [tuple(boxes[i]) for i in keep], [confs[i] for i in keep]

    def _fit_spheres(self, depth, masks, intr):
        """Sphere RANSAC on the back-projected depth of each mask. Returns (centers, radii, mask indices)."""
        fx, fy, cx, cy = intr
        centers, radii, kept = [], [], []
        for i, m in enumerate(masks):
            if m.shape != depth.shape[:2]:
                m = cv2.resize(m, depth.shape[1::-1], interpolation=cv2.INTER_NEAREST)
            ys, xs = np.nonzero(m)
            z = depth[ys, xs].astype(np.float64) / self.depth_scale
            ok = (z > 0) & (z <= self.dist_max)
            if ok.sum() < 50 or np.median(z[ok]) > self.dist_max:
                continue
            xs, ys, z = xs[ok], ys[ok], z[ok]
            pts = np.column_stack([(xs - cx) * z / fx, (ys - cy) * z / fy, z])
            c, r, _ = Sphere().fit(pts, thresh=self.ransac_thresh, maxIteration=self.ransac_iters,
                                   lower_rad_bound=self.rad_min, upper_rad_bound=self.rad_max)
            if c is None or r is None or not np.isfinite(c).all() or not np.isfinite(r):
                continue
            centers.append(c)
            radii.append(float(r))
            kept.append(i)
        return centers, radii, kept

    def _to_pose_array(self, centers) -> PoseArray:
        """Centers (source_frame) as a PoseArray in target_frame. TF failure: empty PoseArray, frame_id ""."""
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = self.target_frame
        tf = None
        if self.source_frame != self.target_frame:
            try:
                tf = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame, rclpy.time.Time(),
                                                     timeout=Duration(seconds=1.0))
            except Exception as e:
                self.get_logger().error(f"TF {self.source_frame} -> {self.target_frame}: {e}. No apple poses.")
                pa.header.frame_id = ""
                return pa
        for c in centers:
            ps = PoseStamped()
            ps.header.frame_id = self.source_frame
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(v) for v in c)
            ps.pose.orientation.w = 1.0
            pa.poses.append(ps.pose if tf is None else do_transform_pose_stamped(ps, tf).pose)
        return pa

    # ================================================================== output

    def _publish_results(self, color, poses, radii, boxes, labels, header):
        """Apple markers and the annotated image."""
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for i, (pose, r) in enumerate(zip(poses.poses, radii)):
            m = Marker(id=i, type=Marker.SPHERE, action=Marker.ADD, pose=pose)
            m.header.frame_id, m.header.stamp = self.target_frame, stamp
            m.scale.x = m.scale.y = m.scale.z = 2.0 * r
            m.color.r = m.color.a = 1.0
            markers.markers.append(m)
        self.marker_pub.publish(markers)

        img = color.copy()
        for (x1, y1, x2, y2), label in zip(boxes, labels):
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            (tw, th), base = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(img, (x1, max(0, y1 - th - base - 6)), (x1 + tw + 8, y1), (0, 255, 0), -1)
            cv2.putText(img, label, (x1 + 4, max(0, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        msg.header = header if header is not None else Header(stamp=stamp, frame_id=self.source_frame)
        self.annotated_pub.publish(msg)

    def _publish_pointcloud(self, color_bgr, depth, intr, publisher, frame_id, stamp):
        """XYZRGB cloud of an RGB-D image (rgb field: (r << 16) | (g << 8) | b; depth up to prediction_distance_max)."""
        H, W = color_bgr.shape[:2]
        fx, fy, cx, cy = intr
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)),
            o3d.geometry.Image(depth.astype(np.uint16)),
            depth_scale=self.depth_scale, depth_trunc=self.dist_max, convert_rgb_to_intensity=False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd, o3d.camera.PinholeCameraIntrinsic(W, H, fx, fy, cx, cy))
        pts = np.asarray(pcd.points)
        rgb = (np.asarray(pcd.colors) * 255).astype(np.uint32)
        cloud = np.zeros(len(pts), dtype=point_cloud2.dtype_from_fields(CLOUD_FIELDS))
        cloud["x"], cloud["y"], cloud["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
        cloud["rgb"] = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]
        publisher.publish(point_cloud2.create_cloud(Header(stamp=stamp, frame_id=frame_id), CLOUD_FIELDS, cloud))


def main():
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(ApplePredictionNode())
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
