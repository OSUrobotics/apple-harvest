#!/usr/bin/env python3

from threading import Event, Lock
import open3d as o3d
import numpy as np
import cv2
import torch

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import (
    qos_profile_sensor_data, QoSProfile,
    ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, PoseArray
from harvest_interfaces.srv import ApplePrediction

from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformListener

from cv_bridge import CvBridge
from ultralytics import YOLO
from message_filters import ApproximateTimeSynchronizer, Subscriber

from .sphere_ransac import Sphere


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 10**9 + int(stamp.nanosec)


class ApplePredictionNode(Node):
    """
    Apple prediction node that supports two operating modes:

    presaved_images = False  (default)
        Subscribes to live color + depth ROS topics, waits for a fresh
        synced pair on each service call, then runs YOLO → back-projection
        RANSAC to estimate apple positions.

    presaved_images = True
        Loads a pre-saved color and depth image from disk at startup.
        Camera intrinsics are taken from hard-coded values selected by the
        ``camera_type`` parameter ("azure" or "realsense").  The inference
        pipeline uses open3d RGBD → point-cloud → RANSAC.  Image and
        point-cloud publishers fire on timers so downstream nodes can
        visualise the static data.
    """

    # ------------------------------------------------------------------ init

    def __init__(self):
        super().__init__("apple_prediction_node")

        # ---- common parameters ----
        self.declare_parameter("presaved_images", False)
        self.declare_parameter("prediction_model_path", "NA")
        self.declare_parameter("prediction_yolo_conf", 0.85)
        self.declare_parameter("prediction_radius_min", 0.03)
        self.declare_parameter("prediction_radius_max", 0.06)
        self.declare_parameter("prediction_distance_max", 1.0)
        self.declare_parameter("source_frame", "mast_camera_color_optical_frame")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("camera_type", "realsense")
        self.declare_parameter("ransac_iters", 1000)
        self.declare_parameter("ransac_thresh", 0.005)
        self.declare_parameter("depth_scale", 1000.0)
        self.declare_parameter("pointcloud_offset", "(0.0, 0.0, 0.0)")

        self.presaved_images = bool(self.get_parameter("presaved_images").value)
        self.conf_thr   = float(self.get_parameter("prediction_yolo_conf").value)
        self.rad_min    = float(self.get_parameter("prediction_radius_min").value)
        self.rad_max    = float(self.get_parameter("prediction_radius_max").value)
        self.dist_max   = float(self.get_parameter("prediction_distance_max").value)
        self.source_frame = self.get_parameter("source_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.camera_type = self.get_parameter("camera_type").get_parameter_value().string_value
        self.ransac_iters = int(self.get_parameter("ransac_iters").value)
        self.ransac_thresh = float(self.get_parameter("ransac_thresh").value)
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        xyz_str = self.get_parameter("pointcloud_offset").value
        xyz = xyz_str[1:-1].split(",")

        self.pointcloud_offset = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
        self.get_logger().info(f"xyz offset {self.pointcloud_offset}")

        # ---- YOLO ----
        self.model = YOLO(self.get_parameter("prediction_model_path").value)
        try:
            self.model.model.eval()
        except Exception:
            pass

        # ---- shared I/O ----
        self.bridge = CvBridge()
        self._predict_lock = Lock()
        self.service_group = MutuallyExclusiveCallbackGroup()

        self.marker_pub = self.create_publisher(MarkerArray, "apple_markers", 10)
        self.annotated_pub = self.create_publisher(
            Image, "apple_annotated",
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=10),
        )
        self.pc_pub = self.create_publisher(PointCloud2, "rgbd_pointcloud", 10)

        # ---- TF ----
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- mode-specific setup ----
        if self.presaved_images:
            self._init_presaved()
        else:
            self._init_live()

    # ------------------------------------------------------------------ live init

    def _init_live(self):
        """Set up live-camera subscriptions and sync."""
        self.declare_parameter("camera_ns", "camera/mast_camera")
        self.declare_parameter("use_aligned_depth", True)
        self.declare_parameter("allow_reuse_latest_frame", False)
        self.declare_parameter("x_tolerance", 0.5)

        self.ns             = self.get_parameter("camera_ns").value
        self.use_aligned    = bool(self.get_parameter("use_aligned_depth").value)
        self.allow_reuse_latest = bool(self.get_parameter("allow_reuse_latest_frame").value)
        self.x_tol          = float(self.get_parameter("x_tolerance").value)

        self.color_topic = f"/{self.ns}/color/image_raw"
        if self.use_aligned:
            self.depth_topic = f"/{self.ns}/aligned_depth_to_color/image_raw"
            self.cinfo_topic = f"/{self.ns}/color/camera_info"
        else:
            self.depth_topic = f"/{self.ns}/depth/image_rect_raw"
            self.cinfo_topic = f"/{self.ns}/depth/camera_info"

        self._last_cinfo = None
        self.cinfo_sub = self.create_subscription(
            CameraInfo, self.cinfo_topic, self._cinfo_cb,
            qos_profile=qos_profile_sensor_data,
        )

        self.color_sub = Subscriber(self, Image, self.color_topic,
                                    qos_profile=qos_profile_sensor_data)
        self.depth_sub = Subscriber(self, Image, self.depth_topic,
                                    qos_profile=qos_profile_sensor_data)

        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=0.50
        )
        self.sync.registerCallback(self._sync_cd_cb)

        self._last_pair          = None
        self._new_pair_event     = Event()
        self._last_used_pair_key = None

        self.srv = self.create_service(
            ApplePrediction, "apple_prediction",
            self.on_predict, callback_group=self.service_group,
        )

        self.get_logger().info(
            "ApplePredictionNode (live) ready.\n"
            f"  color: {self.color_topic}\n"
            f"  depth: {self.depth_topic}\n"
            f"  cinfo: {self.cinfo_topic}"
        )

    # ------------------------------------------------------------------ presaved init

    def _init_presaved(self):
        """Set up presaved-image mode: load intrinsics, images, timers, publishers."""

        self.declare_parameter("presaved_images.rgb_image_path", "")
        self.declare_parameter("presaved_images.depth_image_path", "")

        # ---- camera intrinsics ----
        if self.camera_type == "azure":
            self.target_size = (1920, 1080)
            _k = [902.98, 0.0, 956.55, 0.0, 902.77, 547.68, 0.0, 0.0, 1.0]
            self.fx, self.fy = _k[0], _k[4]
            self.cx, self.cy = _k[2], _k[5]
        elif self.camera_type == "realsense":
            self.target_size = (848, 480)
            self.fx, self.fy = 609.6989, 609.8549
            self.cx, self.cy = 420.2079, 235.2782
        else:
            self.get_logger().error(
                f"Invalid camera_type '{self.camera_type}'. Must be 'azure' or 'realsense'."
            )
            raise ValueError(f"Unknown camera_type: {self.camera_type}")

        self.get_logger().info(
            f"Presaved mode | camera={self.camera_type} | "
            f"size={self.target_size} | "
            f"fx={self.fx} fy={self.fy} cx={self.cx} cy={self.cy}"
        )

        # ---- load images ----
        rgb_path   = self.get_parameter("presaved_images.rgb_image_path").get_parameter_value().string_value
        depth_path = self.get_parameter("presaved_images.depth_image_path").get_parameter_value().string_value

        self.rgb_image   = cv2.imread(rgb_path)
        self.depth_image = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)

        if self.rgb_image is None:
            raise FileNotFoundError(f"Could not load RGB image: {rgb_path}")
        if self.depth_image is None:
            raise FileNotFoundError(f"Could not load depth image: {depth_path}")

        self.rgb_image   = cv2.resize(self.rgb_image,   self.target_size, interpolation=cv2.INTER_LINEAR)
        self.depth_image = cv2.resize(self.depth_image, self.target_size, interpolation=cv2.INTER_NEAREST)

        self.get_logger().info(
            f"Loaded RGB: dtype={self.rgb_image.dtype} shape={self.rgb_image.shape} | "
            f"Depth: dtype={self.depth_image.dtype} min={self.depth_image.min()} "
            f"max={self.depth_image.max()} shape={self.depth_image.shape}"
        )

        # ---- extra publishers (presaved only) ----
        self.rgb_pub   = self.create_publisher(Image, "rgb_image", 10)
        self.depth_pub = self.create_publisher(Image, "depth_image", 10)

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.apple_poses_pub = self.create_publisher(PoseArray, "apple_poses", latched_qos)

        # ---- timers ----
        self.create_timer(0.1, self._presaved_publish_images)
        self.create_timer(0.5, self._presaved_publish_pointcloud)

        # ---- misc state ----
        self._presaved_apple_centers = None
        self._presaved_apple_radii   = None
        self._presaved_yolo_result   = None
        self._marker_counter         = 0

        # ---- service ----
        self.srv = self.create_service(
            ApplePrediction, "apple_prediction",
            self.on_predict, callback_group=self.service_group,
        )

        self.get_logger().info("ApplePredictionNode (presaved) ready.")

    # ================================================================== service

    def on_predict(self, req, res):
        with self._predict_lock:
            if self.presaved_images:
                return self._on_predict_presaved(req, res)
            else:
                return self._on_predict_live(req, res)

    # ------------------------------------------------------------------ live predict

    def _on_predict_live(self, req, res):
        if self._last_cinfo is None:
            self.get_logger().warn("No CameraInfo received yet.")
            res.apple_poses = PoseArray()
            return res

        pair = self._wait_for_new_synced(timeout_sec=3.0)
        if pair is None:
            self.get_logger().warn("Timed out waiting for fresh synced frames.")
            res.apple_poses = PoseArray()
            return res

        color_msg, depth_msg = pair
        cinfo_msg = self._last_cinfo

        self._last_used_pair_key = (
            stamp_to_ns(color_msg.header.stamp),
            stamp_to_ns(depth_msg.header.stamp),
        )

        color_bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        depth_mm  = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        Hc, Wc = color_bgr.shape[:2]
        if (cinfo_msg.height != Hc) or (cinfo_msg.width != Wc):
            self.get_logger().warn(
                f"CameraInfo size ({cinfo_msg.width}x{cinfo_msg.height}) "
                f"!= color size ({Wc}x{Hc}). Using color size for back-projection."
            )
        if self.use_aligned and (depth_mm.shape[:2] != (Hc, Wc)):
            self.get_logger().warn(
                f"Aligned depth size {depth_mm.shape[1]}x{depth_mm.shape[0]} "
                f"!= color size {Wc}x{Hc}; masks will be resized."
            )

        fx = float(cinfo_msg.k[0]); fy = float(cinfo_msg.k[4])
        cx = float(cinfo_msg.k[2]); cy = float(cinfo_msg.k[5])

        # Publish full RGBD point cloud
        self._publish_pointcloud(
            rgb_bgr=color_bgr,
            depth=depth_mm,
            fx=fx, fy=fy, cx=cx, cy=cy,
            stamp=color_msg.header.stamp,
        )

        # 1) YOLO
        with torch.inference_mode():
            results = self.model(color_bgr, conf=self.conf_thr, verbose=False)[0]

        # 2) Masks & boxes
        masks, bboxes = self._build_instance_masks_and_boxes(results, Hc, Wc)

        # 3) Back-project → RANSAC sphere fit
        centers, radii, kept_bboxes = self._estimate_apples_backproject(
            depth_mm, masks, bboxes, fx, fy, cx, cy
        )

        self.get_logger().info(
            f"Live predict ts=({color_msg.header.stamp.sec}."
            f"{color_msg.header.stamp.nanosec:09d}) → {len(centers)} detections."
        )

        # 4) Transform to target frame
        poses_world = self._to_pose_array(centers, self.source_frame, self.target_frame)

        # 5) Filter by |x| in target frame
        poses_world_f, radii_f, bboxes_f, _ = self._filter_by_x_range(
            poses_world, radii, kept_bboxes, self.x_tol
        )
        if len(poses_world.poses) != len(poses_world_f.poses):
            self.get_logger().info(
                f"Filtered {len(poses_world.poses) - len(poses_world_f.poses)} detections "
                f"by |x| <= {self.x_tol} in '{self.target_frame}'. "
                f"Kept {len(poses_world_f.poses)}."
            )

        # 6) Publish
        self._publish_markers(poses_world_f, radii_f, frame_id=self.target_frame)
        self._publish_annotated_image(color_bgr, bboxes_f, header=color_msg.header)

        res.apple_poses = poses_world_f
        return res

    # ------------------------------------------------------------------ presaved predict

    def _on_predict_presaved(self, req, res):
        # 1) Segment with YOLO
        apple_masks = self._presaved_segment_apples(self.rgb_image)
        self.get_logger().info(f"[1] YOLO found {len(apple_masks)} apple mask(s)")

        resized_masks = [
            cv2.resize(m, self.target_size, interpolation=cv2.INTER_NEAREST)
            for m in apple_masks
        ]

        # 2) Build point clouds per mask → RANSAC sphere fit
        rgb_rgb = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB)
        centers, radii = self._presaved_get_apple_centers(rgb_rgb, self.depth_image, resized_masks)
        self.get_logger().info(f"[2] get_apple_centers returned {len(centers)} center(s)")

        # 3) Transform to target frame
        poses_world = self._to_pose_array(centers, self.source_frame, self.target_frame)
        self.get_logger().info(f"[3] transform_apple_poses returned {len(poses_world.poses)} pose(s)")

        # 4) Publish
        self._publish_markers(poses_world, radii, frame_id=self.target_frame)

        # Extract bboxes from the stored YOLO result for the annotated image
        bboxes = []
        if self._presaved_yolo_result is not None \
                and self._presaved_yolo_result.boxes is not None:
            bboxes = [
                tuple(box.xyxy.cpu().numpy()[0].astype(int))
                for box in self._presaved_yolo_result.boxes
            ]
        self._publish_annotated_image(self.rgb_image, bboxes)

        # Latched poses topic
        poses_world.header.stamp    = self.get_clock().now().to_msg()
        poses_world.header.frame_id = self.target_frame
        self.apple_poses_pub.publish(poses_world)
        self.get_logger().info(
            f"Published {len(poses_world.poses)} apple pose(s) to 'apple_poses'."
        )

        # Cache for timer-based republishing
        self._presaved_apple_centers = poses_world
        self._presaved_apple_radii   = radii

        res.apple_poses = poses_world
        return res

    # ================================================================== presaved helpers

    def _presaved_segment_apples(self, image):
        """Run YOLO on *image* and return a list of per-apple uint8 masks."""
        results = self.model(image, conf=self.conf_thr)[0]
        self._presaved_yolo_result = results
        apple_masks = []

        for det in results:
            if results.masks is not None:
                img_h, img_w = results.masks.orig_shape
                mask = np.zeros((img_h, img_w), dtype=np.uint8)
                for poly in det.masks.xy:
                    cv2.fillPoly(mask, [poly.astype(np.int32)], 255)
            else:
                img_h, img_w = results.orig_shape
                mask = np.zeros((img_h, img_w), dtype=np.uint8)
                x1, y1, x2, y2 = det.boxes.xyxy.cpu().numpy()[0]
                cv2.rectangle(mask, (int(x1), int(y1)), (int(x2), int(y2)), 255, -1)
            apple_masks.append(mask)

        return apple_masks

    def _presaved_get_apple_centers(self, rgb, depth, masks):
        """
        For each mask: build an open3d RGBD point cloud, run RANSAC sphere fit.
        Returns (centers, radii) lists.
        """
        self.get_logger().info(
            f"Depth dtype={depth.dtype} min={depth.min()} max={depth.max()} "
            f"nonzero={np.count_nonzero(depth)} shape={depth.shape}"
        )
        apple_centers = []
        apple_radii   = []

        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=self.target_size[0], height=self.target_size[1],
            fx=self.fx, fy=self.fy, cx=self.cx, cy=self.cy,
        )

        for mask in masks:
            depth_seg = np.where(mask.astype(bool), depth, 0).astype(np.uint16)
            valid_vals = depth_seg[depth_seg > 0]
            if valid_vals.size == 0:
                continue
            if np.median(valid_vals) > self.dist_max * self.depth_scale:
                continue

            rgb_o3d   = o3d.geometry.Image(rgb)
            depth_o3d = o3d.geometry.Image(depth_seg.astype(np.uint16))
            rgbd      = o3d.geometry.RGBDImage.create_from_color_and_depth(
                rgb_o3d, depth_o3d,
                depth_scale=self.depth_scale,
                depth_trunc=self.dist_max,
                convert_rgb_to_intensity=False,
            )
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)

            pts = np.asarray(pcd.points)
            if pts.shape[0] < 50:
                continue

            center, radius = self._presaved_ransac_sphere(pts)
            if center is not None and radius is not None:
                apple_centers.append(center)
                apple_radii.append(float(radius))

        return apple_centers, apple_radii

    def _presaved_ransac_sphere(self, pts):
        sph = Sphere()
        center, radius, _ = sph.fit(
            pts,
            thresh=self.ransac_thresh,
            maxIteration=self.ransac_iters,
            lower_rad_bound=self.rad_min,
            upper_rad_bound=self.rad_max,
        )
        return center, radius

    def _presaved_publish_images(self):
        """Timer callback: republish the loaded RGB and depth images at 10 Hz."""
        if self._predict_lock.locked():
            return  # service callback is running — skip this cycle
        rgb_msg   = self.bridge.cv2_to_imgmsg(self.rgb_image,   encoding="bgr8")
        depth_msg = self.bridge.cv2_to_imgmsg(self.depth_image, encoding="mono16")
        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)

    def _publish_pointcloud(self, rgb_bgr, depth, fx, fy, cx, cy, stamp=None):
        """Build and publish an XYZRGB PointCloud2 from a BGR image and depth map.

        Works for both live and presaved modes.  *stamp* defaults to now when
        not supplied (presaved / timer path).  The image is converted BGR→RGB
        internally so callers can pass the raw OpenCV frame either way.

        Args:
            rgb_bgr:  uint8 BGR image (H×W×3).
            depth:    uint16 depth image in the node's depth_scale units.
            fx, fy:   focal lengths in pixels.
            cx, cy:   principal point in pixels.
            stamp:    rospy/rclpy stamp to embed in the header; uses clock.now() if None.
        """
        H, W = rgb_bgr.shape[:2]
        if self.presaved_images:
            rgb_o3d   = o3d.geometry.Image(rgb_bgr)
        else:
            rgb_o3d   = o3d.geometry.Image(cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB))
        depth_o3d = o3d.geometry.Image(depth.astype(np.uint16))
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=W, height=H, fx=fx, fy=fy, cx=cx, cy=cy,
        )
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            rgb_o3d, depth_o3d,
            depth_scale=self.depth_scale,
            depth_trunc=self.dist_max,
            convert_rgb_to_intensity=False,
        )
        pcd    = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
        points = np.asarray(pcd.points)
        colors = (np.asarray(pcd.colors) * 255).astype(np.uint8)

        packed = [
            (x + self.pointcloud_offset[0], y + self.pointcloud_offset[1], z + self.pointcloud_offset[2], int(b) | (int(g) << 8) | (int(r) << 16))
            for (x, y, z), (b, g, r) in zip(points, colors)
        ]
        header          = Header()
        header.stamp    = stamp if stamp is not None else self.get_clock().now().to_msg()
        header.frame_id = self.source_frame
        fields = [
            PointField(name="x",   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y",   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z",   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32,  count=1),
        ]

        self.pc_pub.publish(point_cloud2.create_cloud(header, fields, packed))        
        
    def _presaved_publish_pointcloud(self):
        """Timer callback: republish the full RGBD point cloud at 2 Hz."""
        if self._predict_lock.locked():
            return  # service callback is running — skip this cycle
        self._publish_pointcloud(
            rgb_bgr=self.rgb_image,
            depth=self.depth_image,
            fx=self.fx, fy=self.fy, cx=self.cx, cy=self.cy,
        )

    def _publish_annotated_image(self, bgr_img, bboxes, header=None):
        """Draw bounding boxes on bgr_img and publish to apple_annotated.

        In presaved mode, labels show confidence scores pulled from the stored
        YOLO result.  In live mode, labels show the detection index.
        """
        if bgr_img is None:
            return
        annotated = bgr_img.copy()
        H, W = annotated.shape[:2]

        # Pull per-box confidence scores in presaved mode
        confidences = []
        if self.presaved_images \
                and self._presaved_yolo_result is not None \
                and self._presaved_yolo_result.boxes is not None:
            confidences = [
                float(box.conf.cpu().numpy()[0])
                for box in self._presaved_yolo_result.boxes
            ]

        for i, (x1, y1, x2, y2) in enumerate(bboxes):
            x1 = int(max(0, min(W - 1, x1))); y1 = int(max(0, min(H - 1, y1)))
            x2 = int(max(0, min(W - 1, x2))); y2 = int(max(0, min(H - 1, y2)))
            if x2 <= x1 or y2 <= y1:
                continue

            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

            if self.presaved_images and i < len(confidences):
                label       = f"apple {confidences[i]:.2f}"
                font_scale  = 0.5
                thickness   = 1
            else:
                label       = str(i)
                font_scale  = 0.9
                thickness   = 2

            (tw, th), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness
            )
            bg_y1 = max(0, y1 - th - baseline - 6)
            cv2.rectangle(annotated,
                          (x1, bg_y1),
                          (min(W - 1, x1 + tw + 8), y1),
                          (0, 255, 0), -1)
            cv2.putText(annotated, label, (x1 + 4, max(0, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0),
                        thickness, cv2.LINE_AA)

        msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        if header is not None:
            msg.header = header
        else:
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = self.source_frame
        self.annotated_pub.publish(msg)

    def _build_instance_masks_and_boxes(self, results, H, W):
        inst_n = int(len(results.boxes) if results.boxes is not None else 0)
        if inst_n == 0:
            return [], []

        masks  = [None] * inst_n
        bboxes = [None] * inst_n
        xyxy   = results.boxes.xyxy.cpu().numpy().astype(int)

        if getattr(results, "masks", None) is not None and \
                getattr(results.masks, "xy", None) is not None:
            polys = results.masks.xy
            for i in range(inst_n):
                mask = np.zeros((H, W), dtype=np.uint8)
                pl   = polys[i]
                if isinstance(pl, np.ndarray):
                    cv2.fillPoly(mask, [pl.astype(np.int32)], 255)
                else:
                    for poly in pl:
                        cv2.fillPoly(mask, [poly.astype(np.int32)], 255)
                masks[i]  = mask
                x1, y1, x2, y2 = xyxy[i]
                bboxes[i] = (x1, y1, x2, y2)
        else:
            for i in range(inst_n):
                x1, y1, x2, y2 = xyxy[i]
                mask = np.zeros((H, W), dtype=np.uint8)
                cv2.rectangle(mask,
                              (max(0, x1), max(0, y1)),
                              (min(W - 1, x2), min(H - 1, y2)), 255, -1)
                masks[i]  = mask
                bboxes[i] = (x1, y1, x2, y2)

        pairs = [(m, b) for m, b in zip(masks, bboxes) if m is not None and m.any()]
        if not pairs:
            return [], []
        masks, bboxes = zip(*pairs)
        return list(masks), list(bboxes)

    def _estimate_apples_backproject(self, depth_mm, masks, bboxes, fx, fy, cx, cy):
        """Back-project depth pixels to 3-D then fit a sphere with RANSAC."""
        centers, radii, kept_bboxes = [], [], []
        Hd, Wd = depth_mm.shape[:2]

        for m, bbox in zip(masks, bboxes):
            if m.shape != depth_mm.shape:
                m = cv2.resize(m, (Wd, Hd), interpolation=cv2.INTER_NEAREST)

            ys, xs = np.where(m > 0)
            if xs.size == 0:
                continue
            z = depth_mm[ys, xs].astype(np.float32) / 1000.0
            valid = z > 0
            if not np.any(valid):
                continue
            xs, ys, z = xs[valid], ys[valid], z[valid]
            if z.size < 50:
                continue
            if float(np.median(z)) > self.dist_max:
                continue

            X   = (xs - cx) * z / fx
            Y   = (ys - cy) * z / fy
            pts = np.column_stack((X, Y, z))

            fitter = Sphere()
            c, r, ok = fitter.fit(
                pts,
                thresh=self.ransac_thresh,
                maxIteration=self.ransac_iters,
                lower_rad_bound=self.rad_min,
                upper_rad_bound=self.rad_max,
            )
            if ok is False or c is None or r is None \
                    or not np.isfinite(c).all() or not np.isfinite(r):
                continue

            centers.append(c)
            radii.append(float(r))
            kept_bboxes.append(tuple(bbox))

        return centers, radii, kept_bboxes

    def _to_pose_array(self, centers_xyz, source_frame, target_frame):
        pa  = PoseArray()
        now = self.get_clock().now().to_msg()

        # If source == target, skip the TF lookup entirely (common in presaved/offline mode).
        skip_tf = (source_frame == target_frame)
        tf = None

        if not skip_tf:
            try:
                tf = self.tf_buffer.lookup_transform(
                    target_frame, source_frame,
                    rclpy.time.Time(), timeout=Duration(seconds=1.0),
                )
            except Exception as e:
                # Catch all exceptions (TransformException, LookupException, etc.)
                # so a missing TF in offline/presaved mode doesn't silently kill the callback.
                self.get_logger().warn(
                    f"TF lookup failed {source_frame}→{target_frame}: {type(e).__name__}: {e}. "
                    f"Returning poses in source frame '{source_frame}' instead."
                )
                skip_tf = True

        for c in centers_xyz:
            src = PoseStamped()
            src.header.frame_id    = source_frame
            src.header.stamp       = now
            src.pose.position.x    = float(c[0])
            src.pose.position.y    = float(c[1])
            src.pose.position.z    = float(c[2])
            src.pose.orientation.w = 1.0

            if skip_tf:
                pa.poses.append(src.pose)
            else:
                pa.poses.append(do_transform_pose_stamped(src, tf).pose)

        return pa

    def _filter_by_x_range(self, poses_world: PoseArray, radii, bboxes, x_abs_max: float):
        kept_indices = []
        out = PoseArray()
        out.header = poses_world.header

        for i, p in enumerate(poses_world.poses):
            if abs(float(p.position.x)) <= x_abs_max:
                kept_indices.append(i)
                out.poses.append(p)

        radii_f  = [radii[i]  for i in kept_indices] if radii  else []
        bboxes_f = [bboxes[i] for i in kept_indices] if bboxes else []
        return out, radii_f, bboxes_f, kept_indices

    def _publish_markers(self, poses, radii, frame_id="world"):
        arr = MarkerArray()
        for i, pose in enumerate(poses.poses):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp    = self.get_clock().now().to_msg()
            m.id              = i
            m.type            = Marker.SPHERE
            m.action          = Marker.ADD
            r = float(radii[i]) if i < len(radii) else 0.04
            m.scale.x = m.scale.y = m.scale.z = 2.0 * r
            m.color.r = 1.0; m.color.a = 1.0
            m.pose    = pose
            arr.markers.append(m)
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = ApplePredictionNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()