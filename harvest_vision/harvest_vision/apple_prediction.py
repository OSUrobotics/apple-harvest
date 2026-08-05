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

        # For saving the debug image
        self._yolo_debug_image : np.array = None

        # -- control publication rates ---
        if self.presaved_images:
           qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                    history=HistoryPolicy.KEEP_LAST, depth=1)
        else:
            qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                     history=HistoryPolicy.KEEP_LAST,
                                     depth=3)
 
        # ---- shared I/O ----
        self.bridge = CvBridge()
        self._predict_lock = Lock()
        self.service_group = MutuallyExclusiveCallbackGroup()

        # ---- Data published everytime the service is called ----
        self.marker_pub = self.create_publisher(MarkerArray, "apple_markers", qos_profile)
        self.annotated_pub = self.create_publisher(Image, "apple_annotated", qos_profile)
        self.pc_pub = self.create_publisher(PointCloud2, "rgbd_pointcloud", qos_profile)
        self.apple_poses_pub = self.create_publisher(PoseArray, "apple_poses", qos_profile)

        # ---- TF ----
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ----- Saved image/camera variables for prediction/debugging
        self.rgb_image : np.array = None
        self.depth_image : np.array = None
        self.fx, self.fy = (0.0, 0.0)
        self.cx, self.cy = (0.0, 0.0)
        self._last_cinfo = None
        
        # ---- mode-specific setup ----
        if self.presaved_images:
            self._init_presaved()
        else:
            self._init_live()


        # ---- Getting a new point cloud and predicting locations is done on a service call ----
        self.srv = self.create_service(
            ApplePrediction, "apple_prediction",
            self.on_predict, callback_group=self.service_group,
        )

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

        self.get_logger().info(
            "ApplePredictionNode (live) ready.\n"
            f"  color: {self.color_topic}\n"
            f"  depth: {self.depth_topic}\n"
            f"  cinfo: {self.cinfo_topic}"
        )

    # ------------------------------------------------------------------ live image read

    def _on_predict_live_get_point_cloud(self):
        """ Call just before predicting images if running live"""

        # Clear out old images
        self.rgb_image = None
        self.depth_image = None

        # Get image
        if self._last_cinfo is None:
            self.get_logger().warn("No CameraInfo received yet.")
            return

        pair = self._wait_for_new_synced(timeout_sec=3.0)
        if pair is None:
            self.get_logger().warn("Timed out waiting for fresh synced frames.")
            return

        color_msg, depth_msg = pair
        cinfo_msg = self._last_cinfo

        self._last_used_pair_key = (
            stamp_to_ns(color_msg.header.stamp),
            stamp_to_ns(depth_msg.header.stamp),
        )

        self.rgb_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        self.depth_image  = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        Hc, Wc = self.rgb_image.shape[:2]
        self.target_size = (Wc, Hc)

        if (cinfo_msg.height != Hc) or (cinfo_msg.width != Wc):
            self.get_logger().warn(
                f"CameraInfo size ({cinfo_msg.width}x{cinfo_msg.height}) "
                f"!= color size ({Wc}x{Hc}). Using color size for back-projection."
            )
        if self.use_aligned and (self.depth_image.shape[:2] != (Hc, Wc)):
            self.get_logger().warn(
                f"Aligned depth size {self.depth_image.shape[1]}x{self.depth_image.shape[0]} "
                f"!= color size {Wc}x{Hc}; masks will be resized."
            )

        self.fx = float(cinfo_msg.k[0]); self.fy = float(cinfo_msg.k[4])
        self.cx = float(cinfo_msg.k[2]); self.cy = float(cinfo_msg.k[5])
        self.target_size = (Wc, Hc)

        # Publish full RGBD point cloud
        self._publish_pointcloud(stamp=color_msg.header.stamp)

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

        # ---- timers - used to make the inititial point cloud from saved data ----
        self.image_timer = self.create_timer(0.1, self._presaved_publish_images_timer)
        self.pointcloud_timer = self.create_timer(0.5, self._presaved_publish_pointcloud_timer)

        self.get_logger().info("ApplePredictionNode (presaved) ready.")

    # ================================================================== Timers to kick stuff off
    def _presaved_publish_images_timer(self):
        """Timer callback: republish the loaded RGB and depth images at 10 Hz."""
        if self._predict_lock.locked():
            return  # service callback is running — skip this cycle

        # This timer should be triggered after _init_presaved was run, but if it wasn't, don't kill the timer
        if self.rgb_image is None or self.depth_image is None:
            return
        
        rgb_msg   = self.bridge.cv2_to_imgmsg(self.rgb_image,   encoding="bgr8")
        depth_msg = self.bridge.cv2_to_imgmsg(self.depth_image, encoding="mono16")
        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)

        # Kill - if doing pre-saved, we won't get another set of images
        self.image_timer.cancel()

    def _presaved_publish_pointcloud_timer(self):
        """Timer callback: republish the full RGBD point cloud at 2 Hz."""
        if self._predict_lock.locked():
            return  # service callback is running — skip this cycle
        
        # This timer should be triggered after _init_presaved was run, but if it wasn't, don't kill the timer
        if self.rgb_image is None or self.depth_image is None:
            return
        
        # Publish the point cloud
        self._publish_pointcloud()

        # Kill - if doing pre-saved, we won't get another point cloud
        self.pointcloud_timer.cancel()

    # ================================================================== service

    # ----------------------------- Prediction helper functions
    def _yolo_prediction(self):
        """ Run YOLO """
        with torch.inference_mode():
            yolo_results = self.model(self.rgb_image, conf=self.conf_thr, verbose=False)[0]

        return yolo_results

    def _build_instance_masks_and_boxes(self, yolo_results):
        """ Get masks, boxes, and confidence from yolo results """

        inst_n = int(len(yolo_results.boxes) if yolo_results.boxes is not None else 0)
        if inst_n == 0:
            self.get_logger().warn(f"No apples found - sadness - in yolo result")
            return [], [], []

        W, H = self.target_size
        
        masks  = [None] * inst_n
        bboxes = [None] * inst_n
        xyxy   = yolo_results.boxes.xyxy.cpu().numpy().astype(int)
        confidences = [
                        float(box.conf.cpu().numpy()[0])
                        for box in yolo_results.boxes
                      ]
        if getattr(yolo_results, "masks", None) is not None and \
                getattr(yolo_results.masks, "xy", None) is not None:
            polys = yolo_results.masks.xy
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
        return list(masks), list(bboxes), list(confidences)

    def _ransac_sphere(self, pts):
        # Fit a sphere to the points. The sphere class handles removing outliers and doing ransac
        sph = Sphere()
        center, radius, inliers = sph.fit(
            pts,
            thresh=self.ransac_thresh,
            maxIteration=self.ransac_iters,
            lower_rad_bound=self.rad_min,
            upper_rad_bound=self.rad_max,
        )

        return center, radius
    
    def _get_apple_centers(self, masks):
        """
        For each mask: build an open3d RGBD point cloud, run RANSAC sphere fit.
        Returns (centers, radii) lists and which indices were kept.
        """
        self.get_logger().info(
            f"Depth dtype={self.depth_image.dtype} min={self.depth_image.min()} max={self.depth_image.max()} "
            f"nonzero={np.count_nonzero(self.depth_image)} shape={self.depth_image.shape}"
        )
        apple_centers = []
        apple_radii   = []
        kept_indices  = []

        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=self.target_size[0], height=self.target_size[1],
            fx=self.fx, fy=self.fy, cx=self.cx, cy=self.cy,
        )

        for indx, mask in enumerate(masks):
            depth_seg = np.where(mask.astype(bool), self.depth_image, 0).astype(np.uint16)
            valid_vals = depth_seg[depth_seg > 0]
            if valid_vals.size == 0:
                continue
            if np.median(valid_vals) > self.dist_max * self.depth_scale:
                continue

            rgb_o3d   = o3d.geometry.Image(self.rgb_image)
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

            center, radius = self._ransac_sphere(pts)
            if center is not None and radius is not None:
                # adding in the shift, if any
                center_shifted = (center[0] + self.pointcloud_offset[0],
                                  center[1] + self.pointcloud_offset[1],
                                  center[2] + self.pointcloud_offset[2])
                apple_centers.append(center_shifted)
                apple_radii.append(float(radius))
                kept_indices.append(indx)

        return apple_centers, apple_radii, kept_indices

    def on_predict(self, req, res):
        """ Get either the saved point cloud or a new one and then run YOLO, then fit spheres and publish results """
        if not self.presaved_images:
            # Snag the new point cloud if live
            with self._predict_lock:
                self._on_predict_live_get_point_cloud() 

        # We have either a pre-saved rgb and depth image or we just got one from the camera
        if self.rgb_image is None or self.depth_image is None:
            self.get_logger().warn(f"Predicting images: No rgb or depth image")
            return

        # 1) Run yolo
        yolo_results = self._yolo_prediction()

        # 2) Masks & boxes from yolo result
        masks, bboxes, confidences = self._build_instance_masks_and_boxes(yolo_results)
        self.get_logger().info(f"[1] YOLO found {len(masks)} apple mask(s)")
        
        # 3) Back-project → RANSAC sphere fit
        centers, radii, kept_indices = self._get_apple_centers(masks)
        self.get_logger().info(f"[2] get_apple_centers returned {len(centers)} center(s)")

        # 4) Transform to target frame
        poses_world = self._to_pose_array(centers, self.source_frame, self.target_frame)
        self.get_logger().info(f"[3] transform_apple_poses src {self.source_frame} dst {self.target_frame} returned {len(poses_world.poses)} pose(s)")

        # 5) Publish markers and poses and annotated debug image
        self._publish_markers(poses_world, radii, frame_id=self.target_frame)

        # Latched poses topic
        poses_world.header.stamp    = self.get_clock().now().to_msg()
        poses_world.header.frame_id = self.target_frame
        self.apple_poses_pub.publish(poses_world)
        self.get_logger().info(
            f"Published {len(poses_world.poses)} apple pose(s) to 'apple_poses'."
        )

        # Publish the annotated images
        self._publish_annotated_image(yolo_bboxes=bboxes, confidences=confidences, kept_indices=kept_indices)

        res.apple_poses = poses_world
        return res

    # ================================================================== Publication/debug/visualization

    def _publish_pointcloud(self, stamp=None):
        """Build and publish an XYZRGB PointCloud2 from a BGR image and depth map.

        Works for both live and presaved modes.  *stamp* defaults to now when
        not supplied (presaved / timer path).  The image is converted BGR→RGB
        internally so callers can pass the raw OpenCV frame either way.

        Uses self variables
        rgb_image:  uint8 BGR image (H×W×3).
        depth_image:    uint16 depth image in the node's depth_scale units.
        fx, fy:   focal lengths in pixels.
        cx, cy:   principal point in pixels.

        Args:
            stamp:    rospy/rclpy stamp to embed in the header; uses clock.now() if None.
        """
        if self.rgb_image is None or self.depth_image is None:
            self.get_logger().warn(f"Trying to publish point cloud, but no rgb or depth image")
            return 
        
        H, W = self.rgb_image.shape[:2]
        if self.presaved_images:
            rgb_o3d   = o3d.geometry.Image(self.rgb_image)
        else:
            rgb_o3d   = o3d.geometry.Image(cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB))
        depth_o3d = o3d.geometry.Image(self.depth_image.astype(np.uint16))
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=W, height=H, fx=self.fx, fy=self.fy, cx=self.cx, cy=self.cy,
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

    def _publish_markers(self, poses, radii, frame_id="world"):
        arr = MarkerArray()
        for indx, pose in enumerate(poses.poses):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp    = self.get_clock().now().to_msg()
            m.id              = indx
            m.type            = Marker.SPHERE
            m.action          = Marker.ADD
            r = float(radii[indx]) if indx < len(radii) else 0.04
            m.scale.x = m.scale.y = m.scale.z = 2.0 * r
            # Color red if not filtered as out of reach, otherwise yellow
            m.color.r = 1.0; m.color.a = 1.0
            m.pose    = pose
            arr.markers.append(m)
        self.marker_pub.publish(arr)

    def _publish_annotated_image(self, yolo_bboxes, confidences, kept_indices, header=None):
        """Draw bounding boxes on bgr_img and publish to apple_annotated.

        Show confidence scores pulled from Yolo result.
        """
        if self.rgb_image is None:
            return
        
        annotated_image = self.rgb_image.copy()
        H, W = annotated_image.shape[:2]

        # Color by kept indices
        color_apple = (255, 0, 0)
        color_not_apple = (125, 125, 0)
        for indx, (x1, y1, x2, y2) in enumerate(yolo_bboxes):
            x1 = int(max(0, min(W - 1, x1))); y1 = int(max(0, min(H - 1, y1)))
            x2 = int(max(0, min(W - 1, x2))); y2 = int(max(0, min(H - 1, y2)))
            if x2 <= x1 or y2 <= y1:
                continue

            if indx in kept_indices:
                color = color_apple
            else:
                color = color_not_apple
            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), color, 1)


            label       = f"apple {indx} {confidences[indx]:.2f}"
            font_scale  = 0.5
            thickness   = 1

            (tw, th), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness
            )
            bg_y1 = max(0, y1 - th - baseline - 6)
            cv2.rectangle(annotated_image,
                          (x1, bg_y1),
                          (min(W - 1, x1 + tw + 8), y1),
                          color, -1)
            cv2.putText(annotated_image, label, (x1 + 4, max(0, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0),
                        thickness, cv2.LINE_AA)

        msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
        if header is not None:
            msg.header = header
        else:
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = self.source_frame
        self.annotated_pub.publish(msg)


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