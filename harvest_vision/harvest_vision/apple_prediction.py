#!/usr/bin/env python3
import time
from threading import Event, Lock

import numpy as np
import cv2
import torch

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, PoseArray
from harvest_interfaces.srv import ApplePrediction

from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformListener, TransformException

from cv_bridge import CvBridge
from ultralytics import YOLO
from message_filters import ApproximateTimeSynchronizer, Subscriber

from .sphere_ransac import Sphere


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 10**9 + int(stamp.nanosec)


class ApplePredictionFromTopics(Node):
    def __init__(self):
        super().__init__("apple_prediction_node")

        # --- params ---
        self.declare_parameter("camera_ns", "camera/mast_camera")
        self.declare_parameter("use_aligned_depth", True)
        self.declare_parameter("source_frame", "mast_camera_color_optical_frame")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("prediction_model_path", "NA")
        self.declare_parameter("prediction_yolo_conf", 0.85)
        self.declare_parameter("prediction_radius_min", 0.03)
        self.declare_parameter("prediction_radius_max", 0.06)
        self.declare_parameter("prediction_distance_max", 1.0)
        self.declare_parameter("scan_data_path", "NOTGIVEN")
        self.declare_parameter("allow_reuse_latest_frame", False)

        self.ns = self.get_parameter("camera_ns").value
        self.use_aligned = bool(self.get_parameter("use_aligned_depth").value)
        self.source_frame = self.get_parameter("source_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.save_dir = self.get_parameter("scan_data_path").value
        self.conf_thr = float(self.get_parameter("prediction_yolo_conf").value)
        self.rad_min = float(self.get_parameter("prediction_radius_min").value)
        self.rad_max = float(self.get_parameter("prediction_radius_max").value)
        self.dist_max = float(self.get_parameter("prediction_distance_max").value)
        self.allow_reuse_latest = bool(self.get_parameter("allow_reuse_latest_frame").value)

        # --- I/O ---
        self.service_group = MutuallyExclusiveCallbackGroup()  # single-flight service
        self._predict_lock = Lock()

        self.marker_pub = self.create_publisher(MarkerArray, "apple_markers", 10)
        self.srv = self.create_service(
            ApplePrediction, "apple_prediction", self.on_predict, callback_group=self.service_group
        )
        self.annotated_pub = self.create_publisher(
            Image,
            "apple_annotated",
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST,
                       depth=10)
        )

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- YOLO ---
        self.model = YOLO(self.get_parameter("prediction_model_path").value)
        try:
            self.model.model.eval()
        except Exception:
            pass

        # --- subs + sync ---
        self.bridge = CvBridge()

        self.color_topic = f"/{self.ns}/color/image_raw"
        if self.use_aligned:
            self.depth_topic = f"/{self.ns}/aligned_depth_to_color/image_raw"
            self.cinfo_topic = f"/{self.ns}/color/camera_info"
        else:
            self.depth_topic = f"/{self.ns}/depth/image_rect_raw"
            self.cinfo_topic = f"/{self.ns}/depth/camera_info"

        # IMPORTANT: keep strong refs
        self.color_sub = Subscriber(self, Image, self.color_topic, qos_profile=qos_profile_sensor_data)
        self.depth_sub = Subscriber(self, Image, self.depth_topic, qos_profile=qos_profile_sensor_data)

        info_qos = qos_profile_sensor_data
        self._last_cinfo = None
        self.cinfo_sub = self.create_subscription(CameraInfo, self.cinfo_topic, self._cinfo_cb, qos_profile=info_qos)

        # Sync only color + depth (looser window OK for field rigs)
        self.sync = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub],
                                                queue_size=10, slop=0.50)
        self.sync.registerCallback(self._sync_cd_cb)

        # Latest synced pair + coordination
        self._last_pair = None                  # (color_msg, depth_msg)
        self._new_pair_event = Event()
        self._last_used_pair_key = None         # (color_ns, depth_ns) of last used pair

        # --- RANSAC config ---
        self.ransac_thresh = 1e-4
        self.ransac_iters = 1000

        self.get_logger().info(
            "ApplePredictionFromTopics ready. Subscribing:\n"
            f"  color: {self.color_topic}\n  depth: {self.depth_topic}\n  cinfo: {self.cinfo_topic}"
        )

    # ----- subscribers -----

    def _cinfo_cb(self, msg: CameraInfo):
        self._last_cinfo = msg

    def _sync_cd_cb(self, color_msg: Image, depth_msg: Image):
        self._last_pair = (color_msg, depth_msg)
        self._new_pair_event.set()

    # ----- waiting for fresh color+depth pair -----

    def _wait_for_new_synced(self, timeout_sec=3.0):
        """
        Wait for a color+depth pair whose (color_ts, depth_ts) tuple differs from the last used.
        Optionally allow reusing the most recent once to avoid instant timeouts.
        Returns (color_msg, depth_msg) or None on timeout.
        """
        deadline = time.monotonic() + timeout_sec
        tried_reuse = False
        while time.monotonic() < deadline:
            pair = self._last_pair
            if pair is not None:
                c, d = pair
                key = (stamp_to_ns(c.header.stamp), stamp_to_ns(d.header.stamp))
                if key != self._last_used_pair_key:
                    return pair
                if self.allow_reuse_latest and not tried_reuse:
                    tried_reuse = True
                    return pair
            remaining = max(0.0, deadline - time.monotonic())
            self._new_pair_event.clear()
            self._new_pair_event.wait(timeout=remaining if remaining > 0 else 0)
        return None

    # ----- service -----

    def on_predict(self, req, res):
        with self._predict_lock:
            return self._on_predict_locked(req, res)

    def _on_predict_locked(self, req, res):
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

        # Mark this pair as consumed *before* inference
        self._last_used_pair_key = (
            stamp_to_ns(color_msg.header.stamp),
            stamp_to_ns(depth_msg.header.stamp)
        )

        # Convert images
        color_bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        depth_mm = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")  # uint16 mm

        # Sanity checks on sizes vs intrinsics
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

        fx, fy = float(cinfo_msg.k[0]), float(cinfo_msg.k[4])
        cx, cy = float(cinfo_msg.k[2]), float(cinfo_msg.k[5])

        # 1) YOLO
        with torch.inference_mode():
            results = self.model(color_bgr, conf=self.conf_thr, verbose=False)[0]

        # 2) Build instance-aligned masks & boxes
        masks, bboxes = self._build_instance_masks_and_boxes(results, Hc, Wc)

        # 3) Estimate spheres from masked back-projected points (fresh fitter per instance)
        centers, radii, kept_bboxes = self._estimate_apples_backproject(
            depth_mm, masks, bboxes, fx, fy, cx, cy
        )

        self.get_logger().info(
            f"Predict on pair ts=({color_msg.header.stamp.sec}.{color_msg.header.stamp.nanosec:09d}, "
            f"{depth_msg.header.stamp.sec}.{depth_msg.header.stamp.nanosec:09d}) -> {len(centers)} detections."
        )
        if len(centers) > 0:
            self.get_logger().info(f"Centers (m): {np.round(np.asarray(centers), 3)}")

        # 4) Transform to target frame
        poses_world = self._to_pose_array(centers, self.source_frame, self.target_frame)

        # 5) Publish markers
        self._publish_markers(poses_world, radii, frame_id=self.target_frame)

        # 6) Publish annotated image
        self._publish_annotated_image(color_bgr, kept_bboxes, header=color_msg.header)

        # 7) Fill response
        res.apple_poses = poses_world
        return res

    # ----- helpers -----

    def _build_instance_masks_and_boxes(self, results, H, W):
        inst_n = int(len(results.boxes) if results.boxes is not None else 0)
        masks = [None] * inst_n
        bboxes = [None] * inst_n

        if inst_n == 0:
            return [], []

        xyxy = results.boxes.xyxy.cpu().numpy().astype(int)

        if getattr(results, "masks", None) is not None and getattr(results.masks, "xy", None) is not None:
            polys = results.masks.xy
            for i in range(inst_n):
                mask = np.zeros((H, W), dtype=np.uint8)
                pl = polys[i]
                if isinstance(pl, np.ndarray):
                    cv2.fillPoly(mask, [pl.astype(np.int32)], 255)
                else:
                    for poly in pl:
                        cv2.fillPoly(mask, [poly.astype(np.int32)], 255)
                masks[i] = mask
                x1, y1, x2, y2 = xyxy[i]
                bboxes[i] = (x1, y1, x2, y2)
        else:
            for i in range(inst_n):
                x1, y1, x2, y2 = xyxy[i]
                mask = np.zeros((H, W), dtype=np.uint8)
                cv2.rectangle(mask, (max(0, x1), max(0, y1)), (min(W - 1, x2), min(H - 1, y2)), 255, -1)
                masks[i] = mask
                bboxes[i] = (x1, y1, x2, y2)

        pairs = [(m, b) for m, b in zip(masks, bboxes) if m is not None and m.any()]
        if not pairs:
            return [], []
        masks, bboxes = zip(*pairs)
        return list(masks), list(bboxes)

    def _estimate_apples_backproject(self, depth_mm, masks, bboxes, fx, fy, cx, cy):
        centers, radii, kept_bboxes = [], [], []

        Hd, Wd = depth_mm.shape[:2]
        for idx, (m, bbox) in enumerate(zip(masks, bboxes)):
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

            med = float(np.median(z))
            if med > self.dist_max:
                continue

            X = (xs - cx) * z / fx
            Y = (ys - cy) * z / fy
            pts = np.column_stack((X, Y, z))

            fitter = Sphere()  # stateless per detection
            c, r, ok = fitter.fit(
                pts,
                thresh=self.ransac_thresh,
                maxIteration=self.ransac_iters,
                lower_rad_bound=self.rad_min,
                upper_rad_bound=self.rad_max
            )
            if ok is False or c is None or r is None or not np.isfinite(c).all() or not np.isfinite(r):
                continue

            centers.append(c)
            radii.append(float(r))
            kept_bboxes.append(tuple(bbox))

        return centers, radii, kept_bboxes

    def _to_pose_array(self, centers_xyz, source_frame, target_frame):
        pa = PoseArray()
        now = self.get_clock().now().to_msg()

        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=1.0)
            )
        except TransformException as e:
            self.get_logger().warn(f"TF failed {source_frame}->{target_frame}: {e}")
            return pa

        for c in centers_xyz:
            src = PoseStamped()
            src.header.frame_id = source_frame
            src.header.stamp = now
            src.pose.position.x = float(c[0])
            src.pose.position.y = float(c[1])
            src.pose.position.z = float(c[2])
            src.pose.orientation.x = 0.0
            src.pose.orientation.y = 0.0
            src.pose.orientation.z = 0.0
            src.pose.orientation.w = 1.0

            out = do_transform_pose_stamped(src, tf)
            pa.poses.append(out.pose)

        return pa

    def _publish_markers(self, poses, radii, frame_id="world"):
        arr = MarkerArray()
        for i, pose in enumerate(poses.poses):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            r = float(radii[i]) if i < len(radii) else 0.04
            m.scale.x = m.scale.y = m.scale.z = 2.0 * r
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 1.0
            m.pose = pose
            arr.markers.append(m)
        self.marker_pub.publish(arr)

    def _publish_annotated_image(self, bgr_img, bboxes, header=None):
        if bgr_img is None:
            return

        annotated = bgr_img.copy()
        H, W = annotated.shape[:2]

        for i, (x1, y1, x2, y2) in enumerate(bboxes):
            # Coerce to Python ints and clamp to image bounds
            x1 = int(max(0, min(W - 1, x1)))
            y1 = int(max(0, min(H - 1, y1)))
            x2 = int(max(0, min(W - 1, x2)))
            y2 = int(max(0, min(H - 1, y2)))

            # Skip degenerate boxes
            if x2 <= x1 or y2 <= y1:
                continue

            # Draw box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Label background box (fixed: provide BOTH corners as tuples)
            label = str(i)
            (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
            bg_y1 = max(0, y1 - th - baseline - 6)
            bg_y2 = y1
            bg_x1 = x1
            bg_x2 = min(W - 1, x1 + tw + 8)

            cv2.rectangle(
                annotated,
                (bg_x1, bg_y1),
                (bg_x2, bg_y2),
                (0, 255, 0),
                -1  # thickness as positional (filled)
            )

            # Draw label text
            text_org = (x1 + 4, max(0, y1 - 6))
            cv2.putText(
                annotated, label,
                text_org,
                cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                (0, 0, 0), 2, cv2.LINE_AA
            )

        msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        if header is not None:
            msg.header = header
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.source_frame
        self.annotated_pub.publish(msg)


def main():
    rclpy.init()
    node = ApplePredictionFromTopics()
    from rclpy.executors import MultiThreadedExecutor
    exec = MultiThreadedExecutor(num_threads=4)
    exec.add_node(node)
    exec.spin()
    exec.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
