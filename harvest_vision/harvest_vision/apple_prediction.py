#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from threading import Event
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, PoseArray
from harvest_interfaces.srv import ApplePrediction

from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformListener, TransformException

from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO
import open3d as o3d
from message_filters import ApproximateTimeSynchronizer, Subscriber

from .sphere_ransac import Sphere


def stamp_to_ns(stamp) -> int:
    return stamp.sec * 10**9 + stamp.nanosec


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

        self.ns = self.get_parameter("camera_ns").value
        self.use_aligned = bool(self.get_parameter("use_aligned_depth").value)
        self.source_frame = self.get_parameter("source_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.save_dir = self.get_parameter("scan_data_path").value
        self.conf_thr = float(self.get_parameter("prediction_yolo_conf").value)
        self.rad_min = float(self.get_parameter("prediction_radius_min").value)
        self.rad_max = float(self.get_parameter("prediction_radius_max").value)
        self.dist_max = float(self.get_parameter("prediction_distance_max").value)

        # --- I/O ---
        self.service_group = ReentrantCallbackGroup()

        self.marker_pub = self.create_publisher(MarkerArray, "apple_markers", 10)
        self.srv = self.create_service(
            ApplePrediction, "apple_prediction", self.on_predict, callback_group=self.service_group
        )
        self.annotated_pub = self.create_publisher(
            Image,
            "apple_annotated",
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- YOLO ---
        self.model = YOLO(self.get_parameter("prediction_model_path").value)

        # --- subs + sync ---
        self.bridge = CvBridge()

        color_topic = f"/{self.ns}/color/image_raw"
        if self.use_aligned:
            depth_topic = f"/{self.ns}/aligned_depth_to_color/image_raw"
            cinfo_topic = f"/{self.ns}/color/camera_info"
        else:
            depth_topic = f"/{self.ns}/depth/image_rect_raw"
            cinfo_topic = f"/{self.ns}/depth/camera_info"

        color_sub = Subscriber(self, Image, color_topic, qos_profile=qos_profile_sensor_data)
        depth_sub = Subscriber(self, Image, depth_topic, qos_profile=qos_profile_sensor_data)

        info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        cinfo_sub = Subscriber(self, CameraInfo, cinfo_topic, qos_profile=info_qos)

        # Smaller slop is better when color and aligned depth are tightly stamped
        self.sync = ApproximateTimeSynchronizer(
            [color_sub, depth_sub, cinfo_sub],
            queue_size=20,
            slop=0.01
        )
        self.sync.registerCallback(self._sync_cb)

        # Storage for the most recent synced tuple and coordination for on-demand grabbing
        self._last_triplet = None  # (color_msg, depth_msg, cinfo_msg)
        self._new_triplet_event = Event()
        self._last_used_color_ns = -1  # nanoseconds stamp of last used color frame

        # --- RANSAC config ---
        self.ransac = Sphere()
        self.ransac_thresh = 1e-4
        self.ransac_iters = 1000

        self.get_logger().info(
            f"ApplePredictionFromTopics ready. Subscribing:\n"
            f"  color: {color_topic}\n  depth: {depth_topic}\n  cinfo: {cinfo_topic}"
        )

    # Keep the sync callback light: just stash and signal
    def _sync_cb(self, color_msg, depth_msg, cinfo_msg):
        self._last_triplet = (color_msg, depth_msg, cinfo_msg)
        self._new_triplet_event.set()

    def _wait_for_new_synced(self, timeout_sec=1.0):
        """
        Wait until there is a synced tuple whose color stamp is newer than
        the last one we used. Returns (color_msg, depth_msg, cinfo_msg) or None on timeout.
        """
        # Fast path: if we already have a newer triplet, grab it immediately
        if self._last_triplet is not None:
            c, d, i = self._last_triplet
            if stamp_to_ns(c.header.stamp) > self._last_used_color_ns:
                return c, d, i

        # Otherwise wait for next signal
        self._new_triplet_event.clear()
        if not self._new_triplet_event.wait(timeout=timeout_sec):
            return None

        self._new_triplet_event.clear()
        triplet = self._last_triplet
        if triplet is None:
            return None

        color_msg, depth_msg, cinfo_msg = triplet
        if stamp_to_ns(color_msg.header.stamp) <= self._last_used_color_ns:
            # Keep waiting a bit longer for a truly newer one
            if not self._new_triplet_event.wait(timeout=timeout_sec):
                return None
            self._new_triplet_event.clear()
            triplet = self._last_triplet

        return triplet

    def on_predict(self, req, res):
        # Block for fresh synced frames
        triplet = self._wait_for_new_synced(timeout_sec=2.0)
        if triplet is None:
            self.get_logger().warn("Timed out waiting for fresh synced frames.")
            res.apple_poses = PoseArray()  # ensure valid but empty
            return res

        color_msg, depth_msg, cinfo_msg = triplet
        self._last_used_color_ns = stamp_to_ns(color_msg.header.stamp)

        # Convert here so YOLO runs on fresh frames
        color_bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        depth_mm = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")  # uint16 mm

        # 1) Run YOLO
        results = self.model(color_bgr, conf=self.conf_thr)[0]

        # Build masks and paired bboxes
        masks = []
        bboxes = []
        H, W = color_bgr.shape[:2]

        if results.boxes is not None and len(results.boxes) > 0:
            xyxy = results.boxes.xyxy.cpu().numpy().astype(int)
        else:
            xyxy = np.empty((0, 4), dtype=int)

        if results.masks is not None and len(results) > 0:
            for i in range(len(results)):
                poly_list = results.masks.xy[i]
                mask = np.zeros((H, W), dtype=np.uint8)
                if isinstance(poly_list, np.ndarray):
                    cv2.fillPoly(mask, [np.int32(poly_list)], 255)
                else:
                    for poly in poly_list:
                        cv2.fillPoly(mask, [np.int32(poly)], 255)
                masks.append(mask)

                if i < len(xyxy):
                    x1, y1, x2, y2 = xyxy[i]
                else:
                    ys, xs = np.where(mask > 0)
                    if xs.size > 0:
                        x1, x2 = int(xs.min()), int(xs.max())
                        y1, y2 = int(ys.min()), int(ys.max())
                    else:
                        x1 = y1 = 0; x2 = y2 = 0
                bboxes.append((x1, y1, x2, y2))
        else:
            for i in range(len(xyxy)):
                x1, y1, x2, y2 = xyxy[i]
                mask = np.zeros((H, W), dtype=np.uint8)
                cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)
                masks.append(mask)
                bboxes.append((x1, y1, x2, y2))

        # 2) Intrinsics from CameraInfo
        K = cinfo_msg.k  # [fx,0,cx, 0,fy,cy, 0,0,1]
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]
        w, h = cinfo_msg.width, cinfo_msg.height
        o3d_intr = o3d.camera.PinholeCameraIntrinsic(w, h, fx, fy, cx, cy)

        # 3) Fit spheres
        rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
        centers, radii, kept_bboxes = self._estimate_apples(
            rgb, depth_mm, masks, bboxes, o3d_intr
        )

        # 4) Transform to target frame
        poses_world = self._to_pose_array(centers, self.source_frame, self.target_frame)

        # 5) Publish markers
        self._publish_markers(poses_world, radii, frame_id=self.target_frame)

        # 6) Publish annotated image
        self._publish_annotated_image(color_bgr, kept_bboxes, header=color_msg.header)

        # 7) Fill response
        res.apple_poses = poses_world
        return res

    def _estimate_apples(self, rgb, depth_mm, masks, bboxes, o3d_intr):
        centers, radii, kept_bboxes = [], [], []

        for m, bbox in zip(masks, bboxes):
            if m.shape != depth_mm.shape:
                m = cv2.resize(m, (depth_mm.shape[1], depth_mm.shape[0]), interpolation=cv2.INTER_NEAREST)

            depth_masked = np.where(m > 0, depth_mm, 0)
            valid = depth_masked[depth_masked > 0]
            if valid.size == 0:
                continue
            if np.median(valid) > self.dist_max * 1000.0:
                continue

            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d.geometry.Image(rgb),
                o3d.geometry.Image(depth_masked),
                depth_scale=1000.0,
                convert_rgb_to_intensity=False
            )
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d_intr)

            pts = np.asarray(pcd.points)
            if pts.shape[0] < 50:
                continue

            c, r, _ = self.ransac.fit(
                pts,
                thresh=self.ransac_thresh,
                maxIteration=self.ransac_iters,
                lower_rad_bound=self.rad_min,
                upper_rad_bound=self.rad_max
            )
            if c is not None and r is not None:
                centers.append(c)
                radii.append(r)
                kept_bboxes.append(bbox)

        return centers, radii, kept_bboxes

    def _to_pose_array(self, centers_xyz, source_frame, target_frame):
        pa = PoseArray()
        now = self.get_clock().now().to_msg()

        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
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

        for i, (x1, y1, x2, y2) in enumerate(bboxes):
            h, w = annotated.shape[:2]
            x1c, y1c = max(0, x1), max(0, y1)
            x2c, y2c = min(w - 1, x2), min(h - 1, y2)
            cv2.rectangle(annotated, (x1c, y1c), (x2c, y2c), (0, 255, 0), 2)
            label = str(i)
            (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
            cv2.rectangle(
                annotated,
                (x1c, max(0, y1c - th - baseline - 6)),
                (x1c + tw + 8, y1c),
                (0, 255, 0),
                thickness=-1
            )
            cv2.putText(
                annotated, label,
                (x1c + 4, y1c - 6),
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
    exec = MultiThreadedExecutor(num_threads=2)  # needed so the service can wait while subs spin
    exec.add_node(node)
    exec.spin()
    exec.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
