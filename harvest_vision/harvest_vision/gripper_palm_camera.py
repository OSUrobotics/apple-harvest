#!/usr/bin/env python3

# ROS2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import TransformStamped
from geometry_msgs.msg import Pose, PoseArray
from scipy.spatial.transform import Rotation as R
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import (QoSProfile,
    ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)
from std_msgs.msg import Header

# Image processing
from cv_bridge import CvBridge
import cv2
import numpy as np


class GripperPalmCamera(Node):
    def __init__(self, resolution=(800, 600), target_fr=30):
        super().__init__("gripper_palm_camera_publisher")

        # For virtual camera or real one
        self.declare_parameter("use_fake_hardware", False)
        
        # If we're using fake hardware then only publish new image when end effector moves
        if self.get_parameter('use_fake_hardware').value:
            qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                                           durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                                           history=HistoryPolicy.KEEP_LAST, depth=1)
        else:
            qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                     history=HistoryPolicy.KEEP_LAST,
                                     depth=3)
            
        # This is the real camera image (if there is one), otherwise it's the same as the projected point cloud image
        self.camera_pub = self.create_publisher(Image, "gripper/rgb_palm_camera/image_raw", qos_profile)
        # The point cloud from the rgbd camera projected onto the gripper camera
        self.proj_im_pub = self.create_publisher(Image, "gripper/rgb_palm_camera/image_proj", qos_profile)
        
        # cv bridge to convert to ros image msg
        self.bridge = CvBridge()

        # Note: If a real camera is defined, these will come from that
        self.declare_parameter("resolution", resolution)
        self.declare_parameter("target_frame_rate", 30)
        self.declare_parameter("source_frame", "mast_camera_color_optical_frame")

        # camera vars 
        self.declare_parameter("palm_camera_device_num", 2)
        self.device = self.get_parameter("palm_camera_device_num").get_parameter_value().integer_value
        self.camera = None

        # which camera frame are we getting the point cloud from? (usually mast or down low one)
        self.source_frame = self.get_parameter("source_frame").value
                
        # End effector location, for generating point cloud image
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Listen for 3D marker locations. These will be projected into the image and published as another set of points
        #  at the same time as the new image comes in
        self.create_subscription(PoseArray, "apple_poses", self.project_image_frame_callback, 10)

        # Keep the projected centers and publish them whenever there's a new image
        self.apple_locs = []
        self.apple_loc_pub = self.create_publisher(PoseArray, "gripper/apple_locs", qos_profile=qos_profile)

        # This is for producing the 3d point cloud projected image and (if fake hardware) also generating the fake camera image
        timer_hz = 1.0 / self.get_parameter('target_frame_rate').value
        self.get_logger().info(f"Setting callback rate as {timer_hz}")
        self.projected_image_timer = self.create_timer(timer_hz, self.projected_image_timer_callback)
        # Just use the last image if the arm is not moving and doing fake hardware
        self.last_projected_image: np.array = None
        self.current_pose: TransformStamped = None
        # If there is a point cloud that we're projecting from this camera viewpoint...
        self.point_cloud: PointCloud2 = None

        # camera setup
        if self.get_parameter('use_fake_hardware').value:
            # Grab the data from the point cloud
            self.sub = self.create_subscription(PointCloud2, '/rgbd_pointcloud', self.pc_callback, 10)
            # Publish camera info - needed for visual servo (needs the k matrix)
            self.cam_info_pub = self.create_publisher(CameraInfo, '/gripper/rgb_palm_camera/camera_info', 2)
            self.resolution = resolution
            focal = 0.5 * (resolution[0] + resolution[1])
            self.fx = focal  # Focal length X - square pixels
            self.fy = focal  # Focal length Y
            self.cx = resolution[0] / 2.0
            self.cy = resolution[1] / 2.0
            self.get_logger().info(f"Started virtual camera, resolution {resolution} {self.fx}, {self.fy}")
            self.published_camera_info = False
        else:
            camera_setup_result = self.create_camera()
            if not camera_setup_result:
                self.get_logger().error("Failed to setup camera with device number: {0}".format(self.device)) 
            else:
                self.get_logger().info("Succesfully setup camera with device number: {0}".format(self.device)) 

    def create_camera(self):
        """ See if there's a real camera out there on the gripper """
        try: 
            self.camera = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.get_parameter('resolution')[0])
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.get_parameter('resolution')[1])
            self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            self.camera.set(cv2.CAP_PROP_FPS, self.get_parameter('target_frame_rate').value)
            return True
        except:
            return False
    
    def start(self):
        """ Real camera """
        if self.camera is None:
            return
        try:
            while rclpy.ok():
                ret, frame = self.camera.read()
                image = frame
                if ret:
                    img_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
                    img_msg.header.stamp = self.get_clock().now().to_msg()
                    img_msg.header.frame_id = "gripper_palm_camera_optical_link"
                    self.camera_pub.publish(img_msg)
                    if self.point_cloud is not None:
                        self._publish_projection_image()
        except KeyboardInterrupt:
            return
        cv2.destroyAllWindows()
        self.camera.release()

    @staticmethod
    def _is_transform_equivalent(t1: TransformStamped, t2: TransformStamped, tol=1e-3) -> bool:
        """
        Returns True if translation and rotation values match within a tight tolerance.
        """
        p1, p2 = t1.transform.translation, t2.transform.translation
        r1, r2 = t1.transform.rotation, t2.transform.rotation
        
        # Check spatial position
        pos_match = (np.isclose(p1.x, p2.x, atol=tol) and
                     np.isclose(p1.y, p2.y, atol=tol) and
                     np.isclose(p1.z, p2.z, atol=tol))
                    
        # Check orientation quaternion
        rot_match = (np.isclose(r1.x, r2.x, atol=tol) and
                     np.isclose(r1.y, r2.y, atol=tol) and
                     np.isclose(r1.z, r2.z, atol=tol) and
                     np.isclose(r1.w, r2.w, atol=tol))
                    
        return pos_match and rot_match

    def projected_image_timer_callback(self):
        to_frame = self.source_frame
        from_frame = 'gripper_palm_camera_optical_link'

        if self.point_cloud is None:
            return
        
        try:
            new_pose = self.tf_buffer.transform(from_frame, to_frame, rclpy.duration.Duration(seconds=1))

            b_changed = False
            if self.current_pose != None:
                if not self._is_transform_equivalent(new_pose, self.current_pose):
                    b_changed = True
            else:
                b_changed = True

            self.current_pose = new_pose
            if b_changed:
                p = new_pose.transform.translation
                r = new_pose.transform.rotation
                self.get_logger().info(f"Pose changed {p.x:0.2f} {p.y:0.2f} {p.z:0.2f} {r.w:0.2f}")

                header = Header()
                header.frame_id = self.source_frame
                header.stamp = self.get_clock().now().to_msg()
                self._publish_projection_image(header=header)

        except TransformException as ex:
            self.get_logger().info(f'Could not transform {to_frame} to {from_frame}: {ex}')

    def _get_matrix_transform(self):
        """Get the current mast to gripper camera matrix"""
        if self.current_pose == None:
            return np.identity(4)
        
        # Extract translation
        t = self.current_pose.transform.translation
        q = self.current_pose.transform.rotation

        # 2. Build the 3x3 rotation matrix (SciPy expects [x, y, z, w])
        rotation = R.from_quat([q.x, q.y, q.z, q.w])
        rotation_matrix_3x3 = rotation.as_matrix()

        # 3. Assemble the 4x4 homogeneous transformation matrix
        homogeneous_matrix = np.eye(4)
        homogeneous_matrix[0:3, 0:3] = rotation_matrix_3x3
        homogeneous_matrix[0:3, 3] = [t.x, t.y, t.z]
        return homogeneous_matrix

    def _publish_projection_image(self, header: Header):
        """ Publish both the point cloud projected to this camera (if available) and the 3D points projected to this camera"""
        if self.current_pose == None:
            return
        if self.point_cloud is None:
            return

        self.get_logger().info(f"Rendering point cloud {self.resolution}, N points {self.point_cloud['points'].shape}, {self.point_cloud['colors'].shape}")

        # image - width and height
        img = np.zeros((self.resolution[1], self.resolution[0], 3), dtype=np.uint8)

        # Iterate through points
        # Move the points
        homogeneous_matrix = self._get_matrix_transform()
        points_transformed = (homogeneous_matrix @ self.point_cloud["points"]).transpose()

        # Merge the points with the colors for the following operations (nx7 matrix)
        points_and_colors = np.hstack((points_transformed[:, 0:3], self.point_cloud["colors"]))
        
        # Filter on depth
        valid_mask = points_and_colors[:, 2] > 0.01
        points_and_colors_keep = points_and_colors[valid_mask, :]
        
        # Sort by depth
        points_sorted = points_and_colors_keep[(-points_and_colors_keep[:, 2]).argsort()]

        # Transform to image coordinates
        x, y, z = points_sorted[:, 0], points_sorted[:, 1], points_sorted[:, 2]

        u = (x * self.fx / z) + self.cx
        v = (y * self.fy / z) + self.cy

        # Trim again, this time for u,v out of bounds
        valid_pixels = (u >= 0) & (u < self.resolution[0]) & (v >= 0) & (v < self.resolution[1])

        # Convert to ints for indexing
        u_idx = u[valid_pixels].astype(int)
        v_idx = v[valid_pixels].astype(int)

        colors = points_sorted[valid_pixels, 3:]
        for indx in range(len(u_idx)):
            # Colors for each pixel
            rgb_pix = colors[indx, :]
            # b g r
            img[v_idx[indx], u_idx[indx]] = [rgb_pix[2], rgb_pix[1], rgb_pix[0]]

        for pt in self.apple_locs:
            cv2.drawMarker(img, (pt[0], pt[1]), color=(255, 255, 255), markerType=cv2.MARKER_CROSS, thickness=2)

        # 6. Publish the image and info messages        
        cv2.imwrite('check.png', img)
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")

        img_msg.header = header
        img_msg.header.frame_id = "gripper_palm_camera_optical_link"
        self.proj_im_pub.publish(img_msg)

        if self.get_parameter('use_fake_hardware').value and not self.published_camera_info:
            info_msg = CameraInfo()
            info_msg.header.stamp = self.get_clock().now().to_msg()
            info_msg.header.frame_id = "gripper_palm_camera_optical_link"
            info_msg.width = self.resolution[0]
            info_msg.height = self.resolution[1]
            info_msg.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
            self.cam_info_pub.publish(info_msg)

            # And the fake image
            self.camera_pub.publish(img_msg)
            self.published_camera_info = True

        self.get_logger().info("Done")
        
    def project_image_frame_callback(self, msg: PoseArray):
        """ Get the markers from the 3D apple prediction and project them into the image"""
        if not self.current_pose:
            return
        
        self.get_logger().info(f"Projecting points into image")

        homogeneous_matrix = self._get_matrix_transform()
        pt = np.ones((4,1))

        pa = PoseArray()
        now = self.get_clock().now().to_msg()
        frame_id = "gripper_palm_camera_optical_link"
        pa.header.stamp = now
        pa.header.frame_id = frame_id

        # Project all of the points, even if they end up behind the camera
        for pose in msg.poses:
            pt[0] = pose.position.x
            pt[1] = pose.position.y
            pt[2] = pose.position.z

            pt_in_image = homogeneous_matrix @ pt
            x = pt_in_image[0]
            y = pt_in_image[1]
            z = pt_in_image[2]

            u = (x * self.fx / z) + self.cx
            v = (y * self.fy / z) + self.cy
            
            proj_pt = Pose()
            proj_pt.position.x    = float(u)
            proj_pt.position.y    = float(v)
            proj_pt.position.z    = float(z)
            proj_pt.orientation.w = 1.0

            pa.poses.append(proj_pt)
        self.get_logger().info(f"Publishing {len(pa.poses)} projected apple locations")
        self.apple_loc_pub.publish(pa)

    def pc_callback(self, msg):
        """ Grab the point cloud then call the rendermethod """

        if self.point_cloud is not None:
            return

        self.get_logger().info(f"Getting point cloud")

        points = []
        colors = []
        indx = 0
        for point in pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True):
            x, y, z, rgb_packed = point
            points.append((x, y, z))

            # Re-interpret float32 color bits as uint32 - note, don't do this if the rgb is a packed integer
            # s = struct.pack('f', rgb_packed)
            # pack = struct.unpack('I', s)[0]
        
            # Unpack individual channels (0 to 255)
            r = (rgb_packed >> 16) & 0x00FF
            g = (rgb_packed >> 8) & 0x00FF
            b = rgb_packed & 0x00FF
            colors.append((r, g, b))
            indx += 1


        self.point_cloud = {"points": np.ones((4, len(points))), "colors": np.array(colors)}
        self.point_cloud["points"][0:3, :] = np.array(points).transpose()
        self._publish_projection_image(header=msg.header)


def main(args=None):
    rclpy.init(args=args)
    camera_publisher = GripperPalmCamera()
    camera_publisher.start()
    rclpy.spin(camera_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
