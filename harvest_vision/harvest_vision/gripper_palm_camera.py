#!/usr/bin/env python3

# ROS2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# Image processing
from cv_bridge import CvBridge
import cv2
import numpy as np

class GripperPalmCamera(Node):
    def __init__(self, resolution=(800,600), target_fr=30):
        super().__init__("gripper_palm_camera_publisher")

        # image publisher
        self.camera_pub = self.create_publisher(Image, "gripper/rgb_palm_camera/image_raw", 10)
        
        # cv bridge to convert to ros image msg
        self.bridge = CvBridge()

        # camera vars 
        self.declare_parameter("palm_camera_device_num", 2)
        self.device = self.get_parameter("palm_camera_device_num").get_parameter_value().integer_value
        self.resolution = resolution
        self.target_fr = target_fr
        self.camera = None

        # camera setup
        camera_setup_result = self.create_camera()
        if not camera_setup_result:
            self.get_logger().error("Failed to setup camera with device number: {0}".format(self.device)) 
        else:
            self.get_logger().info("Succesfully setup camera with device number: {0}".format(self.device)) 

        
    def create_camera(self):
        try: 
            self.camera = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            self.camera.set(cv2.CAP_PROP_FPS, self.target_fr)
            return True
        except:
            return False
    
    def start(self):
        try:
            while rclpy.ok():
                ret, frame = self.camera.read()
                image = frame
                if ret:
                    img_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
                    img_msg.header.stamp = self.get_clock().now().to_msg()
                    img_msg.header.frame_id = "gripper_palm_camera_optical_link"
                    self.camera_pub.publish(img_msg)
        except KeyboardInterrupt:
            return
        cv2.destroyAllWindows()
        self.camera.release()
        

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = GripperPalmCamera()
    camera_publisher.start()
    rclpy.spin(camera_publisher)
    rclpy.shutdown()

"""
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudToCameraNode(Node):
    def __init__(self):
        super().__init__('pc_to_camera_sim')
        self.bridge = CvBridge()
        
        # Camera configuration (640x480 resolution)
        self.width = 640
        self.height = 480
        self.fx = 525.0  # Focal length X
        self.fy = 525.0  # Focal length Y
        self.cx = 320.0  # Principal point X
        self.cy = 240.0  # Principal point Y

        # Subscribers and Publishers
        self.sub = self.create_subscription(PointCloud2, '/your_point_cloud_topic', self.pc_callback, 10)
        self.img_pub = self.create_publisher(Image, '/usb_cam/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/usb_cam/camera_info', 10)

    def pc_callback(self, msg):
        # 1. Read points from the cloud (assuming XYZ and intensity/RGB fields if available)
        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        if len(points) == 0:
            return

        # 2. Filter points that are behind the camera (Z <= 0)
        valid_mask = points[:, 2] > 0.1
        points = points[valid_mask]
        
        # 3. Project 3D points to 2D image coordinates
        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        u = (x * self.fx / z) + self.cx
        v = (y * self.fy / z) + self.cy

        # 4. Map projections to image boundary limits
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        valid_pixels = (u >= 0) & (u < self.width) & (v >= 0) & (v < self.height)
        
        u_idx = u[valid_pixels].astype(int)
        v_idx = v[valid_pixels].astype(int)
        z_depth = z[valid_pixels]

        # 5. Color pixels based on distance (or intensity)
        norm_depth = cv2.normalize(z_depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        for i in range(len(u_idx)):
            # Grayscale depth mapping for visualization
            img[v_idx[i], u_idx[i]] = [norm_depth[i], norm_depth[i], norm_depth[i]]

        # 6. Publish the image and info messages
        timestamp = self.get_clock().now().to_msg()
        
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        img_msg.header = msg.header  # Match frame_id and timestamp
        img_msg.header.stamp = timestamp
        self.img_pub.publish(img_msg)

        info_msg = CameraInfo()
        info_msg.header = img_msg.header
        info_msg.width = self.width
        info_msg.height = self.height
        info_msg.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        self.info_pub.publish(info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
if __name__ == '__main__':
    main()
