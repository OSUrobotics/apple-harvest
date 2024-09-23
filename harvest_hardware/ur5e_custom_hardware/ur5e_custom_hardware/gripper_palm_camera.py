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

if __name__ == '__main__':
    main()