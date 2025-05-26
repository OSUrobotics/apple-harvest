#!/usr/bin/env python3

# ROS2
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, PoseArray
from harvest_interfaces.srv import ApplePrediction
# TF2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import tf2_geometry_msgs
# Image processing
from sensor_msgs.msg import PointCloud2, PointField, Image
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
# pointcloud reconstruction
import open3d as o3d
from .sphere_ransac import Sphere


class ApplePredictionPreSaved(Node):
    def __init__(self):
        '''Uses presaved Azure RGBD images, uses YOLO to segment apples and then creates pointcloud.
        Then uses RANSAC to fit a sphere to each apple to estimate the position and radius.'''
        super().__init__("apple_prediction_pre_saved_node")

        self.camera_type = 'azure'
        
        ### AZURE CAMERA INTRINSICS
        self.azure_color_intrinsic = [902.98, 0, 956.55, 0, 902.77, 547.68, 0, 0, 1]
        self.fx = self.azure_color_intrinsic[0]  # Focal length in x
        self.fy = self.azure_color_intrinsic[4]  # Focal length in y
        self.cx = self.azure_color_intrinsic[2]  # Principal point x
        self.cy = self.azure_color_intrinsic[5]  # Principal point y

        if self.camera_type == 'azure':
            self.target_size = (1920, 1080)
        else:
            self.target_size = (848, 480)

        ### SERVICE
        self.prediction_srv = self.create_service(ApplePrediction, "apple_prediction_presaved_images", self.prediction_callback_srv)

        ### PUBLISHERS
        self.marker_pub = self.create_publisher(MarkerArray, "apple_markers", 10)
        self.rgb_publisher = self.create_publisher(Image, 'rgb_image', 10)
        self.depth_publisher = self.create_publisher(Image, 'depth_image', 10)

        ### TIMERS
        self.timer = self.create_timer(0.1, self.publish_images)  # Publish at 10 Hz
        self.pc_timer = self.create_timer(0.5, self.publish_pointcloud)  # Publish at 2 Hz
        self.marker_timer = self.create_timer(0.5, self.marker_timer_callback)

        self.pointcloud_publisher = self.create_publisher(PointCloud2, 'rgbd_pointcloud', 10)
        self.scale = 1000.0  # Depth scale (e.g., mm to meters)

        self.bridge = CvBridge()

        ### PARAMETERS
        self.declare_parameter("prediction_model_path", "NA")
        self.declare_parameter("prediction_yolo_conf", 0.85)
        self.declare_parameter("prediction_radius_min", 0.03)
        self.declare_parameter("prediction_radius_max", 0.06)
        self.declare_parameter("prediction_distance_max", 1.0)
        self.declare_parameter("vision_experiment", "NA")
        self.model_path = self.get_parameter("prediction_model_path").get_parameter_value().string_value
        self.confidence_thresh = self.get_parameter("prediction_yolo_conf").get_parameter_value().double_value
        self.lower_rad_bound = self.get_parameter("prediction_radius_min").get_parameter_value().double_value
        self.upper_rad_bound = self.get_parameter("prediction_radius_max").get_parameter_value().double_value
        self.distance_thresh = self.get_parameter("prediction_distance_max").get_parameter_value().double_value
        self.vision_experiment = self.get_parameter("vision_experiment").get_parameter_value().string_value

        # Retrieve rgb and depth images from vision experiment
        package_name = 'harvest_vision'
        try:
            # Get the package's share directory
            share_directory = get_package_share_directory(package_name)
            
            # Access a file or subdirectory within the share directory
            self.rgb_path = os.path.join(share_directory, 'data/', f'prosser_{self.vision_experiment}/', 'color_raw.png')
            self.depth_path = os.path.join(share_directory, 'data/', f'prosser_{self.vision_experiment}/', 'depth_to_color.png')
        except Exception as e:
            self.get_logger().error(f"Error accessing share directory: {e}")

        # Load RGB and depth images
        self.rgb_image = cv2.imread(self.rgb_path)
        self.depth_image = cv2.imread(self.depth_path, cv2.IMREAD_UNCHANGED)
        self.rgb_image = cv2.resize(self.rgb_image, self.target_size, interpolation=cv2.INTER_LINEAR) # Resize RGB image
        self.depth_image = cv2.resize(self.depth_image, self.target_size, interpolation=cv2.INTER_NEAREST) # Resize depth image

        ### YOLO SETUP
        self.model = YOLO(self.model_path)  # pretrained YOLOv8n model
        if self.model: 
            self.get_logger().info("Succesfully loaded YOLO model.") 
        else:
            self.get_logger().error("Could not load YOLO model.") 

        ### Tf2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        ### VARS
        self.debug_flag = False
        self.marker_counter = 0
        self.ransac_thresh = .0001  # UNITS: m - distance that is considered an inlier point and an outlier point in the sphere fit
        self.ransac_iters = 1000    # number of iterations for ransac to run
        self.apple_centers = None
        self.apple_radii = None
        self.c2 = 0
        self.image_timestamp = "ERROR"
        self.scan_count = 0
        self.yolo_result = None

    def prediction_callback_srv(self, request, response):
        # Segment apples and resize masks
        apple_masks = self.segment_apples(self.rgb_image)
        resized_apple_masks = [cv2.resize(mask, self.target_size, interpolation=cv2.INTER_NEAREST) for mask in apple_masks]

        # Process images and masks
        rgb_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB)
        centers, radii = self.get_apple_centers(rgb_image, self.depth_image, resized_apple_masks)
        transformed_poses = self.transform_apple_poses(centers)

        # Publish results
        self.publish_markers(transformed_poses, radii)
        response.apple_poses = transformed_poses
        self.apple_centers = transformed_poses
        self.apple_radii = radii
        self.c2 = 0

        return response

    def segment_apples(self, image):
        # returns masks of apples, each apple has its own mask. 0 for no apple, 255 for apple.  
        # predict segmentation masks using model
        results = self.model(image, conf=self.confidence_thresh)[0]
        apple_masks = []
        self.yolo_result = results

        # creates a mask for each apple detected over the original image
        for i in results:
            # create empty mask
            if results.masks != None: 
                img_h, img_w = results.masks.orig_shape
                mask = np.zeros((img_h, img_w), dtype=np.uint8)
                # fill in mask with white where predicted apple segmentation is
                cv2.fillPoly(mask, np.int32([i.masks.xy]), (255, 255, 255))
                apple_masks.append(mask)
            else:
                img_h, img_w = results.orig_shape
                mask = np.zeros((img_h, img_w), dtype=np.uint8)
                x,y,w,h = i.boxes.xyxy.cpu().numpy()[0]
                cv2.rectangle(mask, (int(x), int(y)), (int(w), int(h)), (255,255,255), -1)
                apple_masks.append(mask)

        # Optional visualization for debugging
        if self.debug_flag: 
            results.show()  # display to screen
            for i in apple_masks:
                cv2.imshow("apple_masks", i)
                key = cv2.waitKey(0)
                self.get_logger().info("Press esc to view next apple mask.")
                if key == 27:
                    cv2.destroyAllWindows()
        return apple_masks

    def ransac_apple_estimation(self, pcd):
        center = None
        radius = None
        sph_ransac = Sphere()
        center, radius, inliers = sph_ransac.fit(pcd, thresh=self.ransac_thresh, maxIteration=self.ransac_iters, 
                                                 lower_rad_bound=self.lower_rad_bound, upper_rad_bound=self.upper_rad_bound)

        return center, radius

    def get_apple_centers(self, rgb, depth, masks):
        apple_centers = []
        apple_radii = []
        visualization = []
        for mask in masks: 
            # segment only the apple portions
            depth_segmented = np.where(mask, depth, 0)
            print(depth_segmented[depth_segmented>0])
            if np.median(depth_segmented[depth_segmented>0] > self.distance_thresh * 1000):
                continue
            # create RGBD image (NEEDS TO BE IN RGB FORMAT NOT BGR TO LOOK RIGHT, doesnt super matter for anything other than visualization)
            rgb_pc = o3d.geometry.Image(rgb)
            depth_pc = o3d.geometry.Image(depth_segmented)
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_pc, depth_pc, convert_rgb_to_intensity=False)

            if self.camera_type == 'realsense':
                # Creates pointcloud using camera intrinsics from Realsense 435i
                pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
                    rgbd_image,
                    o3d.camera.PinholeCameraIntrinsic(width=848, height=480, fx=609.6989, fy=609.8549, cx=420.2079, cy=235.2782))
            elif self.camera_type == 'azure':
                # Creates pointcloud using camera intrinsics from Microsoft Azure
                pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
                    rgbd_image,
                    o3d.camera.PinholeCameraIntrinsic(width=self.target_size[0], height=self.target_size[1], 
                                                      fx=self.fx,
                                                      fy=self.fy, 
                                                      cx=self.cx, 
                                                      cy=self.cy))
            center, radius = self.ransac_apple_estimation(np.array(pcd.points))

            if center and radius: 
                apple_centers.append(center)
                apple_radii.append(radius)
                mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
                mesh_sphere.compute_vertex_normals()
                mesh_sphere.paint_uniform_color([1,0,0])
                mesh_sphere.translate(np.array(center), relative=False)
                visualization.append(pcd)
                visualization.append(mesh_sphere)

        if self.debug_flag:
            pass
            # o3d.visualization.draw_geometries(visualization)

        return apple_centers, apple_radii

    def transform_apple_poses(self, apple_poses):
        transformed_poses = PoseArray()
        for i in apple_poses:
            origin = PoseStamped()
            # origin.header.frame_id = "camera_color_optical_frame"
            origin.header.frame_id = "camera_link"
            origin.pose.position.x = i[0]
            origin.pose.position.y = i[1]
            origin.pose.position.z = i[2]
            origin.pose.orientation.x = 0.0
            origin.pose.orientation.y = 0.0
            origin.pose.orientation.z = 0.0
            origin.pose.orientation.w = 1.0
            try:
                new_pose = self.tf_buffer.transform(origin, "base_link", rclpy.duration.Duration(seconds=1))
                transformed_poses.poses.append(new_pose.pose)
            except TransformException as e:
                self.get_logger().info(f'Transform failed: {e}')
        return transformed_poses
    
    def publish_markers(self, apple_poses, apple_radii):
        # Clear existing markers
        clear_markers = MarkerArray()
        for i in range(self.marker_counter):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.id = i
            marker.action = Marker.DELETE
            clear_markers.markers.append(marker)
        self.marker_pub.publish(clear_markers)
        self.marker_counter = 0  # Reset the marker counter

        # Publish new markers
        markers = MarkerArray()
        for i in range(len(apple_poses.poses)):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = self.marker_counter
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            # Set the scale
            marker.scale.x = apple_radii[i] * 2
            marker.scale.y = apple_radii[i] * 2
            marker.scale.z = apple_radii[i] * 2
            # Set the color
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            # Set the pose
            marker.pose.position.x = apple_poses.poses[i].position.x
            marker.pose.position.y = apple_poses.poses[i].position.y
            marker.pose.position.z = apple_poses.poses[i].position.z
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            markers.markers.append(marker)
            self.marker_counter += 1
            self.get_logger().info(f"Apple found at: [{apple_poses.poses[i].position.x}, {apple_poses.poses[i].position.y}, {apple_poses.poses[i].position.z}] with radius {apple_radii[i]}")
        self.marker_pub.publish(markers)
    
    def publish_images(self):
        # Convert RGB image to ROS message
        rgb_msg = self.bridge.cv2_to_imgmsg(self.rgb_image, encoding="bgr8")
        # Convert depth image to ROS message
        depth_msg = self.bridge.cv2_to_imgmsg(self.depth_image, encoding="mono16")

        # Publish both images
        self.rgb_publisher.publish(rgb_msg)
        self.depth_publisher.publish(depth_msg)

    def publish_pointcloud(self):
        # Convert RGB and depth images to an Open3D point cloud
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(self.rgb_image),
            o3d.geometry.Image(self.depth_image),
            depth_scale=self.scale,
            depth_trunc=1.5,  # Ignore points beyond 3m
            convert_rgb_to_intensity=False
        )
        camera_intrinsics = o3d.camera.PinholeCameraIntrinsic(
            self.target_size[0],
            self.target_size[1],
            self.fx,
            self.fy,
            self.cx,
            self.cy
        )
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, camera_intrinsics)

        # Extract points and colors
        points = np.asarray(pcd.points)
        colors = (np.asarray(pcd.colors) * 255).astype(np.uint8)  # Convert to 0-255 scale
        # Pack the points but convert from rgb to bgr
        packed_points = [
            (x, y, z, b | (g << 8) | (r << 16))
            for (x, y, z), (b, g, r) in zip(points, colors)
        ]

        # Create ROS PointCloud2 message 
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_link'  # Replace with your frame
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        cloud_msg = point_cloud2.create_cloud(header, fields, packed_points)

        # Publish the point cloud
        self.pointcloud_publisher.publish(cloud_msg)
    
    def marker_timer_callback(self):
        """Periodically publish markers using the latest apple data."""
        if self.apple_centers is not None and self.apple_radii is not None:
            self.publish_markers(self.apple_centers, self.apple_radii)
            

def main(args=None):
    rclpy.init(args=args)
    apple_prediction_node = ApplePredictionPreSaved()
    rclpy.spin(apple_prediction_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()