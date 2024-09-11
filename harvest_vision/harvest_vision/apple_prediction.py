#!/usr/bin/env python3

# ROS2
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, PoseArray
from harvest_interfaces.srv import ApplePrediction
# TF2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import tf2_geometry_msgs
# Image processing
import cv2
import numpy as np
from ultralytics import YOLO
# pointcloud reconstruction
import open3d as o3d
from sphere_ransac import Sphere
# realsense library
import pyrealsense2 as rs


class ApplePredictionRS(Node):
    def __init__(self):
        '''Uses realsense 435i to get RGBD image, uses YOLO to segment apples and then creates pointcloud.
        Then uses RANSAC to fit a sphere to each apple to estimate the position and radius.'''
        super().__init__("apple_prediction_RS_node")

        ### SERVICE
        self.prediction_srv = self.create_service(ApplePrediction, "apple_prediction", self.prediction_callback_srv)

        ### PUBLISHER
        self.marker_pub = self.create_publisher(MarkerArray, "apple_markers", 10)

        ### YOLO SETUP
        self.model = YOLO("train6/weights/best.pt")  # pretrained YOLOv8n model
        if self.model: 
            self.get_logger().info("Succesfully loaded YOLO model.") 
        else:
            self.get_logger().error("Could not load YOLO model.") 
        
        ### REALSENSE SETUP
        # get camera info
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device() 
        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            self.get_logger().error("Realsense RGB camera could not be found")
        else:
            self.get_logger().info("Realsense camera found.")
        # set resolution and format
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)

        ### Tf2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        ### vARS
        self.debug_flag = False
        self.marker_counter = 0
        self.confidence_thresh = 0.7
        self.lower_rad_bound = 0.03 
        self.upper_rad_bound = 0.06
        self.ransac_thresh = .0001
        self.ransac_iters = 10000


    def prediction_callback_srv(self, request, response):
        rgb_image, depth_image = self.take_picture()
        apple_masks = self.segment_apples(rgb_image)
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        centers, radii = self.get_apple_centers(rgb_image, depth_image, apple_masks)
        transformed_poses = self.transform_apple_poses(centers)
        self.publish_markers(transformed_poses, radii)
        response.apple_pose = transformed_poses
        return response


    def transform_apple_poses(self, apple_poses):
        transformed_poses = PoseArray()
        for i in apple_poses:
            origin = PoseStamped()
            origin.header.frame_id = "rs_camera_link"
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
        markers = MarkerArray()
        for i in len(apple_poses.poses):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = self.marker_counter
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            # Set the scale
            marker.scale.x = apple_radii[i]
            marker.scale.y = apple_radii[i]
            marker.scale.z = apple_radii[i]
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
            self.get_logger().info(f"Apple found at: [{apple_poses.poses[i].pose.position.x}, {apple_poses.poses[i].pose.position.y}, {apple_poses.poses[i].pose.position.z}] with radius {apple_radii[i]}")
        self.marker_pub.publish(markers)


    def segment_apples(self, image):
        # returns masks of apples, each apple has its own mask. 0 for no apple, 255 for apple.  
        # predict segmentation masks using model
        results = self.model(image, conf=self.confidence_thresh)[0]
        apple_masks = []

        # creates a mask for each apple detected over the original image
        for i in results:
            # create empty mask
            img_h, img_w = results.masks.orig_shape
            mask = np.zeros((img_h, img_w), dtype=np.uint8)
            # fill in mask with white where predicted apple segmentation is
            cv2.fillPoly(mask, np.int32([i.masks.xy]), (255, 255, 255))
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


    def take_picture(self):
        # returns a segmented 
        # Start streaming
        profile = self.pipeline.start(self.config)
        align_to = rs.stream.color
        align = rs.align(align_to)
        flush_count = 0
        while True:
            # Get frameset of color and depth
            frames = self.pipeline.wait_for_frames()
            # Align the depth frame to color frame
            aligned_frames = align.process(frames)
            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            # aligned_depth_frame = rs.spatial_filter().process(aligned_depth_frame)
            color_frame = aligned_frames.get_color_frame()
            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue
            # flush frames to make sure we get a good image
            elif flush_count < 30:
                flush_count += 1
                continue
            else:
                # get images and get segmentation mask from YOLO
                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                break
        # stop realsense pipeline
        self.pipeline.stop()
        return color_image, depth_image

    def ransac_apple_estimation(self, pcd):
        center = None
        radius = None
        sph_ransac = Sphere()
        center, radius, inliers = sph_ransac.fit(pcd, ransac_thresh=self.ransac_thresh, maxIteration=self.ransac_iters, 
                                                 lower_rad_bound=self.lower_rad_bound, upper_rad_bound=self.upper_rad_bound)

        return center, radius

    def get_apple_centers(self, rgb, depth, masks):
        #TODO check that apple is far enough, check that apples have been identified. 
        apple_centers = []
        apple_radii = []
        visualization = []
        for mask in masks: 
            # segment only the apple portions
            depth_segmented = np.where(mask, depth, 0)
            # create RGBD image (NEEDS TO BE IN RGB FORMAT NOT BGR TO LOOK RIGHT, doesnt super matter for anything other than visualization)
            rgb_pc = o3d.geometry.Image(rgb)
            depth_pc = o3d.geometry.Image(depth_segmented)
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_pc, depth_pc, convert_rgb_to_intensity=False)
            # Creates pointcloud using camera intrinsics from Realsense 435i
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
                rgbd_image,
                o3d.camera.PinholeCameraIntrinsic(width=848, height=480, fx=609.6989, fy=609.8549, cx=420.2079, cy=235.2782))
            # Transforms pointcloud to be facing the correct direction.
            pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            # o3d.visualization.draw_geometries([pcd])
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
            o3d.visualization.draw_geometries(visualization)

        return apple_centers, apple_radii

            

def main(args=None):
    rclpy.init(args=args)
    apple_prediction_node = ApplePredictionRS()
    rclpy.spin(apple_prediction_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()