#!/usr/bin/env python3

# ROS
import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
# Interfaces
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, PoseArray, Twist
from std_msgs.msg import Int32, Float32

from rclpy.qos import (QoSProfile,
    ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)

# Image processing
from cv_bridge import CvBridge
import cv2
import math
import numpy as np
import copy
from pycpd import RigidRegistration

# TF2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import tf2_geometry_msgs
#Kalman
from filterpy.kalman import KalmanFilter

# YOLO model
from ultralytics import YOLO
import torch

from enum import Enum

class LocalPlanner(Node):
    ### Control of state
    class VisualServoState(Enum):
        IDLE = 0
        CONSTELLATION_ALIGNMENT = 1
        BBOX_CENTERING = 2
        APPROACHING = 3
        DONE = 4

    def __init__(self):
        super().__init__('local_planner_node')
        ### Subscribers/ Publishers
        # This is the actual palm camera (fake or real)
        self.camera_subscription = self.create_subscription(Image, 'gripper/rgb_palm_camera/image_raw', self.rgb_servoing_callback, 10)
        self.cam_info_pub = self.create_subscription(CameraInfo, '/gripper/rgb_palm_camera/camera_info', self.camera_info_callback, 1)
            
        # The projected locations
        self.apple_loc_sub = self.create_subscription(PoseArray, "gripper/apple_locs", self.proj_apple_locs_callback, 10)
        # Current selected apple - should be set before this service is called
        self.apple_indx_sub = self.create_subscription(Int32, "start_harvest/current_apple_index", self.apple_index_callback, 10)

        # Depth sub not needed if not going forward. Left in case the use case changes in the future. 
        self.depth_sub = self.create_subscription(Float32, "gripper/tof/depth_raw", self.depth_callback, 10)

        # Timer to get camera image
        self.ts = ApproximateTimeSynchronizer([self.camera_subscription], 30, 0.05, )
        self.ts.registerCallback(self.rgb_servoing_callback)

        ### Tf2 for mapping camera location to end of arm/tool location
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Image showing the yolo boxes and the projected points in the gripper camera
        self.debug_image_pub = self.create_publisher(Image, "visual_servo/image_debug", 
                                                     QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                                                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                                                history=HistoryPolicy.KEEP_LAST, depth=1))
                                                    
        # Publisher to end effector servo controller, sends velocity commands
        self.servo_publisher = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        # specify reentrant callback group 
        r_callback_group = ReentrantCallbackGroup()

        ### Services
        # Service to start the local planner sequence
        self.start_service = self.create_service(Trigger, "start_visual_servo", self.start_sequence_srv_callback, callback_group=r_callback_group)
        
        ### Servo controller params
        self.declare_parameter("vservo_model_path", "NA")
        self.declare_parameter("vservo_yolo_conf", 0.85)
        self.declare_parameter("vservo_accuracy_px", 10)
        self.declare_parameter("vservo_smoothing_factor", 6.0)
        self.declare_parameter("vservo_max_vel", 0.6)
        self.yolo_conf = self.get_parameter("vservo_yolo_conf").get_parameter_value().double_value
        self.target_pixel_accuracy = self.get_parameter("vservo_accuracy_px").get_parameter_value().integer_value
        self.smoothing_factor = self.get_parameter("vservo_smoothing_factor").get_parameter_value().double_value
        self.max_vel = self.get_parameter("vservo_max_vel").get_parameter_value().double_value
        self.model_path = self.get_parameter("vservo_model_path").get_parameter_value().string_value

        # For controlling transition between states
        self.state = LocalPlanner.VisualServoState.IDLE
        self.rate = self.create_rate(1)
        self.stall_count = 0

        ### MUST BE 0.0 unless moving forward and using TOF distance as a stopping condition (In APPROACHING state).
        self.z_speed = 0.0
        self.prev_pos = []

        # Estimate of width of apple in meters
        self.apple_width = 0.075

        # The apple locations produced by the apple prediction node and projected into the gripper frame
        self.last_projected_apple_locs = None
        self.projected_apple_locs = None
        self.vec_proj_image_motion = [0, 0, 0]

        # Current target apple 
        self.current_apple_index = -1
        
        # Camera/image info
        self.width = 100
        self.height = 100
        self.k = np.identity(3)

        # The ones from yolo
        self.yolo_last_apple_centers = None  
        self.yolo_last_apple_radii = []
        self.yolo_apple_centers = None  
        self.yolo_apple_radii = []
        self.vec_yolo_image_motion = [0, 0]
        self.yolo_apple_index = -1
        self.last_yolo_apple_index = -1

        # Track alignment error
        self.vec_align_error = np.zeros((0, 2))

        # Save the value of the current time of flight, averaged over time
        self.tof_depth = -1.0

        ### Image Processing
        self.br = CvBridge()
        self.model = YOLO(self.model_path)  # pretrained YOLOv8n model
        self.model.model = torch.compile(self.model.model)

        ### Kalman
        self.prev_vel = [0,0]
        self.kf_pos = KalmanFilter (dim_x=6, dim_z=4)

    def init_kalman(self):
        # Kalman filter setup, takes in a measured x,y position and velocity and estimates position, velocity and acceleration
        self.kf_pos.x = np.array([[0],
                        [0],
                        [0],
                        [0],
                        [0],
                        [0]])
        self.kf_pos.F = np.array([[1,.14,.0098,0,0,0],
                                  [0,1,.14,0,0,0],
                                  [0,0,1,0,0,0],
                                  [0,0,0,1,.14,.0098],
                                  [0,0,0,0,1,.14],
                                  [0,0,0,0,0,1,]])
        self.kf_pos.H = np.array([[1,0,0,0,0,0],
                                  [0,1,0,0,0,0],
                                  [0,0,0,1,0,0],
                                  [0,0,0,0,1,0]])
        self.kf_pos.P *= 1
        self.kf_pos.R = np.array([[10, 0,0,0],
                                  [0, 10,0,0],
                                  [0, 0,10,0],
                                  [0, 0,0,10]]) * 2
        self.kf_pos.Q = np.eye(6) * 1

    # ================================================================== Service
    def start_sequence_srv_callback(self, request, response):
        # Starts servo node it it hasnt been started already
        self.get_logger().info("Activating servo node...")
        if not (self.state is LocalPlanner.VisualServoState.IDLE or self.state is LocalPlanner.VisualServoState.DONE):
            self.get_logger().warn(f"Starting Visual servo, but not in done or idle state {self.state.name}")
        
        self.state = LocalPlanner.VisualServoState.CONSTELLATION_ALIGNMENT
        self.stall_count = 0
        # Reset all of these
        self.yolo_last_apple_centers = None
        self.yolo_apple_centers = None
        self.last_projected_apple_locs = None
        self.projected_apple_locs = None
        self.vec_proj_image_motion = [0, 0, 0]
        self.vec_yolo_image_motion = [0, 0]
        self.vec_align_error = np.zeros((0, 2))

        self.get_logger().info("Starting visual arm servoing...")
        # Servos until we are in front of apple
        self.init_kalman()
        try:
            while rclpy.ok() and self.start_flag:
                self.get_logger().info("Servoing arm in front of apple...")
                self.rate.sleep()
        except KeyboardInterrupt:
            pass

        self.get_logger().info("Finished servoing in front of the apple, state {self.state.name}")
        if self.state is LocalPlanner.VisualServoState.DONE:
            response.success = True
        else:
            response.success = False

        # Back to not doing anything
        self.state = LocalPlanner.VisualServoState.IDLE
        return response

    # ================================================================== Publication/subscriptions/visualization
    def proj_apple_locs_callback(self, proj_pts_msg: PoseArray):
        """ From the gripper camera, the projected points of the apple centers """
        self.projected_apple_locs = np.zeros((len(proj_pts_msg.poses), 3))
        for indx, p in enumerate(proj_pts_msg.poses):
            self.projected_apple_locs[indx, 0] = p.position.x
            self.projected_apple_locs[indx, 1] = p.position.y
            self.projected_apple_locs[indx, 2] = p.position.z

    def apple_index_callback(self, indx_msg: Int32):
        self.current_apple_index = int(indx_msg.data)
        self.get_logger().info(f"Processing 3D apple {self.current_apple_index}")
        self.debug_image()

    def depth_callback(self, msg: Float32):
        """ Store a running average """
        if self.tof_depth < 0.0:
            # Just starting
            self.tof_depth = float(msg.data)
        else:
            alpha = 0.2
            self.tof_depth = alpha * float(msg.data) + (1.0 - alpha) * self.tof_depth

    def camera_info_callback(self, msg: CameraInfo):
        """ Store the focal length for calculating actual x,y distances from image distaces"""
        self.k = np.array(msg.k)
        self.k.reshape((3, 3))
        self.get_logger().info(f"Gripper palm camera fx fy {self.fx} {self.fy}")

    def create_debug_image(self, img):
        """ Show the projected apple centers, the yolo bounding boxes
        @param img is an opencv image format of the camera image"""
        for pt in self.projected_apple_locs:
            cv2.drawMarker(img, (pt[0], pt[1]), color=(255, 255, 255), markerType=cv2.MARKER_CROSS, 
                           markerSize=4, thickness=2)    
        for pt, r in zip(self.apple_centers, self.apple_radii):
            cv2.circle(img, (pt[0], pt[1]), r, color=(255, 0, 0), thickness=2) 

        if self.current_apple_index >= 0 and self.current_apple_index < len(self.projected_apple_locs):
            pt = self.projected_apple_locs[self.current_apple_index]
            cv2.drawMarker(img, (pt[0], pt[1]), color=(255, 15, 155), markerType=cv2.MARKER_X, 
                           markerSize=4, thickness=2)    

        cv2.imwrite('yolo.png', img)
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        
        img_msg.header.frame_id = "gripper_palm_camera_optical_link"
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.debug_image_pub.publish(img_msg)

    def get_depth(self):
        """ Get an estimate of the depth, either time of flight or where the apple is suppose to be
        Probably should put something in here to make sure the depth value is recent..."""
        if self.tof_depth >= 0.0:
            return self.tof_depth
        
        try:
            return self.projected_apple_locs[self.current_apple_index][2]
        except IndexError:
            self.get_logger().warn(f"No valid depth estimate")
            return 10.0

    # ================================================================== Scoring match functions
    def _estimate_alignment_error(self):
        """Estimate how much the camera alignment is off in the image plane, and pick the best yolo box"""
        self.yolo_apple_index, vec_trans = self._match_points(self.projected_apple_locs[:, 0:2], self.yolo_apple_centers, self.current_apple_index)

        alpha = 0.2
        self.vec_align_error[0:2] = alpha * vec_trans + (1.0 - alpha) * self.vec_align_error[0:2]

        pts_proj = np.array(shape=self.projected_apple_locs)
        pts_proj[:, 0] -= vec_trans[0]
        pts_proj[:, 1] -= vec_trans[1]

        pts_ndc = cv2.undistort(pts_proj, self.k)
        pts_ang = np.abs(np.arctan(pts_ndc))
        best_match = 0
        dist_center = 1e30
        for row_indx in range(0, pts_ang.shape[0]):
            pt_ang = self.projected_apple_locs[row_indx, :]
            if pt_ang[0] + pt_ang[1] < dist_center:
                best_match = row_indx
                dist_center = pt_ang[0] + pt_ang[1]

        pt = self.projected_apple_locs[best_match]
        pt_ang = pts_ang[best_match]
        dist_x = pt[2] * np.cos(pt_ang[0])
        dist_y = pt[2] * np.cos(pt_ang[1])
        if dist_x < self.apple_width and dist_y < self.apple_width:
            self.get_loger().info(f"Updating depth estimate {self.get_depth()}")
            self.vec_align_error[2] = alpha * self.get_depth() + (1.0 - alpha) * self.vec_align_error[2]

    def _score_match(self, pts1: list, pts1_vecs: np.array, pts2: list, pts2_vecs: np.array, match_index: list, missing: float = 0.0):
        """ Score the match by how well the vectors between points match"""
        score = 0.0
        for indx, match in enumerate(match_index):
            if match == -1:
                score += missing
                continue

            pt1 = pts1[indx]
            pt2 = pts2[match]
            for neigh in 

    def _match_points(self, pts1: np.array, pts2: np.array, indx_in_pts2: int):
        # Initialize the solver (handles missing points seamlessly via a probabilistic framework)
        reg = RigidRegistration(X=pts2, Y=pts1)

        # Run registration (returns the transformed array, and the math parameters)
        pts1_to_pts_2, (scale, rotation_matrix, translation_vector) = reg.register()

        pt2 = pts2[indx_in_pts2]
        diff_x = (pts1_to_pts_2[:, 0] - pt2[0])
        diff_y = (pts1_to_pts_2[:, 1] - pt2[1])
        diff = diff_x ** 2 + diff_y * 2
        pt2_indx = np.argmin(diff)

        self.get_logger().info(f"Matching points, found {scale}, {rotation_matrix}, {translation_vector}")
        return pt2_indx, translation_vector
      
    # ================================================================== YOLO
    def _estimate_image_movement_from_yolo(self):
        """Find the best match between the last yolo boxes and this one, and calculate an estimated shift"""
        self.yolo_apple_index, self.vec_yolo_image_motion = self._match_points(self.yolo_last_apple_centers, 
                                                                               self.yolo_apple_centers, 
                                                                               self.last_yolo_apple_index)

    def _run_yolo(self, rgb):
        # Convert to opencv format from msg
        image = self.br.imgmsg_to_cv2(rgb, "bgr8")

        self.height = image.height
        self.width = image.width

        # Get apple bounding boxes from yolo model
        # results = self.model(image, conf=self.yolo_conf, device='cuda', verbose=False)[0]
        results = self.model(image, conf=self.yolo_conf, verbose=False)[0]
        self.yolo_apple_centers = np.zeros((len(results), 2))
        self.yolo_apple_radii = []
        for indx, box in enumerate(results):
            # find center of each bounding box and calculate distance to center of image
            x,y,w,h = box.boxes.xyxy.cpu().numpy()[0]
            self.apple_centers[indx, 0] = (x + w) / 2
            self.apple_centers[indx, 1] = (y + h) / 2
            self.apple_radii.append(0.5 * (w + h))

    # ================================================================== Helper methods
    def normalize(self, val, minimum, maximum):
        # Normalizes val between min and max
        return (val - minimum) / (maximum-minimum)

    def _get_current_pose(self):
        # Get the current optical frame pose
        origin = PoseStamped()
        origin.header.frame_id = "gripper_palm_camera_optical_link"
        new_pose = self.tf_buffer.transform(origin, "tool0", rclpy.duration.Duration(seconds=1))

    def transform_optical_to_ee(self, x, y):
        # Transforms from optical frame to end effector frame
        origin = PoseStamped()
        origin.header.frame_id = "gripper_palm_camera_optical_link"
        origin.pose.position.x = x
        origin.pose.position.y = y
        origin.pose.position.z = 0.0
        origin.pose.orientation.x = 0.0
        origin.pose.orientation.y = 0.0
        origin.pose.orientation.z = 0.0
        origin.pose.orientation.w = 1.0
        new_pose = self.tf_buffer.transform(origin, "tool0", rclpy.duration.Duration(seconds=1))
        return new_pose

    def _exponential_vel(self, vel):
        # Exponential function for determining velocity scaling based on pixel distance from center of the camera 
        # Bounded by 0 and max_vel
        return (1 - np.exp((-self.smoothing_factor / 2) * vel)) * self.max_vel

    def calculate_euclidean(self, pos1, pos2):
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    # ================================================================== Check location methods
    def _estimate_image_movement_from_projected(self):
        """ Take the last set of points and these and see how far the points moved in the image plane """

        self.vec_proj_image_motion = [0, 0, 0]
        if self.last_projected_apple_locs.shape[0] == self.projected_apple_locs.shape[0]:
            # These SHOULD be the same size, unless the apple locations get regenerated
            diff = self.last_projected_apple_locs - self.projected_apple_locs
            # Average along the rows
            self.vec_proj_image_motion = diff.mean(axis=0)
            self.get_logger().info(f"Projected image motion {self.vec_proj_image_motion}")

    def _aligned_xy_and_depth(self, image, distance: float):
        """ Return True if xy is within a threshold and close enough"""
        centered_apple = self.projected_apple_locs[self.yolo_apple_index]
        apple_x = centered_apple[0]
        apple_y = centered_apple[1]
        viewing_ang = np.pi / 2.0
        dist_x_im = viewing_ang * np.abs(apple_x - image.shape[1] // 2) / image.shape[1]s = s
        dist_y_im = viewing_ang * np.abs(apple_y - image.shape[0] // 2) / image.shape[0]

        dist_x = np.arctan2(dist_x_im, distance)
        dist_y = np.arctan2(dist_y_im, distance)

        approx_apple_width = 0.075  # Meters
        if dist_x > approx_apple_width * 0.5:
            return False
        if dist_y > approx_apple_width * 0.5:
            return False
        if self.get_depth() > approx_apple_width:
            return False
        return True 

    def _align_constellation_left_right(self):
        """ Until we have a good match of the projected points and the yolo boxes, do a diagonal x,y movement to
        bring the best-guess yolo box through the middle of the image
        """
        z_dist = []
        for pt, r in zip(self.yolo_apple_centers, self.yolo_apple_radii):
            # find center of each bounding box and calculate distance to center of image
            x,y,w,h = i.boxes.xyxy.cpu().numpy()[0]
            self.apple_centers.append([(x + w)/2, (y+h)/2])
            self.apple_radii.append(0.5 * (w + h))

            if self.first_servo:
                dist_to_apple = self.calculate_euclidean([width//2,height//2], self.apple_centers[-1])
            else: 
                dist_to_apple = self.calculate_euclidean(self.prev_pos, self.apple_centers[-1])

            z_dist.append(dist_to_apple)

    def _match_yolo_projected(self):
        """ Calculate the best match between the current yolo centers and the projected locations
           Calculates an estimated error vector as well"""
        for pt, r in zip(self.yolo_apple_centers, self.yolo_apple_radii):
            # find center of each bounding box and calculate distance to center of image
            x,y,w,h = i.boxes.xyxy.cpu().numpy()[0]
            self.apple_centers.append([(x + w)/2, (y+h)/2])
            self.apple_radii.append(0.5 * (w + h))

            if self.first_servo:
                dist_to_apple = self.calculate_euclidean([width//2,height//2], self.apple_centers[-1])
            else: 
                dist_to_apple = self.calculate_euclidean(self.prev_pos, self.apple_centers[-1])

            z_dist.append(dist_to_apple)


    # ================================================================== Create twist methods
    def _align_constellation(self):
        """ If we're just starting, move at a diagonal (whichever moves the best-match bbox to the middle)
            if we've done this a few times and we have a bbox in the middle, mark as done"""
        
        # Best guess for where the center of the selected apple is (based on match to yolo)
        centered_apple = self.yolo_apple_centers[self.yolo_apple_index]
        apple_x = centered_apple[0]
        apple_y = centered_apple[1]
        center_x = self.width // 2
        center_y = self.height // 2    

        # Get x magnitude and set velocity with exponential function * max_vel
        if apple_x >= center_x:
            new_x = self.max_vel * 0.5
        else:
            new_x = -self.max_vel * 0.5
        # Get y magnitude
        if apple_y >= center_y:
            new_y = self.max_vel * 0.5
        else:
            new_y = -self.max_vel * 0.5

        if self.stall_count > 2:
            if 
        return new_x, new_y

        
    def _align_xy_bbox(self, image):
        # Centerpoints
        centered_apple = self.projected_apple_locs[self.yolo_apple_index]
        apple_x = centered_apple[0]
        apple_y = centered_apple[1]
        center_x = image.shape[1] // 2
        center_y = image.shape[0] // 2
        # Get x magnitude and set velocity with exponential function * max_vel
        if apple_x >= center_x:
            new_x = self._exponential_vel(self.normalize(apple_x, center_x, image.shape[1]))
        else:
            new_x = -self._exponential_vel(1-self.normalize(apple_x, 0, center_x))
        # Get y magnitude
        if apple_y >= center_y:
            new_y = self._exponential_vel(self.normalize(apple_y, center_y, image.shape[0]))
        else:
            new_y = -self._exponential_vel(1 - self.normalize(apple_y, 0, center_y))
        return new_x, new_y


    def create_twist(self, new_x: float, new_y: float, distance: float):
        # save previous velocities to feed into Kalman filter
        self.prev_vel[0] = new_x
        self.prev_vel[1] = new_y
        # Transform from optical frame to end effector frame
        transformed_vector = self.transform_optical_to_ee(new_x, new_y)

        # Create Twiststamped message in end effector frame
        vel_vec = Twist()
        vel_vec.linear.x = transformed_vector.pose.position.x
        vel_vec.linear.y = transformed_vector.pose.position.y

        # If we are within picking distance then stop, otherwise move forward 
        # if we are approaching, also set z
        if self.state is LocalPlanner.VisualServoState.APPROACHING:
            vel_vec.twist.linear.z = self.z_speed
        return vel_vec
    
    def rgb_servoing_callback(self, rgb):
        """ Called when we get a new camera image. If we're not idle/done we do the following
        Calculate the difference (in image space) from the last projected points to the current ones
        Run yolo and try to match the old boxes to the new ones (also calculates an image space difference)
        If the stars <sic> are aligned, then the image space motion of both of those should be the same
        If we're just starting, we've lost alignment, or there's no good bbox match for a few frames,
           do a full match between the projected points and the yolo ones
        Otherwise, we can assume the match is still correct and just propagate the current apple center
        Also track an estimate of the mis-alignment between the point cloud and it's projection on the image plane"""

        # Run YOLO to get all bounding boxes    
        self._run_yolo(rgb)

        # Now try to do a yolo to yolo match
        self._estimate_image_movement_from_yolo()

        # Projected point motion last frame
        self._estimate_image_movement_from_projected()

        vel_vec = TwistStamped()
        vel_vec.header.stamp = self.get_clock().now().to_msg()
        vel_vec.header.frame_id = "tool0"
        vel_vec.twist.linear.z = 0.0
        vel_vec.twist.linear.x = 0.0
        vel_vec.twist.linear.y = 0.0
        
        if self.state is LocalPlanner.VisualServoState.CONSTELLATION_ALIGNMENT:
            # Estimate the yolo->project points alignment error
            self._estimate_alignment_error()

            # Haven't gotten the selected apple in the middle of the image - try to do constellation alignment
            aligned_constellation, vel_vec.twist = self._check_constellation_alignment()

            # If the overall alignment looks good, move the gripper to center on the yolo box that is closest to the center of the image
            if aligned_constellation:
                self.state = LocalPlanner.VisualServoState.BBOX_CENTERING

        if self.state is LocalPlanner.VisualServoState.BBOX_CENTERING:
            aligned_apple, vel_vec.twist = self.check_apple_alignment()

            if aligned_apple:
                # Seed the Kalman filter with the projected location of the selected apple
                self.state = LocalPlanner.VisualServoState.APPROACHING
                # This should probably be the centered YOLO box and/or 0, 0
                xy_loc = self.yolo_apple_centers[self.yolo_apple_index]
                self.kf_pos.x = np.array([[xy_loc[0]],[0],[0],[xy_loc[1]],[0],[0]])

        if self.state is LocalPlanner.VisualServoState.APPROACHING:
            # Largely using the Kalman filter(s) to keep the apple centered
            # update Kalman filter with the measuered position, and previous measured velocities
            self.kf_pos.predict()
            if self._match_yolo():
                current_apple = self.yolo_apple_centers[self.yolo_apple_index]
                self.kf_pos.update(np.array([[current_apple[0]],[-self.prev_vel[0]], [current_apple[1]], [-self.prev_vel[1]]]))
                # get estimate from Kalman filter for closest apple location
                state = self.kf_pos.x
                closest_apple = [float(state[0][0]), float(state[3][0])]

                self.prev_pos = closest_apple

                # check if camera is centered on apple or not within our pixel target accuracy threshold, if it is then publish a 0 velocity and exit loop
                if self.calculate_euclidean([width//2, height//2], closest_apple) < self.target_pixel_accuracy:
                    vel_vec = TwistStamped()
                    vel_vec.header.stamp = self.get_clock().now().to_msg()
                    vel_vec.header.frame_id = "tool0"
                    vel_vec.twist.linear.z = 0.0
                    vel_vec.twist.linear.x = 0.0
                    vel_vec.twist.linear.y = 0.0
                    self.servo_publisher.publish(vel_vec)
                    self.start_flag = False
            else:
                # makes sure that transforms are not failing
                try:
                    # Creates servo vector which servos the arm towards the closest apple
                    ## THE 10 IS A CONSTANT DISTANCE VALUE BECAUSE WE ARE SERVOING IN PLACE ON A PLANE
                    ## IF NEEDED YOU CAN PASS IN A MEASURED DISTANCE AS A STOPPING CONDITION FOR THE SERVOING
                    vec = self.create_servo_vector(closest_apple, image, 10)
                    self.servo_publisher.publish(vec)
                except TransformException as e:
                    self.get_logger().info(f'Transform failed: {e}')
            else:
                # if we dont detect any apples then stay in place with 0 velocity
                vel_vec = TwistStamped()
                vel_vec.header.stamp = self.get_clock().now().to_msg()
                vel_vec.header.frame_id = "tool0"
                vel_vec.twist.linear.z = 0.0
                vel_vec.twist.linear.x = 0.0
                vel_vec.twist.linear.y = 0.0
                self.servo_publisher.publish(vel_vec)
                self.stall_count += 1
            
            if self.stall_count > 10:
                self.get_logger().error("STALLED after 10 attempts to servo, could not locate any apples in FOV.")
                self.start_flag = False

        # Save the last set of projected apple locations
        self.last_projected_apple_locs = copy.deepcopy(self.projected_apple_locs)
        # Save the last set of yolo locations
        self.yolo_last_apple_centers = copy.deepcopy(self.yolo_apple_centers)
        self.yolo_last_apple_radii = copy.deepcopy(self.yolo_apple_radii)
        self.last_yolo_apple_index = self.yolo_apple_index

        self.create_debug_image(img=rgb)

        self.servo_publisher.publish(vel_vec)



def main(args=None):
    rclpy.init(args=args)
    local_planner = LocalPlanner()
    executor = MultiThreadedExecutor()
    rclpy.spin(local_planner, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()