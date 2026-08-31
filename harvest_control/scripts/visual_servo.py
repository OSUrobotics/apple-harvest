#!/usr/bin/env python3

# ROS
import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
# Interfaces
from sensor_msgs.msg import Image, CameraInfo, Range
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, PoseArray, Twist
from std_msgs.msg import Int32, Float32

from rclpy.qos import (QoSProfile,
    ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)

import message_filters

# Image processing
from cv_bridge import CvBridge
import cv2
import torch
import math
import numpy as np
import copy
from pycpd import RigidRegistration

# TF2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
#Kalman
from filterpy.kalman import KalmanFilter

# YOLO model
from ultralytics import YOLO

from enum import Enum
from collections import deque
from math import sqrt
from scipy.optimize import linear_sum_assignment

from ament_index_python.packages import get_package_share_directory
from pathlib import Path

class LocalPlanner(Node):
    ### Control of state
    class VisualServoState(Enum):
        IDLE = 0
        CONSTELLATION_ALIGNMENT = 1
        BBOX_CENTERING = 2
        APPROACHING = 3
        BAILING = 4
        DONE = 5

    class RunningFilterStats:
        def __init__(self, size: int, value: float = 0.0):
            self.n = size
            self.data = deque([value] * size, maxlen=size)
            self.mean = value
            self.variance = 0.0

        def update(self, x: float) -> tuple[float, float]:
            old_val = self.data[0]
            # O(1) mean update
            new_mean = self.mean + (x - old_val) / self.n
            
            # O(1) variance update using Welford-style sliding adjustment
            self.variance += (x - old_val) * ((x - new_mean) + (old_val - self.mean)) / (self.n - 1)
            
            self.mean = new_mean
            self.data.append(x)
            
            std_dev = sqrt(max(0.0, self.variance))
            return self.mean, std_dev

    def __init__(self):
        super().__init__('local_planner_node')

        ### Servo controller params
        try:
            vservo_path = Path(get_package_share_directory("harvest_vision")) / "yolo_networks" / "v9e.pt"
            vservo_path = str(vservo_path)
        except FileNotFoundError:
            vservo_path = "NA"

        self.declare_parameter("vservo_model_path", vservo_path)
        self.declare_parameter("vservo_yolo_conf", 0.85)
        self.declare_parameter("vservo_accuracy_px", 10)
        self.declare_parameter("vservo_smoothing_factor", 6.0)
        self.declare_parameter("vservo_max_vel", 0.6)

        ### Subscribers/ Publishers

        ## The 30 fps sampling for image, 2d points, eef
        # Define the quality of service profile if needed (must match the publisher)
        qos_profile = rclpy.qos.qos_profile_sensor_data

        # This is the actual palm camera (fake or real)
        self.camera_subscription = message_filters.Subscriber(self, Image, 'gripper/rgb_palm_camera/image_raw', qos_profile)
        # The projected locations
        self.apple_loc_subscription = message_filters.Subscriber(self, PoseArray, "gripper/apple_locs", qos_profile)
        # Depth at time of image capture
        self.depth_subscription = message_filters.Subscriber(self, Range, "gripper/tof", qos_profile)

        ## Subscriptions that only need to happen once at the start of the service
        # Current selected apple - should be set before this service is called
        self.apple_indx_sub = self.create_subscription(Int32, "start_harvest/current_apple_index", self.apple_index_callback, 10)

        # Camera info (to get the k matrix)
        self.cam_info_pub = self.create_subscription(CameraInfo, '/gripper/rgb_palm_camera/camera_info', self.camera_info_callback, 1)

        # Synchronize
        self.ats = message_filters.ApproximateTimeSynchronizer([self.camera_subscription, self.apple_loc_subscription, self.depth_subscription], 
                                                               queue_size=5, slop=0.01)
        self.ats.registerCallback(self.servoing_callback)

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
        self.start_service = self.create_service(Trigger, "/start_visual_servo", self.start_sequence_srv_callback, callback_group=r_callback_group)
        
        self.yolo_conf = self.get_parameter("vservo_yolo_conf").get_parameter_value().double_value
        self.target_pixel_accuracy = self.get_parameter("vservo_accuracy_px").get_parameter_value().integer_value
        self.smoothing_factor = self.get_parameter("vservo_smoothing_factor").get_parameter_value().double_value
        self.max_vel = self.get_parameter("vservo_max_vel").get_parameter_value().double_value

        # For controlling transition between states
        self.state = LocalPlanner.VisualServoState.IDLE
        self.rate = self.create_rate(1)
        self.stall_count = 0

        ### Speed when approaching
        self.starting_depth = 0.0
        self.max_z_speed = 0.1

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
        self.yolo_apple_centers = None  
        self.yolo_apple_radii = []
        self.yolo_apple_index = -1
        self.yolo_last_apple_centers = None  
        self.yolo_last_apple_radii = []
        self.last_yolo_apple_index = -1
        self.vec_yolo_image_motion = [0, 0]
        
        # Track camera alignment error in x, y, and z with a smoothed average
        self.vec_proj_to_yolo = None
        # This is tracking how well yolo is matching from one frame to the next
        self.vec_yolo_ee_match = None
        self.yolo_match_error = None
        self.aligned_depth = 0
        self._init_error_tracking()
        
        # Save the value of the current time of flight, averaged over time
        self.tof_depth = -1.0

        ### Image Processing
        self.br = CvBridge()
        self.model = YOLO(self.get_parameter("vservo_model_path").value)
        try:
            self.model.model.eval()
        except Exception:
            pass

        ### Kalman
        self.prev_vel = [0,0]
        self.kf_pos = KalmanFilter(dim_x=6, dim_z=4)

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

    def _init_error_tracking(self):
        self.vec_proj_to_yolo = []
        for _ in range(0, 3):
            self.vec_proj_to_yolo.append(self.RunningFilterStats(size=3, value= 0.0))

        self.vec_yolo_ee_match = self.RunningFilterStats(size=5, value=0.0)
        
        self.yolo_match_error = []
        for _ in range(0, 3):
            self.yolo_match_error.append(self.RunningFilterStats(size=5, value=0.0))

        self.aligned_depth = 0

    def _init_yolo(self):
        self.yolo_last_apple_centers = []
        self.yolo_last_apple_radii = []
        self.last_yolo_apple_index = -1

        self.yolo_apple_centers = []
        self.yolo_apple_radii = []
        self.yolo_apple_index = -1

        self.vec_yolo_image_motion = [0, 0]        

    # ================================================================== Service
    def done(self):
        if self.state is LocalPlanner.VisualServoState.DONE:
            return True
        if self.state is LocalPlanner.VisualServoState.IDLE:
            return True
        if self.state is LocalPlanner.VisualServoState.BAILING:
            return True
        return False
        
    def start_sequence_srv_callback(self, request, response):
        # Starts servo node it it hasnt been started already
        self.get_logger().info("Activating servo node...")
        if not (self.state is LocalPlanner.VisualServoState.IDLE or self.state is LocalPlanner.VisualServoState.DONE):
            self.get_logger().warn(f"Starting Visual servo, but not in done or idle state {self.state.name}")
        
        self.get_logger().info("Starting visual arm servoing...")

        self.state = LocalPlanner.VisualServoState.CONSTELLATION_ALIGNMENT
        self.stall_count = 0
        # Reset all of these
        self._init_yolo()
        self._init_error_tracking()
        self.init_kalman()
        self.starting_depth = 0.0

        # Loop until done
        try:
            while rclpy.ok() and not self.done():
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
    def proj_apple_locs(self, proj_pts_msg: PoseArray):
        """ From the gripper camera, the projected points of the apple centers """
        self.projected_apple_locs = np.zeros((len(proj_pts_msg.poses), 3))
        for indx, p in enumerate(proj_pts_msg.poses):
            self.projected_apple_locs[indx, 0] = p.position.x
            self.projected_apple_locs[indx, 1] = p.position.y
            self.projected_apple_locs[indx, 2] = p.position.z

    def apple_index_callback(self, indx_msg: Int32):
        self.current_apple_index = int(indx_msg.data)
        self.get_logger().info(f"Processing 3D apple {self.current_apple_index}")

    def depth_callback(self, msg: Float32):
        """ Store a running average """
        if self.tof_depth < 0.0:
            # Just starting
            self.tof_depth = float(msg.data)
        else:
            alpha = 0.2
            self.tof_depth = alpha * float(msg.data) + (1.0 - alpha) * self.tof_depth

        if self.starting_depth == 0.0:
            self.starting_depth = self.get_depth()

    def camera_info_callback(self, msg: CameraInfo):
        """ Store the focal length for calculating actual x,y distances from image distaces"""
        self.k = np.array(msg.k)
        self.k.reshape((3, 3))
        self.get_logger().info(f"Gripper palm camera fx fy {self.k}")

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

    # ================================================================== Get data methods
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

    def stopping_depth(self):
        return self.apple_width

    def _projected_xy_loc_with_shift(self):
        xy_loc = self.projected_apple_locs[self.current_apple_index]
        xy_loc[0] += self.vec_proj_to_yolo[0]
        xy_loc[1] += self.vec_proj_to_yolo[1]
        return xy_loc

    def _width_apple_in_image(self):
        return self.apple_width * self.k[0] / self.get_depth()

    # ================================================================== Update state methods
    def _kalman_filter_update(self):
        self.kf_pos.predict()

        xy_loc = self._projected_xy_loc_with_shift()
        if self.yolo_apple_index != -1:
            xy_loc = self.yolo_apple_centers[self.yolo_apple_index]

        self.kf_pos.update(np.array([[xy_loc[0]],[-self.prev_vel[0]], [xy_loc[1]], [-self.prev_vel[1]]]))

    def _estimate_proj_yolo_alignment_error(self):
        """Estimate how much the camera alignment is off in the image plane, and pick the best yolo box"""
        self.yolo_apple_index, vec_trans = self._match_points(self.projected_apple_locs[:, 0:2], self.yolo_apple_centers, self.current_apple_index)

        # Running filter + mean/std of the error between the project points and the yolo bboxes
        for indx in range(0, 2):
            self.vec_proj_to_yolo[indx].update(vec_trans[indx])

        # See if one of the shifted projected points lies under the cone for the time of flight sensor
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
            self.vec_proj_to_yolo[2].update(self.get_depth() - pt[2])
            self.aligned_depth += 1

    def _estimate_image_movement_from_projected(self):
        """ Take the last set of points and these and see how far the points moved in the image plane """

        self.vec_proj_image_motion = [0, 0, 0]
        if self.last_projected_apple_locs.shape[0] == self.projected_apple_locs.shape[0]:
            # These SHOULD be the same size, unless the apple locations get regenerated
            diff = self.last_projected_apple_locs - self.projected_apple_locs
            # Average along the rows
            self.vec_proj_image_motion = diff.mean(axis=0)
            self.get_logger().info(f"Projected image motion {self.vec_proj_image_motion}")
        else:
            self.get_logger().info(f"Projected image motion point set different sizes {self.last_projected_apple_locs.shape[0]}")

    # ================================================================== error functions
    def _score_match(self, pts1: list, pts2: list, vec_shift_pts1_to_pts2: list):
        """ Score the match by how many points match (within 1/2 apple width in image)"""
        dist_vals = np.zeros((len(pts1), len(pts2)))
        for indx2, p2 in enumerate(pts2):
            px = p2[0] - vec_shift_pts1_to_pts2[0]
            py = p2[1] - vec_shift_pts1_to_pts2[1]
            for indx1, p1 in enumerate(pts1):
                diff_x = np.abs(p1[0] - px)
                diff_y = np.abs(p1[1] - py)
                dist_vals[indx1, indx2] = diff_x + diff_y

        row_ind, col_ind = linear_sum_assignment(dist_vals)

        # View the optimal matching results
        print("Optimal Row Indices:", row_ind)  # Output: [0 1 2]
        print("Optimal Col Indices:", col_ind)  # Output: [1 0 2]

        # Calculate total minimum cost
        width_apple_in_image = self._width_apple_in_image()
        return np.count(dist_vals[row_ind, col_ind]) < width_apple_in_image

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
            
    def _update_error_metrics(self):
        """Call once the yolo and projected apple locs have been updated.
           Calculates cummulative error statistics"""
        
        # Yolo to projected points
        self._estimate_proj_yolo_alignment_error()
        
        # Yolo to yolo
        if len(self.yolo_apple_centers) > 0 and len(self.yolo_last_apple_centers) > 0:
            err_match = self._score_match(self.yolo_apple_centers, self.yolo_last_apple_centers, self.vec_yolo_image_motion)
            err_match /= np.min(len(self.yolo_apple_centers), len(self.yolo_last_apple_centers))
        else:
            err_match = 0
        # Put between 0 and 1
        self.vec_yolo_ee_match.update(err_match)
        self.get_logger().info(f"Match error yolo to yolo {err_match} of {len(self.yolo_apple_centers)}")

        # Yolo to projected points
        vec_move = (self.vec_proj_to_yolo[0].mean, self.self.vec_align_error[1].mean)
        err_match = self._score_match(self.projected_apple_locs, self.yolo_apple_centers, vec_move)
        self.yolo_match_error.update(err_match)
        self.get_logger().info(f"Match error yolo to projected {err_match} of {len(self.yolo_apple_centers)}")

    def _check_valid(self):
        """Check the error metrics"""
        if self.stall_count < 5:
            # Just let run a bit
            return True

        if self.stall_count > 10:
            if not self._check_constellation_alignment():
                self.get_logger().info(f"Bailing, variance in yolo to project diff big variance {self.vec_proj_to_yolo} ")
                return False
            
            if self.vec_proj_to_yolo[2].variance > 5 * self.apple_width:
                self.get_logger().info(f"Bailing, variance in yolo to project diff big variance in depth {self.vec_proj_to_yolo} ")
                return False
            
        if self.stall_count < 20:
            if self.state is LocalPlanner.VisualServoState.CONSTELLATION_ALIGNMENT:
                self.get_logger().info(f"Bailing, not centering on apple")
                return False

        if self.stall_count > 3 and self.state is not LocalPlanner.VisualServoState.APPROACHING:
            if self.vec_yolo_ee_match.mean < 0.5:
                self.get_logger().info(f"Bailing, not getting good yolo to yolo match")
                return False

        apple_loc_proj = [0, 0]
        for indx in range(0, 2):
            apple_loc_proj[indx] = self.projected_apple_locs[self.current_apple_index][indx] + self.vec_proj_to_yolo[indx].mean
        apple_im_x = np.abs(apple_loc_proj[0] - self.width // 2) // self.width
        apple_im_y = np.abs(apple_loc_proj[1] - self.height // 2) // self.height
        if apple_im_x > 0.45 or apple_im_y > 0.45:
            self.get_logger().info(f"Bailing, projected point falling out of image {apple_im_x}, {apple_im_y}")
            return False
        return True
        
    # ================================================================== Check for transition methods
    def _check_constellation_alignment(self):
        """ For transitioning from constellation alignment - make sure the noise in the vector has settled down"""
        for indx in range(0, 2):
            if self.vec_proj_to_yolo[indx].variance > self.apple_width:
                return False
        return True

    def _check_apple_centering(self):
        """ For transitioning to the approach phase - is the Kalman filter location centered within 1/2 apple width?"""
        xy_pos = self.kf_pos[0:2]
        apple_width_in_image = self._width_apple_in_image()

        self.get_logger.info(f"Centering apple check, {xy_pos}, apple width {apple_width_in_image}")

        if np.abs(xy_pos[0]) > apple_width_in_image * 0.5:
            return False
        if np.abs(xy_pos[1]) > apple_width_in_image * 0.5:
            return False
        self.get_logger.info(f"Centering apple check, {xy_pos}")
        return True

    def _close_enough_depth(self):
        """ Return True if close enough"""
        if self.get_depth() > self.stopping_depth():
            return False
        return True 

    # ================================================================== YOLO
    def _estimate_image_movement_from_yolo(self):
        """Find the best match between the last yolo boxes and this one, and calculate an estimated shift"""
        if not self.yolo_apple_centers:
            self.get_logger().info("Estimate motion: No apple centers")
            return
        if not self.yolo_last_apple_centers:
            self.get_logger().info("Estimate motion: No last apple centers")
            return
        
        self.yolo_apple_index, self.vec_yolo_image_motion = self._match_points(self.yolo_last_apple_centers, 
                                                                               self.yolo_apple_centers, 
                                                                               self.last_yolo_apple_index)

    def _run_yolo(self, rgb):
        # Convert to opencv format from msg
        image = self.br.imgmsg_to_cv2(rgb, "bgr8")

        self.height = image.shape[1]
        self.width = image.shape[0]
        self.get_logger().info(f"YOLO processing image {self.height}, {self.width}")

        # Get apple bounding boxes from yolo model
        # results = self.model(image, conf=self.yolo_conf, device='cuda', verbose=False)[0]
        if self.model:
            with torch.inference_mode():
                results = self.model(image, conf=self.yolo_conf, verbose=False)[0]
        else:
            self.yolo_apple_centers = None
            self.yolo_apple_radii = None
            return image
        
        self.yolo_apple_centers = np.zeros((len(results), 2))
        self.yolo_apple_radii = []
        for indx, box in enumerate(results):
            # find center of each bounding box and calculate distance to center of image
            x,y,w,h = box.boxes.xyxy.cpu().numpy()[0]
            self.yolo_apple_centers[indx, 0] = (x + w) / 2
            self.yolo_apple_centers[indx, 1] = (y + h) / 2
            self.apple_radii.append(0.5 * (w + h))
        return image

    # ================================================================== Helper methods
    def normalize(self, val, minimum, maximum):
        # Normalizes val between min and max
        return (val - minimum) / (maximum-minimum)

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

    # ================================================================== Create twist methods
    def _align_constellation(self):
        """ If we're just starting, move at a diagonal or horizontal/vertical (whichever moves the best-match bbox to the middle)
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

        if self.stall_count < 2:
            new_x = 0.0
        elif self.stall_count > 5:
            new_y = 0.0

        return new_x, new_y

    def _align_xy_bbox(self):
        """ Move the estimated projected point to the center of the image
            The check function will track if the estimate project point is about to go out of the frame
            This function just assumes the bbox center from the kalman filter is correct.
            Use that, rather than the actual bbox center, to account for yolo cutting out"""
        
        # Centerpoints
        apple_loc_yolo = self.kf_pos.x[0:2]
        center_x = self.width // 2
        center_y = self.height // 2
        # Get x magnitude and set velocity with exponential function * max_vel
        if apple_loc_yolo[0] >= center_x:
            new_x = self._exponential_vel(self.normalize(apple_loc_yolo[0], center_x, self.width))
        else:
            new_x = -self._exponential_vel(1-self.normalize(apple_loc_yolo[0], 0, self.width))
        # Get y magnitude
        if apple_loc_yolo[1] >= center_y:
            new_y = self._exponential_vel(self.normalize(apple_loc_yolo[1], center_y, self.height))
        else:
            new_y = -self._exponential_vel(1 - self.normalize(apple_loc_yolo[1], 0, self.height))
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
        self.prev_pos = []
        # if we are approaching, also set z
        if self.state is LocalPlanner.VisualServoState.APPROACHING:
            vel_vec.twist.linear.z = self.z_speed
        return vel_vec
    
    def servoing_callback(self, rgb_msg: Image, proj_pts_msg: PoseArray, dist_msg: Float32):
        """ Called when we get a new camera image. If we're not idle/done we do the following
        Calculate the new yolo bounding boxes
          - Find the shift between the new boxes and the old ones (via a match)
        Calculate the estimated shift between the last set of projected points and this one (should mirror EE motion)
        Calculate the estimated shift between the yolo bboxes and the projected points (2D from bboxes, 3D from tof)
        If there is both a yolo bbox over the center of the image and a projected point over the center
          - Can estimate the depth from the time of flight
          - Assumes bbox/projected point is in the cone of the time of flight (estimate depth from 3D pc)
        If the stars <sic> are aligned, then the following should hold:
          - The image-space motion of the yolo bboxes should match the image-space motion of the projected points
          - The 3D alignment between the yolo boxes and the projected points should be within a few apple widths
          - When the camera is pointing at the selected apple (accounting for alignment error) then there should be
               - A yolo bbox somewhat centered in the image
               - Depth values from the TOF that are roughly correct (in the expected plane of the tree)
        Motion plan
          - If still in the alignment stage, then move the camera in a zig-zag or box (xy change only) keeping the 
             projected apple location in the center-ish. 
              - eye in hand camera should be far enough back (0.25 m is a good estimate for a 90 degree fov camera) to see
                  approx 5 apples spaced 2 inches apart on 2 different wires space 18 inches apart
              - If yolo box motion is not roughly projected point motion, something is really wrong
          - During alignment and approach, should have 
               - estimated apple location (with offset vector) in center of frame
               - a bbox in center of frame (at least until too close)
               - decreasing distance
        Error assumptions
          - Use width of apple projected into image plane (subtended angle) as overall normalization factor
          - Depth from point cloud/apple locations is broadly correct"""

        if self.state is LocalPlanner.VisualServoState.IDLE:
            return

        # Projected points
        self.proj_apple_locs(proj_pts_msg=proj_pts_msg)

        # Run YOLO to get all bounding boxes    
        img = self._run_yolo(rgb_msg)

        # Now try to do a yolo to yolo match
        self._estimate_image_movement_from_yolo()

        # Projected point motion last frame
        self._estimate_image_movement_from_projected()

        # update Kalman filter with the measuered position, and previous measured velocities
        self._kalman_filter_update()

        # Image showing projected points and yolo bounding boxes
        self.create_debug_image(img=img)

        vel_vec = TwistStamped()
        vel_vec.header.stamp = self.get_clock().now().to_msg()
        vel_vec.header.frame_id = "tool0"
        vel_vec.twist.linear.z = 0.0
        vel_vec.twist.linear.x = 0.0
        vel_vec.twist.linear.y = 0.0

        # Check if still valid or if we've gone off the reservation
        if not self._check_valid():
            self.servo_publisher.publish(vel_vec)
            return                
        
        if self.state is LocalPlanner.VisualServoState.CONSTELLATION_ALIGNMENT:
            # Move in a zig zag to try to establish a decent projected point to yolo alignment
            vel_vec.twist.linear.x, vel_vec.twist.linear.y = self._align_constellation()

            if self.stall_count > 3:
                if self._check_constellation_alignment():
                    # Seed with projected apple's location in the image
                    xy_loc = self._projected_xy_loc_with_shift()
                    self.kf_pos.x = np.array([[xy_loc[0]],[0],[0],[xy_loc[1]],[0],[0]])
                    self.state = LocalPlanner.VisualServoState.BBOX_CENTERING

        if self.state is LocalPlanner.VisualServoState.BBOX_CENTERING:
            vel_vec.twist.linear.x, vel_vec.twist.linear.y = self._align_xy_bbox()

            if self._check_apple_centering():
                # Seed the Kalman filter with the projected location of the selected apple
                self.state = LocalPlanner.VisualServoState.APPROACHING

        if self.state is LocalPlanner.VisualServoState.APPROACHING:
            vel_vec.twist.linear.x, vel_vec.twist.linear.y = self._align_xy_bbox()
            if not self._close_enough_depth():
                # Should be in the range 0 to 1
                dist_to_target = (self.get_depth() - 0.9 * self.stopping_depth()) / (self.starting_depth() - self.stopping_depth())
                # Convert to max z speed
                vel_vec.twist.linear.z = self.max_z_speed * np.tanh(dist_to_target)
            else:
                if self._check_apple_centering():
                    self.state = LocalPlanner.VisualServoState.DONE

        # Save the last set of projected apple locations
        self.last_projected_apple_locs = copy.deepcopy(self.projected_apple_locs)
        # Save the last set of yolo locations
        self.yolo_last_apple_centers = copy.deepcopy(self.yolo_apple_centers)
        self.yolo_last_apple_radii = copy.deepcopy(self.yolo_apple_radii)
        self.last_yolo_apple_index = self.yolo_apple_index

        self.get_logger().info(f"Velocity vec {vel_vec.twist.linear}")
        self.servo_publisher.publish(vel_vec)


def main(args=None):
    rclpy.init(args=args)

    local_planner = LocalPlanner()

    try:
        # Keep the node running to listen and respond to incoming requests
            executor = MultiThreadedExecutor()
            rclpy.spin(local_planner, executor=executor)
    finally:
        # Clean up and shutdown cleanly
        local_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()