#!/usr/bin/env python3

# ROS
import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
# Interfaces
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, TwistStamped

# Image processing
from cv_bridge import CvBridge
import cv2
import math
import numpy as np
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

class LocalPlanner(Node):

    def __init__(self):
        super().__init__('local_planner_node')
        ### Subscribers/ Publishers
        self.camera_subscription = Subscriber(self,Image,'gripper/rgb_palm_camera/image_raw')
        # Depth sub not needed if not going forward. Left in case the use case changes in the future. 
        # self.depth_sub = Subscriber(self,GripperTofDistance, "gripper/tof/depth_raw")
        self.ts = ApproximateTimeSynchronizer([self.camera_subscription],30,0.05,)
        self.ts.registerCallback(self.rgb_servoing_callback)
        # Publisher to end effector servo controller, sends velocity commands
        self.servo_publisher = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        #specify reentrant callback group 
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

        ### Vars
        self.start_flag = False
        self.rate = self.create_rate(1)
        self.stall_count = 0
        self.first_servo = True
        ### MUST BE 0.0 unless moving forward and using TOF distance as a stopping condition.
        self.z_speed = 0.0
        self.prev_pos = []

        ### Image Processing
        self.br = CvBridge()
        self.model = YOLO(self.model_path)  # pretrained YOLOv8n model
        self.model.model = torch.compile(self.model.model)

        ### Tf2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

    def start_sequence_srv_callback(self, request, response):
        # Starts servo node it it hasnt been started already
        self.get_logger().info("Activating servo node...")
        self.start_flag = True
        self.stall_count = 0
        self.first_servo = True
        self.get_logger().info("Starting visual arm servoing...")
        # Servos until we are in front of apple
        self.init_kalman()
        try:
            while rclpy.ok() and self.start_flag:
                self.get_logger().info("Servoing arm in front of apple...")
                self.rate.sleep()
        except KeyboardInterrupt:
            pass
        self.get_logger().info("Successfully servoed in front of the apple!")
        response.success=True
        return response

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

    def exponential_vel(self, vel):
        # Exponential function for determining velocity scaling based on pixel distance from center of the camera 
        # Bounded by 0 and max_vel
        return (1-np.exp((-self.smoothing_factor/2)*vel)) * self.max_vel

    def create_servo_vector(self, closest_apple, image, distance):
        # Centerpoints
        apple_x = closest_apple[0]
        apple_y = closest_apple[1]
        center_x = image.shape[1] // 2
        center_y = image.shape[0] // 2
        # Get x magnitude and set velocity with exponential function * max_vel
        if apple_x >= center_x:
            new_x = self.exponential_vel(self.normalize(apple_x, center_x, image.shape[1]))
        else:
            new_x = -self.exponential_vel(1-self.normalize(apple_x, 0, center_x))
        # Get y magnitude
        if apple_y >= center_y:
            new_y = self.exponential_vel(self.normalize(apple_y, center_y, image.shape[0]))
        else:
            new_y = -self.exponential_vel(1 - self.normalize(apple_y, 0, center_y))

        # save previous velocities to feed into Kalman filter
        self.prev_vel[0] = new_x
        self.prev_vel[1] = new_y
        # Transform from optical frame to end effector frame
        transformed_vector = self.transform_optical_to_ee(new_x, new_y)

        # Create Twiststamped message in end effector frame
        vel_vec = TwistStamped()
        vel_vec.header.stamp = self.get_clock().now().to_msg()
        vel_vec.header.frame_id = "tool0"
        vel_vec.twist.linear.x = transformed_vector.pose.position.x
        vel_vec.twist.linear.y = transformed_vector.pose.position.y

        # If we are within picking distance then stop, otherwise move forward 
        if distance < .2:
            vel_vec.twist.linear.z = 0.0
            vel_vec.twist.linear.x = 0.0
            vel_vec.twist.linear.y = 0.0
            self.start_flag = False
        else:
            vel_vec.twist.linear.z = self.z_speed
        return vel_vec
    
    def calculate_euclidean(self, pos1, pos2):
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    def rgb_servoing_callback(self, rgb):
        if self.start_flag:
            # Convert to opencv format from msg
            image = self.br.imgmsg_to_cv2(rgb, "bgr8")
            width = rgb.width
            height = rgb.height

            # Get apple bounding boxes from yolo model
            # results = self.model(image, conf=self.yolo_conf, device='cuda', verbose=False)[0]
            results = self.model(image, conf=self.yolo_conf, verbose=False)[0]
            apple_centers = []
            z_dist = []
            for i in results:
                # find center of each bounding box and calculate distance to center of image
                x,y,w,h = i.boxes.xyxy.cpu().numpy()[0]
                apple_centers.append([(x + w)/2, (y+h)/2])

                if self.first_servo:
                    dist_to_apple = self.calculate_euclidean([width//2,height//2], apple_centers[-1])
                else: 
                    dist_to_apple = self.calculate_euclidean(self.prev_pos, apple_centers[-1])

                z_dist.append(dist_to_apple)

            if apple_centers:
                # reset stall counter if we saw apples
                self.stall_count = 0
                # get closest apple center
                closest_apple_raw = apple_centers[np.argmin(z_dist)]

                # If this is the first iteration, set our initial estimate of the apple location in the kalman filter to the apple center measured
                if self.first_servo:
                    self.kf_pos.x = np.array([[closest_apple_raw[0]],[0],[0],[closest_apple_raw[1]],[0],[0]])
                    self.first_servo = False
                
                # update Kalman filter with the measuered position, and previous measured velocities
                self.kf_pos.predict()
                self.kf_pos.update(np.array([[closest_apple_raw[0]],[-self.prev_vel[0]], [closest_apple_raw[1]], [-self.prev_vel[1]]]))
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



def main(args=None):
    rclpy.init(args=args)
    local_planner = LocalPlanner()
    executor = MultiThreadedExecutor()
    rclpy.spin(local_planner, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()