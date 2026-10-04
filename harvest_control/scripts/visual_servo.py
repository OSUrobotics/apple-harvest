#!/usr/bin/env python3

# ROS
import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, ActionClient, CancelResponse
# Interfaces
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from visualization_msgs.msg import Marker
from harvest_interfaces.action import VisualServo

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
        # Debug outputs, only drawn/published while something is subscribed
        self.debug_image_publisher = self.create_publisher(Image, "visual_servo/debug_image", 1)
        self.debug_marker_publisher = self.create_publisher(Marker, "visual_servo/command_marker", 1)
        #specify reentrant callback group 
        r_callback_group = ReentrantCallbackGroup()

        ### Services
        # Service to start the local planner sequence
        self.start_service = self.create_service(Trigger, "start_visual_servo", self.start_sequence_srv_callback, callback_group=r_callback_group)
        self.servo_action_server = ActionServer(self, VisualServo, 'visual_servo', self.execute_servo_callback, callback_group=r_callback_group, cancel_callback=self.cancel_servo_callback)
        
        ### Servo controller params
        self.declare_parameter("vservo_model_path", "NA")
        self.declare_parameter("vservo_yolo_conf", 0.85)
        self.declare_parameter("vservo_accuracy_px", 10)
        self.declare_parameter("vservo_smoothing_factor", 6.0)
        self.declare_parameter("vservo_max_vel", 0.6)
        # Dry run: process every frame without a start trigger and compute commands, but never publish to the servo node
        self.declare_parameter("vservo_dry_run", False)
        self.yolo_conf = self.get_parameter("vservo_yolo_conf").get_parameter_value().double_value
        self.target_pixel_accuracy = self.get_parameter("vservo_accuracy_px").get_parameter_value().integer_value
        self.smoothing_factor = self.get_parameter("vservo_smoothing_factor").get_parameter_value().double_value
        self.max_vel = self.get_parameter("vservo_max_vel").get_parameter_value().double_value
        self.model_path = self.get_parameter("vservo_model_path").get_parameter_value().string_value
        self.dry_run = self.get_parameter("vservo_dry_run").get_parameter_value().bool_value

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
        if self.dry_run:
            # Dry run never waits for a start trigger, so the filter has to be ready now
            self.init_kalman()
            self.get_logger().warn("Visual servo DRY RUN: computing commands for debugging, NOT publishing to the servo node")


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
    
    def execute_servo_callback(self, goal_handle):
        self.get_logger().info("Activating servo node...")
        self.start_flag = True
        self.stall_count = 0
        self.first_servo = True
        self.get_logger().info("Starting visual arm servoing...")
        # Servos until we are in front of apple
        self.init_kalman()
        result = VisualServo.Result()

        try:
            while rclpy.ok() and self.start_flag:
                if goal_handle.is_cancel_requested:
                    self.start_flag = False
                    self._publish_stop_twist()
                    goal_handle.canceled()
                    result.success = False
                    result.message = "Visual servo canceled"
                    return result

                self.get_logger().info("Servoing arm in front of apple...")
                self.rate.sleep()
        except KeyboardInterrupt:
            pass

        self.get_logger().info("Successfully servoed in front of the apple!")
        goal_handle.succeed()
        result.success = True
        result.message = "Servo complete"
        return result

    def _publish_stop_twist(self):
        vel_vec = TwistStamped()
        vel_vec.header.stamp = self.get_clock().now().to_msg()
        vel_vec.header.frame_id = "tool0"
        self.servo_publisher.publish(vel_vec)

    def cancel_servo_callback(self, goal_handle):
        self.get_logger().info("Visual servo cancel requested")
        return CancelResponse.ACCEPT

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
        if self.start_flag or self.dry_run:
            # Convert to opencv format from msg
            image = self.br.imgmsg_to_cv2(rgb, "bgr8")
            width = rgb.width
            height = rgb.height

            # Get apple bounding boxes from yolo model
            # results = self.model(image, conf=self.yolo_conf, device='cuda', verbose=False)[0]
            results = self.model(image, conf=self.yolo_conf, verbose=False)[0]
            apple_centers = []
            apple_boxes = []
            z_dist = []
            # Debug info for this frame
            closest_apple_raw = None
            closest_apple = None
            vec = None
            status = "NO APPLE"

            self.get_logger().info(f"Detection results: {results.names}")  # Logs detected objects (apple)
            if len(results) == 0:
                self.get_logger().info("No detections found in the image.")
            else:
                for i in results:
                    boxes = i.boxes.xyxy.cpu().numpy()  # Get bounding boxes
                    confidences = i.boxes.conf.cpu().numpy()  # Get confidence scores
                    self.get_logger().info(f"Detected apple with bounding box: {boxes}, confidence: {confidences}")


            for i in results:
                # find center of each bounding box and calculate distance to center of image
                x,y,w,h = i.boxes.xyxy.cpu().numpy()[0]
                apple_boxes.append([x, y, w, h])
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
                # (the camera is not moving in dry run, so no velocity is fed in)
                meas_vel = [0.0, 0.0] if self.dry_run else self.prev_vel
                self.kf_pos.predict()
                self.kf_pos.update(np.array([[closest_apple_raw[0]],[-meas_vel[0]], [closest_apple_raw[1]], [-meas_vel[1]]]))
                # get estimate from Kalman filter for closest apple location
                state = self.kf_pos.x
                closest_apple = [float(state[0][0]), float(state[3][0])]

                self.prev_pos = closest_apple

                # check if camera is centered on apple or not within our pixel target accuracy threshold, if it is then publish a 0 velocity and exit loop
                if self.calculate_euclidean([width//2, height//2], closest_apple) < self.target_pixel_accuracy:
                    status = "CENTERED"
                    if not self.dry_run:
                        self._publish_stop_twist()
                    self.start_flag = False
                else:
                    # makes sure that transforms are not failing
                    try:
                        # Creates servo vector which servos the arm towards the closest apple
                        ## THE 10 IS A CONSTANT DISTANCE VALUE BECAUSE WE ARE SERVOING IN PLACE ON A PLANE
                        ## IF NEEDED YOU CAN PASS IN A MEASURED DISTANCE AS A STOPPING CONDITION FOR THE SERVOING
                        vec = self.create_servo_vector(closest_apple, image, 10)
                        status = "TRACKING"
                        if not self.dry_run:
                            self.get_logger().info(f"publishing velocity: {vec}")
                            self.servo_publisher.publish(vec)
                    except TransformException as e:
                        status = "TF FAILED"
                        self.get_logger().info(f'Transform failed: {e}')
            else:
                # if we dont detect any apples then stay in place with 0 velocity
                if not self.dry_run:
                    self._publish_stop_twist()
                self.stall_count += 1

            if self.stall_count > 10:
                self.start_flag = False
                if self.dry_run:
                    # Keep running in dry run, reacquire whichever apple is closest to the image center next
                    self.first_servo = True
                    self.stall_count = 0
                else:
                    self.get_logger().error("STALLED after 10 attempts to servo, could not locate any apples in FOV.")

            self.publish_debug(rgb.header, image, apple_boxes, closest_apple_raw, closest_apple, vec, status)

    def publish_debug(self, header, image, boxes, raw_center, est_center, vel_vec, status):
        # Draws what the servo sees and the direction it is commanding. Skipped when nobody is subscribed.
        if self.debug_image_publisher.get_subscription_count() > 0:
            image = image.copy()
            h, w = image.shape[:2]
            center = (w // 2, h // 2)
            # All detections (gray)
            for x1, y1, x2, y2 in boxes:
                cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (160, 160, 160), 1)
            # Image center and pixel accuracy target (white)
            cv2.drawMarker(image, center, (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
            cv2.circle(image, center, self.target_pixel_accuracy, (255, 255, 255), 1)
            # Raw YOLO center of the chosen apple (red) and the Kalman estimate the servo actually steers to (green)
            if raw_center is not None:
                cv2.circle(image, (int(raw_center[0]), int(raw_center[1])), 4, (0, 0, 255), -1)
            if est_center is not None:
                cv2.circle(image, (int(est_center[0]), int(est_center[1])), 8, (0, 255, 0), 2)

            lines = [status + ("  [DRY RUN]" if self.dry_run else "")]
            if est_center is not None:
                lines.append(f"offset px: x={est_center[0] - center[0]:+.0f} y={est_center[1] - center[1]:+.0f}")
            if vel_vec is not None:
                # Optical frame shares the image axes (+x right, +y down), so the camera command draws straight onto the image.
                # Correct behavior: the yellow arrow points from the image center toward the green circle.
                scale = (min(w, h) / 2) / self.max_vel
                tip = (int(center[0] + self.prev_vel[0] * scale), int(center[1] + self.prev_vel[1] * scale))
                cv2.arrowedLine(image, center, tip, (0, 255, 255), 2, tipLength=0.2)
                lines.append(f"optical cmd: x={self.prev_vel[0]:+.2f} y={self.prev_vel[1]:+.2f}")
                lines.append(f"tool0 cmd: x={vel_vec.twist.linear.x:+.2f} y={vel_vec.twist.linear.y:+.2f} z={vel_vec.twist.linear.z:+.2f}")
            for n, text in enumerate(lines):
                org = (10, 25 + 22 * n)
                cv2.putText(image, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(image, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            debug_msg = self.br.cv2_to_imgmsg(image, "bgr8")
            debug_msg.header = header
            self.debug_image_publisher.publish(debug_msg)

        if self.debug_marker_publisher.get_subscription_count() > 0:
            # Arrow in tool0 showing the exact twist sent to the servo node, so RViz shows the physical direction
            marker = Marker()
            marker.header.frame_id = "tool0"
            marker.ns = "visual_servo"
            marker.id = 0
            if vel_vec is None:
                marker.action = Marker.DELETE
            else:
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                # Start the arrow at the camera so it reads from the camera's point of view
                start = Point()
                try:
                    cam = self.tf_buffer.lookup_transform("tool0", "gripper_palm_camera_optical_link", rclpy.time.Time())
                    start.x = cam.transform.translation.x
                    start.y = cam.transform.translation.y
                    start.z = cam.transform.translation.z
                except TransformException:
                    pass
                # Full speed draws as a 30 cm arrow
                length = 0.3 / self.max_vel
                end = Point(x=start.x + vel_vec.twist.linear.x * length,
                            y=start.y + vel_vec.twist.linear.y * length,
                            z=start.z + vel_vec.twist.linear.z * length)
                marker.points = [start, end]
                marker.scale.x = 0.01
                marker.scale.y = 0.02
                marker.scale.z = 0.03
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.a = 1.0
            self.debug_marker_publisher.publish(marker)



def main(args=None):
    rclpy.init(args=args)
    local_planner = LocalPlanner()
    executor = MultiThreadedExecutor()
    rclpy.spin(local_planner, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()