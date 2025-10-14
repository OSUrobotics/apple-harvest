#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, WrenchStamped, TwistStamped, TransformStamped, Twist
from std_msgs.msg import Float64, Bool
from std_srvs.srv import Empty
from scipy.spatial.transform import Rotation
from rcl_interfaces.srv import SetParameters

class StiffnessController(Node):
    
    def __init__(self, angle_limits=None):
        
        super().__init__('stiffness_pick_controller')
        
        self.speed = 0.2 # * 0.6 m/s
        self.vel_cmd = Twist() # * 0.6 m/s

        self.wrench_subscription = self.create_subscription(WrenchStamped, '/filtered_wrench', self.process_force_meas, 10)
        self.pose_subscription = self.create_subscription(TransformStamped,'/tool_pose', self.configure_self, 10)
        
        self.cmd_publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.status_publisher = self.create_publisher(Bool, '/stiffness_seeking/status', 10)

        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.force_from_gravity = np.array([0.0, 0.0, 0.0])
        self.preferred_pull = np.array([0.0, -0.7, -0.7])

        self.running = False

        self.start_service = self.create_service(Empty, 'start_stiffness_controller', self.start)
        self.stop_service = self.create_service(Empty, 'stop_stiffness_controller', self.stop)
        
        self.R = np.identity(3)

        self.cli = self.create_client(SetParameters, '/servo_node/set_parameters')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetParameters.Request()        

        self.current_force = 0.0
        self.prev_force = 0.0
        self.angle_list = []
        self.stiffness_list = []
        self.exploration_angles = []

        self.angle_limits = angle_limits
        self.iter_counter = 0
        self.iter_per_step = 50  # Number of iterations before changing velocity


    ## SERVICES

    def start(self, request, response):

        self.get_logger().info("starting controller...")
        self.running = True
        return response

    def stop(self, request, response):

        self.running = False
        self.get_logger().info("finished")
        return response
     
    ## SUBSCRIBERS & PUBLISHERS

    def process_force_meas(self, msg):
        
        wrench = msg.wrench 
        
        current_force = np.array([wrench.force.x, wrench.force.y,
                                  wrench.force.z]) - self.force_from_gravity

        self.current_force = np.linalg.norm(current_force)


    def timer_callback(self):

        if self.running:

            self.iter_counter += 1

            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"

            #todo: change this to run only at a certain frequency
            if self.iter_counter % self.iter_per_step == 0:

                self.get_logger().info("Trying a new direction")
                self.iter_counter = 0

                #logic to update velocity here
                next_angles = [0.0, 0.0]

                if len(self.angle_list) == 0:
                    self.exploration_angles = self.generate_exploration_angles()
                    self.get_logger().info("exploration angles: {}".format(self.exploration_angles))
                    next_angles = self.exploration_angles[0]
                    self.prev_force = self.current_force
                    self.angle_list.append(next_angles)
                elif len(self.angle_list) < 3:
                    self.stiffness_list.append(self.current_force-self.prev_force)
                    next_angles = self.exploration_angles[len(self.angle_list)]
                    self.angle_list.append(next_angles)
                    self.prev_force = self.current_force
                else:
                    self.stiffness_list.append(self.current_force-self.prev_force)
                    self.get_logger().info("force changes: {}".format(self.stiffness_list))

                    if self.stiffness_list[-1] >= self.stiffness_list[0]:
                    
                        self.prev_force = self.current_force

                        v_angle_mat = np.array(self.angle_list)
                        stiffness_mat = np.array(self.stiffness_list)
                        
                        next_angles, new_angle_list, new_stiffness_list = choose_velocity_angle(v_angle_mat, stiffness_mat)
                        self.angle_list = new_angle_list
                        self.stiffness_list = new_stiffness_list

                    else:
                        self.stiffness_list.pop()
                        next_angles = self.angle_list[0]
                        self.angle_list[-1] = self.angle_list[0]                    

                if self.angle_limits is not None:
                    # Ensure angles are within specified limits
                    next_angles[0] = np.clip(next_angles[0], self.angle_limits[0][0], self.angle_limits[0][1])
                    next_angles[1] = np.clip(next_angles[1], self.angle_limits[1][0], self.angle_limits[1][1])

                #convert angles to unit vector
                unit_vector = angles_to_unit_vector(next_angles)
                msg.twist.linear.x = self.speed * unit_vector[0]
                msg.twist.linear.y = self.speed * unit_vector[1]
                msg.twist.linear.z = self.speed * unit_vector[2]

                self.vel_cmd = msg.twist

                #self.get_logger().info("angles: {}".format(next_angles))
                #self.get_logger().info("speed: {}".format(msg.twist.linear))
            else:
                msg.twist = self.vel_cmd

            #publish the velocity command     
            self.cmd_publisher.publish(msg)

        sts_msg = Bool()
        sts_msg.data = self.running
        self.status_publisher.publish(sts_msg)

    ## HELPERS
    
    def configure_self(self, pose_msg):

        quat_msg = pose_msg.transform.rotation
        quat_vec = [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
        
        position_msg = pose_msg.transform.translation
        position_vec = [position_msg.x, position_msg.y, position_msg.z]

        r = Rotation.from_quat(quat_vec)
        self.R = r.as_matrix()

        self.preferred_pull = -1 * np.array(position_vec) / np.linalg.norm(position_vec)

    def generate_exploration_angles(self):
        angle_1 = unit_vector_to_angles(self.preferred_pull)
        angle_2 = [(angle_1[0] - 0.2) % (2 * np.pi), angle_1[1]]
        angle_3 = [angle_1[0], (angle_1[1] - 0.2) % (2 * np.pi)]
        return [angle_1, angle_2, angle_3]
        
# More helper functions
def choose_velocity_angle(v_angle_mat, stiffness_list):

    if v_angle_mat.shape != (3, 2):
        raise ValueError("v_angle_mat must be a 3x2 matrix.")
        
    if len(stiffness_list) != 3:
        raise ValueError("stiffness_list must contain exactly 3 stiffness scores.")
        
        # Sort the stiffness scores and their corresponding angles
    sorted_indices = np.argsort(stiffness_list)[::-1]
    best_indices = sorted_indices[:2]  # Get the indices of the two highest stiffness scores

    #calculate the centroid of the best two angles
    centroid = np.mean(v_angle_mat[best_indices], axis=0)
    # Calculate the reflection point
    reflection = centroid + (centroid - v_angle_mat[sorted_indices[2]])

    new_angle_list = [v_angle_mat[sorted_indices[0]], v_angle_mat[sorted_indices[1]], reflection]
    new_stiffness_list = [stiffness_list[i] for i in best_indices]
    return reflection, new_angle_list, new_stiffness_list
    
def angles_to_unit_vector(angles):
    """
    Convert a list of angles to a unit vector.
        
    Args:
        angles (list): A list of angles in radians.
            
    Returns:
        np.ndarray: A unit vector corresponding to the angles.
    """
    if len(angles) != 2:
        raise ValueError("angles must be a list of 2 angles.")
        
    x = np.cos(angles[0]) * np.cos(angles[1])
    y = np.sin(angles[0]) * np.cos(angles[1])
    z = np.sin(angles[1])
        
    return np.array([x, y, z])

def unit_vector_to_angles(unit_vector):
    """
    Convert a unit vector to a list of angles.
        
    Args:
        unit_vector (np.ndarray): A unit vector (3D).
            
    Returns:
        list: A list of angles in radians.
    """
    if unit_vector.shape != (3,):
        raise ValueError("unit_vector must be a 3D vector.")
        
    theta = np.arctan2(unit_vector[2], np.sqrt(unit_vector[0]**2 + unit_vector[1]**2))
    phi = np.arctan2(unit_vector[1], unit_vector[0])
        
    return [phi, theta]



def main():

    rclpy.init()

    node = StiffnessController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()
