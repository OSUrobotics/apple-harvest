#!/usr/bin/env python3

import numpy as np
import time
import sys
import os
import pybullet as p

from pyb_scripts.pyb_utils import PybUtils
from pyb_scripts.load_objects import LoadObjects
from pyb_scripts.load_robot import LoadRobot

import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from std_srvs.srv import Empty
from harvest_interfaces.srv import CoordinateToTrajectory, SendTrajectory, VoxelMask, ApplePrediction


class TrajGen(Node):
    def __init__(self, robot_urdf_path: str, robot_home_pos, renders=True):
        super().__init__('trajectory_generation_node')


        # Probably have a service or two



        self.pyb = PybUtils(self, renders=renders)
        self.object_loader = LoadObjects(self.pyb.con)

        self.robot_home_pos = robot_home_pos
        self.robot = LoadRobot(self.pyb.con, robot_urdf_path, [0, 0, 0], self.pyb.con.getQuaternionFromEuler([0, 0, 0]), self.robot_home_pos, collision_objects=self.object_loader.collision_objects)


        # Get the package share directory
        package_share_directory = get_package_share_directory('harvest_control')

        # Load data from share/resource directory
        self.voxel_data = np.loadtxt(os.path.join(package_share_directory, 'resource', 'reachable_voxel_centers.csv'))
        self.trajectories = np.load(os.path.join(package_share_directory, 'resource', 'reachable_paths.npy'))
        self.ik_data = np.loadtxt(os.path.join(package_share_directory, 'resource', 'voxel_ik_data.csv'), delimiter=',', skiprows=1)

        # Extract the data
        self.voxel_centers = self.voxel_data[:, :3]
        self.voxel_centers_orig = np.copy(self.voxel_centers)

        self.target_configurations = self.ik_data[:, :6]

    def path_to_closest_voxel(self, target_point):
        """ Find the path to a voxel that the target point is closest to 

        Args:
            target_point (float list): target 3D coordinate

        Returns:
            path: the path from the set home configuration to the voxel the target point is closest to
            distance_error: error between target point and closest voxel center
            closest_voxel_index: index from presaved data in which the closest voxel is assosicated
        """
        # Calculate distances
        distances = np.linalg.norm(self.voxel_centers - target_point, axis=1)
        
        # Find the index of the closest voxel
        closest_voxel_index = np.argmin(distances)

        distance_error = distances[closest_voxel_index]

        # Get the associated path to closest voxel
        return self.trajectories[:, :, closest_voxel_index], distance_error, closest_voxel_index


        