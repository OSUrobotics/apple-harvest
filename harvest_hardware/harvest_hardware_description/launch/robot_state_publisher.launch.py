#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    args = [
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        DeclareLaunchArgument('robot_ip', default_value='yyy.yyy.yyy.yyy'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),
        DeclareLaunchArgument('description_package', default_value='harvest_hardware_description'),
        DeclareLaunchArgument('description_file', default_value='amiga_ur_gripper.urdf.xacro'),
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    # Build robot_description from your xacro
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ',
                PathJoinSubstitution([
                    FindPackageShare(LaunchConfiguration('description_package')),
                    'urdf',
                    LaunchConfiguration('description_file'),
                ]),
                ' ',
                'name:=ur5e ',
                'ur_type:=', LaunchConfiguration('ur_type'), ' ',
                'prefix:=', LaunchConfiguration('prefix'), ' ',
                'robot_ip:=', LaunchConfiguration('robot_ip'), ' ',
                'use_fake_hardware:=', LaunchConfiguration('use_fake_hardware'), ' ',
                'headless_mode:=false ',
                'fake_sensor_commands:=false ',
                'sim_gazebo:=false ',
                'sim_ignition:=false ',
                'safety_limits:=false ',
                'tool_device_name:=/tmp/ttyUR ',
            ]),
            value_type=str
        )
    }

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription(args + [rsp])
