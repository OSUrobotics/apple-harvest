#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution, IfElseSubstitution, EqualsSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # ---------- Arguments ----------
    args = [
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        DeclareLaunchArgument('robot_ip', default_value='yyy.yyy.yyy.yyy'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument('description_file', default_value='amiga_ur_gripper.urdf.xacro'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),

        DeclareLaunchArgument('controller_manager_ns', default_value='/controller_manager'),
        DeclareLaunchArgument('spawn_moveit_controllers', default_value='true'),

        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('voxel_distance_tol', default_value='0.5'),
        DeclareLaunchArgument('max_accel', default_value='0.05'),
        DeclareLaunchArgument('max_vel', default_value='0.05'),
        DeclareLaunchArgument('traj_time_step', default_value='0.05'),
    ]

    # Pick controller based on fake hardware argument
    initial_controller = IfElseSubstitution(
        EqualsSubstitution(LaunchConfiguration('use_fake_hardware'), TextSubstitution(text='true')),
        TextSubstitution(text='joint_trajectory_controller'),
        TextSubstitution(text='scaled_joint_trajectory_controller'),
    )

    # ---------- Paths ----------
    ur_driver_launch = os.path.join(
        get_package_share_directory('ur_robot_driver'),
        'launch', 'ur_control.launch.py'
    )

    desc_launch = os.path.join(
        get_package_share_directory('harvest_hardware_description'),
        'launch', 'robot_state_publisher.launch.py'
    )

    moveit_launch = os.path.join(
        get_package_share_directory('harvest_hardware_moveit_config'),
        'launch', 'ur_moveit.launch.py'
    )

    # ---------- Includes ----------
    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_driver_launch),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'description_package': 'harvest_hardware_description',
            'description_file': LaunchConfiguration('description_file'),
            'prefix': LaunchConfiguration('prefix'),
            'use_robot_state_publisher': 'true',
            'launch_rviz': 'false',
            'initial_joint_controller': initial_controller,
        }.items(),
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(desc_launch),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'description_file': LaunchConfiguration('description_file'),
            'prefix': LaunchConfiguration('prefix'),
        }.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'prefix': LaunchConfiguration('prefix'),
            'launch_rviz': 'true',
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'description_file': LaunchConfiguration('description_file'),
            'controller_manager_ns': LaunchConfiguration('controller_manager_ns'),
            'spawn_controllers': LaunchConfiguration('spawn_moveit_controllers'),
        }.items(),
    )

    # ---------- Other nodes (commented out in your original) ----------
    coord_to_traj = Node(
        package='harvest_control',
        executable='coordinate_to_trajectory.py',
        name='trajectory_query_node',
        parameters=[{
            'sim': LaunchConfiguration('sim'),
            'voxel_distance_tol': LaunchConfiguration('voxel_distance_tol'),
        }],
        output='screen',
    )

    event_detector = Node(package='harvest_control', executable='event_detector.py', name='event_detector')
    force_filter = Node(package='harvest_control', executable='force_filter.py', name='forcefilter')
    pick_controller = Node(package='harvest_control', executable='heuristic_controller.py', name='pick_controller')
    linear_controller = Node(package='harvest_control', executable='linear_controller.py', name='linear_controller')
    tf_listener = Node(package='harvest_control', executable='pose_listener.py', name='tf_listener')
    pressure_avg = Node(package='harvest_control', executable='pressure_averager.py', name='pressure_averager')
    pull_twist = Node(package='harvest_control', executable='pull_twist_controller.py', name='pull_twist_controller')
    recorder = Node(package='harvest', executable='record.py', name='record_topics_node')

    return LaunchDescription(
        args + [
            ur_driver,
            moveit,
            coord_to_traj,
            event_detector,
            force_filter,
            pick_controller,
            linear_controller,
            tf_listener,
            pressure_avg,
            pull_twist,
            recorder,
        ]
    )
