#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, IfElseSubstitution, EqualsSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # ---------- Arguments ----------
    args = [
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        DeclareLaunchArgument('robot_ip', default_value='yyy.yyy.yyy.yyy'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('headless_mode', default_value='true'),
        DeclareLaunchArgument('activate_joint_controller', default_value='true'),
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument('description_package', default_value='harvest_hardware_description'),
        DeclareLaunchArgument('description_file', default_value='amiga_ur_gripper.urdf.xacro'),
        DeclareLaunchArgument('view_rviz', default_value='true'),
        DeclareLaunchArgument("rviz_file", default_value="view_robot.rviz"),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument("use_3d_sensors", default_value="false"),

        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('voxel_distance_tol', default_value='0.5'),
        DeclareLaunchArgument('max_accel', default_value='0.05'),
        DeclareLaunchArgument('max_vel', default_value='0.05'),
        DeclareLaunchArgument('traj_time_step', default_value='0.05'),
        DeclareLaunchArgument('source_frame', default_value='amiga__base'),
        DeclareLaunchArgument('gripper_tip_frame', default_value='gripper_scups_link'),
        DeclareLaunchArgument('gripper_type', default_value='finray'),
        DeclareLaunchArgument('camera_mount', default_value='wrist'),
    ]

    # Pick controller based on fake hardware argument
    initial_controller = IfElseSubstitution(
        EqualsSubstitution(LaunchConfiguration('use_fake_hardware'), TextSubstitution(text='true')),
        TextSubstitution(text='joint_trajectory_controller'),
        TextSubstitution(text='scaled_joint_trajectory_controller'),
    )

    # ---------- Paths ----------
    ur_driver_launch = os.path.join(
        get_package_share_directory('harvest_hardware_moveit_config'),
        'launch', 'ur_control_no_rsp.launch.py'
    )

    desc_launch = os.path.join(
        get_package_share_directory('harvest_hardware_description'),
        'launch', 'robot_state_publisher.launch.py'
    )

    moveit_launch = os.path.join(
        get_package_share_directory('harvest_hardware_moveit_config'),
        'launch', 'ur_moveit.launch.py'
    )

    ur_controller_config = os.path.join(
        get_package_share_directory('harvest_hardware_moveit_config'),
        'config', 'ur_controllers.yaml'
    )

    # ---------- Includes ----------
    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_driver_launch),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'use_mock_hardware': LaunchConfiguration('use_fake_hardware'),
            'headless_mode': LaunchConfiguration('headless_mode'),
            'activate_joint_controller': LaunchConfiguration('activate_joint_controller'),
            'description_package': LaunchConfiguration('description_package'),
            'description_file': LaunchConfiguration('description_file'),
            'prefix': LaunchConfiguration('prefix'),
            'launch_rviz': 'false',
            'initial_joint_controller': initial_controller,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'controllers_file': ur_controller_config,
            'gripper_type': LaunchConfiguration('gripper_type'),
        }.items(),
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(desc_launch),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'use_mock_hardware': LaunchConfiguration('use_fake_hardware'),
            'description_package': LaunchConfiguration('description_package'),
            'description_file': LaunchConfiguration('description_file'),
            'prefix': LaunchConfiguration('prefix'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gripper_type': LaunchConfiguration('gripper_type'),
        }.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'prefix': LaunchConfiguration('prefix'),
            'launch_rviz': LaunchConfiguration('view_rviz'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'description_file': LaunchConfiguration('description_file'),
            'rviz_file': LaunchConfiguration('rviz_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_3d_sensors': LaunchConfiguration('use_3d_sensors'),
            'gripper_type': LaunchConfiguration('gripper_type'),
            'camera_mount': LaunchConfiguration('camera_mount'),
        }.items(),
    )

    coord_to_traj = Node(
        package='harvest_control',
        executable='coordinate_to_trajectory.py',
        name='trajectory_query_node',
        parameters=[{
            'sim': LaunchConfiguration('sim'),
            'voxel_distance_tol': LaunchConfiguration('voxel_distance_tol'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen',
    )
    event_detector = Node(
        package='harvest_control', 
        executable='event_detector.py', 
        name='event_detector', 
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
    )
    force_filter = Node(
        package='harvest_control', 
        executable='force_filter.py', 
        name='forcefilter', 
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
    )
    pick_controllers = Node(
        package='harvest_control',
        executable='pick_controller.py',
        name = 'pick_controllers',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    pick_controller = Node(
        package='harvest_control', 
        executable='heuristic_controller.py', 
        name='pick_controller', 
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
    )
    stiffness_controller = Node(
        package='harvest_control',
        executable='stiffness_controller.py',
        name='stiffness_controller',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    linear_controller = Node(
        package='harvest_control', 
        executable='linear_controller.py', 
        name='linear_controller', 
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
    )
    tf_listener = Node(
        package='harvest_control', 
        executable='pose_listener.py', 
        name='tf_listener', 
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'source_frame': LaunchConfiguration('source_frame'),
            'gripper_tip_frame': LaunchConfiguration('gripper_tip_frame'),
            }],
    )
    pressure_avg = Node(
        package='harvest_control', 
        executable='pressure_averager.py', 
        name='pressure_averager', 
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
    )
    pull_twist = Node(
        package='harvest_control', 
        executable='pull_twist_controller.py', 
        name='pull_twist_controller', 
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
    )
    sweep_controller = Node(
        package='harvest_control',
        executable='sweep_controller.py',
        name='sweep_controller',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
    )
    recorder = Node(
        package='harvest', 
        executable='record.py', 
        name='record_topics_node', 
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
    )

    return LaunchDescription(
        args + [
            ur_driver,
            rsp,
            moveit,
            coord_to_traj,
            # event_detector,
            force_filter,
            pick_controllers,
            pick_controller,
            stiffness_controller,
            linear_controller,
            tf_listener,
            pressure_avg,
            pull_twist,
            sweep_controller,
            recorder,
        ]
    )
