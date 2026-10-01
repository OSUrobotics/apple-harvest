#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    launch_realsense_arg = DeclareLaunchArgument(
        "launch_realsense",
        default_value="true",
        description="Whether to launch realsense topics."
    )

    # The base camera is currently the D435
    base_serial_arg = DeclareLaunchArgument(
        "base_serial",
        default_value="'829212072203'",
        description="Serial number for the base RealSense camera"
    )
    # The mast camera is currently the D435I
    mast_serial_arg = DeclareLaunchArgument(
        "mast_serial",
        default_value="'040322070611'",
        description="Serial number for the mast RealSense camera"
    )

    # Common toggles
    publish_tf_arg = DeclareLaunchArgument(
        "publish_tf", default_value="false",
        description="Let realsense2_camera publish TF frames (false if your URDF handles the mount)"
    )
    # Named to match realsense2_camera's "align_depth.enable" parameter directly.
    align_depth_arg = DeclareLaunchArgument(
        "align_depth.enable", default_value="true",
        description="Publish /aligned_depth_to_color/image_raw"
    )
    # Named to match realsense2_camera's "pointcloud.enable" parameter directly.
    pointcloud_enable_arg = DeclareLaunchArgument(
        "pointcloud.enable", default_value="false",
        description="Enable on-node point cloud generation"
    )

    # Per-node hardware sync toggle (realsense2_camera's "enable_sync" parameter)
    base_enable_sync_arg = DeclareLaunchArgument(
        "base_enable_sync", default_value="true",
        description="Enable frame sync on the base RealSense camera"
    )
    mast_enable_sync_arg = DeclareLaunchArgument(
        "mast_enable_sync", default_value="true",
        description="Enable frame sync on the mast RealSense camera"
    )

    # Paths
    rs_launch = PythonLaunchDescriptionSource([
        PathJoinSubstitution([FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])
    ])

    # --- base camera ---
    base_camera = IncludeLaunchDescription(
        rs_launch,
        launch_arguments={
            "serial_no": LaunchConfiguration("base_serial"),
            "camera_name": "base_camera",
            "enable_color": "true",
            "enable_depth": "true",
            "align_depth.enable": LaunchConfiguration("align_depth.enable"),
            "publish_tf": LaunchConfiguration("publish_tf"),
            "pointcloud.enable": LaunchConfiguration("pointcloud.enable"),
            "enable_sync": LaunchConfiguration("base_enable_sync"),
        }.items(),
            condition=IfCondition(LaunchConfiguration('launch_realsense')),
    )

    # --- Mast camera ---
    mast_camera = IncludeLaunchDescription(
        rs_launch,
        launch_arguments={
            "serial_no": LaunchConfiguration("mast_serial"),
            "camera_name": "mast_camera",
            "enable_color": "true",
            "enable_depth": "true",
            "align_depth.enable": LaunchConfiguration("align_depth.enable"),
            "publish_tf": LaunchConfiguration("publish_tf"),
            "pointcloud.enable": LaunchConfiguration("pointcloud.enable"),
            "enable_sync": LaunchConfiguration("mast_enable_sync"),
        }.items(),
            condition=IfCondition(LaunchConfiguration('launch_realsense')),
    )

    return LaunchDescription([
        launch_realsense_arg,
        base_serial_arg,
        mast_serial_arg,
        publish_tf_arg,
        align_depth_arg,
        pointcloud_enable_arg,
        base_enable_sync_arg,
        mast_enable_sync_arg,
        base_camera,
        mast_camera,
    ])
