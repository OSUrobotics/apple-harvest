#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
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
    align_depth_arg = DeclareLaunchArgument(
        "align_depth", default_value="true",
        description="Publish /aligned_depth_to_color/image_raw"
    )
    pointcloud_enable_arg = DeclareLaunchArgument(
        "pointcloud_enable", default_value="true",
        description="Enable on-node point cloud generation"
    )
    enable_imu_arg = DeclareLaunchArgument(
        "enable_imu", default_value="false",
        description="Enable IMU (set true if your device is D435i)"
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
            "align_depth.enable": LaunchConfiguration("align_depth"),
            "publish_tf": LaunchConfiguration("publish_tf"),
            "pointcloud.enable": LaunchConfiguration("pointcloud_enable"),
            # "enable_gyro": LaunchConfiguration("enable_imu"),
            # "enable_accel": LaunchConfiguration("enable_imu"),
            # "unite_imu_method": "linear_interpolation",
        }.items(),
    )

    # --- Mast camera ---
    mast_camera = IncludeLaunchDescription(
        rs_launch,
        launch_arguments={
            "serial_no": LaunchConfiguration("mast_serial"),
            "camera_name": "mast_camera",
            "enable_color": "true",
            "enable_depth": "true",
            "align_depth.enable": LaunchConfiguration("align_depth"),
            "publish_tf": LaunchConfiguration("publish_tf"),
            "pointcloud.enable": LaunchConfiguration("pointcloud_enable"),
            # "enable_gyro": LaunchConfiguration("enable_imu"),
            # "enable_accel": LaunchConfiguration("enable_imu"),
            # "unite_imu_method": "linear_interpolation",
        }.items(),
    )

    return LaunchDescription([
        base_serial_arg,
        mast_serial_arg,
        publish_tf_arg,
        align_depth_arg,
        pointcloud_enable_arg,
        enable_imu_arg,
        base_camera,
        mast_camera,
    ])
