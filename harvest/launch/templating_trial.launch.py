#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Templating vs voxelization reachability trial on presaved data (notes/templating_trial_plan.md).
#
# trial_dir is one fast_slam_rgbd_extraction output folder: tree_*/meta.yaml, tree_*/mast_cam, tree_*/base_cam and
# the slam_trunks_*.yaml of the same SLAM run. All the nodes read the images and the map from it, because the
# tree ids are not the same between SLAM runs.
#
# This launch starts:
#   arm control and MoveIt          harvest_control arm_control_templating.launch.py (use_tree_scene:=false)
#   trellis collision objects       tree_template tree_scene_node (trellis_frame from robot_config)
#   presaved images                 apple_prediction (presaved_images:=True): mast cam + base cam buffer topics
#   trunk detector                  trunk_width_ws ros_publisher_node.py (PRESAVED_IMAGES=true, /base_cam topics)
#   trunk measurements              tree_template trunk_detection_relay
#   trellis template map            tree_template trellis_from_rgbd_extraction (extraction_dir:=trial_dir)
#   voxel collision objects         harvest_vision voxelize_scan
# It does not start SLAM (row_fast_slam), odom_reframer or save_best_particle_map.
#
# Then run the trial:
#   ros2 run harvest orchard_templating.py --ros-args -p tree_id:=0
#   ros2 run harvest orchard_templating.py --ros-args -p "tree_ids:=[0, 1, 2]"


def generate_launch_description():
    trial_dir = LaunchConfiguration("trial_dir")
    tree_template_params = LaunchConfiguration("tree_template_params")
    robot_config = LaunchConfiguration("robot_config")

    home = os.path.expanduser("~")
    default_trial_dir = os.path.join(
        home, "apple_harvest_ws", "data", "post_slam_image_processing", "test_06262026_full_res")
    default_width_estimation_path = os.path.join(home, "trunk_width_ws", "trunk_width_estimation")
    default_width_estimation_data_path = os.path.join(home, "trunk_width_ws", "width_estimation_package_data")
    default_detector_script = os.path.join(default_width_estimation_path, "scripts", "ros2", "ros_publisher_node.py")

    return LaunchDescription([
        # ----------------------------
        # Data
        # ----------------------------
        DeclareLaunchArgument(
            "trial_dir",
            default_value=default_trial_dir,
            description="fast_slam_rgbd_extraction output folder (tree_*/ and slam_trunks_*.yaml)",
        ),

        # ----------------------------
        # tree_template config (2025 Prosser bags, Amiga)
        # ----------------------------
        DeclareLaunchArgument(
            "tree_template_params",
            default_value=PathJoinSubstitution(
                [FindPackageShare("tree_template"), "config", "launch_config_2025_with_polar_meas.yaml"]),
            description="tree_template params YAML with the algorithm tuning",
        ),
        DeclareLaunchArgument(
            "robot_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("tree_template"), "config", "robot", "prosser_2025_amiga.yaml"]),
            description="tree_template robot config (frames and topics). Loaded after tree_template_params",
        ),

        # ----------------------------
        # Arm control
        # ----------------------------
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument("robot_ip", default_value="yyy.yyy.yyy.yyy"),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),

        # ----------------------------
        # Vision
        # ----------------------------
        DeclareLaunchArgument("prediction_model", default_value="v9e.pt",
                              description="YOLO model in harvest_vision/yolo_networks"),
        DeclareLaunchArgument("prediction_yolo_conf", default_value="0.85"),
        DeclareLaunchArgument("prediction_radius_min", default_value="0.03"),
        DeclareLaunchArgument("prediction_radius_max", default_value="0.06"),
        DeclareLaunchArgument("prediction_distance_max", default_value="4.0"),
        DeclareLaunchArgument("camera_type", default_value="realsense",
                              description="azure | realsense: camera intrinsics for the presaved mast images"),
        DeclareLaunchArgument("source_frame", default_value="mast_camera_color_optical_frame"),
        DeclareLaunchArgument("target_frame", default_value="amiga__base"),

        # ----------------------------
        # Trunk detector (trunk_width_ws)
        # ----------------------------
        DeclareLaunchArgument("use_detector", default_value="true",
                              description="Start the trunk detector. false if you start it in a different terminal"),
        DeclareLaunchArgument("detector_script", default_value=default_detector_script),
        DeclareLaunchArgument("width_estimation_package_path", default_value=default_width_estimation_path),
        DeclareLaunchArgument("width_estimation_package_data_path", default_value=default_width_estimation_data_path),

        # ----------------------------
        # Arm control and MoveIt
        # ----------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("harvest_control"), "launch", "arm_control_templating.launch.py"])),
            launch_arguments={
                "ur_type": LaunchConfiguration("ur_type"),
                "robot_ip": LaunchConfiguration("robot_ip"),
                "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                "launch_rviz": LaunchConfiguration("launch_rviz"),
                "use_tree_scene": "false",
            }.items(),
        ),

        # ----------------------------
        # tree_template nodes
        # ----------------------------
        # MoveIt trellis collision objects from the trunk registry
        Node(
            package="tree_template",
            executable="tree_template",
            name="tree_scene_node",
            output="screen",
            parameters=[tree_template_params, robot_config],
        ),

        # TreeImageData -> TrunkInfo measurements in the robot base frame
        Node(
            package="tree_template",
            executable="trunk_detection_relay",
            name="trunk_detection_relay",
            output="screen",
            parameters=[tree_template_params, robot_config],
        ),

        # Trellis template map from the SLAM map in trial_dir (slam_results_yaml: the slam_trunks_*.yaml in trial_dir)
        Node(
            package="tree_template",
            executable="trellis_from_rgbd_extraction",
            name="trellis_from_rgbd_extraction",
            output="screen",
            parameters=[tree_template_params, robot_config, {"extraction_dir": trial_dir}],
        ),

        # ----------------------------
        # Trunk detector, presaved mode: runs on the base cam frame that apple_prediction publishes, when
        # /trigger_estimation is called
        # ----------------------------
        ExecuteProcess(
            cmd=["python3", LaunchConfiguration("detector_script")],
            additional_env={
                "PRESAVED_IMAGES": "true",
                "RGB_IMAGE_TOPIC": "/base_cam/rgb_image",
                "DEPTH_IMAGE_TOPIC": "/base_cam/depth_image",
                "WIDTH_ESTIMATION_PACKAGE_PATH": LaunchConfiguration("width_estimation_package_path"),
                "WIDTH_ESTIMATION_PACKAGE_DATA_PATH": LaunchConfiguration("width_estimation_package_data_path"),
            },
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_detector")),
        ),

        # ----------------------------
        # Vision (presaved)
        # ----------------------------
        Node(
            package="harvest_vision",
            executable="apple_prediction",
            name="apple_prediction",
            output="screen",
            parameters=[{
                "presaved_images": True,
                "prediction_model_path": PathJoinSubstitution(
                    [FindPackageShare("harvest_vision"), "yolo_networks", LaunchConfiguration("prediction_model")]),
                "prediction_yolo_conf": LaunchConfiguration("prediction_yolo_conf"),
                "prediction_radius_min": LaunchConfiguration("prediction_radius_min"),
                "prediction_radius_max": LaunchConfiguration("prediction_radius_max"),
                "prediction_distance_max": LaunchConfiguration("prediction_distance_max"),
                "source_frame": LaunchConfiguration("source_frame"),
                "target_frame": LaunchConfiguration("target_frame"),
                "camera_type": LaunchConfiguration("camera_type"),
                "presaved_images.rgbd_dir_path": trial_dir,
                "presaved_images.tree_id": -1,   # orchard_templating sets it for each tree
            }],
        ),

        Node(
            package="harvest_vision",
            executable="voxelize_scan",
            name="voxelize_scan",
            output="screen",
            parameters=[{
                "camera_type": LaunchConfiguration("camera_type"),
                "source_frame": LaunchConfiguration("source_frame"),
                "target_frame": LaunchConfiguration("target_frame"),
            }],
        ),
    ])
