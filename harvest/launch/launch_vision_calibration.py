import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_ros.actions
from launch.substitutions import TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    # 1. Declare a boolean flag (defaults to true)
    use_tcp_broadcaster = LaunchConfiguration('use_tcp_broadcaster')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_tcp_broadcaster',
            default_value='false', # Set to false for your simulation
            description='Whether to load the TCP pose broadcaster'
        ),

        # 2. Only spawn the controller if the condition is met
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['tcp_pose_broadcaster'],
            condition=IfCondition(use_tcp_broadcaster),
            output='screen',
        ),
    ])


def generate_launch_description():
    declared_arguments = []
    # Control
    declared_arguments.append(DeclareLaunchArgument("use_fake_hardware", default_value="True",
                                description="Use fake hardware"))

    ### apple_prediction node parameters & voxelize_scan (vision_experiment)
    # Segmentation model is trained on open source datasets and can give slightly more accurate 3D reconstruction results. 
    # detection models are trained on Prosser data and may be more robust to field conditions.
    declared_arguments.append(DeclareLaunchArgument("presaved_images", default_value="True",
                                  description="Whether to use presaved images for apple prediction. If False, will subscribe to camera topics for apple prediction. If True, will use presaved images"))
    declared_arguments.append(DeclareLaunchArgument("prediction_model", default_value="v9e.pt", 
                                  description="Yolo model used, can specify any model in the harvest_vision/yolo_models directory."))
    declared_arguments.append(DeclareLaunchArgument("prediction_yolo_conf", default_value="0.65", 
                                  description="Confidence threshold for yolo model in apple_prediction node."))
    declared_arguments.append(DeclareLaunchArgument("prediction_radius_min", default_value="0.03", 
                                  description="Minimum radius bound (meters) for ransac sphere fit in apple_prediction node."))
    declared_arguments.append(DeclareLaunchArgument("prediction_radius_max", default_value="0.06", 
                                  description="Maximum radius bound (meters) for ransac sphere fit in apple_prediction node."))
    declared_arguments.append(DeclareLaunchArgument("prediction_distance_max", default_value="4.0", 
                                  description="Distance threshold in meters for detecting apples. Filters out backgound apples."))
    declared_arguments.append(DeclareLaunchArgument("vision_experiment", default_value="a", 
                                  description="Microsoft Azure Kinect camera frame (color image, depth image, and point cloud) used in apple detection and voxelize scan."))
    declared_arguments.append(DeclareLaunchArgument("source_frame", default_value="mast_camera_color_optical_frame",
                                    description="Source frame for apple prediction. Should be the color camera frame"))
    declared_arguments.append(DeclareLaunchArgument("target_frame", default_value="amiga__base",
                                    description="Target frame for apple prediction. Should be the robot base frame"))
    declared_arguments.append(DeclareLaunchArgument("camera_type", default_value="azure",
                                    description="Type of camera used for apple detection. Options are 'azure' or 'realsense'. This affects the camera intrinsics used in the apple_prediction node."))

    ### apple_prediction live camera parameters
    declared_arguments.append(DeclareLaunchArgument("camera_ns", default_value="camera/mast_camera",
                                    description="Namespace of the camera topics to subscribe to for apple_prediction node"))
    declared_arguments.append(DeclareLaunchArgument("use_aligned_depth", default_value="True",
                                    description="Whether to use aligned depth images for apple prediction. If False, will use raw"))
    declared_arguments.append(DeclareLaunchArgument("allow_reuse_latest_frame", default_value="False",
                                    description="Whether to allow reuse of the latest camera frame in apple_prediction node. If True, will use the latest camera frame if a new frame is not available."))
    declared_arguments.append(DeclareLaunchArgument("x_tolerance", default_value="0.5",
                                    description="Tolerance in meters for determining whether a detected apple is the same as a previously detected apple based on x coordinate"))
    

    ### apple_prediction presaved_images parameters
    target_package_path = get_package_share_directory('harvest_vision')
    images_rgb = os.path.join(target_package_path, "data", "prosser_a", "color_raw.png")
    images_depth = os.path.join(target_package_path, "data", "prosser_a", "depth_to_color.png")
    declared_arguments.append(DeclareLaunchArgument("presaved_images.rgb_image_path", default_value=images_rgb,
                                    description="Path to the RGB image for presaved images mode."))
    declared_arguments.append(DeclareLaunchArgument("presaved_images.depth_image_path", default_value=images_depth,
                                    description="Path to the depth image for presaved images mode."))
    ### visual_servo node parameters
    declared_arguments.append(DeclareLaunchArgument("vservo_model", default_value="v9e.pt", 
                                  description="Yolo model used, can specify any model in the harvest_vision/yolo_models directory."))
    declared_arguments.append(DeclareLaunchArgument("vservo_yolo_conf", default_value="0.85", 
                                description="Confidence threshold for yolo model in visual_servo node."))
    declared_arguments.append(DeclareLaunchArgument("vservo_accuracy_px", default_value="10", 
                                  description="Specifies in pixels how close the center of the camera must be to the apple center to stop visual servoing."))
    declared_arguments.append(DeclareLaunchArgument("vservo_smoothing_factor", default_value="6.0", 
                                  description="Smoothing factor on velocity based on how far away the target apple center is from the camera center. Higher smoothing factor, faster movement when apple is far away."))
    declared_arguments.append(DeclareLaunchArgument("vservo_max_vel", default_value="0.6", 
                                  description="Maximum velocity that arm end effector can move during visual servo."))

    ### getting paths to yolo_networks
    declared_arguments.append(DeclareLaunchArgument('prediction_model_path', default_value=[PathJoinSubstitution([FindPackageShare("harvest_vision"), "yolo_networks", LaunchConfiguration("prediction_model")])]))
    declared_arguments.append(DeclareLaunchArgument('vservo_model_path', default_value=[PathJoinSubstitution([FindPackageShare("harvest_vision"), "yolo_networks", LaunchConfiguration("vservo_model")])]))
    
    ### Offsets for where to place the point cloud. y is up/down
    declared_arguments.append(DeclareLaunchArgument("pointcloud_offset", default_value="(-0.1, -0.1, 0.55)", 
                                  description="Tuple for x,y,z offset of apple point cloud from mast camera base"))

    ### Nodes
    palm_camera = launch_ros.actions.Node(
        package="harvest_vision",
        executable="gripper_palm_camera",
        name="gripper_palm_camera",
        parameters=[
            {"use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
             "source_frame": LaunchConfiguration("source_frame")
            }            
        ]
    )

    tcp_pose_relay = launch_ros.actions.Node(
        package="harvest_hardware_description",
        executable="tcp_pose_relay",
        parameters=[
            {"use_fake_hardware": LaunchConfiguration("use_fake_hardware")}            
        ]
    )
    apple_prediction_node = launch_ros.actions.Node(
                package="harvest_vision",
                executable="apple_prediction",
                name="apple_prediction",
                parameters=[
                    {"presaved_images": LaunchConfiguration("presaved_images"),
                     "prediction_model_path": LaunchConfiguration("prediction_model_path"),
                     "prediction_yolo_conf": LaunchConfiguration("prediction_yolo_conf"),
                     "prediction_radius_min": LaunchConfiguration("prediction_radius_min"),
                     "prediction_radius_max": LaunchConfiguration("prediction_radius_max"),
                     "prediction_distance_max": LaunchConfiguration("prediction_distance_max"),
                     "vision_experiment": LaunchConfiguration("vision_experiment"),
                     "source_frame": LaunchConfiguration("source_frame"),
                     "target_frame": LaunchConfiguration("target_frame"),
                    
                     # Parameters for live camera mode (will be ignored if presaved_images is False)
                     "camera_ns": LaunchConfiguration("camera_ns"),
                     "use_aligned_depth": LaunchConfiguration("use_aligned_depth"),
                     "allow_reuse_latest_frame": LaunchConfiguration("allow_reuse_latest_frame"),
                     "x_tolerance": LaunchConfiguration("x_tolerance"),

                     # Parameters for presaved images mode (will be ignored if presaved_images is False)
                     "camera_type": LaunchConfiguration("camera_type"),
                     "presaved_images.rgb_image_path": LaunchConfiguration("presaved_images.rgb_image_path"),
                     "presaved_images.depth_image_path": LaunchConfiguration("presaved_images.depth_image_path"),
                     "pointcloud_offset": LaunchConfiguration("pointcloud_offset"),
                      }
                ])
    
    voxelize_scan_node = launch_ros.actions.Node(
                package="harvest_vision",
                executable="voxelize_scan",
                name="voxelize_scan",
                parameters=[
                    {"vision_experiment": LaunchConfiguration("vision_experiment"),
                     "camera_type": LaunchConfiguration("camera_type"),
                      }
                ])
    
    arm_control = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('harvest_control'), 'launch'), '/arm_control.launch.py']),
         launch_arguments ={'max_vel': '0.1', 
                            'max_accel': '0.1', 
                            'traj_time_step': '0.025', 
                            'use_fake_hardware': LaunchConfiguration("use_fake_hardware"), 
                            'rviz_file': 'view_robot_with_apples.rviz'
                            }.items()
    )

    vservo_node = launch_ros.actions.Node(
                package="harvest_control",
                executable="visual_servo.py",
                name="visual_servo",
                parameters=[
                    {"vservo_model_path": LaunchConfiguration("vservo_model_path"),
                     "vservo_yolo_conf": LaunchConfiguration("vservo_yolo_conf"),
                     "vservo_accuracy_px": LaunchConfiguration("vservo_accuracy_px"),
                     "vservo_smoothing_factor": LaunchConfiguration("vservo_smoothing_factor"),
                     "vservo_max_vel": LaunchConfiguration("vservo_max_vel")
                      }
                ])

    return LaunchDescription(declared_arguments + [
                             apple_prediction_node,
                             palm_camera,
                             #voxelize_scan_node,
                             tcp_pose_relay,
                             arm_control,
                             vservo_node
    ])