from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_ros.actions
from launch.substitutions import TextSubstitution

def generate_launch_description():
    declared_arguments = []
    ### apple_prediction node parameters & voxelize_scan (vision_experiment)
    # Segmentation model is trained on open source datasets and can give slightly more accurate 3D reconstruction results. 
    # detection models are trained on Prosser data and may be more robust to field conditions.
    declared_arguments.append(DeclareLaunchArgument("prediction_model", default_value="best_segmentation.pt", 
                                  description="Yolo model used, can specify any model in the harvest_vision/yolo_models directory."))
    declared_arguments.append(DeclareLaunchArgument("prediction_yolo_conf", default_value="0.85", 
                                  description="Confidence threshold for yolo model in apple_prediction node."))
    declared_arguments.append(DeclareLaunchArgument("prediction_radius_min", default_value="0.03", 
                                  description="Minimum radius bound (meters) for ransac sphere fit in apple_prediction node."))
    declared_arguments.append(DeclareLaunchArgument("prediction_radius_max", default_value="0.06", 
                                  description="Maximum radius bound (meters) for ransac sphere fit in apple_prediction node."))
    declared_arguments.append(DeclareLaunchArgument("prediction_distance_max", default_value="1.0", 
                                  description="Distance threshold in meters for detecting apples. Filters out backgound apples."))
    declared_arguments.append(DeclareLaunchArgument("vision_experiment", default_value="a", 
                                  description="Microsoft Azure Kinect camera frame (color image, depth image, and point cloud) used in apple detection and voxelize scan."))

    ### getting paths to yolo_networks
    declared_arguments.append(DeclareLaunchArgument('prediction_model_path', default_value=[PathJoinSubstitution([FindPackageShare("harvest_vision"), "yolo_networks", LaunchConfiguration("prediction_model")])]))

    ### Nodes
    apple_prediction_node = launch_ros.actions.Node(
                package="harvest_vision",
                executable="apple_prediction_presaved_images",
                name="apple_prediction_presaved_images",
                parameters=[
                    {"prediction_model_path": LaunchConfiguration("prediction_model_path"),
                     "prediction_yolo_conf": LaunchConfiguration("prediction_yolo_conf"),
                     "prediction_radius_min": LaunchConfiguration("prediction_radius_min"),
                     "prediction_radius_max": LaunchConfiguration("prediction_radius_max"),
                     "prediction_distance_max": LaunchConfiguration("prediction_distance_max"),
                     "vision_experiment": LaunchConfiguration("vision_experiment"),
                      }
                ])
    
    voxelize_scan_node = launch_ros.actions.Node(
                package="harvest_vision",
                executable="voxelize_scan",
                name="voxelize_scan",
                parameters=[
                    {"vision_experiment": LaunchConfiguration("vision_experiment"),
                      }
                ])
    
    return LaunchDescription(declared_arguments + [
                             apple_prediction_node, 
                             voxelize_scan_node,
    ])