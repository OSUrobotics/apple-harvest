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
    # ### apple_prediction node parameters
    # # Segmentation model is trained on open source datasets and can give slightly more accurate 3D reconstruction results. 
    # # detection models are trained on Prosser data and may be more robust to field conditions.
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
    declared_arguments.append(DeclareLaunchArgument("scan_data_path", default_value="NOTGIVEN", 
                                  description="Data path to save pointcloud, realsense rgb image, realsense depth image and masks from prediction node. In format home/dir/data do not add slash at the end."))


    ### visual_servo node parameters
    declared_arguments.append(DeclareLaunchArgument("vservo_model", default_value="best_segmentation.pt", 
                                  description="Yolo model used, can specify any model in the harvest_vision/yolo_models directory."))
    declared_arguments.append(DeclareLaunchArgument("vservo_yolo_conf", default_value="0.85", 
                                description="Confidence threshold for yolo model in visual_servo node."))
    declared_arguments.append(DeclareLaunchArgument("vservo_accuracy_px", default_value="10", 
                                  description="Specifies in pixels how close the center of the camera must be to the apple center to stop visual servoing."))
    declared_arguments.append(DeclareLaunchArgument("vservo_smoothing_factor", default_value="6.0", 
                                  description="Smoothing factor on velocity based on how far away the target apple center is from the camera center. Higher smoothing factor, faster movement when apple is far away."))
    declared_arguments.append(DeclareLaunchArgument("vservo_max_vel", default_value="0.6", 
                                  description="Maximum velocity that arm end effector can move during visual servo."))
    
    ### palm camera publisher node parameters
    declared_arguments.append(DeclareLaunchArgument("palm_camera_device_num", default_value="2", 
                                  description="Device number for palm RGB camer in gripper."))

    ### getting paths to yolo_networks
    declared_arguments.append(DeclareLaunchArgument('prediction_model_path', default_value=[PathJoinSubstitution([FindPackageShare("harvest_vision"), "yolo_networks", LaunchConfiguration("prediction_model")])]))
    declared_arguments.append(DeclareLaunchArgument('vservo_model_path', default_value=[PathJoinSubstitution([FindPackageShare("harvest_vision"), "yolo_networks", LaunchConfiguration("vservo_model")])]))
    
    ### Nodes
    apple_prediction_node = launch_ros.actions.Node(
                package="harvest_vision",
                executable="apple_prediction",
                name="apple_prediction",
                parameters=[
                    {"prediction_model_path": LaunchConfiguration("prediction_model_path"),
                     "prediction_yolo_conf": LaunchConfiguration("prediction_yolo_conf"),
                     "prediction_radius_min": LaunchConfiguration("prediction_radius_min"),
                     "prediction_radius_max": LaunchConfiguration("prediction_radius_max"),
                     "prediction_distance_max": LaunchConfiguration("prediction_distance_max"),
                     "scan_data_path": LaunchConfiguration("scan_data_path")
                      }
                ])
    
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

    palm_camera_node = launch_ros.actions.Node(
                package="ur5e_custom_hardware",
                executable="gripper_palm_camera",
                name="gripper_palm_camera",
                parameters=[
                    {"palm_camera_device_num": LaunchConfiguration("palm_camera_device_num")
                      }
                ])
    
    return LaunchDescription(declared_arguments + [
                             apple_prediction_node, 
                             vservo_node, 
                             palm_camera_node,
    ])