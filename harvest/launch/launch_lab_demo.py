import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   robot_bringup = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('ur_robot_driver'), 'launch'),
         '/ur_control_custom_hw.launch.py']),
      launch_arguments ={'ur_type': 'ur5e', 'robot_ip': '169.254.177.230', 'launch_rviz':
                         'false', 'headless_mode':'true'}.items()
   )
   moveit_bringup = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('ur_moveit_config'), 'launch'),
         '/ur_moveit_custom_hw.launch.py']),
      launch_arguments ={'ur_type': 'ur5e', 'robot_ip': '169.254.177.230', 'launch_rviz':
                         'true'}.items()
   )

   arm_control = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('harvest_control'), 'launch'),
         '/arm_control.launch.py']),
      launch_arguments ={'max_vel': '0.1', 'max_accel': '0.1', 'traj_time_step': '0.025'}.items()
   )

   robot_vision = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('harvest'), 'launch'),
         '/launch_vision.launch.py']),
      launch_arguments ={'prediction_distance_max': '1.5', 'vservo_yolo_conf': '0.5', 'vservo_max_vel':
                         '0.3', 'palm_camera_device_num': '0'}.items()
   )   

   return LaunchDescription( [
      robot_bringup,
      moveit_bringup,
      arm_control,
      robot_vision
   ])