import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the installed Python scripts in the C++ package
    package_name = 'harvest_control'
    coord_to_traj_script = os.path.join(get_package_prefix(package_name), 'lib', package_name, 'coordinate_to_trajectory.py')
    event_detector_script = os.path.join(get_package_prefix(package_name), 'lib', package_name, 'event_detector.py')
    force_filter_script = os.path.join(get_package_prefix(package_name), 'lib', package_name, 'force_filter.py')
    heuristic_controller_script = os.path.join(get_package_prefix(package_name), 'lib', package_name, 'heuristic_controller.py')
    linear_controller_script = os.path.join(get_package_prefix(package_name), 'lib', package_name, 'linear_controller.py')
    pose_listener_script = os.path.join(get_package_prefix(package_name), 'lib', package_name, 'pose_listener.py')
    pressure_averager_script = os.path.join(get_package_prefix(package_name), 'lib', package_name, 'pressure_averager.py')
    pull_twist_controller_script = os.path.join(get_package_prefix(package_name), 'lib', package_name, 'pull_twist_controller.py')

    return LaunchDescription([
        # Launch the coordinate_to_trajectory_node
        ExecuteProcess(
            cmd=['python3', coord_to_traj_script],
            name='coordinate_to_trajectory',
            output='screen'
        ),

        ExecuteProcess(
            cmd=['python3', event_detector_script],
            name='event_detector',
            output='screen'
        ),

        ExecuteProcess(
            cmd=['python3', force_filter_script],
            name='forcefilter',
            output='screen'
        ),

        ExecuteProcess(
            cmd=['python3', heuristic_controller_script],
            name='pick_controller',
            output='screen'
        ),

        ExecuteProcess(
            cmd=['python3', linear_controller_script],
            name='linear_controller',
            output='screen'
        ),

        ExecuteProcess(
            cmd=['python3', pose_listener_script],
            name='tf_listener',
            output='screen'
        ),

        ExecuteProcess(
            cmd=['python3', pressure_averager_script],
            name='pressure_averager',
            output='screen'
        ),

        ExecuteProcess(
            cmd=['python3', pull_twist_controller_script],
            name='pull_twist_controller',
            output='screen'
        ),
        
        # Launch C++ node
        Node(
            package='harvest_control',
            executable='move_arm',
            name='move_arm_node',
        ),
    ])
