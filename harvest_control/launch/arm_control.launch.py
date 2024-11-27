import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the installed Python scripts in the C++ package
    package_name = 'harvest_control'

    declared_arguments = []
    ### harvest node parameter
    # The pick pattern is dependent on the controller selected with the below parameter
    declared_arguments.append(DeclareLaunchArgument("pick_pattern", default_value="pull-twist", 
                                  description="Pick pattern used, can specify: 'pull-twist', 'force-heuristic', or 'linear-pull'."))
    
    ### coordinate_to_trajectory node parameter
    # If using MoveIt with simulation or hardware
    declared_arguments.append(DeclareLaunchArgument('sim', default_value="False", 
                                  description="Simulation bool for MoveIt."))
    declared_arguments.append(DeclareLaunchArgument("voxel_distance_tol", default_value="0.5",
                                  description="Maximum distance tolerance between target coordinate and precomputed voxel coordinate."))

    ### move_arm node parameters
    declared_arguments.append(DeclareLaunchArgument('max_accel', default_value="0.05", 
                                  description="Set the max accelleration of the UR5."))
    declared_arguments.append(DeclareLaunchArgument('max_vel', default_value="0.05", 
                                  description="Set the max velocity of the UR5."))
    declared_arguments.append(DeclareLaunchArgument('traj_time_step', default_value="0.05", 
                                  description="Time step (in seconds) between UR5 joint trajectory waypoints."))

    return LaunchDescription(declared_arguments + [
        # Launch the coordinate_to_trajectory_node
        Node(
            package='harvest_control',
            executable='coordinate_to_trajectory.py',
            name='trajectory_query_node',
            parameters=[
                    {"sim": LaunchConfiguration("sim"),
                     "voxel_distance_tol": LaunchConfiguration("voxel_distance_tol")
                      }
                    ]
        ),

        # Node(
        #     package='harvest_control',
        #     executable='event_detector.py',
        #     name='event_detector',
        # ),

        # Node(
        #     package='harvest_control',
        #     executable='force_filter.py',
        #     name='forcefilter',
        # ),

        # Node(
        #     package='harvest_control',
        #     executable='heuristic_controller.py',
        #     name='pick_controller',
        # ),

        # Node(
        #     package='harvest_control',
        #     executable='linear_controller.py',
        #     name='linear_controller',
        # ),

        # Node(
        #     package='harvest_control',
        #     executable='pose_listener.py',
        #     name='tf_listener',
        # ),

        # Node(
        #     package='harvest_control',
        #     executable='pressure_averager.py',
        #     name='pressure_averager',
        # ),

        # Node(
        #     package='harvest_control',
        #     executable='pull_twist_controller.py',
        #     name='pull_twist_controller',
        # ),
        
        # Launch C++ node
        Node(
            package='harvest_control',
            executable='move_arm_action_based',
            name='move_arm_node',
            parameters=[
                    {"max_accel": LaunchConfiguration("max_accel"),
                     "max_vel": LaunchConfiguration("max_vel"),
                     "traj_time_step": LaunchConfiguration("traj_time_step"),
                      }
                    ]
        ),
    ])
