import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the path to the installed Python scripts in the C++ package
    package_name = 'harvest_control'

    declared_arguments = []

    ### UR driver and MoveIt arguments
    declared_arguments.append(DeclareLaunchArgument('ur_type', default_value="ur5e", 
                                  description="Type of Universal Robot."))
    declared_arguments.append(DeclareLaunchArgument('robot_ip', default_value="yyy.yyy.yyy.yyy", 
                                  description="IP address of the robot."))
    declared_arguments.append(DeclareLaunchArgument('use_fake_hardware', default_value="true", 
                                  description="Use fake hardware for the UR robot."))
    declared_arguments.append(DeclareLaunchArgument('launch_rviz', default_value="true", 
                                  description="Launch RViz for visualization."))


    ### generate_trellis_collision_obj node parameters
    declared_arguments.append(DeclareLaunchArgument('leader_branch_radii', default_value="0.08", 
                                  description="Leader branch radius for trellis template."))
    declared_arguments.append(DeclareLaunchArgument("leader_branch_len", default_value="2.0",
                                  description="Leader branch length for trellis template."))
    declared_arguments.append(DeclareLaunchArgument("num_side_branches", default_value="4.0",
                                  description="Number of side branches for trellis template."))
    declared_arguments.append(DeclareLaunchArgument("side_branch_radii", default_value="0.04",
                                  description="Side branch radius for trellis template."))
    declared_arguments.append(DeclareLaunchArgument("side_branch_len", default_value="2.0",
                                  description="Side branch length for trellis template."))

    ### move_arm node parameters
    declared_arguments.append(DeclareLaunchArgument('max_accel', default_value="0.05", 
                                  description="Set the max accelleration of the UR5."))
    declared_arguments.append(DeclareLaunchArgument('max_vel', default_value="0.05", 
                                  description="Set the max velocity of the UR5."))
    declared_arguments.append(DeclareLaunchArgument('traj_time_step', default_value="0.05", 
                                  description="Time step (in seconds) between UR5 joint trajectory waypoints."))

    # Path to UR5 launch files
    ur_driver_launch_path = os.path.join(
        get_package_share_directory('ur_robot_driver'),
        'launch',
        'ur_control_custom_hw.launch.py')
    
    ur_moveit_launch_path = os.path.join(
        get_package_share_directory('ur_moveit_config'),
        'launch',
        'ur_moveit_custom_hw.launch.py')


    return LaunchDescription(declared_arguments + [
        # Include UR driver launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_driver_launch_path),
            launch_arguments={
                'ur_type': LaunchConfiguration('ur_type'),
                'robot_ip': LaunchConfiguration('robot_ip'),
                'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
                'launch_rviz': LaunchConfiguration('launch_rviz'),
            }.items(),
        ),

        # Include UR MoveIt launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_moveit_launch_path),
            launch_arguments={
                'ur_type': LaunchConfiguration('ur_type'),
                'launch_rviz': LaunchConfiguration('launch_rviz'),
            }.items(),
        ),

        # Launch the generate_trellis_collision_obj_node from external package
        Node(
            package='tree_template',
            executable='tree_template',
            name='update_trellis_position',
            parameters=[
                    {"leader_branch_radii": LaunchConfiguration("leader_branch_radii"),
                     "leader_branch_len": LaunchConfiguration("leader_branch_len"),
                     "num_side_branches": LaunchConfiguration("num_side_branches"),
                     "side_branch_radii": LaunchConfiguration("side_branch_radii"),
                     "side_branch_len": LaunchConfiguration("side_branch_len"),
                      }
                    ]
        ),
        
        Node(
            package='harvest_control',
            executable='pose_listener.py',
            name='tf_listener',
        ),
        
        # Launch C++ node
        Node(
            package='harvest_control',
            executable='move_arm',
            name='move_arm_node',
            parameters=[
                    {"max_accel": LaunchConfiguration("max_accel"),
                     "max_vel": LaunchConfiguration("max_vel"),
                     "traj_time_step": LaunchConfiguration("traj_time_step"),
                      }
                    ]
        ),
    ])
