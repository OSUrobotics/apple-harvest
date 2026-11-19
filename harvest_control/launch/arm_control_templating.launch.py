import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import EqualsSubstitution, IfElseSubstitution, LaunchConfiguration, TextSubstitution
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
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
    declared_arguments.append(DeclareLaunchArgument('rviz_file', default_value="view_templating.rviz",
                                  description="RViz config in harvest_hardware_moveit_config/rviz (planning scene, "
                                              "mast and base-cam point clouds, apple and voxel markers)."))

    ### generate_trellis_collision_obj node parameters
    declared_arguments.append(DeclareLaunchArgument('use_tree_scene', default_value="true",
                                  description="Start the trellis collision object node. Set false when a different launch "
                                              "starts it (for example templating_trial.launch.py)."))
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
                                  description="Set the max acceleration of the UR5."))
    declared_arguments.append(DeclareLaunchArgument('max_vel', default_value="0.05",
                                  description="Set the max velocity of the UR5."))
    declared_arguments.append(DeclareLaunchArgument('traj_time_step', default_value="0.05",
                                  description="Time step (in seconds) between UR5 joint trajectory waypoints."))

    # UR driver (controllers), robot description and MoveIt: the same includes as arm_control.launch.py
    moveit_config = get_package_share_directory('harvest_hardware_moveit_config')
    description = get_package_share_directory('harvest_hardware_description')
    initial_controller = IfElseSubstitution(
        EqualsSubstitution(LaunchConfiguration('use_fake_hardware'), TextSubstitution(text='true')),
        TextSubstitution(text='joint_trajectory_controller'),
        TextSubstitution(text='scaled_joint_trajectory_controller'),
    )

    return LaunchDescription(declared_arguments + [
        # Scoped group: the launch arguments of an include stay in the context after the include. Without the group,
        # launch_rviz 'false' of the UR driver would also set launch_rviz for ur_moveit.launch.py (view_rviz).
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(moveit_config, 'launch', 'ur_control_no_rsp.launch.py')),
            launch_arguments={
                'ur_type': LaunchConfiguration('ur_type'),
                'robot_ip': LaunchConfiguration('robot_ip'),
                'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
                'use_mock_hardware': LaunchConfiguration('use_fake_hardware'),
                'description_package': 'harvest_hardware_description',
                'description_file': 'amiga_ur_gripper.urdf.xacro',
                'launch_rviz': 'false',
                'initial_joint_controller': initial_controller,
                'controllers_file': os.path.join(moveit_config, 'config', 'ur_controllers.yaml'),
            }.items(),
        )]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(description, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={
                'ur_type': LaunchConfiguration('ur_type'),
                'robot_ip': LaunchConfiguration('robot_ip'),
                'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
                'use_mock_hardware': LaunchConfiguration('use_fake_hardware'),
                'description_package': 'harvest_hardware_description',
                'description_file': 'amiga_ur_gripper.urdf.xacro',
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(moveit_config, 'launch', 'ur_moveit.launch.py')),
            launch_arguments={
                'ur_type': LaunchConfiguration('ur_type'),
                'view_rviz': LaunchConfiguration('launch_rviz'),
                'rviz_file': LaunchConfiguration('rviz_file'),
                'robot_ip': LaunchConfiguration('robot_ip'),
                'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
                'description_file': 'amiga_ur_gripper.urdf.xacro',
                'launch_servo': 'false',
                # move_arm_node (started by ur_moveit.launch.py)
                'max_accel': LaunchConfiguration('max_accel'),
                'max_vel': LaunchConfiguration('max_vel'),
                'traj_time_step': LaunchConfiguration('traj_time_step'),
            }.items(),
        ),

        # Trellis collision objects (tree_template generate_trellis_collision_obj)
        Node(
            package='tree_template',
            executable='tree_template',
            name='update_trellis_position',
            condition=IfCondition(LaunchConfiguration('use_tree_scene')),
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

    ])
