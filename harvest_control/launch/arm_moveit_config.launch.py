import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

def load_yaml(pkg: str, relpath: str):
    path = os.path.join(get_package_share_directory(pkg), relpath)
    with open(path, "r") as f:
        return yaml.safe_load(f)

def generate_launch_description():
    # Common args
    args = [
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        DeclareLaunchArgument('robot_ip', default_value='yyy.yyy.yyy.yyy'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('description_package', default_value='robot_custom_hardware'),
        DeclareLaunchArgument('description_file', default_value='amiga_ur_gripper.urdf.xacro'),
        DeclareLaunchArgument('max_accel', default_value='0.05'),
        DeclareLaunchArgument('max_vel', default_value='0.05'),
        DeclareLaunchArgument('traj_time_step', default_value='0.05'),
    ]

    # URDF from your custom xacro
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ',
                PathJoinSubstitution([
                    FindPackageShare(LaunchConfiguration('description_package')),
                    'urdf',
                    LaunchConfiguration('description_file'),
                ]),
                # ===== Arguments matching amiga_ur_gripper.urdf.xacro =====
                ' ',
                TextSubstitution(text='name:='), 'ur',
                ' ',
                TextSubstitution(text='ur_type:='), LaunchConfiguration('ur_type'),
                ' ',
                TextSubstitution(text='prefix:='), '',               # default empty
                ' ',
                TextSubstitution(text='robot_ip:='), LaunchConfiguration('robot_ip'),
                ' ',
                TextSubstitution(text='use_fake_hardware:='), LaunchConfiguration('use_fake_hardware'),
                ' ',
                TextSubstitution(text='headless_mode:=false'),
                ' ',
                TextSubstitution(text='fake_sensor_commands:=false'),
                ' ',
                TextSubstitution(text='sim_gazebo:=false'),
                ' ',
                TextSubstitution(text='sim_ignition:=false'),
                ' ',
                TextSubstitution(text='safety_limits:=false'),
                ' ',
                TextSubstitution(text='tool_device_name:=/tmp/ttyUR'),
            ]),
            value_type=str
        )
    }

    # SRDF is a xacro in ur_moveit_config
    ur_moveit_share = get_package_share_directory("ur_moveit_config")
    srdf_relpath = "config/ur.srdf.xacro" if os.path.exists(
        os.path.join(ur_moveit_share, "config/ur.srdf.xacro")
    ) else "srdf/ur.srdf.xacro"

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command([
                FindExecutable(name="xacro"), " ",
                PathJoinSubstitution([
                    FindPackageShare("ur_moveit_config"),
                    srdf_relpath,
                ]),
                " ",
                TextSubstitution(text="name:="), TextSubstitution(text="ur"),
                " ",
                TextSubstitution(text="prefix:="), TextSubstitution(text=""),
            ]),
            value_type=str
        )
    }

    # YAML configs
    raw_kin = load_yaml('ur_moveit_config', 'config/kinematics.yaml')

    # Support both key names; MoveIt looks up by group name from the SRDF
    if isinstance(raw_kin, dict) and 'robot_description_kinematics' in raw_kin:
        kin = raw_kin['robot_description_kinematics']
    else:
        kin = raw_kin  # already the inner dict

    # If vendor file only has 'ur_manipulator', also expose it as 'manipulator'
    if 'manipulator' not in kin and 'ur_manipulator' in kin:
        kin['manipulator'] = kin['ur_manipulator']

    # (Or vice-versa, just in case)
    if 'ur_manipulator' not in kin and 'manipulator' in kin:
        kin['ur_manipulator'] = kin['manipulator']

    kinematics_yaml = {'robot_description_kinematics': kin}

    ompl_planning  = {'robot_description_planning':  load_yaml('ur_moveit_config', 'config/ompl_planning.yaml')}
    joint_limits   = {'robot_description_joint_limits': load_yaml('ur_moveit_config', 'config/joint_limits.yaml')}
    planning_pipeline = {'planning_pipelines': ['ompl']}

    move_arm = Node(
        package='harvest_control',
        executable='move_arm',
        name='move_arm_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning,
            joint_limits,
            planning_pipeline,
            {
                'max_accel': LaunchConfiguration('max_accel'),
                'max_vel': LaunchConfiguration('max_vel'),
                'traj_time_step': LaunchConfiguration('traj_time_step'),
            }
        ]
    )

    return LaunchDescription(args + [move_arm])
