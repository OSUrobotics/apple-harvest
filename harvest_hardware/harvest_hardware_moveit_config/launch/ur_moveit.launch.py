#!/usr/bin/env python3
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch.actions import TimerAction


def load_yaml(pkg, relpath):
    path = os.path.join(get_package_share_directory(pkg), relpath)
    with open(path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    args = [
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument("prefix", default_value=""),
        DeclareLaunchArgument("view_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_file", default_value="view_robot.rviz"),
        DeclareLaunchArgument("robot_ip", default_value="yyy.yyy.yyy.yyy"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("description_file", default_value="amiga_ur_gripper.urdf.xacro"),
        DeclareLaunchArgument("launch_servo", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_3d_sensors", default_value="false"),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])


def _launch_setup(context):
    # Resolve launch configurations
    use_fake = LaunchConfiguration("use_fake_hardware").perform(context).lower() in ("1", "true", "yes")
    launch_servo = LaunchConfiguration("launch_servo")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_3d_sensors = LaunchConfiguration("use_3d_sensors").perform(context).lower() in ("1", "true", "yes")

    # --- URDF ---
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ",
                PathJoinSubstitution([
                    FindPackageShare("harvest_hardware_description"),
                    "urdf",
                    LaunchConfiguration("description_file"),
                ]),
                " ",
                "name:=ur5e ",
                "ur_type:=", LaunchConfiguration("ur_type"), " ",
                "prefix:=", LaunchConfiguration("prefix"), " ",
                "robot_ip:=", LaunchConfiguration("robot_ip"), " ",
                "use_fake_hardware:=", LaunchConfiguration("use_fake_hardware"), " ",
            ]),
            value_type=str,
        )
    }

    # --- SRDF ---
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command([
                "xacro ",
                PathJoinSubstitution([
                    FindPackageShare("harvest_hardware_moveit_config"),
                    "srdf",
                    "ur.srdf.xacro",
                ]),
                " ",
                "name:=ur5e ",
                "prefix:=", LaunchConfiguration("prefix"),
            ]),
            value_type=str,
        )
    }

    # --- MoveIt configs ---
    kinematics_yaml = load_yaml("harvest_hardware_moveit_config", "config/kinematics.yaml")
    joint_limits_yaml = {
        "robot_description_planning": load_yaml(
            "harvest_hardware_moveit_config", "config/joint_limits.yaml"
        )
    }

    if use_fake:
        adapter_chain = (
            "default_planner_request_adapters/ResolveConstraintFrames "
            "default_planner_request_adapters/FixWorkspaceBounds "
            "default_planner_request_adapters/FixStartStateBounds "
            "default_planner_request_adapters/FixStartStateCollision "
            "default_planner_request_adapters/FixStartStatePathConstraints "
            "default_planner_request_adapters/AddTimeOptimalParameterization"
        )
    else:
        adapter_chain = (
            "default_planner_request_adapters/ResolveConstraintFrames "
            "default_planner_request_adapters/FixWorkspaceBounds "
            "default_planner_request_adapters/FixStartStateBounds "
            "default_planner_request_adapters/FixStartStateCollision "
            "default_planner_request_adapters/FixStartStatePathConstraints "
            "default_planner_request_adapters/AddRuckigTrajectorySmoothing"
        )

    ompl_yaml = {"ompl": load_yaml("harvest_hardware_moveit_config", "config/ompl_planning.yaml")}
    ompl_yaml["ompl"].update({
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": adapter_chain,
        "start_state_max_bounds_error": 0.1,
    })

    controllers_yaml = load_yaml("harvest_hardware_moveit_config", "config/controllers.yaml")
    controllers_yaml.setdefault(
        "moveit_controller_manager",
        "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    )
    controllers_yaml.setdefault("moveit_simple_controller_manager", {})

    # Octomap & 3D sensor config
    octomap_params = {
        "octomap_frame": "world",
        "octomap_resolution": 0.05,
        "max_range": 5.0,
        "occupancy_map_monitor": {
            "max_update_rate": 1.0
        }
    }

    sensors_3d_yaml = load_yaml(
        "harvest_hardware_moveit_config",
        "config/sensors_3d.yaml",
    )

    # ---- move_group parameters (build list dynamically) ----
    move_group_params = [
        robot_description,
        robot_description_semantic,
        kinematics_yaml,
        joint_limits_yaml,
        {"planning_pipelines": ["ompl"], "default_planning_pipeline": "ompl"},
        ompl_yaml,
        controllers_yaml,
        {"use_sim_time": use_sim_time},
    ]

    # Only add 3D sensor / octomap params if enabled
    if use_3d_sensors:
        move_group_params.append(octomap_params)
        move_group_params.append(sensors_3d_yaml)

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="ur_move_group",
        output="screen",
        parameters=move_group_params,
    )

    # Servo node
    servo_yaml = load_yaml("harvest_hardware_moveit_config", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    move_arm = Node(
        package="harvest_control",
        executable="move_arm",
        name="move_arm_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            {"use_sim_time": use_sim_time},
            {"max_accel": 0.05, "max_vel": 0.05, "traj_time_step": 0.05},  # or LaunchConfigurations
        ],
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution([
                FindPackageShare("harvest_hardware_moveit_config"),
                "rviz",
                LaunchConfiguration("rviz_file"),
            ]),
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(LaunchConfiguration("view_rviz")),
    )
    delay_rviz_before_start = TimerAction(
        period=3.0,  # Delays RViz by 5 seconds
        actions=[rviz_node]
    )

    print("UR Move")
    return [
        move_group, 
        servo_node,
        move_arm,
        # rsp,
        delay_rviz_before_start, 
        ]
