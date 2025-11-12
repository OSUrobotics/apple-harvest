#!/usr/bin/env python3
import os, yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

def load_yaml(pkg, relpath):
    path = os.path.join(get_package_share_directory(pkg), relpath)
    with open(path, "r") as f:
        return yaml.safe_load(f)

def generate_launch_description():
    args = [
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument("prefix", default_value=""),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("robot_ip", default_value="yyy.yyy.yyy.yyy"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("description_file", default_value="amiga_ur_gripper.urdf.xacro"),
        # If your controller_manager isn’t at '/controller_manager', set it here:
        DeclareLaunchArgument("controller_manager_ns", default_value="/controller_manager"),
        DeclareLaunchArgument("launch_servo", default_value="true"),
        
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])

def _launch_setup(context):
    use_fake = LaunchConfiguration("use_fake_hardware").perform(context).lower() in ("1", "true", "yes")
    launch_servo = LaunchConfiguration("launch_servo")

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

    # --- Controllers (MoveIt’s view only; driver actually spawns) ---
    controllers_yaml = load_yaml("harvest_hardware_moveit_config", "config/controllers.yaml")
    controllers_yaml.setdefault(
        "moveit_controller_manager",
        "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    )
    controllers_yaml.setdefault("moveit_simple_controller_manager", {})
    
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            {"planning_pipelines": ["ompl"], "default_planning_pipeline": "ompl"},
            ompl_yaml,
            controllers_yaml,
        ],
    )

    servo_yaml = load_yaml("harvest_hardware_moveit_config", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node_main",
        parameters=[servo_params, robot_description, robot_description_semantic, kinematics_yaml],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="screen",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("harvest_hardware_moveit_config"), "rviz", "view_robot.rviz"
        ])],
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, joint_limits_yaml],
    )

    # Important: no spawners/unspawners here
    return [move_group, rviz, servo_node]
