from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Clearpath Warthog (2023 Prosser bags): robot_state_publisher with urdf/warthog_2023.urdf.xacro.
# The namespace keeps its robot_description apart from the Amiga description.
# Play the bag without its TF: --remap /tf:=/bag/tf /tf_static:=/bag/tf_static


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("harvest_hardware_description"), "urdf", "warthog_2023.urdf.xacro"])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="warthog_state_publisher",
            namespace="warthog",
            output="screen",
            parameters=[
                {"robot_description": ParameterValue(Command(["xacro ", xacro_file]), value_type=str)},
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),
    ])
