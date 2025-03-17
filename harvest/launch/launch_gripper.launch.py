from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ld = LaunchDescription()
    
    # Launch the node to control gripper functionality
    ld.add_action(Node(
        package='gripper',
        executable='suction_gripper.py',
    ))

    # Launch the node for pressure servoing 
    ld.add_action(Node(
        package='gripper',
        executable='grasp_controller.py',
    ))
    
    return ld
