import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the installed Python scripts in the C++ package
    package_name = 'harvest_control'
    coord_to_traj_script = os.path.join(get_package_prefix(package_name), 'lib', package_name, 'coordinate_to_trajectory.py')
    visual_servo_script = os.path.join(get_package_prefix(package_name), 'lib', package_name, 'visual_servo.py')

    return LaunchDescription([
        # Launch the coordinate_to_trajectory_node
        ExecuteProcess(
            cmd=['python3', coord_to_traj_script],
            name='coordinate_to_trajectory',
            output='screen'
        ),
        
        # # Launch the visual_servo node
        # ExecuteProcess(
        #     cmd=['python3', visual_servo_script],
        #     name='coordinate_to_trajectory_client',
        #     output='screen'
        # ),
        # Launch C++ node
        Node(
            package='harvest_control',
            executable='move_arm',
            name='move_arm_node',
        ),
    ])
