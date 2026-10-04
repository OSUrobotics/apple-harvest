from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    ld = LaunchDescription()

    #Launch arg for Gripper type
    gripper_type = LaunchConfiguration("gripper_type")
    gripper_arg = DeclareLaunchArgument('gripper_type', default_value="old", 
                                  description="Which gripper is attached [old, finray]")
    grasp_strategy = LaunchConfiguration("grasp_strategy")
    grasp_strategy_arg = DeclareLaunchArgument('grasp_strategy', default_value="pressure", 
                                  description="Which grasp strategy [pressure, time]")
    

    
    ld.add_action(gripper_arg)
    ld.add_action(grasp_strategy_arg)
    # Launch the node to control gripper functionality
    ld.add_action(Node(
        package='gripper',
        executable='suction_gripper.py',
        condition=IfCondition(PythonExpression([
            "'",
            gripper_type,
            "' == 'old'"
        ]))
    ))

    #Deploy Wifi | Note ifname is device specific (wlo1) run nmcli device status
    ld.add_action(ExecuteProcess(
        cmd=['nmcli', 'device', 'wifi', 'hotspot',
            'ifname', 'wlo1',
            'ssid', 'northjar',
            'password', 'finray_gripper'],
        output='screen',
        condition=IfCondition(PythonExpression(["'", gripper_type, "' == 'finray'"]))
    ))


    #Launch Micro ros agent
    ld.add_action(Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['udp4', '--port', '8888'],
        condition=IfCondition(PythonExpression([
            "'",
            gripper_type,
            "' == 'finray'",
        ]))
    ))



    # Launch the node for pressure servoing 
    ld.add_action(Node(
        package='gripper',
        executable='grasp_controller.py',
        parameters=[{'gripper_type' : gripper_type}, {'grasp_strategy' : grasp_strategy}]
    ))
    
    return ld
