To run Marcus Path-Query:

In the first terminal: (sim vs. real)
ros2 launch ur_robot_driver ur_control_custom_hw.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=true
ros2 launch ur_robot_driver ur_control_custom_hw.launch.py ur_type:=ur5e robot_ip:=169.254.174.50 launch_rviz:=true

In the second terminal:
ros2 launch ur_moveit_config ur_moveit_custom_hw.launch.py ur_type:=ur5e launch_rviz:=true

In a third terminal:
ros2 run harvest_control move_arm

In a fourth terminal:
ros2 run harvest_control coordinate_to_trajectory.py
    
In a fifth terminal: (set desired x,y,z coorinates)
ros2 service call /coordinate_to_trajectory harvest_interfaces/srv/CoordinateToTrajectory "{coordinate: {x: 0.2, y: 0.5, z: 0.8}}"

To send the robot to home configuration:
ros2 service call /move_arm_to_home std_srvs/srv/Empty 
