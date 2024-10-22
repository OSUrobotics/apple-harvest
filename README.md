To run apple locatization and path query

With UR5e plugged into computer, ensure the pendant is set to *remote* mode!

1. In the first terminal: (sim vs. real):

```bash
ros2 launch ur_robot_driver ur_control_custom_hw.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=true
```

OR

```bash
ros2 launch ur_robot_driver ur_control_custom_hw.launch.py ur_type:=ur5e robot_ip:=169.254.177.230 launch_rviz:=true headless_mode:=true
```

2. In the second terminal:
```bash
ros2 launch ur_moveit_config ur_moveit_custom_hw.launch.py ur_type:=ur5e launch_rviz:=true
```

3. In a third terminal:
```bash
ros2 launch harvest_control arm_control.launch.py
```

4. In fourth terminal run the vision (need to ensure palm camera connects to proper idx):
```bash
ros2 launch harvest launch_vision.launch.py prediction_distance_max:=1.5 vservo_yolo_conf:=0.5 vservo_max_vel:=0.4 palm_camera_device_num:=8
```

5. To send the robot to home configuration:
```bash
ros2 service call /move_arm_to_home std_srvs/srv/Trigger
```

6. To set desired a desired end-effector x,y,z coorinate:
```bash
ros2 service call /coordinate_to_trajectory harvest_interfaces/srv/CoordinateToTrajectory "{coordinate: {x: 0.2, y: 0.5, z: 0.8}}"
```

7. To send the robot to home configuration by reversing the approach trajectory:
```bash
ros2 service call /return_home_trajectory std_srvs/srv/Trigger 
```

8. To set a voxel mask for tree and wire locations: Valid inputs are tree_pos 1-5, each shifting the tree location from left to right. If 0 is entered, it will remove the tree mask and maintain the wire mask. Any values outside of this range, it will default to no voxel mask.
```bash
ros2 service call /voxel_mask harvest_interfaces/srv/VoxelMask "{tree_pos: 1}"
```
