# Robotic Apple Harvesting Control

This repository contains code to detect and localize apples from RGB-D data and operate a UR5e manipulator. This can be done with both real hardware or simulated data.

The vision code has only been setup for running a Realsense d435i (real hardware) or a Microsoft Azure Kinect (simulated data).

There are currently two main control schemes: apple harvesting (real or simulated) and tree templating (tested only in simulation). The apple harvesting pipeline includes nodes for additional UR controllers and data recording, while the tree templating method runs a minimal set of nodes for control in simulation.

### To run apple locatization and path planning

#### Tree Templating:
*Currently only tested in simulation*

1. In the first terminal:
```bash
ros2 launch harvest_control arm_control_templating.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=true
```

2. In a second terminal run the vision:
```bash
ros2 launch harvest launch_vision_templating.launch.py
```

3. In a third terminal run the control script to execute actions from the running nodes:
```bash
ros2 run harvest orchard_templating.py
```

#### Apple Harvesting:
1. In the first terminal (real or sim):
```bash
ros2 launch harvest_control arm_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=true
```
OR
```bash
ros2 launch harvest_control arm_control.launch.py ur_type:=ur5e robot_ip:=169.254.177.230 launch_rviz:=true headless_mode:=true
```

2. In a second terminal run the vision:
(need to ensure palm camera connects to proper idx):
```bash
ros2 launch harvest launch_vision.launch.py
```

3. In a third terminal run the control script to execute actions from the running nodes:
```bash
ros2 run harvest start_harvest.py
```


### Details
- The default kinematics plug-in is `KDLKinematicsPlugin`
- Each of the three launch files have specific parameters that can be updated for the desired usecase. Each parameter having documentation within the associcated launch file.
- `start_harvest.py` is a client that was used for physical hardware trials. If different functionality is desired, a new client control script will need to be written.
- If running in simulation with presaved vision data:
    - Saved vision data files are stored in `harvest_vision/data/`, under the directory `prosser_a/`
    - Where the letter after prosser_ is considered the **vision_experiment** parameter when launching the vision package
    - If adding a new data directory, the `setup.py` file will need to be updated
    - Within this directory are three files needed for experiments: `color_raw.png`, `depth_to_color.png`, and `pointcloud.ply`
