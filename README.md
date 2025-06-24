# Robotic Apple Harvesting Control

This repository contains code to detect and localize apples from RGB-D data and operate a UR5e manipulator. This can be done with both real hardware or simulated data.

The vision code has only been setup for running a Realsense d435i (real hardware) or a Microsoft Azure Kinect (simulated data).

There are currently two main control schemes: apple harvesting (real or simulated) and tree templating (tested only in simulation). The apple harvesting pipeline includes nodes for additional UR controllers and data recording, while the tree templating method runs a minimal set of nodes for control in simulation.

### Apple Harvesting:
1. In the first terminal:

    **Simulated**
    ```bash
    ros2 launch harvest_control arm_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=true
    ```
    **Real Hardware**
    ```bash
    ros2 launch harvest_control arm_control.launch.py ur_type:=ur5e robot_ip:=169.254.177.230 launch_rviz:=true headless_mode:=true
    ```

2. Getting apple locations:

    A. If using predicting apple locations via YOLO, run the vision node in another script:
    (need to ensure palm camera connects to proper idx):
    ```bash
    ros2 launch harvest launch_vision.launch.py palm_camera_device_num:=<camera port idx>
    ```
    B. If getting manual apple locations, run position recording in another script (free-drive the robot to probe apple locations):
    ```bash
    ros2 run harvest_control get_manual_apple_locations.py --ros-args \
      -p output_directory:=/absolute/path/to/data
    ```

3. In a third terminal run the control script.  You can now tweak these parameters at launch with `--ros-args -p <name>:=<value>`:

    ```bash
    ros2 run harvest start_harvest.py --ros-args \
      -p pick_pattern:=force-heuristic \
      -p event_sensitivity:=0.43 \
      -p recording_startup_delay:=0.5 \
      -p base_data_dir:=/absolute/path/to/data \
      -p enable_recording:=true \
      -p enable_visual_servo:=true \
      -p enable_apple_prediction:=true \
      -p enable_pressure_servo:=true \
      -p enable_picking:=true
    ```

    **Defaults** (if you don’t pass `-p` flags):  
    ```yaml
    pick_pattern:            force-heuristic        # pick controller selection (force-heuristic, pull-twist, linear-pull)
    event_sensitivity:       0.43                   # event detection sensitivity (0.0–1.0)
    recording_startup_delay: 0.5                    # secs to wait after record start before action
    base_data_dir:           <workspace_root>/data  # where to store batch_<N> directories
    enable_recording:        true                   # enable/disable any rosbag recording stages
    enable_visual_servo:     true                   # enable/disable the visual-servo stage
    enable_apple_prediction: true                   # enable/disable live apple-prediction
    enable_pressure_servo:   true                   # enable/disable pressure servoing stage
    enable_picking:          true                   # enable/disable picking phase 
    ```

    - **base_data_dir**: by default this creates (if needed) a `data/` folder next to your `src/` and writes `batch_1/`, `batch_2/`, … under it.  
    - All stages (approach, visual‑servo, pressure‑servo, pick, release) will automatically record topics (if `enable_recording`) under `<base_data_dir>/<stage>_<batch>/<prefix>*.bag`.  
    - Use `pick_pattern` to switch between your three pick‑controller strategies without code changes.

---

With that, you can flexibly tune your harvest run from the command line, point recordings wherever you like, and turn subsystems on or off without editing any Python.

### Tree Templating:
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


### Details
- The default kinematics plug-in is `KDLKinematicsPlugin`
- Each of the three launch files have specific parameters that can be updated for the desired usecase. Each parameter having documentation within the associcated launch file.
- If running tree templating in simulation with presaved vision data:
    - Saved vision data files are stored in `harvest_vision/data/`, under the directory `prosser_a/`
    - Where the letter after prosser_ is considered the **vision_experiment** parameter when launching the vision package
    - If adding a new data directory, the `setup.py` file will need to be updated
    - Within this directory are three files needed for experiments: `color_raw.png`, `depth_to_color.png`, and `pointcloud.ply`
