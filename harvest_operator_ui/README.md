# Harvest Operator UI

Offline PySide6 control application for the `apple-harvest` ROS 2 pipeline.

## Included

- YAML profiles for autonomous and freedrive operation.
- Editable arm, gripper, vision, harvest, topic, and environment parameters.
- RViz checkbox (`view_rviz`) enabled by default as a backup 3D visualizer.
- UR headless control and initial scaled trajectory controller activation enabled by default.
- Mode-aware launch selection: freedrive omits visual-servo and apple-prediction stages while allowing the vision stack to run independently for data collection.
- Supervised arm, gripper, camera, vision, and abort-capable harvest processes.
- Separate Status and Controls pages.
- Four live image viewers with stale-frame indication: mast/wrist RGB, mast/wrist depth, palm camera, and annotated apple predictions.
- Fin Ray pressure/ToF, filtered force, ODrive feedback, and gripper IMU plots without an extra plotting dependency. The IMU view shows linear acceleration and roll/pitch/yaw from `/microROS/imu1`.
- Full `/microROS/can_status` display including position, velocity, torque, current, setpoint, active error, and disarm reason.
- ROS node count, MoveIt Servo status, freedrive heartbeat, combined logs, and process state.
- Pipeline, harvest, arm, and gripper controls, including open/close, valve on/off, clear errors, home, and guarded freedrive entry/exit.
- The Controls page mirrors component process states and includes a large mast/wrist RGB operator view plus the annotated detected-apples view for future target selection.

## Run

The configured virtual environment already contains PySide6. From this directory:

```bash
./run.sh
```

`run.sh` activates the Fin Ray virtual environment, sources ROS Humble, and then sources the local apple-harvest, apple-gripper, and Fin Ray overlays before starting the UI. Paths can be overridden with `ROS_SETUP`, `HARVEST_VENV`, `APPLE_GRIPPER_WS`, and `FINRAY_WS` environment variables.

If dependencies need to be installed on another machine, do so while internet access is available:

```bash
python -m pip install -r requirements.txt
```

ROS packages (`rclpy`, message types, `cv_bridge`, launch files, and DDS middleware) remain system/workspace dependencies and are not installed by pip.

## Recommended workflow

1. Open **Configuration**, select autonomous or freedrive, and adjust parameters.
2. Save the settings as a named profile and select **Apply configuration**.
3. On **Controls**, start the support stack. Verify the cameras, telemetry, ROS nodes, and process state on **Status**, and verify the arm in RViz.
4. Select **Start harvest**. Use **Continue / Enter** for prompts emitted by `start_harvest_abort.py`.
5. **Enter freedrive** calls `/set_harvest_freedrive`. The harvest node rejects entry while a stage, action, or arm motion is active and returns to the trajectory controller before later motion.
6. Use **ABORT STAGE** for the cancelable abort path. Use **Stop all** or **Exit cleanly** to send SIGINT to every managed process group.

## Camera serial handling

The UI starts one `launch_vision.launch.py` process. Its `mast_serial` argument is forwarded internally to `realsense_topics.launch.py`, so the configured RealSense and all vision nodes belong to one managed launch process.

## Gripper interfaces

The UI matches the firmware in `suction-gripper/platformio_ws`: `/microROS/actuate_odrive`, `/microROS/toggle_valve`, `/microROS/home_odrive`, and `/microROS/clear_odrive_errors` are `std_srvs/SetBool` services. No Fin Ray submodule firmware is used or modified.

## Tests

```bash
python -m unittest discover -s tests -v
```

The application must be restarted if you change the Python/ROS environment used by the UI itself. Applying a profile rebuilds the environment used by newly launched child processes.
