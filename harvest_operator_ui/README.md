# Harvest Operator UI

Offline PySide6 control application for the `apple-harvest` ROS 2 pipeline.

## Included

- YAML profiles for autonomous and freedrive operation.
- Editable arm, gripper, vision, harvest, topic, and environment parameters.
- Configurable pose-listener source frame (`cart_base` by default; for example, `amiga__base`).
- Palm camera device is a free-form string, supporting both numeric indices and stable udev paths such as `/dev/harvest/palm_camera`.
- Sweep pick-controller selection with a configurable sweep angle, alongside the legacy pick patterns.
- RViz checkbox (`view_rviz`) enabled by default as a backup 3D visualizer.
- UR headless control and initial scaled trajectory controller activation enabled by default.
- Mode-aware launch selection: freedrive can optionally run visual servo after manual arm positioning; apple prediction and ID selection remain autonomous-only. The vision stack can also run independently for data collection.
- Supervised arm, gripper, camera, vision, and abort-capable harvest processes.
- Separate Status and Controls pages.
- Persistent Light mode toggle available from every page.
- Four live image viewers with stale-frame indication: mast/wrist RGB, mast/wrist depth, palm camera, and annotated apple predictions.
- Fin Ray pressure/ToF, filtered force, ODrive feedback, and gripper IMU plots without an extra plotting dependency. The IMU view shows linear acceleration and roll/pitch/yaw from `/microROS/imu1`.
- Full `/microROS/can_status` display including position, velocity, torque, current, setpoint, active error, and disarm reason.
- ROS node count, MoveIt Servo status, freedrive heartbeat, combined logs, and process state.
- Pipeline, harvest, arm, and gripper controls, including open/close, valve on/off, clear errors, home, and guarded freedrive entry/exit.
- The Controls page mirrors component process states and includes a large mast/wrist RGB operator view plus the annotated detected-apples view and an apple-ID target field.
- Current batch, apple, and the exact timestamp embedded in the latest bag filename are shown on Status and Controls. The Controls page appends bagged picks or blank-number discards to `pick_log.csv` in the active data directory; each row also stores the bag path and a separate operator-entry time.

## Run

From this directory:

```bash
./run.sh
```

`run.sh` sources ROS Jazzy and the `apple_harvest_ws` overlay, activates `apple_harvest_ws/.venv`, and puts the venv's site-packages on `PYTHONPATH` so ROS nodes launched by the UI (which use `#!/usr/bin/python3`) also see the venv packages. Paths can be overridden with `ROS_SETUP`, `APPLE_WS`, and `HARVEST_VENV`.

### Creating the virtual environment

The venv lives at the workspace root and uses `--system-site-packages` so `rclpy`, `cv_bridge`, and the apt numpy/scipy stay visible. numpy is pinned below 2 because the ROS Python bindings are built against numpy 1.x.

```bash
cd ~/apple_harvest_ws
python3 -m venv --system-site-packages .venv   # add --without-pip if python3-venv is not installed
touch .venv/COLCON_IGNORE
.venv/bin/python -m pip install --upgrade pip
# CPU-only torch; use the matching CUDA index on machines with an NVIDIA GPU.
.venv/bin/python -m pip install "numpy>=1.24,<2" torch torchvision \
    --index-url https://download.pytorch.org/whl/cpu --extra-index-url https://pypi.org/simple
.venv/bin/python -m pip install -r src/apple-harvest/harvest_operator_ui/requirements.txt
.venv/bin/python -m pip install "setuptools>=68,<80"   # colcon-core requires setuptools<80
```

ROS packages (`rclpy`, message types, `cv_bridge`, launch files, and DDS middleware) remain system/workspace dependencies and are not installed by pip.

## Recommended workflow

1. Open **Configuration**, select autonomous or freedrive, and adjust parameters.
2. Save the settings as a named profile and select **Apply configuration**.
3. On **Controls**, start the support stack. Verify the cameras, telemetry, ROS nodes, and process state on **Status**, and verify the arm in RViz.
4. Select **Start harvest**. Use **Continue / Enter** for prompts emitted by `start_harvest_abort.py`.
5. In autonomous mode with **Select each apple in Controls** enabled, wait for prediction, choose a zero-based ID from the annotated image and available-ID list, then press **Pick this apple next**. The harvest loop validates the ID and waits for **Continue / Enter** before moving. Every predicted ID remains selectable for another attempt, including after an abort. Press **Finish selected apples** when the batch is done.
6. With **Abort recovery: freedrive**, an aborted autonomous attempt leaves the arm in freedrive and returns to the apple-ID selector. Reposition the arm, enter any ID (including the one just aborted), and continue the autonomous pick. Aborting prediction instead lets you reposition the arm and repeat the scan and prediction.
7. **Enter freedrive** calls `/set_harvest_freedrive`. The harvest node rejects entry while a stage, action, or arm motion is active and returns to the trajectory controller before later motion.
8. Use **ABORT STAGE** for the cancelable abort path. Use **Stop all** or **Exit cleanly** to send SIGINT to every managed process group.

## Camera serial handling

The UI starts one `launch_vision.launch.py` process. Its `mast_serial` argument is forwarded internally to `realsense_topics.launch.py`, so the configured RealSense and all vision nodes belong to one managed launch process.

## Gripper interfaces

The UI matches the firmware in `suction-gripper/platformio_ws`: `/microROS/actuate_odrive`, `/microROS/toggle_valve`, `/microROS/home_odrive`, and `/microROS/clear_odrive_errors` are `std_srvs/SetBool` services. No Fin Ray submodule firmware is used or modified.

## Tests

```bash
python -m unittest discover -s tests -v
```

The application must be restarted if you change the Python/ROS environment used by the UI itself. Applying a profile rebuilds the environment used by newly launched child processes.
