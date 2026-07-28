from __future__ import annotations

import copy
import ipaddress
import os
import shlex
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import yaml


DEFAULT_CONFIG: dict[str, Any] = {
    "schema_version": 1,
    "profile_name": "Field harvest",
    "mode": "autonomous",
    "components": {
        "arm": True,
        "gripper": True,
        "vision": True,
        "harvest": True,
    },
    "environment": {
        "setup_scripts": [
            "/opt/ros/humble/setup.bash",
            "/home/jn2/college/apple-harvest/install/local_setup.bash",
            "/home/jn2/college/Forked_Repos/apple_gripper/install/local_setup.bash",
            "/home/jn2/college/Fin_Ray_Gripper/install/local_setup.bash",
        ],
        "venv": "/home/jn2/college/Fin_Ray_Gripper/.venv",
        "pythonpath": "/home/jn2/college/Fin_Ray_Gripper/.venv/lib/python3.10/site-packages",
    },
    "arm": {
        "ur_type": "ur5e",
        "robot_ip": "169.254.177.230",
        "use_fake_hardware": False,
        "headless_mode": True,
        "activate_joint_controller": True,
        "view_rviz": True,
        "description_file": "cart_ur_gripper.urdf.xacro",
        "rviz_file": "cart_robot.rviz",
        "camera_mount": "wrist",
    },
    "gripper": {
        "gripper_type": "finray",
        "grasp_strategy": "pressure",
    },
    "vision": {
        "launch_camera": True,
        "palm_camera_device_num": 2,
        "camera_ns": "camera/gripper_camera",
        "prediction_model": "best_segmentation.pt",
        "vservo_model": "best_segmentation.pt",
        "target_frame": "cart_body",
        "mast_serial": "040322070611",
    },
    "harvest": {
        "event_sensitivity": 0.43,
        "recording_startup_delay": 0.5,
        "base_data_dir": "/home/jn2/college/data",
        "enable_recording": True,
        "enable_visual_servo": True,
        "enable_pressure_servo": True,
        "enable_picking": True,
        "enable_apple_prediction": True,
        "optimal_trajectory": True,
        "pick_pattern": "stiffness-seeking",
        "abort_on_decelerate": False,
        "abort_recovery": "freedrive",
    },
    "topics": {
        "mast_rgb": "/camera/gripper_camera/color/image_raw",
        "mast_depth": "/camera/gripper_camera/aligned_depth_to_color/image_raw",
        "palm_image": "/gripper/rgb_palm_camera/image_raw",
        "apple_prediction_image": "/apple_annotated",
        "pressure": "/microROS/sensor_data",
        "gripper_imu": "/microROS/imu1",
        "can_status": "/microROS/can_status",
        "wrench": "/filtered_wrench",
        "servo_status": "/servo_node/status",
        "freedrive_status": "/freedrive_mode_controller/enable_freedrive_mode",
    },
    "services": {
        "gripper_actuate": "/microROS/actuate_odrive",
        "valve": "/microROS/toggle_valve",
        "clear_odrive_errors": "/microROS/clear_odrive_errors",
        "home_gripper": "/microROS/home_odrive",
        "harvest_freedrive": "/set_harvest_freedrive",
        "abort_harvest": "/abort_harvest",
        "move_arm_home": "/move_arm_to_home",
        "release_apple": "/release_apple",
    },
}


@dataclass(frozen=True)
class ProcessSpec:
    key: str
    label: str
    argv: tuple[str, ...]

    @property
    def command_text(self) -> str:
        import shlex

        return shlex.join(self.argv)


def _deep_merge(base: dict[str, Any], update: dict[str, Any]) -> dict[str, Any]:
    result = copy.deepcopy(base)
    for key, value in update.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def normalize_config(raw: dict[str, Any] | None) -> dict[str, Any]:
    config = _deep_merge(DEFAULT_CONFIG, raw or {})
    config["mode"] = str(config.get("mode", "autonomous")).lower()
    if config["mode"] not in {"autonomous", "freedrive"}:
        config["mode"] = "autonomous"

    if config["mode"] == "freedrive":
        # The manual harvest flow does not run vision-driven stages, but the
        # vision stack may still be launched independently for data capture.
        config["harvest"]["enable_visual_servo"] = False
        config["harvest"]["enable_apple_prediction"] = False
    return config


def load_config(path: str | Path) -> dict[str, Any]:
    path = Path(path)
    if not path.exists():
        return normalize_config(None)
    with path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"Configuration root must be a mapping: {path}")
    return normalize_config(raw)


def save_config(config: dict[str, Any], path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".tmp")
    with temp.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(normalize_config(config), stream, sort_keys=False)
    temp.replace(path)


def validate_config(config: dict[str, Any]) -> list[str]:
    config = normalize_config(config)
    errors: list[str] = []
    try:
        ipaddress.ip_address(config["arm"]["robot_ip"])
    except ValueError:
        errors.append("Arm robot IP is not a valid IP address.")
    if not 0.0 <= float(config["harvest"]["event_sensitivity"]) <= 1.0:
        errors.append("Event sensitivity must be between 0 and 1.")
    if float(config["harvest"]["recording_startup_delay"]) < 0:
        errors.append("Recording startup delay cannot be negative.")
    if config["harvest"]["pick_pattern"] not in {
        "stiffness-seeking",
        "force-heuristic",
        "pull-twist",
        "linear-pull",
    }:
        errors.append("Unknown pick pattern.")
    if config["harvest"]["abort_recovery"] not in {"freedrive", "home"}:
        errors.append("Abort recovery must be 'freedrive' or 'home'.")
    for script in config["environment"]["setup_scripts"]:
        if script and not Path(script).exists():
            errors.append(f"Setup script does not exist: {script}")
    return errors


def build_child_environment(raw_config: dict[str, Any]) -> dict[str, str]:
    """Load the configured ROS overlays into an environment for child commands."""
    config = normalize_config(raw_config)
    environment_config = config["environment"]
    statements: list[str] = []
    venv = str(environment_config.get("venv", "")).strip()
    if venv:
        statements.append(f"source {shlex.quote(str(Path(venv) / 'bin' / 'activate'))}")
    for setup_script in environment_config.get("setup_scripts", []):
        if setup_script:
            statements.append(f"source {shlex.quote(str(setup_script))}")
    pythonpath = str(environment_config.get("pythonpath", "")).strip()
    if pythonpath:
        statements.append(f"export PYTHONPATH={shlex.quote(pythonpath)}${{PYTHONPATH:+:$PYTHONPATH}}")
    statements.append("env -0")
    completed = subprocess.run(
        ["/bin/bash", "-c", " && ".join(statements)],
        env=dict(os.environ),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )
    result: dict[str, str] = {}
    for item in completed.stdout.split(b"\0"):
        if b"=" in item:
            key, value = item.split(b"=", 1)
            result[key.decode(errors="replace")] = value.decode(errors="replace")
    return result


def _ros_bool(value: Any) -> str:
    return "true" if bool(value) else "false"


def _assignments(values: Iterable[tuple[str, Any]]) -> tuple[str, ...]:
    return tuple(f"{name}:={value}" for name, value in values)


def build_process_specs(raw_config: dict[str, Any]) -> list[ProcessSpec]:
    config = normalize_config(raw_config)
    specs: list[ProcessSpec] = []
    components = config["components"]

    if components["arm"]:
        arm = config["arm"]
        argv = ("ros2", "launch", "harvest_control", "arm_control.launch.py") + _assignments(
            (
                ("ur_type", arm["ur_type"]),
                ("robot_ip", arm["robot_ip"]),
                ("use_fake_hardware", _ros_bool(arm["use_fake_hardware"])),
                ("headless_mode", _ros_bool(arm["headless_mode"])),
                ("activate_joint_controller", _ros_bool(arm["activate_joint_controller"])),
                ("view_rviz", _ros_bool(arm["view_rviz"])),
                ("description_file", arm["description_file"]),
                ("rviz_file", arm["rviz_file"]),
                ("camera_mount", arm["camera_mount"]),
            )
        )
        specs.append(ProcessSpec("arm", "Arm control + RViz", argv))

    if components["gripper"]:
        gripper = config["gripper"]
        argv = ("ros2", "launch", "harvest", "launch_gripper.launch.py") + _assignments(
            (
                ("gripper_type", gripper["gripper_type"]),
                ("grasp_strategy", gripper["grasp_strategy"]),
            )
        )
        specs.append(ProcessSpec("gripper", "Gripper", argv))

    if components["vision"]:
        vision = config["vision"]
        vision_argv = ("ros2", "launch", "harvest", "launch_vision.launch.py") + _assignments(
            (
                ("palm_camera_device_num", vision["palm_camera_device_num"]),
                ("camera_ns", vision["camera_ns"]),
                ("prediction_model", vision["prediction_model"]),
                ("vservo_model", vision["vservo_model"]),
                ("target_frame", vision["target_frame"]),
                ("mast_serial", vision["mast_serial"]),
                ("launch_realsense", _ros_bool(vision["launch_camera"])),
            )
        )
        specs.append(ProcessSpec("vision", "Vision", vision_argv))

    if components["harvest"]:
        harvest = config["harvest"]
        argv = ("ros2", "run", "harvest", "start_harvest_abort.py", "--ros-args") + tuple(
            item
            for name, value in (
                ("event_sensitivity", harvest["event_sensitivity"]),
                ("recording_startup_delay", harvest["recording_startup_delay"]),
                ("base_data_dir", harvest["base_data_dir"]),
                ("enable_recording", _ros_bool(harvest["enable_recording"])),
                ("enable_visual_servo", _ros_bool(harvest["enable_visual_servo"])),
                ("enable_pressure_servo", _ros_bool(harvest["enable_pressure_servo"])),
                ("enable_picking", _ros_bool(harvest["enable_picking"])),
                ("enable_apple_prediction", _ros_bool(harvest["enable_apple_prediction"])),
                ("optimal_trajectory", _ros_bool(harvest["optimal_trajectory"])),
                ("pick_pattern", harvest["pick_pattern"]),
                ("abort_on_decelerate", _ros_bool(harvest["abort_on_decelerate"])),
                ("abort_recovery", harvest["abort_recovery"]),
                ("freedrive", _ros_bool(config["mode"] == "freedrive")),
            )
            for item in ("-p", f"{name}:={value}")
        )
        specs.append(ProcessSpec("harvest", "Harvest pipeline", argv))

    return specs
