from __future__ import annotations

import copy
import re
import subprocess
import time
from pathlib import Path
from typing import Any

from PySide6.QtCore import QSettings, QSignalBlocker, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QCloseEvent, QFont
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QSplitter,
    QTabWidget,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

from .config import (
    build_child_environment,
    build_process_specs,
    load_config,
    normalize_config,
    save_config,
    validate_config,
)
from .processes import ProcessSupervisor
from .ros_bridge import RosBridge
from .widgets import ImagePanel, LinePlot


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = ROOT / "config" / "default.yaml"
LAST_PROFILE = ROOT / "config" / "last_used.yaml"
ANSI_ESCAPE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")


def _get_path(config: dict[str, Any], path: str) -> Any:
    value: Any = config
    for part in path.split("."):
        value = value[part]
    return value


def _set_path(config: dict[str, Any], path: str, value: Any) -> None:
    target = config
    parts = path.split(".")
    for part in parts[:-1]:
        target = target[part]
    target[parts[-1]] = value


class ConfigurationPage(QWidget):
    apply_requested = Signal(object)

    def __init__(self, config: dict[str, Any], parent: QWidget | None = None):
        super().__init__(parent)
        self._config = normalize_config(config)
        self._profile_path = LAST_PROFILE if LAST_PROFILE.exists() else DEFAULT_PROFILE
        self._bindings: dict[str, QWidget] = {}

        outer = QVBoxLayout(self)
        toolbar = QHBoxLayout()
        self.profile_label = QLabel()
        self.profile_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        load_button = QPushButton("Load profile…")
        save_button = QPushButton("Save profile as…")
        apply_button = QPushButton("Apply configuration")
        apply_button.setProperty("primary", True)
        preview_button = QPushButton("Refresh command preview")
        load_button.clicked.connect(self._load_profile)
        save_button.clicked.connect(self._save_profile)
        apply_button.clicked.connect(self._apply)
        preview_button.clicked.connect(self._update_preview)
        toolbar.addWidget(self.profile_label, 1)
        toolbar.addWidget(load_button)
        toolbar.addWidget(save_button)
        toolbar.addWidget(preview_button)
        toolbar.addWidget(apply_button)
        outer.addLayout(toolbar)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        form_container = QWidget()
        form_layout = QVBoxLayout(form_container)
        form_layout.addWidget(self._build_pipeline_group())
        form_layout.addWidget(self._build_arm_group())
        form_layout.addWidget(self._build_gripper_group())
        form_layout.addWidget(self._build_vision_group())
        form_layout.addWidget(self._build_harvest_group())
        form_layout.addWidget(self._build_topics_group())
        form_layout.addWidget(self._build_services_group())
        form_layout.addWidget(self._build_environment_group())
        form_layout.addStretch(1)
        scroll.setWidget(form_container)

        preview_container = QWidget()
        preview_layout = QVBoxLayout(preview_container)
        preview_layout.addWidget(QLabel("Commands generated from this profile"))
        self.preview = QPlainTextEdit()
        self.preview.setReadOnly(True)
        self.preview.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        preview_layout.addWidget(self.preview, 1)
        note = QLabel(
            "Vision is one component and uses launch_vision.launch.py. Its camera, model, frame, and mast serial "
            "arguments are shown above and passed through the single launch command."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color:#8d9aaa")
        preview_layout.addWidget(note)

        splitter.addWidget(scroll)
        splitter.addWidget(preview_container)
        splitter.setSizes([620, 580])
        outer.addWidget(splitter, 1)
        self.set_config(self._config, self._profile_path)

    def _group(self, title: str) -> tuple[QGroupBox, QFormLayout]:
        group = QGroupBox(title)
        layout = QFormLayout(group)
        layout.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
        return group, layout

    def _bind(self, path: str, widget: QWidget) -> QWidget:
        self._bindings[path] = widget
        return widget

    def _line(self, path: str) -> QLineEdit:
        return self._bind(path, QLineEdit())  # type: ignore[return-value]

    def _check(self, path: str, text: str = "Enabled") -> QCheckBox:
        return self._bind(path, QCheckBox(text))  # type: ignore[return-value]

    def _combo(self, path: str, values: list[str]) -> QComboBox:
        combo = QComboBox()
        combo.addItems(values)
        return self._bind(path, combo)  # type: ignore[return-value]

    def _double(self, path: str, minimum: float, maximum: float, decimals: int = 2) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(decimals)
        spin.setSingleStep(0.01)
        return self._bind(path, spin)  # type: ignore[return-value]

    def _integer(self, path: str, minimum: int, maximum: int) -> QSpinBox:
        spin = QSpinBox()
        spin.setRange(minimum, maximum)
        return self._bind(path, spin)  # type: ignore[return-value]

    def _build_pipeline_group(self) -> QGroupBox:
        group, form = self._group("Pipeline")
        form.addRow("Profile name", self._line("profile_name"))
        self.mode = self._combo("mode", ["autonomous", "freedrive"])
        self.mode.currentTextChanged.connect(self._mode_changed)
        form.addRow("Operating mode", self.mode)
        form.addRow("Arm control", self._check("components.arm"))
        form.addRow("Gripper", self._check("components.gripper"))
        self.vision_component = self._check("components.vision", "Launch vision stack")
        form.addRow("Vision", self.vision_component)
        form.addRow("Harvest orchestrator", self._check("components.harvest"))
        self.mode_note = QLabel()
        self.mode_note.setWordWrap(True)
        form.addRow("Effective mode", self.mode_note)
        return group

    def _build_arm_group(self) -> QGroupBox:
        group, form = self._group("Arm control")
        form.addRow("UR type", self._combo("arm.ur_type", ["ur5e", "ur10e", "ur3e", "ur16e", "ur20"]))
        form.addRow("Robot IP", self._line("arm.robot_ip"))
        form.addRow("Fake hardware", self._check("arm.use_fake_hardware"))
        form.addRow("Headless control", self._check("arm.headless_mode", "Start UR control without pendant program"))
        form.addRow("Initial controller", self._check("arm.activate_joint_controller", "Activate scaled trajectory controller"))
        self.rviz_checkbox = self._check("arm.view_rviz", "Launch RViz backup visualizer")
        form.addRow("RViz", self.rviz_checkbox)
        form.addRow("Description xacro", self._line("arm.description_file"))
        form.addRow("RViz config", self._line("arm.rviz_file"))
        form.addRow("Camera mount", self._combo("arm.camera_mount", ["wrist", "mast"]))
        form.addRow("Pose-listener source frame", self._line("arm.source_frame"))
        return group

    def _build_gripper_group(self) -> QGroupBox:
        group, form = self._group("Gripper")
        form.addRow("Gripper type", self._combo("gripper.gripper_type", ["finray", "old"]))
        form.addRow("Grasp strategy", self._combo("gripper.grasp_strategy", ["pressure", "time"]))
        return group

    def _build_vision_group(self) -> QGroupBox:
        group, form = self._group("Vision")
        self.vision_controls: list[QWidget] = []
        controls = [
            ("Launch RealSense", self._check("vision.launch_camera")),
            ("Palm camera device", self._integer("vision.palm_camera_device_num", 0, 20)),
            ("Camera namespace", self._line("vision.camera_ns")),
            ("Prediction model", self._line("vision.prediction_model")),
            ("Visual-servo model", self._line("vision.vservo_model")),
            ("Target frame", self._line("vision.target_frame")),
            ("Mast serial", self._line("vision.mast_serial")),
        ]
        for label, widget in controls:
            form.addRow(label, widget)
            self.vision_controls.append(widget)
        return group

    def _build_harvest_group(self) -> QGroupBox:
        group, form = self._group("Harvest run")
        form.addRow("Event sensitivity", self._double("harvest.event_sensitivity", 0.0, 1.0, 3))
        form.addRow("Recording startup delay (s)", self._double("harvest.recording_startup_delay", 0.0, 30.0, 2))
        form.addRow("Base data directory", self._line("harvest.base_data_dir"))
        form.addRow("Recording", self._check("harvest.enable_recording"))
        self.visual_servo = self._check("harvest.enable_visual_servo")
        self.apple_prediction = self._check("harvest.enable_apple_prediction")
        self.visual_servo.toggled.connect(self._vision_requirement_changed)
        self.apple_prediction.toggled.connect(self._vision_requirement_changed)
        form.addRow("Visual servo", self.visual_servo)
        form.addRow("Pressure servo", self._check("harvest.enable_pressure_servo"))
        form.addRow("Picking", self._check("harvest.enable_picking"))
        form.addRow("Apple prediction", self.apple_prediction)
        form.addRow("Optimal trajectory", self._check("harvest.optimal_trajectory"))
        form.addRow(
            "Pick pattern",
            self._combo(
                "harvest.pick_pattern",
                ["stiffness-seeking", "force-heuristic", "pull-twist", "linear-pull", "sweep"],
            ),
        )
        form.addRow("Sweep angle (deg)", self._double("harvest.sweep_theta_deg", -360.0, 360.0, 1))
        form.addRow("Abort on deceleration", self._check("harvest.abort_on_decelerate"))
        form.addRow("Abort recovery", self._combo("harvest.abort_recovery", ["freedrive", "home"]))
        return group

    def _build_topics_group(self) -> QGroupBox:
        group, form = self._group("Dashboard topics")
        form.addRow("Mast/wrist RGB", self._line("topics.mast_rgb"))
        form.addRow("Mast/wrist depth", self._line("topics.mast_depth"))
        form.addRow("Palm camera", self._line("topics.palm_image"))
        form.addRow("Apple detections", self._line("topics.apple_prediction_image"))
        form.addRow("Fin Ray sensor data", self._line("topics.pressure"))
        form.addRow("Gripper IMU", self._line("topics.gripper_imu"))
        form.addRow("ODrive CAN status", self._line("topics.can_status"))
        form.addRow("Filtered wrench", self._line("topics.wrench"))
        form.addRow("MoveIt Servo status", self._line("topics.servo_status"))
        form.addRow("Freedrive heartbeat", self._line("topics.freedrive_status"))
        return group

    def _build_services_group(self) -> QGroupBox:
        group, form = self._group("Control services")
        form.addRow("Open/close gripper", self._line("services.gripper_actuate"))
        form.addRow("Vacuum valve", self._line("services.valve"))
        form.addRow("Clear ODrive errors", self._line("services.clear_odrive_errors"))
        form.addRow("Home gripper", self._line("services.home_gripper"))
        form.addRow("Harvest freedrive", self._line("services.harvest_freedrive"))
        form.addRow("Abort harvest", self._line("services.abort_harvest"))
        form.addRow("Move arm home", self._line("services.move_arm_home"))
        form.addRow("Release apple", self._line("services.release_apple"))
        return group

    def _build_environment_group(self) -> QGroupBox:
        group, form = self._group("ROS environment")
        form.addRow("Virtual environment", self._line("environment.venv"))
        form.addRow("Python path", self._line("environment.pythonpath"))
        setups = QPlainTextEdit()
        setups.setPlaceholderText("One setup.bash path per line")
        setups.setMaximumHeight(100)
        self._bind("environment.setup_scripts", setups)
        form.addRow("Setup scripts", setups)
        return group

    def set_config(self, config: dict[str, Any], path: Path | None = None) -> None:
        self._config = normalize_config(config)
        if path:
            self._profile_path = path
        self.profile_label.setText(f"Profile: {self._profile_path}")
        for field_path, widget in self._bindings.items():
            value = _get_path(self._config, field_path)
            blocker = QSignalBlocker(widget)
            if isinstance(widget, QLineEdit):
                widget.setText(str(value))
            elif isinstance(widget, QCheckBox):
                widget.setChecked(bool(value))
            elif isinstance(widget, QComboBox):
                index = widget.findText(str(value))
                widget.setCurrentIndex(max(0, index))
            elif isinstance(widget, QDoubleSpinBox):
                widget.setValue(float(value))
            elif isinstance(widget, QSpinBox):
                widget.setValue(int(value))
            elif isinstance(widget, QPlainTextEdit):
                widget.setPlainText("\n".join(str(item) for item in value))
            del blocker
        self._mode_changed(self.mode.currentText())
        self._update_preview()

    def config(self) -> dict[str, Any]:
        config = copy.deepcopy(self._config)
        for field_path, widget in self._bindings.items():
            if isinstance(widget, QLineEdit):
                value: Any = widget.text().strip()
            elif isinstance(widget, QCheckBox):
                value = widget.isChecked()
            elif isinstance(widget, QComboBox):
                value = widget.currentText()
            elif isinstance(widget, QDoubleSpinBox):
                value = widget.value()
            elif isinstance(widget, QSpinBox):
                value = widget.value()
            elif isinstance(widget, QPlainTextEdit):
                value = [line.strip() for line in widget.toPlainText().splitlines() if line.strip()]
            else:
                continue
            _set_path(config, field_path, value)
        return normalize_config(config)

    def _mode_changed(self, mode: str) -> None:
        freedrive = mode == "freedrive"
        if freedrive:
            self.visual_servo.setChecked(False)
            self.apple_prediction.setChecked(False)
        self.vision_component.setEnabled(True)
        self.visual_servo.setEnabled(not freedrive)
        self.apple_prediction.setEnabled(not freedrive)
        for widget in self.vision_controls:
            widget.setEnabled(True)
        self.mode_note.setText(
            "Freedrive: visual-servo and apple-prediction stages are omitted; the vision stack remains optional for data collection."
            if freedrive
            else "Autonomous: vision launches when enabled; RViz is controlled independently in Arm control."
        )
        self.mode_note.setStyleSheet("color:#f2cc60" if freedrive else "color:#58a6ff")

    def _vision_requirement_changed(self) -> None:
        if self.mode.currentText() == "autonomous" and (self.visual_servo.isChecked() or self.apple_prediction.isChecked()):
            self.vision_component.setChecked(True)

    def _load_profile(self) -> None:
        filename, _ = QFileDialog.getOpenFileName(self, "Load harvest profile", str(ROOT / "config"), "YAML (*.yaml *.yml)")
        if not filename:
            return
        try:
            self.set_config(load_config(filename), Path(filename))
        except (OSError, ValueError) as exc:
            QMessageBox.critical(self, "Profile error", str(exc))

    def _save_profile(self) -> None:
        filename, _ = QFileDialog.getSaveFileName(self, "Save harvest profile", str(ROOT / "config"), "YAML (*.yaml)")
        if not filename:
            return
        if not filename.endswith((".yaml", ".yml")):
            filename += ".yaml"
        try:
            save_config(self.config(), filename)
            self._profile_path = Path(filename)
            self.profile_label.setText(f"Profile: {self._profile_path}")
        except OSError as exc:
            QMessageBox.critical(self, "Save failed", str(exc))

    def _apply(self) -> None:
        config = self.config()
        errors = validate_config(config)
        if errors:
            QMessageBox.warning(self, "Invalid configuration", "\n".join(f"• {error}" for error in errors))
            return
        try:
            save_config(config, LAST_PROFILE)
        except OSError as exc:
            QMessageBox.warning(self, "Profile not persisted", str(exc))
        self._config = config
        self._profile_path = LAST_PROFILE
        self.profile_label.setText(f"Profile: {self._profile_path}")
        self._update_preview()
        self.apply_requested.emit(config)

    def _update_preview(self) -> None:
        try:
            specs = build_process_specs(self.config())
            self.preview.setPlainText("\n\n".join(f"# {spec.label}\n{spec.command_text}" for spec in specs))
        except Exception as exc:
            self.preview.setPlainText(f"Cannot generate commands: {exc}")


class MainWindow(QMainWindow):
    SERVO_CODES = {
        -1: "Invalid",
        0: "No warning",
        1: "Approaching singularity",
        2: "HALT: singularity",
        3: "Approaching collision",
        4: "HALT: collision",
        5: "HALT: joint bound",
        6: "Leaving singularity",
    }

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Apple Harvest Operator")
        self.resize(1500, 920)
        self.setMinimumSize(1100, 720)
        self._closing = False
        self._shutdown_started = 0.0
        self._process_rows: dict[str, list[QTreeWidgetItem]] = {}
        self._field_control_state: dict[str, tuple[str, str | None, str | None]] = {}

        initial_path = LAST_PROFILE if LAST_PROFILE.exists() else DEFAULT_PROFILE
        try:
            self.config = load_config(initial_path)
        except (OSError, ValueError):
            self.config = normalize_config(None)

        self.supervisor = ProcessSupervisor(self)
        self.supervisor.output.connect(self._process_output)
        self.supervisor.state_changed.connect(self._process_state)
        self.ros = RosBridge(self.config["topics"], self)

        self.tabs = QTabWidget()
        self.configuration_page = ConfigurationPage(self.config)
        self.configuration_page.apply_requested.connect(self._apply_config)
        self.tabs.addTab(self.configuration_page, "Configuration")
        self.tabs.addTab(self._build_status_page(), "Status")
        self.tabs.addTab(self._build_controls_page(), "Controls")
        self.ui_settings = QSettings("apple-harvest", "harvest-operator-ui")
        self.light_mode_toggle = QPushButton("Light mode")
        self.light_mode_toggle.setObjectName("themeToggle")
        self.light_mode_toggle.setCheckable(True)
        light_mode = self.ui_settings.value("light_mode", False, type=bool)
        self.light_mode_toggle.setChecked(light_mode)
        self.light_mode_toggle.toggled.connect(self._set_light_mode)
        self.tabs.setCornerWidget(self.light_mode_toggle, Qt.Corner.TopRightCorner)
        self.setCentralWidget(self.tabs)
        self._set_light_mode(light_mode, persist=False)
        self._connect_ros()
        self._apply_config(self.config, announce=False)
        self.ros.start()

    def _build_status_page(self) -> QWidget:
        page = QWidget()
        outer = QVBoxLayout(page)

        outer.addLayout(self._create_field_control_header(control_page=False))

        self.process_tree = self._create_process_tree(185)
        outer.addWidget(self.process_tree)

        dashboard = QTabWidget()
        camera_page = QWidget()
        camera_layout = QGridLayout(camera_page)
        camera_layout.setContentsMargins(4, 4, 4, 4)
        self.image_panels: dict[str, list[ImagePanel]] = {}
        camera_definitions = (
            ("mast_rgb", "Mast / wrist RGB"),
            ("mast_depth", "Mast / wrist depth"),
            ("palm_image", "Palm camera"),
            ("apple_prediction_image", "Detected apples"),
        )
        for index, (key, title_text) in enumerate(camera_definitions):
            container = QGroupBox(title_text)
            layout = QVBoxLayout(container)
            panel = ImagePanel()
            panel.label.setMinimumSize(320, 180)
            layout.addWidget(panel)
            self.image_panels.setdefault(key, []).append(panel)
            camera_layout.addWidget(container, index // 2, index % 2)
        dashboard.addTab(camera_page, "Cameras")

        telemetry_page = QWidget()
        telemetry_layout = QGridLayout(telemetry_page)
        self.pressure_plot = LinePlot("Fin Ray pressure / ToF", ["P1", "P2", "P3", "ToF"])
        self.force_plot = LinePlot("Filtered force", ["Fx", "Fy", "Fz"])
        self.odrive_plot = LinePlot("ODrive feedback", ["velocity", "torque", "current", "setpoint"])
        self.imu_acceleration_plot = LinePlot("Gripper IMU — linear acceleration", ["Ax", "Ay", "Az"])
        self.imu_orientation_plot = LinePlot("Gripper IMU — orientation (degrees)", ["Roll", "Pitch", "Yaw"])
        telemetry_layout.addWidget(self.pressure_plot, 0, 0)
        telemetry_layout.addWidget(self.force_plot, 0, 1)
        telemetry_layout.addWidget(self.odrive_plot, 1, 0)
        telemetry_layout.addWidget(self.imu_acceleration_plot, 1, 1)
        telemetry_layout.addWidget(self.imu_orientation_plot, 2, 0)

        can_group = QGroupBox("ODrive CAN status")
        can_form = QFormLayout(can_group)
        self.can_status_labels: dict[str, QLabel] = {}
        for key, label_text in (
            ("position", "Position"),
            ("velocity", "Velocity"),
            ("torque", "Torque"),
            ("current", "Current"),
            ("torque_setpoint", "Torque setpoint"),
            ("active_error", "Active error"),
            ("disarm_reason", "Disarm reason"),
        ):
            value_label = QLabel("No data")
            value_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            can_form.addRow(label_text, value_label)
            self.can_status_labels[key] = value_label
        telemetry_layout.addWidget(can_group, 2, 1)
        dashboard.addTab(telemetry_page, "Telemetry")

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(5000)
        self.log.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        dashboard.addTab(self.log, "Process log")
        outer.addWidget(dashboard, 1)
        return page

    def _build_controls_page(self) -> QWidget:
        page = QWidget()
        outer = QVBoxLayout(page)

        outer.addLayout(self._create_field_control_header(control_page=True))

        self.control_process_tree = self._create_process_tree(155)
        outer.addWidget(self.control_process_tree)

        center = QSplitter(Qt.Orientation.Horizontal)
        visual_container = QWidget()
        visual_layout = QVBoxLayout(visual_container)
        visual_layout.setContentsMargins(0, 0, 0, 0)

        mast_group = QGroupBox("Mast / wrist RGB — operator view")
        mast_layout = QVBoxLayout(mast_group)
        mast_panel = ImagePanel()
        mast_panel.label.setMinimumSize(480, 270)
        mast_layout.addWidget(mast_panel)
        self.image_panels.setdefault("mast_rgb", []).append(mast_panel)
        visual_layout.addWidget(mast_group, 2)

        apples_group = QGroupBox("Detected apples — selection view")
        apples_layout = QVBoxLayout(apples_group)
        apples_panel = ImagePanel()
        apples_panel.label.setMinimumSize(320, 180)
        apples_layout.addWidget(apples_panel)
        self.image_panels.setdefault("apple_prediction_image", []).append(apples_panel)
        visual_layout.addWidget(apples_group, 1)
        center.addWidget(visual_container)

        control_container = QWidget()
        control_layout = QVBoxLayout(control_container)
        control_layout.setContentsMargins(4, 0, 0, 0)

        self.safety_note = QLabel(
            "Freedrive requests are handled by the harvest orchestrator and are rejected while an arm motion or harvest stage is active."
        )
        self.safety_note.setWordWrap(True)
        control_layout.addWidget(self.safety_note)

        system_group, system_layout = self._button_group("Pipeline")
        self._add_control_button(system_layout, "Start support stack", self._start_support)
        self._add_control_button(system_layout, "Start harvest", lambda: self.supervisor.start("harvest"))
        self._add_control_button(system_layout, "Start all", lambda: self.supervisor.start_sequence(self.supervisor.keys(), 1500), primary=True)
        self._add_control_button(system_layout, "Stop all", self.supervisor.stop_all, danger=True)
        self._add_control_button(system_layout, "Exit cleanly", self.close)
        control_layout.addWidget(system_group)

        harvest_group, harvest_layout = self._button_group("Harvest / arm")
        self._add_control_button(harvest_layout, "Continue / Enter", lambda: self.supervisor.write("harvest", "\n"), primary=True)
        self._add_control_button(harvest_layout, "ABORT STAGE", lambda: self._call_trigger_service("abort_harvest"), danger=True)
        self._add_control_button(harvest_layout, "Move arm home", lambda: self._call_trigger_service("move_arm_home"))
        self._add_control_button(harvest_layout, "Enter freedrive", lambda: self._call_bool_service("harvest_freedrive", True))
        self._add_control_button(harvest_layout, "Exit freedrive", lambda: self._call_bool_service("harvest_freedrive", False))
        self._add_control_button(harvest_layout, "Release apple", lambda: self._call_trigger_service("release_apple"))
        control_layout.addWidget(harvest_group)

        gripper_group, gripper_layout = self._button_group("Gripper")
        self._add_control_button(gripper_layout, "Open gripper", lambda: self._call_bool_service("gripper_actuate", False))
        self._add_control_button(gripper_layout, "Close gripper", lambda: self._call_bool_service("gripper_actuate", True), primary=True)
        self._add_control_button(gripper_layout, "Valve on", lambda: self._call_bool_service("valve", True))
        self._add_control_button(gripper_layout, "Valve off", lambda: self._call_bool_service("valve", False))
        self._add_control_button(gripper_layout, "Clear ODrive errors", lambda: self._call_bool_service("clear_odrive_errors", True))
        self._add_control_button(gripper_layout, "Home gripper", lambda: self._call_bool_service("home_gripper", True))
        control_layout.addWidget(gripper_group)

        control_layout.addWidget(QLabel("Control request log"))
        self.control_log = QPlainTextEdit()
        self.control_log.setReadOnly(True)
        self.control_log.setMaximumBlockCount(1000)
        self.control_log.setMaximumHeight(135)
        control_layout.addWidget(self.control_log)
        center.addWidget(control_container)
        center.setSizes([850, 600])
        outer.addWidget(center, 1)
        return page

    def _create_field_control_header(self, control_page: bool) -> QHBoxLayout:
        prefix = "control_" if control_page else ""
        header = QHBoxLayout()
        title = QLabel("FIELD CONTROL")
        title.setFont(QFont("Sans Serif", 14, QFont.Weight.Bold))
        labels = (
            ("ros_status", "ROS starting…", 0),
            ("node_status", "0 nodes", 0),
            ("stage_status", "Stage: idle", 1),
            ("servo_status", "Servo: no data", 0),
            ("freedrive_status", "Freedrive: no data", 0),
        )
        header.addWidget(title)
        header.addSpacing(20)
        for name, initial_text, stretch in labels:
            label = QLabel(initial_text)
            setattr(self, f"{prefix}{name}", label)
            header.addWidget(label, stretch)
        return header

    def _set_field_control_label(
        self,
        name: str,
        text: str,
        *,
        tone: str | None = None,
        tooltip: str | None = None,
    ) -> None:
        self._field_control_state[name] = (text, tone, tooltip)
        style = f"color:{self._semantic_color(tone)}; font-weight:600" if tone else None
        for prefix in ("", "control_"):
            label = getattr(self, f"{prefix}{name}")
            label.setText(text)
            if style is not None:
                label.setStyleSheet(style)
            if tooltip is not None:
                label.setToolTip(tooltip)

    @staticmethod
    def _create_process_tree(maximum_height: int) -> QTreeWidget:
        tree = QTreeWidget()
        tree.setHeaderLabels(["Component", "State", "Details / command"])
        tree.setRootIsDecorated(False)
        tree.setMaximumHeight(maximum_height)
        tree.header().setStretchLastSection(True)
        tree.setColumnWidth(0, 160)
        tree.setColumnWidth(1, 90)
        return tree

    @staticmethod
    def _button_group(title: str) -> tuple[QGroupBox, QGridLayout]:
        group = QGroupBox(title)
        return group, QGridLayout(group)

    @staticmethod
    def _add_control_button(layout: QGridLayout, text: str, callback, *, primary: bool = False, danger: bool = False) -> None:
        button = QPushButton(text)
        button.setMinimumHeight(46)
        if primary:
            button.setProperty("primary", True)
        if danger:
            button.setProperty("danger", True)
        button.clicked.connect(callback)
        index = layout.count()
        layout.addWidget(button, index // 3, index % 3)

    def _set_light_mode(self, enabled: bool, *, persist: bool = True) -> None:
        self._light_mode = bool(enabled)
        if persist:
            self.ui_settings.setValue("light_mode", self._light_mode)
        self.light_mode_toggle.setText("Light mode: on" if self._light_mode else "Light mode: off")
        self._apply_styles()
        self.safety_note.setStyleSheet(
            f"color:{self._semantic_color('warning')}; font-weight:600"
        )
        for panel in self.findChildren(ImagePanel):
            panel.set_light_mode(self._light_mode)
        for plot in self.findChildren(LinePlot):
            plot.set_light_mode(self._light_mode)
        for name, (text, tone, tooltip) in list(self._field_control_state.items()):
            self._set_field_control_label(name, text, tone=tone, tooltip=tooltip)

    def _semantic_color(self, tone: str) -> str:
        palette = {
            "light": {
                "normal": "#1f2937",
                "success": "#1a7f37",
                "warning": "#9a6700",
                "danger": "#cf222e",
                "muted": "#64748b",
            },
            "dark": {
                "normal": "#d7dde5",
                "success": "#56d364",
                "warning": "#f2cc60",
                "danger": "#ff6b6b",
                "muted": "#8d9aaa",
            },
        }
        return palette["light" if self._light_mode else "dark"][tone]

    def _apply_styles(self) -> None:
        if self._light_mode:
            stylesheet = """
            QMainWindow, QWidget { background:#f4f7fb; color:#1f2937; }
            QGroupBox { border:1px solid #cbd5e1; border-radius:5px; margin-top:11px; padding-top:9px; font-weight:600; }
            QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 5px; }
            QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QPlainTextEdit, QTreeWidget {
                background:#ffffff; border:1px solid #b8c2d1; border-radius:3px; padding:4px; color:#1f2937;
            }
            QPushButton { background:#e5eaf1; border:1px solid #aeb9c8; border-radius:4px; padding:7px 10px; color:#1f2937; }
            QPushButton:hover { background:#d8e0ea; }
            QPushButton#themeToggle:checked { background:#ffffff; border-color:#0969da; }
            QPushButton[primary="true"] { background:#0969da; border-color:#0550ae; color:white; }
            QPushButton[danger="true"] { background:#cf222e; border-color:#a40e26; color:white; font-weight:700; }
            QTabBar::tab { background:#e5eaf1; padding:8px 18px; margin-right:2px; }
            QTabBar::tab:selected { background:#0969da; color:white; }
            QHeaderView::section { background:#dce3ec; color:#1f2937; padding:5px; border:0; }
            """
        else:
            stylesheet = """
            QMainWindow, QWidget { background:#171d25; color:#d7dde5; }
            QGroupBox { border:1px solid #303b4a; border-radius:5px; margin-top:11px; padding-top:9px; font-weight:600; }
            QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 5px; }
            QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QPlainTextEdit, QTreeWidget {
                background:#10151c; border:1px solid #303b4a; border-radius:3px; padding:4px; color:#e6edf3;
            }
            QPushButton { background:#263142; border:1px solid #3b4b61; border-radius:4px; padding:7px 10px; }
            QPushButton:hover { background:#33425a; }
            QPushButton#themeToggle:checked { background:#e5eaf1; color:#1f2937; border-color:#58a6ff; }
            QPushButton[primary="true"] { background:#1f6feb; border-color:#388bfd; color:white; }
            QPushButton[danger="true"] { background:#b4232d; border-color:#ff5d67; color:white; font-weight:700; }
            QTabBar::tab { background:#202936; padding:8px 18px; margin-right:2px; }
            QTabBar::tab:selected { background:#1f6feb; }
            QHeaderView::section { background:#202936; color:#d7dde5; padding:5px; border:0; }
            """
        self.setStyleSheet(stylesheet)

    def _connect_ros(self) -> None:
        self.ros.image_received.connect(self._image_update)
        self.ros.pressure_received.connect(self.pressure_plot.append_values)
        self.ros.imu_acceleration_received.connect(self.imu_acceleration_plot.append_values)
        self.ros.imu_orientation_received.connect(self.imu_orientation_plot.append_values)
        self.ros.force_received.connect(self.force_plot.append_values)
        self.ros.odrive_received.connect(self.odrive_plot.append_values)
        self.ros.can_status_received.connect(self._can_status_update)
        self.ros.servo_status_received.connect(self._servo_update)
        self.ros.freedrive_status_received.connect(self._freedrive_update)
        self.ros.graph_received.connect(self._graph_update)
        self.ros.service_result.connect(self._service_result)
        self.ros.availability_changed.connect(self._ros_availability)

    def _apply_config(self, config: dict[str, Any], announce: bool = True) -> None:
        if self.supervisor.any_running:
            QMessageBox.warning(self, "Processes running", "Stop all pipeline processes before applying a new profile.")
            return
        errors = validate_config(config)
        if errors:
            QMessageBox.warning(self, "Invalid configuration", "\n".join(errors))
            return
        try:
            environment = build_child_environment(config)
        except (OSError, subprocess.CalledProcessError) as exc:
            if isinstance(exc, subprocess.CalledProcessError):
                output = exc.stderr or exc.stdout
                detail = output.decode(errors="replace") if output else str(exc)
            else:
                detail = str(exc)
            QMessageBox.critical(self, "ROS environment failed", detail)
            return
        self.config = normalize_config(config)
        specs = build_process_specs(self.config)
        self.supervisor.configure(specs, environment)
        self._populate_processes(specs)
        self.ros.update_topics(self.config["topics"])
        if announce:
            self._append_log("UI", f"Applied profile '{self.config['profile_name']}' ({self.config['mode']})")
            self.tabs.setCurrentIndex(1)

    def _populate_processes(self, specs) -> None:
        process_trees = (self.process_tree, self.control_process_tree)
        for tree in process_trees:
            tree.clear()
        self._process_rows.clear()
        for spec in specs:
            rows = []
            for tree in process_trees:
                item = QTreeWidgetItem([spec.label, "stopped", spec.command_text])
                tree.addTopLevelItem(item)
                rows.append(item)
            self._process_rows[spec.key] = rows

    def _start_support(self) -> None:
        self.supervisor.start_sequence([key for key in self.supervisor.keys() if key != "harvest"], 1500)

    def _call_trigger_service(self, config_key: str) -> None:
        service_name = self.config["services"][config_key]
        self._append_control_log(f"Requesting {service_name}")
        self.ros.call_trigger(service_name)

    def _call_bool_service(self, config_key: str, value: bool) -> None:
        service_name = self.config["services"][config_key]
        self._append_control_log(f"Requesting {service_name} data={str(value).lower()}")
        self.ros.call_set_bool(service_name, value)

    def _process_state(self, key: str, state: str, detail: str) -> None:
        items = self._process_rows.get(key, [])
        for item in items:
            item.setText(1, state)
            item.setText(2, detail)
            color = {
                "running": QColor("#56d364"),
                "starting": QColor("#f2cc60"),
                "stopping": QColor("#f2cc60"),
                "error": QColor("#ff6b6b"),
                "stopped": QColor("#8d9aaa"),
            }.get(state, QColor("#d7dde5"))
            item.setForeground(1, color)
        self._append_log(key, f"[{state}] {detail}")

    def _process_output(self, key: str, text: str) -> None:
        clean = ANSI_ESCAPE.sub("", text).replace("\r", "")
        self._append_log(key, clean.rstrip())
        lower = clean.lower()
        stage_match = re.search(r"--- running stage:\s*(.*?)\s*---", clean, re.IGNORECASE)
        if stage_match:
            self._set_field_control_label(
                "stage_status",
                f"Stage: {stage_match.group(1)}",
                tone="normal",
            )
        elif key == "harvest" and ("hit enter" in lower or "continue" in lower):
            prompt = clean.strip().splitlines()[-1][-90:]
            self._set_field_control_label(
                "stage_status",
                f"Waiting: {prompt}",
                tone="warning",
            )
        elif "batch complete" in lower:
            self._set_field_control_label(
                "stage_status",
                "Stage: batch complete",
                tone="success",
            )

    def _append_log(self, source: str, text: str) -> None:
        if not text:
            return
        timestamp = time.strftime("%H:%M:%S")
        lines = text.splitlines() or [text]
        self.log.appendPlainText("\n".join(f"{timestamp} [{source}] {line}" for line in lines))
        scrollbar = self.log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def _append_control_log(self, text: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.control_log.appendPlainText(f"{timestamp} {text}")
        scrollbar = self.control_log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def _ros_availability(self, available: bool, message: str) -> None:
        self._set_field_control_label(
            "ros_status",
            message,
            tone="success" if available else "danger",
        )
        self._append_log("ROS", message)

    def _graph_update(self, nodes: list[str]) -> None:
        self._set_field_control_label(
            "node_status",
            f"{len(nodes)} ROS nodes",
            tooltip="\n".join(nodes),
        )

    def _servo_update(self, code: int, _timestamp: float) -> None:
        text = self.SERVO_CODES.get(code, f"Unknown ({code})")
        dangerous = code in {2, 4, 5}
        warning = code in {1, 3, 6}
        tone = "danger" if dangerous else "warning" if warning else "success"
        self._set_field_control_label(
            "servo_status",
            f"Servo: {text}",
            tone=tone,
        )

    def _image_update(self, key: str, image, timestamp: float) -> None:
        for panel in self.image_panels.get(key, []):
            panel.set_image(image, timestamp)

    def _can_status_update(self, status: dict[str, Any], _timestamp: float) -> None:
        for key in ("position", "velocity", "torque", "current", "torque_setpoint"):
            self.can_status_labels[key].setText(f"{status[key]:.4f}")
        for key in ("active_error", "disarm_reason"):
            value = int(status[key])
            self.can_status_labels[key].setText(f"0x{value:08X}")
            color = self._semantic_color("danger" if value else "success")
            self.can_status_labels[key].setStyleSheet(f"color:{color}; font-weight:600")

    def _freedrive_update(self, enabled: bool, _timestamp: float) -> None:
        self._set_field_control_label(
            "freedrive_status",
            f"Freedrive: {'enabled' if enabled else 'disabled'}",
            tone="warning" if enabled else "success",
        )

    def _service_result(self, name: str, success: bool, message: str) -> None:
        self._append_log("CONTROL", f"{name}: {'OK' if success else 'FAILED'} — {message}")
        self._append_control_log(f"{name}: {'OK' if success else 'FAILED'} — {message}")
        if not success:
            QMessageBox.warning(self, "Control request failed", f"{name}\n{message}")

    def closeEvent(self, event: QCloseEvent) -> None:  # noqa: N802 (Qt API)
        if self._closing:
            if self.supervisor.any_running:
                event.ignore()
            else:
                self.ros.stop()
                event.accept()
            return
        if self.supervisor.any_running:
            answer = QMessageBox.question(
                self,
                "Stop the harvest system?",
                "The UI will send SIGINT to the harvest pipeline, ROS launch processes, RViz, and their child nodes before exiting.",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
                QMessageBox.StandardButton.Cancel,
            )
            if answer != QMessageBox.StandardButton.Yes:
                event.ignore()
                return
            self._closing = True
            self._shutdown_started = time.monotonic()
            self.supervisor.stop_all()
            self._append_log("UI", "Clean shutdown requested")
            event.ignore()
            QTimer.singleShot(100, self._poll_shutdown)
            return
        self.ros.stop()
        event.accept()

    def _poll_shutdown(self) -> None:
        if not self.supervisor.any_running:
            self.close()
            return
        if time.monotonic() - self._shutdown_started > 8.0:
            self.supervisor.kill_all()
        QTimer.singleShot(100, self._poll_shutdown)
