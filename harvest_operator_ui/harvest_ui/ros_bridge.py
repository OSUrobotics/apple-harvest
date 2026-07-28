from __future__ import annotations

import queue
import threading
import time
from typing import Any

import numpy as np
from PySide6.QtCore import QObject, Signal
from PySide6.QtGui import QImage

ROS_IMPORT_ERROR = ""
try:
    import rclpy
    from geometry_msgs.msg import WrenchStamped
    from gripper_interfaces.msg import CanStatusMsg
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import Bool, Float32MultiArray, Int16MultiArray, Int8
    from std_srvs.srv import SetBool, Trigger

    ROS_AVAILABLE = True
except ImportError as exc:  # Allows the configuration UI to open without ROS sourced.
    ROS_AVAILABLE = False
    ROS_IMPORT_ERROR = str(exc)


class RosBridge(QObject):
    image_received = Signal(str, QImage, float)
    pressure_received = Signal(object, float)
    imu_acceleration_received = Signal(object, float)
    imu_orientation_received = Signal(object, float)
    force_received = Signal(object, float)
    odrive_received = Signal(object, float)
    can_status_received = Signal(object, float)
    servo_status_received = Signal(int, float)
    freedrive_status_received = Signal(bool, float)
    graph_received = Signal(object)
    service_result = Signal(str, bool, str)
    availability_changed = Signal(bool, str)

    def __init__(self, topics: dict[str, str], parent: QObject | None = None):
        super().__init__(parent)
        self._topics = dict(topics)
        self._commands: queue.Queue[tuple[str, Any]] = queue.Queue()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._node = None
        self._executor = None
        self._subscriptions: list[Any] = []

    def start(self) -> None:
        if not ROS_AVAILABLE:
            self.availability_changed.emit(False, f"ROS Python imports unavailable: {ROS_IMPORT_ERROR}")
            return
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._run, name="harvest-ui-ros", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._executor:
            self._executor.wake()
        if self._thread:
            self._thread.join(timeout=2.0)

    def update_topics(self, topics: dict[str, str]) -> None:
        self._commands.put(("topics", dict(topics)))
        if self._executor:
            self._executor.wake()

    def call_trigger(self, service_name: str) -> None:
        self._commands.put(("trigger", service_name))
        if self._executor:
            self._executor.wake()

    def call_set_bool(self, service_name: str, value: bool) -> None:
        self._commands.put(("set_bool", (service_name, bool(value))))
        if self._executor:
            self._executor.wake()

    def _run(self) -> None:
        try:
            rclpy.init(args=None)
            self._node = rclpy.create_node("harvest_operator_ui")
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self._create_subscriptions(self._topics)
            self.availability_changed.emit(True, "ROS connected")
            last_graph_update = 0.0
            while rclpy.ok() and not self._stop_event.is_set():
                self._drain_commands()
                self._executor.spin_once(timeout_sec=0.05)
                if time.monotonic() - last_graph_update > 1.0:
                    nodes = sorted(f"{ns.rstrip('/')}/{name}" for name, ns in self._node.get_node_names_and_namespaces())
                    self.graph_received.emit(nodes)
                    last_graph_update = time.monotonic()
        except Exception as exc:  # Keep GUI alive and make ROS setup failures visible.
            self.availability_changed.emit(False, f"ROS bridge failed: {exc}")
        finally:
            if self._executor:
                self._executor.shutdown(timeout_sec=1.0)
            if self._node:
                self._node.destroy_node()
            if ROS_AVAILABLE and rclpy.ok():
                rclpy.shutdown()

    def _drain_commands(self) -> None:
        while True:
            try:
                command, payload = self._commands.get_nowait()
            except queue.Empty:
                return
            if command == "topics":
                self._topics = payload
                self._create_subscriptions(payload)
            elif command == "trigger":
                self._call_trigger_in_ros_thread(payload)
            elif command == "set_bool":
                self._call_set_bool_in_ros_thread(*payload)

    def _create_subscriptions(self, topics: dict[str, str]) -> None:
        for subscription in self._subscriptions:
            self._node.destroy_subscription(subscription)
        self._subscriptions.clear()
        for key in ("mast_rgb", "mast_depth", "palm_image", "apple_prediction_image"):
            callback = lambda msg, image_key=key: self._image_callback(image_key, msg)
            self._subscriptions.append(
                self._node.create_subscription(Image, topics[key], callback, qos_profile_sensor_data)
            )
        self._subscriptions.extend(
            [
                self._node.create_subscription(Int16MultiArray, topics["pressure"], self._pressure_callback, qos_profile_sensor_data),
                self._node.create_subscription(Float32MultiArray, topics["gripper_imu"], self._imu_callback, qos_profile_sensor_data),
                self._node.create_subscription(CanStatusMsg, topics["can_status"], self._can_status_callback, qos_profile_sensor_data),
                self._node.create_subscription(WrenchStamped, topics["wrench"], self._wrench_callback, qos_profile_sensor_data),
                self._node.create_subscription(Int8, topics["servo_status"], self._servo_callback, 10),
                self._node.create_subscription(Bool, topics["freedrive_status"], self._freedrive_callback, 10),
            ]
        )

    def _call_trigger_in_ros_thread(self, service_name: str) -> None:
        client = self._node.create_client(Trigger, service_name)
        if not client.wait_for_service(timeout_sec=0.2):
            self.service_result.emit(service_name, False, "Service is not available")
            self._node.destroy_client(client)
            return
        future = client.call_async(Trigger.Request())

        def done(completed):
            try:
                result = completed.result()
                self.service_result.emit(service_name, bool(result.success), result.message or "Request completed")
            except Exception as exc:
                self.service_result.emit(service_name, False, str(exc))
            self._node.destroy_client(client)

        future.add_done_callback(done)

    def _call_set_bool_in_ros_thread(self, service_name: str, value: bool) -> None:
        client = self._node.create_client(SetBool, service_name)
        if not client.wait_for_service(timeout_sec=0.2):
            self.service_result.emit(service_name, False, "Service is not available")
            self._node.destroy_client(client)
            return
        future = client.call_async(SetBool.Request(data=value))

        def done(completed):
            try:
                result = completed.result()
                self.service_result.emit(service_name, bool(result.success), result.message or "Request completed")
            except Exception as exc:
                self.service_result.emit(service_name, False, str(exc))
            self._node.destroy_client(client)

        future.add_done_callback(done)

    def _image_callback(self, key: str, msg: Any) -> None:
        try:
            image = self._to_qimage(msg)
            if not image.isNull():
                self.image_received.emit(key, image, time.monotonic())
        except (ValueError, TypeError):
            return

    @staticmethod
    def _to_qimage(msg: Any) -> QImage:
        encoding = msg.encoding.lower()
        height, width, step = int(msg.height), int(msg.width), int(msg.step)
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if encoding in {"rgb8", "bgr8"}:
            rows = raw.reshape(height, step)[:, : width * 3]
            array = rows.reshape(height, width, 3)
            image = QImage(array.data, width, height, int(array.strides[0]), QImage.Format.Format_RGB888).copy()
            return image.rgbSwapped() if encoding == "bgr8" else image
        if encoding in {"rgba8", "bgra8"}:
            rows = raw.reshape(height, step)[:, : width * 4]
            array = rows.reshape(height, width, 4)
            fmt = QImage.Format.Format_RGBA8888 if encoding == "rgba8" else QImage.Format.Format_ARGB32
            return QImage(array.data, width, height, int(array.strides[0]), fmt).copy()
        if encoding in {"mono8", "8uc1"}:
            array = raw.reshape(height, step)[:, :width]
            return QImage(array.data, width, height, int(array.strides[0]), QImage.Format.Format_Grayscale8).copy()
        if encoding in {"mono16", "16uc1"}:
            data16 = np.frombuffer(msg.data, dtype=np.uint16).reshape(height, step // 2)[:, :width]
            low, high = np.percentile(data16, (2, 98))
            scaled = np.clip((data16.astype(np.float32) - low) * 255.0 / max(1.0, high - low), 0, 255).astype(np.uint8)
            return QImage(scaled.data, width, height, int(scaled.strides[0]), QImage.Format.Format_Grayscale8).copy()
        return QImage()

    def _pressure_callback(self, msg: Any) -> None:
        self.pressure_received.emit([float(value) for value in msg.data[:4]], time.monotonic())

    def _imu_callback(self, msg: Any) -> None:
        if len(msg.data) < 6:
            return
        timestamp = time.monotonic()
        self.imu_acceleration_received.emit([float(value) for value in msg.data[:3]], timestamp)
        self.imu_orientation_received.emit([float(value) for value in msg.data[3:6]], timestamp)

    def _wrench_callback(self, msg: Any) -> None:
        force = msg.wrench.force
        self.force_received.emit([float(force.x), float(force.y), float(force.z)], time.monotonic())

    def _can_status_callback(self, msg: Any) -> None:
        timestamp = time.monotonic()
        status = {
            "position": float(msg.position),
            "velocity": float(msg.velocity),
            "torque": float(msg.torque),
            "current": float(msg.current),
            "torque_setpoint": float(msg.torque_setpoint),
            "active_error": int(msg.active_error),
            "disarm_reason": int(msg.disarm_reason),
        }
        self.can_status_received.emit(status, timestamp)
        self.odrive_received.emit(
            [status["velocity"], status["torque"], status["current"], status["torque_setpoint"]],
            timestamp,
        )

    def _servo_callback(self, msg: Any) -> None:
        self.servo_status_received.emit(int(msg.data), time.monotonic())

    def _freedrive_callback(self, msg: Any) -> None:
        self.freedrive_status_received.emit(bool(msg.data), time.monotonic())
