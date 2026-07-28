from __future__ import annotations

import os
import signal
from collections.abc import Iterable

from PySide6.QtCore import QObject, QProcess, QProcessEnvironment, QTimer, Signal

from .config import ProcessSpec


class ManagedProcess(QObject):
    output = Signal(str, str)
    state_changed = Signal(str, str, str)

    def __init__(self, spec: ProcessSpec, environment: dict[str, str], parent: QObject | None = None):
        super().__init__(parent)
        self.spec = spec
        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        qenv = QProcessEnvironment.systemEnvironment()
        for key, value in environment.items():
            qenv.insert(key, value)
        qenv.insert("PYTHONUNBUFFERED", "1")
        self.process.setProcessEnvironment(qenv)
        self.process.readyReadStandardOutput.connect(self._read_output)
        self.process.started.connect(self._started)
        self.process.errorOccurred.connect(self._error)
        self.process.finished.connect(self._finished)

    @property
    def running(self) -> bool:
        return self.process.state() != QProcess.ProcessState.NotRunning

    def start(self) -> None:
        if self.running:
            return
        self.state_changed.emit(self.spec.key, "starting", self.spec.command_text)
        # setsid creates one process group for ros2 and every node/GUI it starts.
        self.process.start("setsid", list(self.spec.argv))

    def write(self, text: str) -> None:
        if self.running:
            self.process.write(text.encode("utf-8"))
            self.process.waitForBytesWritten(100)

    def stop(self) -> None:
        if not self.running:
            return
        pid = int(self.process.processId())
        self.state_changed.emit(self.spec.key, "stopping", "Sending SIGINT to process group")
        self._signal_group(pid, signal.SIGINT)
        QTimer.singleShot(2500, lambda: self._escalate(pid, signal.SIGTERM))
        QTimer.singleShot(5000, lambda: self._escalate(pid, signal.SIGKILL))

    def kill(self) -> None:
        if self.running:
            self._signal_group(int(self.process.processId()), signal.SIGKILL)

    def _signal_group(self, pid: int, sig: signal.Signals) -> None:
        try:
            os.killpg(pid, sig)
        except (ProcessLookupError, PermissionError):
            if self.running:
                self.process.kill()

    def _escalate(self, original_pid: int, sig: signal.Signals) -> None:
        if self.running and int(self.process.processId()) == original_pid:
            self._signal_group(original_pid, sig)

    def _read_output(self) -> None:
        data = bytes(self.process.readAllStandardOutput()).decode("utf-8", errors="replace")
        if data:
            self.output.emit(self.spec.key, data)

    def _started(self) -> None:
        self.state_changed.emit(self.spec.key, "running", f"PID {self.process.processId()}")

    def _error(self, error: QProcess.ProcessError) -> None:
        self.state_changed.emit(self.spec.key, "error", f"{error.name}: {self.process.errorString()}")

    def _finished(self, exit_code: int, exit_status: QProcess.ExitStatus) -> None:
        state = "stopped" if exit_code in {0, 2, 130, -2} else "error"
        detail = f"exit={exit_code}, status={exit_status.name}"
        self.state_changed.emit(self.spec.key, state, detail)


class ProcessSupervisor(QObject):
    output = Signal(str, str)
    state_changed = Signal(str, str, str)

    def __init__(self, parent: QObject | None = None):
        super().__init__(parent)
        self._processes: dict[str, ManagedProcess] = {}
        self._environment = dict(os.environ)

    def configure(self, specs: Iterable[ProcessSpec], environment: dict[str, str] | None = None) -> None:
        if self.any_running:
            raise RuntimeError("Stop running processes before applying a new configuration.")
        for process in self._processes.values():
            process.deleteLater()
        self._processes.clear()
        self._environment = dict(environment or os.environ)
        for spec in specs:
            process = ManagedProcess(spec, self._environment, self)
            process.output.connect(self.output)
            process.state_changed.connect(self.state_changed)
            self._processes[spec.key] = process
            self.state_changed.emit(spec.key, "stopped", spec.command_text)

    @property
    def any_running(self) -> bool:
        return any(process.running for process in self._processes.values())

    def keys(self) -> list[str]:
        return list(self._processes)

    def start(self, key: str) -> None:
        process = self._processes.get(key)
        if process:
            process.start()

    def start_sequence(self, keys: Iterable[str], interval_ms: int = 1200) -> None:
        delay = 0
        for key in keys:
            if key in self._processes and not self._processes[key].running:
                QTimer.singleShot(delay, lambda process_key=key: self.start(process_key))
                delay += interval_ms

    def write(self, key: str, text: str) -> None:
        process = self._processes.get(key)
        if process:
            process.write(text)

    def stop(self, key: str) -> None:
        process = self._processes.get(key)
        if process:
            process.stop()

    def stop_all(self) -> None:
        # Stop the orchestrator first, then its dependencies in reverse order.
        for key in reversed(self.keys()):
            self._processes[key].stop()

    def kill_all(self) -> None:
        for process in self._processes.values():
            process.kill()
