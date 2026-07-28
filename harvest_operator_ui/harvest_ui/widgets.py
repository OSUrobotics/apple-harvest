from __future__ import annotations

import math
import time
from collections import deque

from PySide6.QtCore import QPointF, QRectF, Qt, QTimer
from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QLabel, QSizePolicy, QVBoxLayout, QWidget


class ImagePanel(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._image = QImage()
        self._last_frame = 0.0
        self.label = QLabel("Waiting for image topic…")
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label.setMinimumSize(480, 270)
        self.label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.label.setStyleSheet("background:#10151c; color:#8d9aaa; border:1px solid #293241;")
        self.status = QLabel("No frames received")
        self.status.setStyleSheet("color:#8d9aaa")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.label, 1)
        layout.addWidget(self.status)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh_status)
        self._timer.start(500)

    def set_image(self, image: QImage, timestamp: float) -> None:
        self._image = image
        self._last_frame = timestamp
        self._render()

    def resizeEvent(self, event) -> None:  # noqa: N802 (Qt API)
        super().resizeEvent(event)
        self._render()

    def _render(self) -> None:
        if self._image.isNull():
            return
        pixmap = self._image.scaled(
            self.label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        from PySide6.QtGui import QPixmap

        self.label.setPixmap(QPixmap.fromImage(pixmap))

    def _refresh_status(self) -> None:
        if not self._last_frame:
            return
        age = time.monotonic() - self._last_frame
        if age > 2.0:
            self.status.setText(f"STALE — last frame {age:.1f}s ago")
            self.status.setStyleSheet("color:#ff6b6b; font-weight:600")
        else:
            self.status.setText(f"Live — frame age {age * 1000:.0f} ms")
            self.status.setStyleSheet("color:#56d364")


class LinePlot(QWidget):
    COLORS = [QColor("#58a6ff"), QColor("#f2cc60"), QColor("#ff7b72"), QColor("#56d364")]

    def __init__(self, title: str, series_names: list[str], max_points: int = 240, parent: QWidget | None = None):
        super().__init__(parent)
        self.title = title
        self.series_names = series_names
        self.values = [deque(maxlen=max_points) for _ in series_names]
        self.setMinimumHeight(190)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def append_values(self, values: list[float], _timestamp: float) -> None:
        for index, series in enumerate(self.values):
            if index < len(values) and math.isfinite(float(values[index])):
                series.append(float(values[index]))
        self.update()

    def paintEvent(self, _event) -> None:  # noqa: N802 (Qt API)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#10151c"))
        plot = QRectF(48, 30, max(10, self.width() - 62), max(10, self.height() - 58))
        painter.setPen(QPen(QColor("#293241"), 1))
        for i in range(5):
            y = plot.top() + plot.height() * i / 4
            painter.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y))
        painter.setPen(QColor("#d7dde5"))
        painter.drawText(10, 20, self.title)

        all_values = [value for series in self.values for value in series]
        if not all_values:
            painter.setPen(QColor("#6e7b8c"))
            painter.drawText(plot, Qt.AlignmentFlag.AlignCenter, "Waiting for topic…")
            return
        low, high = min(all_values), max(all_values)
        if math.isclose(low, high):
            margin = max(abs(low) * 0.05, 1.0)
            low, high = low - margin, high + margin
        else:
            margin = (high - low) * 0.08
            low, high = low - margin, high + margin
        painter.setPen(QColor("#8d9aaa"))
        painter.drawText(5, int(plot.top() + 10), f"{high:.1f}")
        painter.drawText(5, int(plot.bottom()), f"{low:.1f}")

        for index, series in enumerate(self.values):
            if len(series) < 2:
                continue
            color = self.COLORS[index % len(self.COLORS)]
            painter.setPen(QPen(color, 2))
            points: list[QPointF] = []
            denominator = max(1, len(series) - 1)
            for point_index, value in enumerate(series):
                x = plot.left() + plot.width() * point_index / denominator
                y = plot.bottom() - plot.height() * (value - low) / (high - low)
                points.append(QPointF(x, y))
            painter.drawPolyline(QPolygonF(points))
            legend_x = int(plot.left() + index * 105)
            painter.setPen(color)
            painter.drawText(legend_x, self.height() - 8, f"{self.series_names[index]} {series[-1]:.1f}")
