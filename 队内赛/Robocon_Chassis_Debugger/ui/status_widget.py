"""机器人状态面板。"""
from __future__ import annotations

from PyQt5 import QtCore, QtWidgets


class StatusWidget(QtWidgets.QFrame):
    def __init__(self, parent=None):
        super().__init__(parent); self.setObjectName("statusPanel")
        layout = QtWidgets.QVBoxLayout(self); layout.setContentsMargins(10, 8, 10, 8); layout.setSpacing(4)
        self.layout = layout
        title = QtWidgets.QLabel("ROBOT STATE"); title.setObjectName("sectionTitle"); layout.addWidget(title)
        self.status_badge = QtWidgets.QLabel("●  NORMAL"); self.status_badge.setObjectName("statusBadge"); layout.addWidget(self.status_badge)
        self.position = self._group("POSITION", [("X", "0.00 m"), ("Y", "0.00 m"), ("YAW GYRO INT", "0.0°"), ("YAW IMU", "0.0°"), ("YAW WHEEL INT", "0.0°"), ("YAW EKF", "0.0°")], layout)
        self.velocity = self._group("VELOCITY", [("VX", "0.00 m/s"), ("VY", "0.00 m/s"), ("OMEGA", "0.0 deg/s")], layout)
        self.sensors = self._group("SENSORS", [("DT35-1", "0.000 m"), ("DT35-2", "0.000 m")], layout)
        self.telemetry = self._group("TELEMETRY", [("TIMESTAMP", "0 ms"), ("UPTIME", "0.0 s")], layout)
        layout.addStretch(1)

    def _group(self, heading, rows, parent_layout):
        group = QtWidgets.QGroupBox(heading); form = QtWidgets.QFormLayout(group); form.setContentsMargins(6, 4, 6, 4)
        form.setVerticalSpacing(2)
        values = {}
        for key, value in rows:
            label = QtWidgets.QLabel(value); label.setObjectName("metricValue"); form.addRow(key, label); values[key] = label
        parent_layout.addWidget(group); return values

    def update_state(self, state) -> None:
        self.position["X"].setText(f"{state.position_x_m:.2f} m"); self.position["Y"].setText(f"{state.position_y_m:.2f} m"); self.position["YAW GYRO INT"].setText(f"{state.yaw_deg:.1f}°"); self.position["YAW IMU"].setText(f"{state.imu_yaw_deg:.1f}°"); self.position["YAW WHEEL INT"].setText(f"{state.chassis_yaw_deg:.1f}°"); self.position["YAW EKF"].setText(f"{state.fused_yaw_deg:.1f}°")
        self.velocity["VX"].setText(f"{state.velocity_x_mps:.2f} m/s"); self.velocity["VY"].setText(f"{state.velocity_y_mps:.2f} m/s"); self.velocity["OMEGA"].setText(f"{state.yaw_rate_degps:.1f} deg/s")
        self.sensors["DT35-1"].setText(f"{state.dt35_distance_0_m:.3f} m"); self.sensors["DT35-2"].setText(f"{state.dt35_distance_1_m:.3f} m")
        self.telemetry["TIMESTAMP"].setText(f"{state.timestamp_ms} ms"); self.telemetry["UPTIME"].setText(f"{state.timestamp_ms / 1000:.1f} s")
        fault = bool(state.status_flags & 1); running = bool(state.status_flags & 2)
        self.status_badge.setText("●  FAULT" if fault else ("●  RUNNING" if running else "●  NORMAL")); self.status_badge.setProperty("fault", fault); self.status_badge.style().unpolish(self.status_badge); self.status_badge.style().polish(self.status_badge)
