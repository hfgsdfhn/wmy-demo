"""Chassis manual-control and health-monitoring desktop application."""
from __future__ import annotations

import math
import json
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets

from ..communication.protocol import (
    FrameParser, ID_DUAL_MCU_DIAGNOSTIC, ID_HEALTH, ID_STATE,
    decode_dual_mcu_diagnostic, decode_health, decode_state,
    encode_command, encode_manual, encode_motion_limits, encode_yaw_pid_params,
    encode_yaw_pid_profile, encode_yaw_pid_profile_params,
    encode_yaw_target, encode_yaw_pid_mode, encode_dt35_direction,
    encode_dt35_sequence, encode_str_pid_params, encode_dt35_targets,
    encode_yaw_switch_thresholds,
)
from ..communication.serial_port import SerialPort
from .status_widget import StatusWidget


CONTROL_KEYS = {
    QtCore.Qt.Key_W, QtCore.Qt.Key_A, QtCore.Qt.Key_S,
    QtCore.Qt.Key_D, QtCore.Qt.Key_Q, QtCore.Qt.Key_E,
}

YAW_PID_PROFILE_90 = 0
YAW_PID_PROFILE_QE = 1
YAW_PID_PROFILE_180 = 2
YAW_QE_STEP_DEG = 5.0
YAW_QE_STEP_PERIOD_MS = 100
S_CURVE_SEGMENT_PERIOD_MS = 1000
S_CURVE_SEQUENCES = {
    "blue": (1 << 9, 1 << 13, 1 << 9, 1 << 11, 1 << 9),
    "red": (1 << 9, 1 << 11, 1 << 9, 1 << 13, 1 << 9),
}


class HealthItem(QtWidgets.QFrame):
    def __init__(self, name: str, parent=None):
        super().__init__(parent)
        self.setObjectName("healthItem")
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(14, 12, 14, 12)
        self.name_label = QtWidgets.QLabel(name)
        self.value_label = QtWidgets.QLabel("OFFLINE")
        self.value_label.setObjectName("healthValue")
        layout.addWidget(self.name_label)
        layout.addStretch()
        layout.addWidget(self.value_label)

    def set_state(self, online: bool, detail: str = "") -> None:
        self.value_label.setText("ONLINE" if online else "OFFLINE")
        self.value_label.setProperty("online", online)
        self.value_label.setToolTip(detail)
        self.value_label.style().unpolish(self.value_label)
        self.value_label.style().polish(self.value_label)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, config_path: Path):
        super().__init__()
        self.setWindowTitle("Robocon Chassis Console")
        self.resize(940, 680)
        self.setMinimumSize(760, 560)
        self.config_path = config_path
        self.serial_baudrate = self._load_serial_baudrate(config_path)
        self.parser = FrameParser()
        self.serial = SerialPort(self)
        self.pressed_keys: set[int] = set()
        self.button_keys: set[int] = set()
        self.drive_enabled = False
        self.yaw_target_deg = 0.0
        self.yaw_target_initialized = False
        self.latest_yaw_deg: float | None = None
        self.yaw_pid_preset: str | None = None
        self.qe_step_elapsed_ms = 0
        self.s_curve_name: str | None = None
        self.s_curve_step = 0
        self.s_curve_elapsed_ms = 0
        self.tx_frames = 0
        self.rx_frames = 0
        self.last_rx_elapsed_ms = -1
        self.link_clock = QtCore.QElapsedTimer()
        self.link_clock.start()

        self._build_ui()
        self.estop_shortcut = QtWidgets.QShortcut(QtGui.QKeySequence("Space"), self)
        self.estop_shortcut.setContext(QtCore.Qt.ApplicationShortcut)
        self.estop_shortcut.activated.connect(self._estop)
        self.serial.data_received.connect(self._receive_bytes)
        self.serial.connection_changed.connect(self._connection_changed)

        self.manual_timer = QtCore.QTimer(self)
        self.manual_timer.timeout.connect(self._send_manual_keepalive)
        self.manual_timer.start(20)
        self.link_timer = QtCore.QTimer(self)
        self.link_timer.timeout.connect(self._update_link_status)
        self.link_timer.start(250)
        self.port_timer = QtCore.QTimer(self)
        self.port_timer.timeout.connect(self._refresh_ports)
        self.port_timer.start(1500)
        self._refresh_ports()

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(18, 16, 18, 16)

        header = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("Robocon Chassis Console")
        title.setObjectName("appTitle")
        self.connection_badge = QtWidgets.QLabel("UART OFFLINE")
        self.connection_badge.setObjectName("statusBadge")
        self.connection_badge.setProperty("fault", True)
        header.addWidget(title)
        header.addStretch()
        header.addWidget(self.connection_badge)
        root.addLayout(header)

        self.tabs = QtWidgets.QTabWidget()
        # Keep the live health view first so faults are visible immediately.
        self.tabs.addTab(self._build_health_page(), "模块状态")
        self.tabs.addTab(self._build_control_page(), "底盘控制")
        self.tabs.addTab(self._build_config_page(), "DT35/PID设置")
        root.addWidget(self.tabs, 1)
        self.setCentralWidget(central)
        self.statusBar().showMessage("选择串口并连接，确认底盘架空后再使能")

    def _create_yaw_pid_panel(self, title: str, profile: int,
                              defaults: tuple[float, float, float, float, float, float, float]
                              ) -> QtWidgets.QGroupBox:
        panel = QtWidgets.QGroupBox(title)
        form = QtWidgets.QFormLayout(panel)
        controls = {
            "kp": self._spin(0.0, 1000.0, defaults[0], ""),
            "ki": self._spin(0.0, 1000.0, defaults[1], ""),
            "kd": self._spin(0.0, 1000.0, defaults[2], ""),
            "max": self._spin(0.01, 10000.0, defaults[3], " deg/s"),
            "i_limit": self._spin(0.0, 100.0, defaults[4], ""),
            "i_sep": self._spin(0.0, 180.0, defaults[5], " deg"),
            "deadzone": self._spin(0.0, 180.0, defaults[6], " deg"),
        }
        for control in controls.values():
            control.setDecimals(6)
        for label, name in (("KP", "kp"), ("KI", "ki"), ("KD", "kd"),
                            ("MAX OUTPUT", "max"), ("I LIMIT", "i_limit"),
                            ("I SEPARATION", "i_sep"), ("DEADZONE", "deadzone")):
            form.addRow(label, controls[name])
        apply_button = QtWidgets.QPushButton("APPLY")
        apply_button.clicked.connect(
            lambda: self._apply_yaw_pid_profile(profile, title, controls))
        form.addRow(apply_button)
        if not hasattr(self, "yaw_pid_profile_controls"):
            self.yaw_pid_profile_controls = {}
        self.yaw_pid_profile_controls[profile] = controls
        return panel

    def _build_control_page(self) -> QtWidgets.QWidget:
        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll_area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        page = QtWidgets.QWidget()
        layout = QtWidgets.QGridLayout(page)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 2)

        connection = QtWidgets.QGroupBox("通信链路")
        form = QtWidgets.QFormLayout(connection)
        self.port_combo = QtWidgets.QComboBox()
        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.addItems(["921600", "115200"])
        self.baud_combo.setCurrentText(str(self.serial_baudrate))
        buttons = QtWidgets.QHBoxLayout()
        connect = QtWidgets.QPushButton("连接")
        connect.setObjectName("primaryButton")
        disconnect = QtWidgets.QPushButton("断开")
        connect.clicked.connect(self._connect_serial)
        disconnect.clicked.connect(self.serial.close)
        buttons.addWidget(connect)
        buttons.addWidget(disconnect)
        form.addRow("端口", self.port_combo)
        form.addRow("波特率", self.baud_combo)
        form.addRow(buttons)
        self.link_detail = QtWidgets.QLabel("RX 0  |  TX 0  |  无遥测")
        self.link_detail.setObjectName("mutedLabel")
        form.addRow(self.link_detail)

        limits = QtWidgets.QGroupBox("速度限制")
        limit_form = QtWidgets.QFormLayout(limits)
        self.linear_spin = self._spin(0.05, 5.0, 0.30, " m/s")
        self.angular_spin = self._spin(3.0, 860.0, 45.0, " deg/s")
        self.accel_spin = self._spin(0.05, 20.0, 1.0, " m/s2")
        apply_limits = QtWidgets.QPushButton("下发限制")
        apply_limits.clicked.connect(self._apply_limits)
        limit_form.addRow("平移", self.linear_spin)
        limit_form.addRow("旋转", self.angular_spin)
        limit_form.addRow("加速度", self.accel_spin)
        limit_form.addRow(apply_limits)

        manual = QtWidgets.QGroupBox("四轮底盘手动控制")
        manual_layout = QtWidgets.QGridLayout(manual)
        self.enable_button = QtWidgets.QPushButton("使能底盘")
        self.enable_button.setCheckable(True)
        self.enable_button.setObjectName("enableButton")
        self.enable_button.toggled.connect(self._set_drive_enabled)
        manual_layout.addWidget(self.enable_button, 0, 0, 1, 3)
        manual_layout.addWidget(
            QtWidgets.QLabel("WORLD: W/S X+/-  A/D Y+/-  Q/E YAW  Space STOP"), 1, 0, 1, 3)
        self.velocity_label = QtWidgets.QLabel("world vx +0.00   vy +0.00   omega +0.00")
        self.velocity_label.setObjectName("metricValue")
        manual_layout.addWidget(self.velocity_label, 2, 0, 1, 3)

        for text, key, row, column in (
            ("前进 W", QtCore.Qt.Key_W, 3, 1),
            ("左移 A", QtCore.Qt.Key_A, 4, 0),
            ("停止", None, 4, 1),
            ("右移 D", QtCore.Qt.Key_D, 4, 2),
            ("后退 S", QtCore.Qt.Key_S, 5, 1),
            ("左旋 Q", QtCore.Qt.Key_Q, 6, 0),
            ("右旋 E", QtCore.Qt.Key_E, 6, 2),
        ):
            button = QtWidgets.QPushButton(text)
            button.setObjectName("driveButton")
            if key is None:
                button.clicked.connect(self._stop_motion)
            else:
                button.pressed.connect(lambda value=key: self._set_button_control(value, True))
                button.released.connect(lambda value=key: self._set_button_control(value, False))
            manual_layout.addWidget(button, row, column)

        s_curve = QtWidgets.QGroupBox("S CURVE")
        s_curve_layout = QtWidgets.QGridLayout(s_curve)
        for text, direction, row, column in (
            ("FORWARD", 1 << 9, 0, 1),
            ("LEFT", 1 << 11, 1, 0),
            ("BACK", 1 << 12, 1, 1),
            ("RIGHT", 1 << 13, 1, 2),
        ):
            button = QtWidgets.QPushButton(text)
            button.setObjectName("driveButton")
            button.clicked.connect(
                lambda value=direction: self._send_dt35_direction(value))
            s_curve_layout.addWidget(button, row, column)
        blue = QtWidgets.QPushButton("blue")
        red = QtWidgets.QPushButton("red")
        blue.clicked.connect(lambda: self._start_s_curve("blue"))
        red.clicked.connect(lambda: self._start_s_curve("red"))
        s_curve_layout.addWidget(blue, 2, 0, 1, 2)
        s_curve_layout.addWidget(red, 2, 2)
        manual_layout.addWidget(s_curve, 7, 0, 1, 3)

        estop = QtWidgets.QPushButton("急停")
        estop.setObjectName("dangerButton")
        estop.clicked.connect(self._estop)
        manual_layout.addWidget(estop, 8, 0, 1, 3)

        self.yaw_target_spin = self._spin(-180.0, 180.0, 0.0, " deg")
        self.yaw_pid_enable_check = QtWidgets.QCheckBox("ENABLE YAW POSITION PID")
        self.yaw_pid_enable_check.setChecked(True)
        self.yaw_pid_enable_check.toggled.connect(self._send_yaw_pid_mode)
        set_target = QtWidgets.QPushButton("SET TARGET YAW")
        set_target.clicked.connect(self._send_yaw_target_from_input)
        turn_left_90 = QtWidgets.QPushButton("左转 90°")
        turn_left_90.clicked.connect(lambda: self._turn_relative_90(-90.0))
        turn_right_90 = QtWidgets.QPushButton("右转 90°")
        turn_right_90.clicked.connect(lambda: self._turn_relative_90(90.0))
        turn_buttons = QtWidgets.QHBoxLayout()
        turn_buttons.addWidget(turn_left_90)
        turn_buttons.addWidget(turn_right_90)
        turn_left_180 = QtWidgets.QPushButton("TURN 180 L")
        turn_left_180.clicked.connect(lambda: self._turn_relative_90(-180.0))
        turn_right_180 = QtWidgets.QPushButton("TURN 180 R")
        turn_right_180.clicked.connect(lambda: self._turn_relative_90(180.0))
        turn_buttons.addWidget(turn_left_180)
        turn_buttons.addWidget(turn_right_180)
        yaw_target = QtWidgets.QGroupBox("YAW TARGET")
        yaw_target_form = QtWidgets.QFormLayout(yaw_target)
        yaw_target_form.addRow("TARGET", self.yaw_target_spin)
        yaw_target_form.addRow(self.yaw_pid_enable_check)
        yaw_target_form.addRow(set_target)
        yaw_target_form.addRow(turn_buttons)
        manual_layout.addWidget(yaw_target, 9, 0, 1, 3)

        str_pid = QtWidgets.QGroupBox("CHASSIS STRPID")
        str_form = QtWidgets.QFormLayout(str_pid)
        self.str_kp_spin = self._spin(0.0, 100.0, 1.2, "")
        self.str_ki_spin = self._spin(0.0, 100.0, 0.0, "")
        self.str_kd_spin = self._spin(0.0, 100.0, 0.001, "")
        self.str_max_output_spin = self._spin(0.01, 100.0, 2.0, "")
        self.str_integral_limit_spin = self._spin(0.0, 100.0, 0.0, "")
        self.str_integral_separation_spin = self._spin(0.0, 100.0, 0.0, "")
        self.str_deadzone_spin = self._spin(0.0, 1.0, 0.02, " m")
        for label, control in (("KP", self.str_kp_spin), ("KI", self.str_ki_spin),
                               ("KD", self.str_kd_spin), ("MAX", self.str_max_output_spin),
                               ("I LIMIT", self.str_integral_limit_spin),
                               ("I SEP", self.str_integral_separation_spin),
                               ("DEADZONE", self.str_deadzone_spin)):
            str_form.addRow(label, control)
        apply_str = QtWidgets.QPushButton("APPLY STRPID")
        apply_str.clicked.connect(self._apply_str_pid)
        str_form.addRow(apply_str)
        pid_row = QtWidgets.QHBoxLayout()
        pid_row.addWidget(self._create_yaw_pid_panel(
            "HEADING HOLD PID", YAW_PID_PROFILE_QE,
            (5.0, 6.0, 0.008, 150.0, 20.0, 10.0, 0.0)))
        pid_row.addWidget(self._create_yaw_pid_panel(
            "90 DEG PID", YAW_PID_PROFILE_90,
            (4.5, 2.5, 0.136, 150.0, 10.0, 10.0, 2.0)))
        pid_row.addWidget(self._create_yaw_pid_panel(
            "180 DEG PID", YAW_PID_PROFILE_180,
            (4.5, 2.5, 0.136, 150.0, 10.0, 10.0, 2.0)))
        pid_row.addWidget(str_pid)
        # The operational controls stay near the top; tuning panels are kept
        # below them so they do not crowd the status and safety controls.
        layout.addWidget(connection, 0, 0)
        layout.addWidget(limits, 1, 0)
        layout.addWidget(manual, 1, 1, 2, 1)
        self.status_widget = StatusWidget()
        layout.addWidget(self.status_widget, 0, 1)
        layout.addLayout(pid_row, 3, 0, 1, 2)
        scroll_area.setWidget(page)
        return scroll_area

    def _build_config_page(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        yaw = QtWidgets.QGroupBox("YAW PID PROFILE")
        yaw_form = QtWidgets.QFormLayout(yaw)
        self.config_yaw_profile = QtWidgets.QComboBox()
        self.config_yaw_profile.addItem("Heading hold", 1)
        self.config_yaw_profile.addItem("Small-angle PID", 0)
        self.config_yaw_profile.addItem("Large-angle PID", 2)
        select = QtWidgets.QPushButton("SELECT PROFILE")
        select.clicked.connect(self._apply_config_yaw_profile)
        yaw_form.addRow("PROFILE", self.config_yaw_profile)
        self.config_yaw_large_threshold = self._spin(0.0, 180.0, 135.0, " deg")
        self.config_yaw_small_threshold = self._spin(0.0, 179.0, 45.0, " deg")
        yaw_form.addRow("LARGE THRESHOLD", self.config_yaw_large_threshold)
        yaw_form.addRow("SMALL THRESHOLD", self.config_yaw_small_threshold)
        threshold_button = QtWidgets.QPushButton("APPLY THRESHOLDS")
        threshold_button.clicked.connect(self._apply_config_yaw_thresholds)
        yaw_form.addRow(threshold_button)
        yaw_form.addRow(select)
        targets = QtWidgets.QGroupBox("DT35 TARGETS")
        form = QtWidgets.QFormLayout(targets)
        self.config_dt35_btf = self._spin(0.0, 10.0, 1.3, " m")
        self.config_dt35_ftb = self._spin(0.0, 10.0, 0.2, " m")
        self.config_dt35_rtl = self._spin(0.0, 10.0, 0.2, " m")
        self.config_dt35_ltr = self._spin(0.0, 10.0, 2.25, " m")
        form.addRow("BACK TO FRONT", self.config_dt35_btf)
        form.addRow("FRONT TO BACK", self.config_dt35_ftb)
        form.addRow("RIGHT TO LEFT", self.config_dt35_rtl)
        form.addRow("LEFT TO RIGHT", self.config_dt35_ltr)
        apply_targets = QtWidgets.QPushButton("APPLY DT35 TARGETS")
        apply_targets.clicked.connect(self._apply_config_dt35_targets)
        form.addRow(apply_targets)
        layout.addWidget(yaw)
        layout.addWidget(targets)
        layout.addStretch()
        return page

    def _apply_config_yaw_profile(self) -> None:
        profile = int(self.config_yaw_profile.currentData())
        if self.serial.write(encode_yaw_pid_profile(profile)):
            self.tx_frames += 1
            self.statusBar().showMessage("YAW PID profile selected")

    def _apply_config_yaw_thresholds(self) -> None:
        large = self.config_yaw_large_threshold.value()
        small = self.config_yaw_small_threshold.value()
        if large <= small:
            self.statusBar().showMessage("Large threshold must exceed small threshold")
            return
        if self.serial.write(encode_yaw_switch_thresholds(large, small)):
            self.tx_frames += 1
            self.statusBar().showMessage("YAW switch thresholds updated")

    def _apply_config_dt35_targets(self) -> None:
        frame = encode_dt35_targets(self.config_dt35_btf.value(), self.config_dt35_ftb.value(),
                                    self.config_dt35_rtl.value(), self.config_dt35_ltr.value())
        if self.serial.write(frame):
            self.tx_frames += 1
            self.statusBar().showMessage("DT35 targets updated")

    def _build_health_page(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setSpacing(8)
        self.health_items = {
            "pc": HealthItem("上位机"),
            "remote": HealthItem("遥控器（UART1）"),
            "dt35": HealthItem("DT35"),
            "main": HealthItem("上层主控"),
            "imu": HealthItem("IMU"),
            "fdcan1": HealthItem("FDCAN1"),
            "safety": HealthItem("系统安全"),
        }
        for item in self.health_items.values():
            layout.addWidget(item)
        self.health_timestamp = QtWidgets.QLabel("尚未收到健康帧")
        self.health_timestamp.setObjectName("mutedLabel")
        layout.addWidget(self.health_timestamp)
        self.can_diagnostics = QtWidgets.QLabel("CAN3 diagnostics unavailable")
        self.can_diagnostics.setObjectName("metricValue")
        self.can_diagnostics.setWordWrap(True)
        layout.addWidget(self.can_diagnostics)
        dual_mcu_group = QtWidgets.QGroupBox("DUAL MCU RX")
        dual_mcu_layout = QtWidgets.QVBoxLayout(dual_mcu_group)
        self.dual_mcu_diagnostics = QtWidgets.QLabel("Waiting for chassis diagnostic frame")
        self.dual_mcu_diagnostics.setObjectName("metricValue")
        self.dual_mcu_diagnostics.setWordWrap(True)
        dual_mcu_layout.addWidget(self.dual_mcu_diagnostics)
        layout.addWidget(dual_mcu_group)
        layout.addStretch()
        return page

    @staticmethod
    def _load_serial_baudrate(config_path: Path) -> int:
        try:
            with config_path.open(encoding="utf-8") as config_file:
                baudrate = int(json.load(config_file).get("serial_baudrate", 921600))
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return 921600
        return baudrate if baudrate > 0 else 921600

    @staticmethod
    def _spin(minimum: float, maximum: float, value: float,
              suffix: str) -> QtWidgets.QDoubleSpinBox:
        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(2)
        spin.setSingleStep(0.05)
        spin.setValue(value)
        spin.setSuffix(suffix)
        return spin

    def _refresh_ports(self) -> None:
        current = self.port_combo.currentText()
        ports = self.serial.ports()
        if ports == [self.port_combo.itemText(i) for i in range(self.port_combo.count())]:
            return
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current in ports:
            self.port_combo.setCurrentText(current)

    def _connect_serial(self) -> None:
        if self.port_combo.currentText():
            self.serial.open(self.port_combo.currentText(), int(self.baud_combo.currentText()))

    def _connection_changed(self, connected: bool, message: str) -> None:
        if not connected:
            self._set_drive_enabled(False)
        self.connection_badge.setText("UART CONNECTED" if connected else "UART OFFLINE")
        self.connection_badge.setProperty("fault", not connected)
        self.connection_badge.style().unpolish(self.connection_badge)
        self.connection_badge.style().polish(self.connection_badge)
        self.statusBar().showMessage(message)

    def _apply_limits(self) -> None:
        frame = encode_motion_limits(
            self.linear_spin.value(), self.angular_spin.value(), self.accel_spin.value())
        if self.serial.write(frame):
            self.tx_frames += 1
            self.statusBar().showMessage("运动限制已下发到主控 RAM")
        else:
            self.statusBar().showMessage("UART 未连接")

    def _apply_yaw_pid(self) -> None:
        controls = self.yaw_pid_profile_controls[YAW_PID_PROFILE_QE]
        self._apply_yaw_pid_profile(YAW_PID_PROFILE_QE, "HEADING HOLD PID", controls)

    def _apply_yaw_pid_profile(self, profile: int, title: str,
                               controls: dict[str, QtWidgets.QDoubleSpinBox]) -> None:
        frame = encode_yaw_pid_profile_params(
            profile, controls["kp"].value(), controls["ki"].value(),
            controls["kd"].value(), controls["max"].value(),
            controls["i_limit"].value(), controls["i_sep"].value(),
            controls["deadzone"].value())
        if self.serial.write(frame):
            self.tx_frames += 1
            self.statusBar().showMessage(f"{title} updated")

    def _apply_str_pid(self) -> None:
        frame = encode_str_pid_params(
            self.str_kp_spin.value(), self.str_ki_spin.value(),
            self.str_kd_spin.value(), self.str_max_output_spin.value(),
            self.str_integral_limit_spin.value(),
            self.str_integral_separation_spin.value(), self.str_deadzone_spin.value())
        if self.serial.write(frame):
            self.tx_frames += 1
            self.statusBar().showMessage("CHASSIS STRPID updated")

    def _send_yaw_pid_params(self, kp: float, ki: float, kd: float,
                             max_output: float, integral_limit: float,
                             integral_separation: float, deadzone: float,
                             success_message: str) -> bool:
        frame = encode_yaw_pid_params(
            kp, ki, kd, max_output, integral_limit, integral_separation, deadzone)
        if self.serial.write(frame):
            self.tx_frames += 1
            self.statusBar().showMessage(success_message)
            return True
        return False

    def _apply_yaw_pid_preset(self, preset: str, force: bool = False) -> bool:
        profile = {"90": YAW_PID_PROFILE_90, "QE": YAW_PID_PROFILE_QE,
                   "180": YAW_PID_PROFILE_180}[preset]
        if not force and self.yaw_pid_preset == preset:
            return True
        if self.serial.write(encode_yaw_pid_profile(profile)):
            self.tx_frames += 1
            self.statusBar().showMessage(f"Firmware YAW PID {preset} profile selected")
            self.yaw_pid_preset = preset
            return True
        return False

    @staticmethod
    def _normalize_yaw_deg(yaw_deg: float) -> float:
        return (yaw_deg + 180.0) % 360.0 - 180.0

    def _turn_relative_90(self, delta_deg: float) -> None:
        if not self.serial.connected:
            self.statusBar().showMessage("Connect UART before using 90 degree rotation")
            return
        if not self.drive_enabled:
            self.statusBar().showMessage("Enable chassis before using 90 degree rotation")
            return
        if self.latest_yaw_deg is None:
            self.statusBar().showMessage("Waiting for a valid chassis YAW state")
            return
        if not self.yaw_pid_enable_check.isChecked():
            self.statusBar().showMessage("Enable YAW position PID before using 90 degree rotation")
            return

        self.yaw_target_deg = self._normalize_yaw_deg(self.latest_yaw_deg + delta_deg)
        self.yaw_target_spin.setValue(self.yaw_target_deg)
        self.yaw_target_initialized = True
        if not self._apply_yaw_pid_preset(
                "180" if abs(delta_deg) == 180.0 else "90", force=True):
            return
        self._send_yaw_target()

    def _send_yaw_pid_mode(self) -> None:
        if not hasattr(self, "yaw_pid_enable_check"):
            return
        if self.serial.connected and self.serial.write(
                encode_yaw_pid_mode(self.yaw_pid_enable_check.isChecked())):
            self.tx_frames += 1
            state_text = "enabled" if self.yaw_pid_enable_check.isChecked() else "disabled"
            self.statusBar().showMessage(f"YAW position PID {state_text} (IMU)")

    def _send_yaw_target_from_input(self) -> None:
        self.yaw_target_deg = self.yaw_target_spin.value()
        self.yaw_target_initialized = True
        self._send_yaw_target()

    def _send_yaw_target(self) -> None:
        if self.serial.connected and self.serial.write(encode_yaw_target(self.yaw_target_deg)):
            self.tx_frames += 1

    def _step_yaw_target(self, delta_deg: float) -> bool:
        if not self.yaw_target_initialized:
            return False
        self.yaw_target_deg = self._normalize_yaw_deg(self.yaw_target_deg + delta_deg)
        self.yaw_target_spin.setValue(self.yaw_target_deg)
        self._send_yaw_target()
        return True

    def _manual_velocity(self) -> tuple[float, float, float]:
        if not self.drive_enabled:
            return 0.0, 0.0, 0.0
        speed = self.linear_spin.value()
        omega = self.angular_spin.value()
        active_keys = self.button_keys if self.button_keys else self.pressed_keys
        vx = speed * ((QtCore.Qt.Key_W in active_keys) -
                      (QtCore.Qt.Key_S in active_keys))
        # These are world-frame axes. Firmware converts them to body-frame
        # velocity using its current yaw feedback.
        vy = speed * ((QtCore.Qt.Key_A in active_keys) -
                      (QtCore.Qt.Key_D in active_keys))
        wz = omega * ((QtCore.Qt.Key_Q in active_keys) -
                      (QtCore.Qt.Key_E in active_keys))
        magnitude = math.hypot(vx, vy)
        if magnitude > speed:
            vx, vy = vx * speed / magnitude, vy * speed / magnitude
        return vx, vy, wz

    def _start_s_curve(self, name: str) -> None:
        if not self.drive_enabled or not self.serial.connected:
            self.statusBar().showMessage("Enable the chassis and connect UART first")
            return
        self.pressed_keys.clear()
        self.button_keys.clear()
        self.serial.write(encode_manual(0.0, 0.0, 0.0))
        if self.serial.write(encode_dt35_sequence(S_CURVE_SEQUENCES[name])):
            self.tx_frames += 1
            self.statusBar().showMessage(f"{name} DT35 S curve started")

    def _send_dt35_direction(self, direction: int) -> None:
        if not self.drive_enabled or not self.serial.connected:
            self.statusBar().showMessage("Enable the chassis and connect UART first")
            return
        self.pressed_keys.clear()
        self.button_keys.clear()
        self.serial.write(encode_manual(0.0, 0.0, 0.0))
        if self.serial.write(encode_dt35_direction(direction)):
            self.tx_frames += 1
            self.statusBar().showMessage("DT35 position command started")

    def _cancel_s_curve(self) -> None:
        self.s_curve_name = None
        self.s_curve_step = 0
        self.s_curve_elapsed_ms = 0

    def _advance_s_curve(self) -> None:
        if self.s_curve_name is None:
            return
        self.s_curve_elapsed_ms += 20
        if self.s_curve_elapsed_ms < int(self.s_curve_segment_spin.value() * 1000.0):
            return
        self.s_curve_elapsed_ms = 0
        self.s_curve_step += 1
        if self.s_curve_step >= len(S_CURVE_SEQUENCES[self.s_curve_name]):
            finished = self.s_curve_name
            self._cancel_s_curve()
            self.statusBar().showMessage(f"{finished} S curve complete")

    def _send_manual_keepalive(self) -> None:
        vx, vy, omega = self._manual_velocity()
        yaw_pid_enabled = self.yaw_pid_enable_check.isChecked()
        active_keys = self.button_keys if self.button_keys else self.pressed_keys
        if self.drive_enabled and yaw_pid_enabled and active_keys & {QtCore.Qt.Key_Q, QtCore.Qt.Key_E}:
            direction = ((QtCore.Qt.Key_Q in active_keys)
                         - (QtCore.Qt.Key_E in active_keys))
            if direction:
                self.qe_step_elapsed_ms += 20
                if self.qe_step_elapsed_ms >= YAW_QE_STEP_PERIOD_MS:
                    self._step_yaw_target(direction * YAW_QE_STEP_DEG)
                    self.qe_step_elapsed_ms = 0
            else:
                self.qe_step_elapsed_ms = 0
        else:
            self.qe_step_elapsed_ms = 0
        self.velocity_label.setText(
            f"world vx {vx:+.2f}   vy {vy:+.2f}   omega {omega:+.2f}")
        command_omega = 0.0 if yaw_pid_enabled else omega
        if self.serial.connected and self.serial.write(encode_manual(vx, vy, command_omega)):
            self.tx_frames += 1
        self._advance_s_curve()

    def _send_command(self, command: str) -> bool:
        sent = self.serial.write(encode_command(command))
        if sent:
            self.tx_frames += 1
        return sent

    def _set_drive_enabled(self, enabled: bool) -> None:
        if enabled and not self.serial.connected:
            self.enable_button.blockSignals(True)
            self.enable_button.setChecked(False)
            self.enable_button.blockSignals(False)
            self.statusBar().showMessage("请先连接 UART")
            return
        self.drive_enabled = enabled
        self._cancel_s_curve()
        if not enabled:
            self.yaw_target_initialized = False
        self.enable_button.setText("底盘已使能" if enabled else "使能底盘")
        self.enable_button.setProperty("armed", enabled)
        self.enable_button.style().unpolish(self.enable_button)
        self.enable_button.style().polish(self.enable_button)
        self.pressed_keys.clear()
        self.button_keys.clear()
        if enabled:
            self._send_yaw_pid_mode()
            self._send_command("RESET")
            self._send_command("START")
            if self.yaw_target_initialized:
                self._send_yaw_target()
            self.statusBar().showMessage("底盘控制已使能")
        elif self.serial.connected:
            if self.serial.write(encode_manual(0.0, 0.0, 0.0)):
                self.tx_frames += 1
            self._send_command("STOP")

    def _set_control_key(self, key: int, pressed: bool) -> None:
        if pressed:
            self._cancel_s_curve()
        qe_was_pressed = bool(self.pressed_keys & {QtCore.Qt.Key_Q, QtCore.Qt.Key_E})
        key_was_pressed = key in self.pressed_keys
        if pressed:
            self.pressed_keys.add(key)
        else:
            self.pressed_keys.discard(key)
        qe_is_pressed = bool(self.pressed_keys & {QtCore.Qt.Key_Q, QtCore.Qt.Key_E})
        if (key in {QtCore.Qt.Key_Q, QtCore.Qt.Key_E}
                and self.drive_enabled and self.yaw_pid_enable_check.isChecked()):
            if not qe_was_pressed and qe_is_pressed:
                self._apply_yaw_pid_preset("QE")
            if pressed and not key_was_pressed:
                self.qe_step_elapsed_ms = 0
                self._step_yaw_target(
                    YAW_QE_STEP_DEG if key == QtCore.Qt.Key_Q else -YAW_QE_STEP_DEG)
            elif qe_was_pressed and not qe_is_pressed:
                self._apply_yaw_pid_preset("90")
                self.qe_step_elapsed_ms = 0

    def _set_button_control(self, key: int, pressed: bool) -> None:
        """Button input overrides keyboard input until every button is released."""
        qe_was_pressed = bool(self.button_keys & {QtCore.Qt.Key_Q, QtCore.Qt.Key_E})
        key_was_pressed = key in self.button_keys
        if pressed:
            self._cancel_s_curve()
            self.button_keys.add(key)
        else:
            self.button_keys.discard(key)
        qe_is_pressed = bool(self.button_keys & {QtCore.Qt.Key_Q, QtCore.Qt.Key_E})
        if (key in {QtCore.Qt.Key_Q, QtCore.Qt.Key_E}
                and self.drive_enabled and self.yaw_pid_enable_check.isChecked()):
            if not qe_was_pressed and qe_is_pressed:
                self._apply_yaw_pid_preset("QE")
            if pressed and not key_was_pressed:
                self.qe_step_elapsed_ms = 0
                self._step_yaw_target(
                    YAW_QE_STEP_DEG if key == QtCore.Qt.Key_Q else -YAW_QE_STEP_DEG)
            elif qe_was_pressed and not qe_is_pressed:
                self._apply_yaw_pid_preset("90")
                self.qe_step_elapsed_ms = 0

    def _stop_motion(self) -> None:
        self.pressed_keys.clear()
        self.button_keys.clear()
        self._cancel_s_curve()
        if self.serial.connected and self.serial.write(encode_manual(0.0, 0.0, 0.0)):
            self.tx_frames += 1

    def _estop(self) -> None:
        self.pressed_keys.clear()
        self.button_keys.clear()
        self._cancel_s_curve()
        self.drive_enabled = False
        self.yaw_target_initialized = False
        self.enable_button.blockSignals(True)
        self.enable_button.setChecked(False)
        self.enable_button.blockSignals(False)
        self.enable_button.setText("使能底盘")
        if self.serial.connected and self.serial.write(encode_manual(0.0, 0.0, 0.0)):
            self.tx_frames += 1
        self._send_command("ESTOP")
        self.statusBar().showMessage("急停已发送")

    def keyPressEvent(self, event) -> None:
        key = event.key()
        if key == QtCore.Qt.Key_Space and not event.isAutoRepeat():
            self._estop()
            event.accept()
            return
        if key in CONTROL_KEYS and not event.isAutoRepeat():
            self._set_control_key(key, True)
            event.accept()
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event) -> None:
        if event.key() in CONTROL_KEYS and not event.isAutoRepeat():
            self._set_control_key(event.key(), False)
            event.accept()
            return
        super().keyReleaseEvent(event)

    def _receive_bytes(self, data: bytes) -> None:
        for frame in self.parser.feed(data):
            self.rx_frames += 1
            self.last_rx_elapsed_ms = self.link_clock.elapsed()
            try:
                if frame.frame_id == ID_STATE:
                    state = decode_state(frame.payload)
                    self.latest_yaw_deg = state.yaw_deg
                    if not self.yaw_target_initialized:
                        self.yaw_target_deg = state.yaw_deg
                        self.yaw_target_spin.setValue(self.yaw_target_deg)
                        self.yaw_target_initialized = True
                    self.status_widget.update_state(state)
                    self.statusBar().showMessage(
                        f"位置 x={state.position_x_m:.2f} y={state.position_y_m:.2f} "
                        f"yaw={state.yaw_deg:.1f} deg")
                elif frame.frame_id == ID_HEALTH:
                    self._update_health(decode_health(frame.payload))
                elif frame.frame_id == ID_DUAL_MCU_DIAGNOSTIC:
                    self._update_dual_mcu_diagnostic(decode_dual_mcu_diagnostic(frame.payload))
            except ValueError as error:
                self.statusBar().showMessage(str(error))

    def _update_health(self, health) -> None:
        remote_online = health.online(4)
        remote_armed = health.online(5)
        remote_fault = health.online(6)
        self.health_items["pc"].set_state(health.online(0))
        self.health_items["remote"].set_state(
            remote_online and not remote_fault,
            ("未收到有效遥控帧" if not remote_online else
             "遥控故障标志有效" if remote_fault else
             "已收到有效遥控帧"))
        self.health_items["dt35"].set_state(health.online(1))
        self.health_items["main"].set_state(health.online(2))
        self.health_items["imu"].set_state(health.online(3))
        feedback_age_ms = health.timestamp_ms - health.motor_last_feedback_ms
        fdcan1_error = bool(health.can_error_mask & (1 << 0))
        fdcan1_ok = (health.motor_feedback_mask == 0x0F
                     and health.motor_last_feedback_ms != 0
                     and 0 <= feedback_age_ms < 500
                     and not fdcan1_error)
        self.health_items["fdcan1"].set_state(
            fdcan1_ok,
            f"feedback=0x{health.motor_feedback_mask:01X}, age={feedback_age_ms} ms, "
            f"error={'yes' if fdcan1_error else 'no'}")
        fault = bool(health.system_events & ((1 << 2) | (1 << 11))) or bool(health.can_error_mask)
        self.health_items["safety"].set_state(not fault, "急停/CAN 故障" if fault else "正常")
        self.health_timestamp.setText(f"主控时间：{health.timestamp_ms} ms")
        command = {0: "NONE", 1: "START", 2: "STOP", 3: "RESET", 4: "ESTOP"}.get(
            health.pc_command, str(health.pc_command))
        erpm = " / ".join(f"{value:+.0f}" for value in health.motor_target_erpm)
        wheel_speeds = " / ".join(
            f"M{index + 1} {speed:+.3f}" for index, speed
            in enumerate(health.motor_measured_speed_mps))
        erpm_active = any(abs(value) >= 1.0 for value in health.motor_target_erpm)
        estop_active = bool(health.system_events & (1 << 2))
        can_fault_active = bool(health.system_events & (1 << 11))
        if estop_active:
            drive_state = "BLOCKED: ESTOP active"
        elif can_fault_active:
            drive_state = "BLOCKED: CAN fault active"
        elif command != "START":
            drive_state = "BLOCKED: command is not START"
        elif not erpm_active:
            drive_state = "NO MOTOR COMMAND: target ERPM is zero"
        elif not fdcan1_ok:
            drive_state = "COMMAND SENT: waiting for FDCAN1 motor feedback"
        else:
            drive_state = "COMMAND AND FDCAN1 FEEDBACK OK"
        self.can_diagnostics.setText(
            f"CAN1  CMD={command}  EVENTS=0x{health.system_events:08X}  "
            f"ERR=0x{health.can_error_mask:08X}\n"
            f"Remote UART1: {'ONLINE' if remote_online else 'OFFLINE'}  "
            f"ARMED={'YES' if remote_armed else 'NO'}  "
            f"FAULT={'YES' if remote_fault else 'NO'}\n"
            f"Motor feedback=0x{health.motor_feedback_mask:01X}  age={feedback_age_ms} ms  "
            f"TX ok/fail={health.motor_tx_ok_count}/{health.motor_tx_failed_count}\n"
            f"Target ERPM: {erpm}\n"
            f"Measured wheel speed (m/s): {wheel_speeds}\n"
            f"Drive status: {drive_state}")

    def _update_dual_mcu_diagnostic(self, diagnostic) -> None:
        freshness_text = {
            0: "NO FRAME",
            1: "ACTIVE",
            2: "STOP WINDOW",
            3: "OFFLINE",
        }.get(diagnostic.freshness, f"UNKNOWN ({diagnostic.freshness})")
        age_text = "--" if diagnostic.last_valid_rx_ms == 0 else str(max(
            0, diagnostic.timestamp_ms - diagnostic.last_valid_rx_ms))
        self.dual_mcu_diagnostics.setText(
            f"State: {freshness_text}  last valid age: {age_text} ms\n"
            f"UART8 RX bytes: {diagnostic.rx_byte_count}  valid/rejected: "
            f"{diagnostic.valid_frame_count}/{diagnostic.rejected_frame_count}\n"
            f"SEQ: {diagnostic.sequence}  mode: {diagnostic.control_mode}  "
            f"action: {diagnostic.action_id}  flags: 0x{diagnostic.flags:02X}\n"
            f"Left stick: X={diagnostic.joy_left_x} Y={diagnostic.joy_left_y}  "
            f"Right stick: X={diagnostic.joy_right_x} Y={diagnostic.joy_right_y}\n"
            f"Shoulders: left={diagnostic.shoulder_left} right={diagnostic.shoulder_right}")

    def _update_link_status(self) -> None:
        age = (-1 if self.last_rx_elapsed_ms < 0
               else self.link_clock.elapsed() - self.last_rx_elapsed_ms)
        telemetry = "无遥测" if age < 0 else f"遥测 {age} ms 前"
        self.link_detail.setText(f"RX {self.rx_frames}  |  TX {self.tx_frames}  |  {telemetry}")
        healthy = self.serial.connected and 0 <= age < 1000
        if self.serial.connected:
            self.connection_badge.setText("LINK ONLINE" if healthy else "UART CONNECTED")
            self.connection_badge.setProperty("fault", not healthy)
            self.connection_badge.style().unpolish(self.connection_badge)
            self.connection_badge.style().polish(self.connection_badge)

    def focusOutEvent(self, event) -> None:
        self._stop_motion()
        super().focusOutEvent(event)

    def closeEvent(self, event) -> None:
        if self.serial.connected:
            self._stop_motion()
            self._send_command("STOP")
        self.serial.close()
        super().closeEvent(event)
