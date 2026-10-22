"""连接、轨迹控制和键盘手动调试面板。"""
from __future__ import annotations

from PyQt5 import QtCore, QtGui, QtWidgets


class ControlWidget(QtWidgets.QFrame):
    connect_requested = QtCore.pyqtSignal(str, int)
    disconnect_requested = QtCore.pyqtSignal()
    load_trajectory_requested = QtCore.pyqtSignal()
    check_requested = QtCore.pyqtSignal()
    send_trajectory_requested = QtCore.pyqtSignal()
    command_requested = QtCore.pyqtSignal(str)
    manual_velocity = QtCore.pyqtSignal(float, float, float)
    save_csv_requested = QtCore.pyqtSignal()
    edit_mode_changed = QtCore.pyqtSignal(bool)
    field_size_changed = QtCore.pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent); self.setObjectName("controlPanel"); self.setFocusPolicy(QtCore.Qt.StrongFocus)
        layout = QtWidgets.QVBoxLayout(self); layout.setContentsMargins(14, 12, 14, 12); layout.setSpacing(9)
        title = QtWidgets.QLabel("DEBUG CONTROLS"); title.setObjectName("sectionTitle"); layout.addWidget(title)
        connection = QtWidgets.QGroupBox("UART CONNECTION"); form = QtWidgets.QFormLayout(connection)
        self.port_combo = QtWidgets.QComboBox(); self.port_combo.setEditable(True); self.port_combo.addItems(["未连接"])
        self.baud_combo = QtWidgets.QComboBox(); self.baud_combo.addItems(["921600", "115200", "460800"])
        form.addRow("端口", self.port_combo); form.addRow("波特率", self.baud_combo)
        self.connect_button = QtWidgets.QPushButton("连接 UART"); self.connect_button.clicked.connect(self._toggle_connection); form.addRow(self.connect_button)
        layout.addWidget(connection)
        field = QtWidgets.QGroupBox("FIELD CONFIG"); field_form = QtWidgets.QFormLayout(field)
        self.field_width_spin = QtWidgets.QDoubleSpinBox(); self.field_width_spin.setRange(1.0, 100.0); self.field_width_spin.setValue(11.2); self.field_width_spin.setSuffix(" m")
        self.field_height_spin = QtWidgets.QDoubleSpinBox(); self.field_height_spin.setRange(1.0, 100.0); self.field_height_spin.setValue(6.1); self.field_height_spin.setSuffix(" m")
        field_form.addRow("宽度 X", self.field_width_spin); field_form.addRow("高度 Y", self.field_height_spin)
        apply_field = QtWidgets.QPushButton("应用地图尺寸"); apply_field.clicked.connect(lambda: self.field_size_changed.emit(self.field_width_spin.value(), self.field_height_spin.value())); field_form.addRow(apply_field); layout.addWidget(field)
        path = QtWidgets.QGroupBox("TRAJECTORY"); path_layout = QtWidgets.QVBoxLayout(path)
        self.file_label = QtWidgets.QLabel("未加载轨迹"); self.file_label.setObjectName("mutedLabel"); path_layout.addWidget(self.file_label)
        row = QtWidgets.QHBoxLayout(); self.load_button = QtWidgets.QPushButton("加载轨迹"); self.load_button.clicked.connect(self.load_trajectory_requested); self.edit_button = QtWidgets.QPushButton("编辑控制点"); self.edit_button.setCheckable(True); self.edit_button.toggled.connect(self.edit_mode_changed); row.addWidget(self.load_button); row.addWidget(self.edit_button); path_layout.addLayout(row)
        row = QtWidgets.QHBoxLayout(); self.check_button = QtWidgets.QPushButton("安全检测"); self.check_button.clicked.connect(self.check_requested); self.send_button = QtWidgets.QPushButton("发送到 STM32"); self.send_button.clicked.connect(self.send_trajectory_requested); row.addWidget(self.check_button); row.addWidget(self.send_button); path_layout.addLayout(row)
        layout.addWidget(path)
        command = QtWidgets.QGroupBox("COMMAND"); command_layout = QtWidgets.QGridLayout(command)
        for col, name in enumerate(("START", "STOP", "RESET", "ESTOP")):
            button = QtWidgets.QPushButton(name); button.setObjectName("commandButton"); button.clicked.connect(lambda _, value=name: self.command_requested.emit(value)); command_layout.addWidget(button, 0, col)
        layout.addWidget(command)
        manual = QtWidgets.QGroupBox("MANUAL VELOCITY"); manual_form = QtWidgets.QFormLayout(manual)
        self.vx_spin = self._speed_spin(); self.vy_spin = self._speed_spin(); self.omega_spin = self._omega_spin()
        manual_form.addRow("VX", self.vx_spin); manual_form.addRow("VY", self.vy_spin); manual_form.addRow("OMEGA", self.omega_spin)
        apply_button = QtWidgets.QPushButton("发送手动速度"); apply_button.clicked.connect(self._emit_manual); manual_form.addRow(apply_button)
        hint = QtWidgets.QLabel("W/S 前后 · A/D 横移 · Q/E 旋转 · SPACE 急停"); hint.setObjectName("mutedLabel"); hint.setWordWrap(True); manual_form.addRow(hint)
        layout.addWidget(manual)
        self.save_button = QtWidgets.QPushButton("保存实时数据 CSV"); self.save_button.clicked.connect(self.save_csv_requested); layout.addWidget(self.save_button)
        layout.addStretch(1)
        self._keys = set()

    def set_ports(self, ports: list[str]) -> None:
        current = self.port_combo.currentText(); self.port_combo.clear(); self.port_combo.addItems(ports or ["未连接"]); self.port_combo.setCurrentText(current if current in ports else self.port_combo.itemText(0))

    def set_trajectory_name(self, name: str) -> None: self.file_label.setText(name)

    def set_connection_state(self, connected: bool, message: str = "") -> None:
        self.connect_button.setText("断开 UART" if connected else "连接 UART"); self.connect_button.setProperty("connected", connected); self.connect_button.style().unpolish(self.connect_button); self.connect_button.style().polish(self.connect_button)
        if message: self.file_label.setToolTip(message)

    def _toggle_connection(self) -> None:
        if self.connect_button.text().startswith("断"):
            self.disconnect_requested.emit()
        else:
            self.connect_requested.emit(self.port_combo.currentText(), int(self.baud_combo.currentText()))

    def _emit_manual(self) -> None: self.manual_velocity.emit(self.vx_spin.value(), self.vy_spin.value(), self.omega_spin.value())

    @staticmethod
    def _speed_spin():
        spin = QtWidgets.QDoubleSpinBox(); spin.setRange(-2.0, 2.0); spin.setSingleStep(.1); spin.setDecimals(2); spin.setSuffix(" m/s"); return spin

    @staticmethod
    def _omega_spin():
        spin = QtWidgets.QDoubleSpinBox(); spin.setRange(-230.0, 230.0); spin.setSingleStep(5.0); spin.setDecimals(1); spin.setSuffix(" deg/s"); return spin

    def keyPressEvent(self, event: QtGui.QKeyEvent) -> None:
        key = event.key()
        mapping = {QtCore.Qt.Key_W: (1, 0, 0), QtCore.Qt.Key_S: (-1, 0, 0), QtCore.Qt.Key_A: (0, 1, 0), QtCore.Qt.Key_D: (0, -1, 0), QtCore.Qt.Key_Q: (0, 0, 1), QtCore.Qt.Key_E: (0, 0, -1)}
        if key == QtCore.Qt.Key_Space:
            self.command_requested.emit("ESTOP"); event.accept(); return
        if key in mapping and key not in self._keys:
            self._keys.add(key); vx, vy, omega = mapping[key]; self.manual_velocity.emit(vx * .8, vy * .8, omega * 1.4)
        event.accept()

    def keyReleaseEvent(self, event: QtGui.QKeyEvent) -> None:
        if event.key() in self._keys:
            self._keys.remove(event.key()); self.manual_velocity.emit(0.0, 0.0, 0.0)
        event.accept()
