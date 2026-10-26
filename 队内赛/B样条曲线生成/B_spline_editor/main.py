"""Robocon B-spline path editor entry point."""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
# PyQtGraph must use the same Qt binding as the rest of this application.
# Set this before importing pyqtgraph to prevent accidental PySide/PyQt6 mixing.
import os
os.environ["PYQTGRAPH_QT_LIB"] = "PyQt5"
import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets

from bspline.curve import BSplineCurve
from config.defaults import (
    RED_START_ORIGIN_PIXEL,
    FIELD_HEIGHT_MM,
    FIELD_IMAGE_CROP,
    FIELD_IMAGE_PATH,
    FIELD_WIDTH_MM,
    FORWARD_AXIS_ANGLE,
)
from config.path_config import (
    CONFIG_FORMAT,
    CONFIG_VERSION,
    DEFAULT_CONFIG_DIRECTORY,
    PathConfigError,
    load_path_config,
    save_path_config,
)
from config.coordinate_system import CoordinateSystem, DIRECTIONS
from path.exporter import export_c_array, export_mpc_c_array, export_mpc_csv
from path.mpc_generator import (
    DEFAULT_SAMPLE_TIME,
    MAX_SAMPLE_TIME,
    MIN_SAMPLE_TIME,
    generate_mpc_states,
)
from path.path_generator import PathPoint, generate_path, yaw_from_heading

class CentimeterAxis(pg.AxisItem):
    """Display the millimetre world coordinates as centimetres on plot axes."""

    def tickStrings(self, values, scale, spacing):
        return [f"{value / 10.0:g}" for value in values]

class PathViewBox(pg.ViewBox):
    """ViewBox which turns mouse actions into control point edits."""

    def __init__(self, editor, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.editor = editor
        self.drag_index = None
        self.origin_drag_start = None
        self.origin_start_field = None
        self.setMouseEnabled(x=False, y=False)

    def _world_position(self, event):
        position = self.mapSceneToView(event.scenePos())
        return float(position.x()), float(position.y())

    def mouseClickEvent(self, event):
        x, y = self._world_position(event)
        if event.button() == QtCore.Qt.LeftButton and self.editor.near_origin(x, y):
            event.accept()
            return
        if event.button() == QtCore.Qt.RightButton:
            index = self.editor.nearest_control_point(x, y)
            if index is not None:
                self.editor.delete_control_point(index)
            event.accept()
            return
        if event.button() == QtCore.Qt.LeftButton:
            index = self.editor.nearest_control_point(x, y)
            if index is None:
                self.editor.add_control_point(x, y)
            event.accept()
            return
        event.ignore()

    def mouseDragEvent(self, event, axis=None):
        if event.button() == QtCore.Qt.MiddleButton:
            super().mouseDragEvent(event, axis=axis)
            return
        if event.button() != QtCore.Qt.LeftButton:
            event.ignore()
            return
        x, y = self._world_position(event)
        if event.isStart():
            if self.editor.origin_edit_mode.isChecked() or self.editor.near_origin(x, y):
                self.origin_drag_start = (x, y)
                self.origin_start_field = self.editor.coordinate_system.origin_point(
                    self.editor.field_width, self.editor.field_height
                )
                event.accept()
                return
            self.drag_index = self.editor.nearest_control_point(x, y)
            if self.drag_index is None:
                self.drag_index = self.editor.add_control_point(x, y)
        if self.origin_drag_start is not None:
            self.editor.move_origin_by_display_delta(
                x - self.origin_drag_start[0], y - self.origin_drag_start[1], self.origin_start_field
            )
            if event.isFinish():
                self.origin_drag_start = None
                self.origin_start_field = None
            event.accept()
            return
        if self.drag_index is not None:
            self.editor.move_control_point(self.drag_index, x, y)
        if event.isFinish():
            self.drag_index = None
        event.accept()

    def wheelEvent(self, event, axis=None):
        if axis is not None:
            event.ignore()
            return
        position = self.mapSceneToView(event.scenePos())
        factor = 1.15 if event.delta() > 0 else 1.0 / 1.15
        self.scaleBy((factor, factor), center=position)
        event.accept()


class PathEditor(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robocon B-Spline Path Editor")
        self.resize(1280, 800)
        self.field_width = FIELD_WIDTH_MM
        self.field_height = FIELD_HEIGHT_MM
        self.image_origin_x = 0.0
        self.image_origin_y = 0.0
        self._set_default_image_origin()
        self.coordinate_system = CoordinateSystem()
        self.control_points: list[tuple[float, float]] = []
        self.path_points = []
        self.path_plot_points = []
        self.mpc_states = []
        self.background_item = None
        self._build_ui()
        self._update_plot()
        self._load_background(
            FIELD_IMAGE_PATH,
            show_status=False,
            crop=FIELD_IMAGE_CROP,
            origin=(self.image_origin_x, self.image_origin_y),
        )

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)
        layout.setContentsMargins(8, 8, 8, 8)

        controls = QtWidgets.QWidget()
        controls.setFixedWidth(310)
        control_layout = QtWidgets.QVBoxLayout(controls)
        control_layout.setContentsMargins(0, 0, 0, 0)

        image_group = QtWidgets.QGroupBox("场地设置")
        image_form = QtWidgets.QFormLayout(image_group)
        import_button = QtWidgets.QPushButton("导入场地图片")
        import_button.clicked.connect(self.import_background)
        self.width_spin = self._spin(100.0, 100000.0, self.field_width, 1.0)
        self.height_spin = self._spin(100.0, 100000.0, self.field_height, 1.0)
        self.width_spin.valueChanged.connect(self._set_field_size)
        self.height_spin.valueChanged.connect(self._set_field_size)
        image_form.addRow(import_button)
        image_form.addRow("场地宽度 X (mm)", self.width_spin)
        image_form.addRow("场地长度 Y (mm)", self.height_spin)

        coordinate_group = QtWidgets.QGroupBox("坐标系设置")
        coordinate_form = QtWidgets.QFormLayout(coordinate_group)
        self.origin_combo = QtWidgets.QComboBox()
        self.origin_combo.addItem("左下角", "bottom_left")
        self.origin_combo.addItem("左上角", "top_left")
        self.origin_combo.addItem("右下角", "bottom_right")
        self.origin_combo.addItem("右上角", "top_right")
        self.direction_combo = QtWidgets.QComboBox()
        direction_labels = ("X向右，Y向上", "X向右，Y向下", "X向左，Y向上", "X向左，Y向下")
        for label, (x_direction, y_direction) in zip(direction_labels, DIRECTIONS):
            self.direction_combo.addItem(label, (x_direction, y_direction))
        self.origin_combo.currentIndexChanged.connect(self._coordinate_system_changed)
        self.direction_combo.currentIndexChanged.connect(self._coordinate_system_changed)
        coordinate_form.addRow("坐标原点", self.origin_combo)
        coordinate_form.addRow("坐标轴方向", self.direction_combo)
        self.origin_x_spin = self._spin(0.0, self.field_width, 0.0, 1.0)
        self.origin_y_spin = self._spin(0.0, self.field_height, 0.0, 1.0)
        self.origin_x_spin.valueChanged.connect(self._origin_input_changed)
        self.origin_y_spin.valueChanged.connect(self._origin_input_changed)
        coordinate_form.addRow("原点场地 X", self.origin_x_spin)
        coordinate_form.addRow("原点场地 Y", self.origin_y_spin)
        self.origin_edit_mode = QtWidgets.QCheckBox("启用原点拖动模式")
        self.origin_edit_mode.setToolTip("开启后左键拖动视图即可移动坐标原点")
        coordinate_form.addRow(self.origin_edit_mode)
        self.origin_offset_label = QtWidgets.QLabel("微调偏移：X 0.0 mm，Y 0.0 mm")
        coordinate_form.addRow(self.origin_offset_label)

        path_group = QtWidgets.QGroupBox("路径设置")
        path_form = QtWidgets.QFormLayout(path_group)
        self.sample_mode = QtWidgets.QComboBox()
        self.sample_mode.addItem("按采样间隔", "interval")
        self.sample_mode.addItem("指定路径点数量", "count")
        self.sample_mode.currentIndexChanged.connect(self._update_sampling_controls)
        self.interval_spin = self._spin(1.0, 1000.0, 50.0, 1.0)
        self.interval_spin.setToolTip("相邻离散路径点的目标距离")
        self.interval_spin.valueChanged.connect(self._update_path)
        self.count_spin = QtWidgets.QSpinBox()
        self.count_spin.setRange(2, 10000)
        self.count_spin.setValue(200)
        self.count_spin.setToolTip("导出数组中的离散路径点数量")
        self.count_spin.valueChanged.connect(self._update_path)
        self.point_count_label = QtWidgets.QLabel("路径点: 0")
        path_form.addRow("采样方式", self.sample_mode)
        path_form.addRow("采样间隔 (mm)", self.interval_spin)
        path_form.addRow("路径点数量", self.count_spin)
        path_form.addRow(self.point_count_label)

        speed_group = QtWidgets.QGroupBox("速度规划")
        speed_form = QtWidgets.QFormLayout(speed_group)
        self.lateral_accel_spin = self._speed_spin(0.01, 100.0, 2.0, 0.1, " m/s^2")
        self.speed_scale_spin = self._speed_spin(0.0, 1.0, 1.0, 0.05, "")
        self.acceleration_spin = self._speed_spin(0.01, 100.0, 1.0, 0.1, " m/s^2")
        self.deceleration_spin = self._speed_spin(0.01, 100.0, 1.0, 0.1, " m/s^2")
        self.lateral_accel_spin.setToolTip("仅用于根据曲率限制路径速度")
        self.speed_scale_spin.setToolTip("按比例降低所有路径点的目标速度")
        self.acceleration_spin.setToolTip("从起点开始的纵向加速度上限")
        self.deceleration_spin.setToolTip("到终点前的纵向减速度上限")
        for spin in (
            self.lateral_accel_spin,
            self.speed_scale_spin,
            self.acceleration_spin,
            self.deceleration_spin,
        ):
            spin.valueChanged.connect(self._update_path)
        speed_form.addRow("横向加速度 a_max", self.lateral_accel_spin)
        speed_form.addRow("速度比例", self.speed_scale_spin)
        speed_form.addRow("起步加速度", self.acceleration_spin)
        speed_form.addRow("刹车减速度", self.deceleration_spin)

        mpc_group = QtWidgets.QGroupBox("MPC 状态量 (按时间采样)")
        mpc_layout = QtWidgets.QVBoxLayout(mpc_group)
        mpc_form = QtWidgets.QFormLayout()
        self.mpc_sample_spin = QtWidgets.QDoubleSpinBox()
        self.mpc_sample_spin.setRange(MIN_SAMPLE_TIME, MAX_SAMPLE_TIME)
        self.mpc_sample_spin.setDecimals(3)
        self.mpc_sample_spin.setSingleStep(0.005)
        self.mpc_sample_spin.setValue(DEFAULT_SAMPLE_TIME)
        self.mpc_sample_spin.setSuffix(" s")
        self.mpc_sample_spin.setToolTip(
            "MPC 参考轨迹的固定采样周期，默认 0.01 s；相邻状态的时间间隔"
        )
        self.mpc_sample_spin.valueChanged.connect(self._update_path)
        mpc_form.addRow("采样周期 dt", self.mpc_sample_spin)
        mpc_layout.addLayout(mpc_form)
        self.mpc_info_label = QtWidgets.QLabel("状态数: 0（至少需要 4 个控制点）")
        self.mpc_info_label.setWordWrap(True)
        self.mpc_show_check = QtWidgets.QCheckBox("在图中显示时间采样点")
        self.mpc_show_check.setChecked(True)
        self.mpc_show_check.toggled.connect(self._set_mpc_points_visible)
        self.mpc_copy_button = QtWidgets.QPushButton("复制 MPC 数组")
        self.mpc_header_button = QtWidgets.QPushButton("导出 MPC .h")
        self.mpc_csv_button = QtWidgets.QPushButton("导出 MPC .csv")
        self.mpc_copy_button.clicked.connect(self.copy_mpc_array)
        self.mpc_header_button.clicked.connect(self.save_mpc_header)
        self.mpc_csv_button.clicked.connect(self.save_mpc_csv)
        mpc_layout.addWidget(self.mpc_info_label)
        mpc_layout.addWidget(self.mpc_show_check)
        mpc_layout.addWidget(self.mpc_copy_button)
        mpc_layout.addWidget(self.mpc_header_button)
        mpc_layout.addWidget(self.mpc_csv_button)

        list_group = QtWidgets.QGroupBox("控制点 (机器人坐标 mm)")
        list_layout = QtWidgets.QVBoxLayout(list_group)
        self.point_list = QtWidgets.QTableWidget(0, 3)
        self.point_list.setHorizontalHeaderLabels(["序号", "X", "Y"])
        self.point_list.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.point_list.setEditTriggers(
            QtWidgets.QAbstractItemView.DoubleClicked | QtWidgets.QAbstractItemView.EditKeyPressed
        )
        self.point_list.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.point_list.itemSelectionChanged.connect(self._highlight_selected)
        self.point_list.cellChanged.connect(self._edit_control_point_from_table)
        clear_button = QtWidgets.QPushButton("清空控制点")
        clear_button.clicked.connect(self.clear_control_points)
        list_layout.addWidget(self.point_list)
        list_layout.addWidget(clear_button)

        export_group = QtWidgets.QGroupBox("导出")
        export_layout = QtWidgets.QVBoxLayout(export_group)
        save_config_button = QtWidgets.QPushButton("保存配置")
        open_config_button = QtWidgets.QPushButton("打开配置")
        copy_button = QtWidgets.QPushButton("复制 C 数组到剪贴板")
        save_button = QtWidgets.QPushButton("导出 .h 文件")
        save_config_button.clicked.connect(self.save_configuration)
        open_config_button.clicked.connect(self.open_configuration)
        copy_button.clicked.connect(self.copy_c_array)
        save_button.clicked.connect(self.save_c_array)
        export_layout.addWidget(save_config_button)
        export_layout.addWidget(open_config_button)
        export_layout.addWidget(copy_button)
        export_layout.addWidget(save_button)

        self.status_label = QtWidgets.QLabel("左键添加/拖动，右键删除控制点")
        self.status_label.setWordWrap(True)
        control_layout.addWidget(image_group)
        control_layout.addWidget(coordinate_group)
        control_layout.addWidget(path_group)
        control_layout.addWidget(speed_group)
        control_layout.addWidget(mpc_group)
        control_layout.addWidget(list_group, 1)
        control_layout.addWidget(export_group)
        control_layout.addWidget(self.status_label)

        self.view_box = PathViewBox(self, lockAspect=True, invertY=False)
        self.view_box.setMouseEnabled(x=True, y=True)
        axes = {
            "bottom": CentimeterAxis(orientation="bottom"),
            "left": CentimeterAxis(orientation="left"),
        }
        self.plot = pg.PlotWidget(viewBox=self.view_box, axisItems=axes)
        self.plot.setBackground("w")
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setLabel("bottom", "机器人 X", units="cm")
        self.plot.setLabel("left", "机器人 Y", units="cm")
        self.plot.addLegend(offset=(10, 10))
        self.curve_item = self.plot.plot([], [], pen=pg.mkPen("#1368ce", width=3), name="三阶 B 样条")
        self.mpc_item = pg.ScatterPlotItem(
            size=6, pen=None, brush=pg.mkBrush("#e67e22"), name="MPC 采样点"
        )
        self.mpc_item.setZValue(2)
        self.plot.addItem(self.mpc_item)
        self.control_item = pg.ScatterPlotItem(
            size=14, pen=pg.mkPen("#7a1f1f", width=1), brush=pg.mkBrush("#e74c3c"), name="控制点"
        )
        self.plot.addItem(self.control_item)
        # Keep the robot origin visible even when the field has display margins.
        origin_pen = pg.mkPen("#303030", width=1.5)
        self.robot_x_axis = pg.InfiniteLine(pos=0, angle=0, pen=origin_pen)
        self.robot_y_axis = pg.InfiniteLine(pos=0, angle=90, pen=origin_pen)
        self.robot_x_axis.setZValue(5)
        self.robot_y_axis.setZValue(5)
        self.plot.addItem(self.robot_x_axis)
        self.plot.addItem(self.robot_y_axis)
        self.origin_item = pg.ScatterPlotItem(
            size=18, pen=pg.mkPen("#111111", width=2), brush=pg.mkBrush("#f1c40f"),
        )
        self.origin_item.setAcceptedMouseButtons(QtCore.Qt.NoButton)
        self.origin_item.setData([0.0], [0.0])
        self.origin_item.setZValue(10)
        self.plot.addItem(self.origin_item)
        control_panel = QtWidgets.QScrollArea()
        control_panel.setWidgetResizable(True)
        control_panel.setWidget(controls)
        control_panel.setFixedWidth(328)
        control_panel.setFrameShape(QtWidgets.QFrame.NoFrame)
        control_panel.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        layout.addWidget(control_panel)
        layout.addWidget(self.plot, 1)
        self._update_sampling_controls()

    @staticmethod
    def _spin(minimum, maximum, value, step):
        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setValue(value)
        spin.setSingleStep(step)
        spin.setDecimals(1)
        spin.setSuffix(" mm")
        return spin

    @staticmethod
    def _speed_spin(minimum, maximum, value, step, suffix):
        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setValue(value)
        spin.setSingleStep(step)
        spin.setDecimals(2)
        spin.setSuffix(suffix)
        return spin

    @staticmethod
    def _combo_index(combo, value):
        """Find a combo entry by data, including tuples that findData misses."""
        index = combo.findData(value)
        if index >= 0:
            return index
        for candidate in range(combo.count()):
            if combo.itemData(candidate) == value:
                return candidate
        return -1

    def _set_field_size(self):
        old_width, old_height = self.field_width, self.field_height
        field_points = [
            self.coordinate_system.to_field(point, old_width, old_height)
            for point in self.control_points
        ]
        self.field_width = self.width_spin.value()
        self.field_height = self.height_spin.value()
        self.control_points = [
            self.coordinate_system.to_display(point, self.field_width, self.field_height)
            for point in field_points
        ]
        self._set_default_image_origin()
        self._update_origin_inputs()
        self._update_plot()

    def _origin_field_point(self):
        return self.coordinate_system.origin_point(self.field_width, self.field_height)

    def _origin_input_changed(self):
        if not hasattr(self, "origin_x_spin"):
            return
        self._set_origin_field_point(self.origin_x_spin.value(), self.origin_y_spin.value())

    def _set_origin_field_point(self, field_x, field_y):
        field_x = min(max(float(field_x), 0.0), self.field_width)
        field_y = min(max(float(field_y), 0.0), self.field_height)
        base_x, base_y = self.coordinate_system.base_origin(self.field_width, self.field_height)
        self.coordinate_system = CoordinateSystem(
            self.coordinate_system.origin,
            self.coordinate_system.x_direction,
            self.coordinate_system.y_direction,
            (field_x - base_x, field_y - base_y),
        )
        self._update_origin_inputs()
        self._update_origin_offset_label()
        self._update_plot()

    def _update_origin_inputs(self):
        if not hasattr(self, "origin_x_spin"):
            return
        origin_x, origin_y = self._origin_field_point()
        blockers = [QtCore.QSignalBlocker(self.origin_x_spin), QtCore.QSignalBlocker(self.origin_y_spin)]
        self.origin_x_spin.setRange(0.0, self.field_width)
        self.origin_y_spin.setRange(0.0, self.field_height)
        self.origin_x_spin.setValue(origin_x)
        self.origin_y_spin.setValue(origin_y)
        del blockers

    def _coordinate_system_changed(self):
        if not hasattr(self, "origin_combo"):
            return
        origin = self.origin_combo.currentData()
        direction = self.direction_combo.currentData()
        if origin is None or direction is None:
            return
        x_direction, y_direction = direction
        new_system = CoordinateSystem(origin, x_direction, y_direction, (0.0, 0.0))
        field_points = [
            self.coordinate_system.to_field(point, self.field_width, self.field_height)
            for point in self.control_points
        ]
        self.coordinate_system = new_system
        self.control_points = [
            self.coordinate_system.to_display(point, self.field_width, self.field_height)
            for point in field_points
        ]
        self._update_origin_inputs()
        self._update_plot()

    def near_origin(self, x, y):
        threshold = max(self.field_width, self.field_height) * 0.025
        return np.hypot(x, y) <= threshold

    def move_origin_by_display_delta(self, delta_x, delta_y, start_field):
        field_x = start_field[0] - delta_x
        field_y = start_field[1] - delta_y
        self._set_origin_field_point(field_x, field_y)

    def _update_origin_offset_label(self):
        if hasattr(self, "origin_offset_label"):
            offset_x, offset_y = self.coordinate_system.origin_offset
            self.origin_offset_label.setText(
                f"微调偏移：X {offset_x:.1f} mm，Y {offset_y:.1f} mm"
            )

    def _to_display(self, point):
        return self.coordinate_system.to_display(point, self.field_width, self.field_height)

    def _to_field(self, point):
        return self.coordinate_system.to_field(point, self.field_width, self.field_height)

    def _display_bounds(self):
        corners = (
            self._to_display((0.0, 0.0)),
            self._to_display((self.field_width, 0.0)),
            self._to_display((0.0, self.field_height)),
            self._to_display((self.field_width, self.field_height)),
        )
        xs, ys = zip(*corners)
        return min(xs), max(xs), min(ys), max(ys)

    def _background_rect(self):
        """将背景图的场地矩形转换到当前坐标系。"""
        left_bottom = self._to_display((-self.image_origin_x, -self.image_origin_y))
        top_left = self._to_display((-self.image_origin_x, self.field_height - self.image_origin_y))
        return QtCore.QRectF(left_bottom[0], top_left[1], self.field_width, -self.field_height)

    def _set_background_rect(self):
        """重置图像变换后设置背景图，避免坐标系切换时变换叠加。"""
        if self.background_item is None:
            return
        self.background_item.resetTransform()
        self.background_item.setRect(self._background_rect())

    def _set_default_image_origin(self):
        """将机器人坐标原点映射到红色启动区左下角。"""
        self.image_origin_x, self.image_origin_y = RED_START_ORIGIN_PIXEL

    def import_background(self):
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "选择比赛场地图片", "", "图片文件 (*.png *.jpg *.jpeg *.bmp)"
        )
        if not filename:
            return
        # A user-imported image starts with its bottom-left at robot (0, 0).
        self.image_origin_x = 0.0
        self.image_origin_y = 0.0
        self._load_background(Path(filename), origin=(0.0, 0.0))

    def _load_background(self, filename: Path, show_status=True, crop=None, origin=(0.0, 0.0)):
        """Load a field image and map it to the configured world dimensions."""
        image = QtGui.QImage(str(filename)).convertToFormat(QtGui.QImage.Format_RGBA8888)
        if image.isNull():
            if show_status:
                QtWidgets.QMessageBox.warning(self, "导入失败", "无法读取该图片文件。")
            else:
                self.status_label.setText("默认场地图片未找到，请点击“导入场地图片”选择图片。")
            return
        if crop is not None:
            image = image.copy(*crop)
        if self.background_item is not None:
            self.plot.removeItem(self.background_item)
        array = pg.imageToArray(image, copy=True)
        self.background_item = pg.ImageItem(array, axisOrder="col-major")
        # Negative height keeps the supplied overhead field image upright while
        # presenting robot Y coordinates with their origin at the bottom-left.
        self._set_background_rect()
        self.background_item.setZValue(-10)
        self.plot.addItem(self.background_item)
        self._update_plot()
        self.status_label.setText(f"已导入: {filename.name}")

    def nearest_control_point(self, x, y):
        if not self.control_points:
            return None
        threshold = max(self.field_width, self.field_height) * 0.018
        distances = [np.hypot(px - x, py - y) for px, py in self.control_points]
        index = int(np.argmin(distances))
        return index if distances[index] <= threshold else None

    def _clamp(self, x, y):
        field_x, field_y = self._to_field((x, y))
        field_point = (
            min(max(field_x, 0.0), self.field_width),
            min(max(field_y, 0.0), self.field_height),
        )
        return self._to_display(field_point)

    def add_control_point(self, x, y):
        self.control_points.append(self._clamp(x, y))
        index = len(self.control_points) - 1
        self._update_plot()
        self._select_row(index)
        return index

    def move_control_point(self, index, x, y):
        if 0 <= index < len(self.control_points):
            self.control_points[index] = self._clamp(x, y)
            self._update_plot()
            self._select_row(index)

    def delete_control_point(self, index):
        if 0 <= index < len(self.control_points):
            self.control_points.pop(index)
            self._update_plot()

    def clear_control_points(self):
        self.control_points.clear()
        self._update_plot()

    def _update_path(self):
        if len(self.control_points) >= 4:
            field_points = [self._to_field(point) for point in self.control_points]
            curve = BSplineCurve(field_points, degree=3)
            if self.sample_mode.currentData() == "count":
                samples = curve.sample_by_count(self.count_spin.value())
            else:
                samples = curve.sample_by_distance(self.interval_spin.value())
            display_samples = np.asarray([self._to_display(point) for point in samples])
            self.path_plot_points = generate_path(
                display_samples,
                lateral_acceleration=self.lateral_accel_spin.value(),
                speed_scale=self.speed_scale_spin.value(),
                acceleration=self.acceleration_spin.value(),
                deceleration=self.deceleration_spin.value(),
            )
            sx, sy = self.coordinate_system.signs
            self.path_points = [
                PathPoint(
                    sx * point.x,
                    sy * point.y,
                    yaw_from_heading(
                        self.coordinate_system.transform_angle(point.theta),
                        FORWARD_AXIS_ANGLE,
                    ),
                    self.coordinate_system.transform_curvature(point.curvature),
                    point.s,
                    point.velocity,
                )
                for point in self.path_plot_points
            ]
        else:
            self.path_points = []
            self.path_plot_points = []
        self._update_mpc_states()
        self._update_plot(update_path=False)

    def _update_mpc_states(self):
        """Rebuild the time-sampled reference states used by the MPC."""
        self.mpc_states = []
        if not self.path_points:
            self.mpc_info_label.setStyleSheet("")
            self.mpc_info_label.setText("状态数: 0（至少需要 4 个控制点）")
            return
        try:
            self.mpc_states = generate_mpc_states(
                self.path_points, sample_time=self.mpc_sample_spin.value()
            )
        except ValueError as error:
            self.mpc_info_label.setStyleSheet("color: #c0392b;")
            self.mpc_info_label.setText(f"无法生成 MPC 状态：{error}")
            return
        self.mpc_info_label.setStyleSheet("")
        duration = self.mpc_states[-1].time
        self.mpc_info_label.setText(
            f"状态数: {len(self.mpc_states)}，总时长: {duration:.2f} s"
        )

    def _update_mpc_points(self):
        """Show the time-sampled reference as dots, decimated for display speed."""
        if not self.mpc_show_check.isChecked() or not self.mpc_states:
            self.mpc_item.setData([], [])
            return
        stride = max(1, math.ceil(len(self.mpc_states) / 2000))
        sx, sy = self.coordinate_system.signs
        samples = self.mpc_states[::stride]
        self.mpc_item.setData(
            [sx * state.x for state in samples],
            [sy * state.y for state in samples],
        )

    def _set_mpc_points_visible(self, _checked):
        self._update_mpc_points()

    def _update_plot(self, update_path=True):
        if update_path:
            self._update_path()
            return
        if self.background_item is not None:
            self._set_background_rect()
        if self.control_points:
            points = np.asarray(self.control_points)
            self.control_item.setData(points[:, 0], points[:, 1])
        else:
            self.control_item.setData([], [])
        if self.path_plot_points:
            self.curve_item.setData([p.x for p in self.path_plot_points], [p.y for p in self.path_plot_points])
        else:
            self.curve_item.setData([], [])
        self._update_mpc_points()
        # Allow the view to extend above the field while preserving its aspect ratio.
        self.plot.setLimits(
            xMin=self._display_bounds()[0], xMax=self._display_bounds()[1],
            yMin=self._display_bounds()[2], yMax=self._display_bounds()[3],
        )
        self._update_table()
        self.point_count_label.setText(f"路径点: {len(self.path_points)}")
        self.origin_item.setData([0.0], [0.0])
        self._update_origin_inputs()
        self._update_origin_offset_label()

    def _update_sampling_controls(self):
        by_count = self.sample_mode.currentData() == "count"
        self.interval_spin.setEnabled(not by_count)
        self.count_spin.setEnabled(by_count)
        self._update_path()

    def _fit_field_to_view(self):
        """Show the whole field at true scale with its bottom and left on the axes."""
        rect = self.view_box.boundingRect()
        if rect.width() <= 1 or rect.height() <= 1:
            return
        min_x, max_x, min_y, max_y = self._display_bounds()
        field_width = max_x - min_x
        field_height = max_y - min_y
        visible_height = max(field_height, field_width * rect.height() / rect.width())
        # Keep the bottom start areas away from the plot and window edges.
        bottom_margin = visible_height * 0.06
        # Shift the world view downward so the field is displayed higher in
        # the window instead of leaving a large blank area above it.
        vertical_shift = -visible_height * 0.20
        self.view_box.setRange(
            xRange=(min_x, max_x),
            yRange=(
                min_y - bottom_margin + vertical_shift,
                min_y + visible_height - bottom_margin + vertical_shift,
            ),
            padding=0.0,
        )

    def resizeEvent(self, event):
        super().resizeEvent(event)
        QtCore.QTimer.singleShot(0, self._fit_field_to_view)

    def _update_table(self):
        selected = self.point_list.currentRow()
        self.point_list.blockSignals(True)
        self.point_list.setRowCount(len(self.control_points))
        for row, point in enumerate(self.control_points):
            x, y = self.coordinate_system.to_coordinate(
                self._to_field(point), self.field_width, self.field_height
            )
            for col, value in enumerate((row + 1, x, y)):
                item = QtWidgets.QTableWidgetItem(str(value) if col == 0 else f"{value:.1f}")
                item.setTextAlignment(QtCore.Qt.AlignCenter)
                if col == 0:
                    item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
                self.point_list.setItem(row, col, item)
        self.point_list.blockSignals(False)
        if 0 <= selected < len(self.control_points):
            self._select_row(selected)

    def _edit_control_point_from_table(self, row, column):
        """Apply a manually entered X or Y coordinate to the selected control point."""
        if column not in (1, 2) or not (0 <= row < len(self.control_points)):
            return
        try:
            value = float(self.point_list.item(row, column).text())
        except (AttributeError, ValueError):
            self.status_label.setText("坐标必须是数字，已恢复原值。")
            self._update_plot()
            return

        x, y = self.control_points[row]
        if column == 1:
            field_x, field_y = self.coordinate_system.from_coordinate(
                (value, self.point_list.item(row, 2).text()), self.field_width, self.field_height
            )
            x, y = self._to_display((field_x, field_y))
        else:
            field_x, field_y = self.coordinate_system.from_coordinate(
                (self.point_list.item(row, 1).text(), value), self.field_width, self.field_height
            )
            x, y = self._to_display((field_x, field_y))
        self.move_control_point(row, x, y)

    def _select_row(self, row):
        if 0 <= row < self.point_list.rowCount():
            self.point_list.selectRow(row)

    def _highlight_selected(self):
        selected = self.point_list.currentRow()
        spots = []
        for index, (x, y) in enumerate(self.control_points):
            color = "#f1c40f" if index == selected else "#e74c3c"
            spots.append({"pos": (x, y), "brush": pg.mkBrush(color), "pen": pg.mkPen("#7a1f1f"), "size": 14})
        self.control_item.setData(spots)

    def _c_source(self):
        return export_c_array(self.path_points)

    def _configuration(self):
        """Capture all editor values needed to rebuild the current path."""
        return {
            "format": CONFIG_FORMAT,
            "version": CONFIG_VERSION,
            "control_points": [list(self._to_field(point)) for point in self.control_points],
            "field": {"width_mm": self.width_spin.value(), "height_mm": self.height_spin.value()},
            "sampling": {
                "mode": self.sample_mode.currentData(),
                "interval_mm": self.interval_spin.value(),
                "count": self.count_spin.value(),
            },
            "speed": {
                "lateral_acceleration": self.lateral_accel_spin.value(),
                "speed_scale": self.speed_scale_spin.value(),
                "acceleration": self.acceleration_spin.value(),
                "deceleration": self.deceleration_spin.value(),
            },
            "mpc": {"sample_time": self.mpc_sample_spin.value()},
            "coordinate_system": {
                "origin": self.coordinate_system.origin,
                "origin_offset_mm": list(self.coordinate_system.origin_offset),
                "x_direction": self.coordinate_system.x_direction,
                "y_direction": self.coordinate_system.y_direction,
            },
        }

    def save_configuration(self):
        try:
            DEFAULT_CONFIG_DIRECTORY.mkdir(parents=True, exist_ok=True)
        except OSError as error:
            QtWidgets.QMessageBox.warning(self, "保存失败", str(error))
            return
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "保存路径配置", str(DEFAULT_CONFIG_DIRECTORY / "path_config.json"), "路径配置 (*.json)"
        )
        if not filename:
            return
        try:
            save_path_config(Path(filename), self._configuration())
        except (OSError, PathConfigError) as error:
            QtWidgets.QMessageBox.warning(self, "保存失败", str(error))
            return
        self.status_label.setText(f"已保存配置：{filename}")

    def open_configuration(self):
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "打开路径配置", str(DEFAULT_CONFIG_DIRECTORY), "路径配置 (*.json)"
        )
        if not filename:
            return
        try:
            config = load_path_config(Path(filename))
        except PathConfigError as error:
            QtWidgets.QMessageBox.warning(self, "打开失败", str(error))
            return
        self._apply_configuration(config)
        self.status_label.setText(f"已打开配置：{filename}")

    def _apply_configuration(self, config):
        """Apply a pre-validated configuration without intermediate redraws."""
        widgets = (
            self.width_spin, self.height_spin, self.sample_mode, self.interval_spin,
            self.count_spin, self.lateral_accel_spin, self.speed_scale_spin,
            self.acceleration_spin, self.deceleration_spin, self.mpc_sample_spin,
            self.origin_combo, self.direction_combo,
        )
        blockers = [QtCore.QSignalBlocker(widget) for widget in widgets]
        field = config["field"]
        sampling = config["sampling"]
        speed = config["speed"]
        mpc = config["mpc"]
        self.width_spin.setValue(field["width_mm"])
        self.height_spin.setValue(field["height_mm"])
        self.sample_mode.setCurrentIndex(self.sample_mode.findData(sampling["mode"]))
        self.interval_spin.setValue(sampling["interval_mm"])
        self.count_spin.setValue(sampling["count"])
        self.lateral_accel_spin.setValue(speed["lateral_acceleration"])
        self.speed_scale_spin.setValue(speed["speed_scale"])
        self.acceleration_spin.setValue(speed["acceleration"])
        self.deceleration_spin.setValue(speed["deceleration"])
        self.mpc_sample_spin.setValue(mpc["sample_time"])
        coordinate_system = config["coordinate_system"]
        self.origin_combo.setCurrentIndex(
            self._combo_index(self.origin_combo, coordinate_system["origin"])
        )
        self.direction_combo.setCurrentIndex(
            self._combo_index(
                self.direction_combo,
                (coordinate_system["x_direction"], coordinate_system["y_direction"]),
            )
        )
        del blockers

        self.field_width = field["width_mm"]
        self.field_height = field["height_mm"]
        self._set_default_image_origin()
        self.coordinate_system = CoordinateSystem(
            coordinate_system["origin"],
            coordinate_system["x_direction"],
            coordinate_system["y_direction"],
            tuple(coordinate_system["origin_offset_mm"]),
        )
        self.control_points = [self._to_display(point) for point in config["control_points"]]
        self._update_sampling_controls()

    def copy_c_array(self):
        if not self.path_points:
            self.status_label.setText("至少需要 4 个控制点才能导出路径。")
            return
        QtWidgets.QApplication.clipboard().setText(self._c_source())
        self.status_label.setText("C 数组已复制到剪贴板。")

    def save_c_array(self):
        if not self.path_points:
            self.status_label.setText("至少需要 4 个控制点才能导出路径。")
            return
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(self, "导出路径头文件", "path.h", "C Header (*.h)")
        if filename:
            Path(filename).write_text(self._c_source(), encoding="utf-8")
            self.status_label.setText(f"已导出: {filename}")

    def copy_mpc_array(self):
        if not self.mpc_states:
            self.status_label.setText("没有可导出的 MPC 状态，请先检查控制点和速度规划。")
            return
        QtWidgets.QApplication.clipboard().setText(
            export_mpc_c_array(self.mpc_states, self.mpc_sample_spin.value())
        )
        self.status_label.setText(
            f"MPC 数组已复制到剪贴板（{len(self.mpc_states)} 个状态，"
            f"dt={self.mpc_sample_spin.value():g} s）。"
        )

    def save_mpc_header(self):
        if not self.mpc_states:
            self.status_label.setText("没有可导出的 MPC 状态，请先检查控制点和速度规划。")
            return
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "导出 MPC 状态头文件", "mpc_reference.h", "C Header (*.h)"
        )
        if not filename:
            return
        Path(filename).write_text(
            export_mpc_c_array(self.mpc_states, self.mpc_sample_spin.value()),
            encoding="utf-8",
        )
        self.status_label.setText(f"已导出 MPC 状态: {filename}")

    def save_mpc_csv(self):
        if not self.mpc_states:
            self.status_label.setText("没有可导出的 MPC 状态，请先检查控制点和速度规划。")
            return
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "导出 MPC 状态 CSV", "mpc_reference.csv", "CSV (*.csv)"
        )
        if not filename:
            return
        Path(filename).write_text(export_mpc_csv(self.mpc_states), encoding="utf-8")
        self.status_label.setText(f"已导出 MPC 状态: {filename}")


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    editor = PathEditor()
    editor.show()
    QtCore.QTimer.singleShot(0, editor._fit_field_to_view)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
