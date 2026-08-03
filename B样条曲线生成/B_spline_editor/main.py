"""Robocon B-spline path editor entry point."""
from __future__ import annotations

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
    BLUE_START_ORIGIN_PIXEL,
    FIELD_HEIGHT_MM,
    FIELD_IMAGE_CROP,
    FIELD_IMAGE_PATH,
    FIELD_WIDTH_MM,
)
from path.exporter import export_c_array
from path.path_generator import generate_path


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
        self.setMouseEnabled(x=False, y=False)

    def _world_position(self, event):
        position = self.mapSceneToView(event.scenePos())
        return float(position.x()), float(position.y())

    def mouseClickEvent(self, event):
        x, y = self._world_position(event)
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
        if event.button() != QtCore.Qt.LeftButton:
            event.ignore()
            return
        x, y = self._world_position(event)
        if event.isStart():
            self.drag_index = self.editor.nearest_control_point(x, y)
            if self.drag_index is None:
                self.drag_index = self.editor.add_control_point(x, y)
        if self.drag_index is not None:
            self.editor.move_control_point(self.drag_index, x, y)
        if event.isFinish():
            self.drag_index = None
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
        self.control_points: list[tuple[float, float]] = []
        self.path_points = []
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
        controls.setFixedWidth(295)
        control_layout = QtWidgets.QVBoxLayout(controls)

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
        copy_button = QtWidgets.QPushButton("复制 C 数组到剪贴板")
        save_button = QtWidgets.QPushButton("导出 .h 文件")
        copy_button.clicked.connect(self.copy_c_array)
        save_button.clicked.connect(self.save_c_array)
        export_layout.addWidget(copy_button)
        export_layout.addWidget(save_button)

        self.status_label = QtWidgets.QLabel("左键添加/拖动，右键删除控制点")
        self.status_label.setWordWrap(True)
        control_layout.addWidget(image_group)
        control_layout.addWidget(path_group)
        control_layout.addWidget(speed_group)
        control_layout.addWidget(list_group, 1)
        control_layout.addWidget(export_group)
        control_layout.addWidget(self.status_label)

        self.view_box = PathViewBox(self, lockAspect=True, invertY=False)
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
        layout.addWidget(controls)
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

    def _set_field_size(self):
        self.field_width = self.width_spin.value()
        self.field_height = self.height_spin.value()
        self._set_default_image_origin()
        self._update_plot()

    def _set_default_image_origin(self):
        """Map robot (0, 0) to the lower-left corner of the blue start block."""
        pixel_x, pixel_y = BLUE_START_ORIGIN_PIXEL
        _, _, image_width, image_height = FIELD_IMAGE_CROP
        self.image_origin_x = self.field_width * pixel_x / image_width
        # QGraphics uses image edge coordinates, so do not subtract one pixel here.
        self.image_origin_y = self.field_height * (image_height - pixel_y) / image_height

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
        origin_x, origin_y = origin
        self.background_item.setRect(
            QtCore.QRectF(-origin_x, self.field_height - origin_y, self.field_width, -self.field_height)
        )
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
        return (min(max(x, 0.0), self.field_width), min(max(y, 0.0), self.field_height))

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
            curve = BSplineCurve(self.control_points, degree=3)
            if self.sample_mode.currentData() == "count":
                samples = curve.sample_by_count(self.count_spin.value())
            else:
                samples = curve.sample_by_distance(self.interval_spin.value())
            self.path_points = generate_path(
                samples,
                lateral_acceleration=self.lateral_accel_spin.value(),
                speed_scale=self.speed_scale_spin.value(),
                acceleration=self.acceleration_spin.value(),
                deceleration=self.deceleration_spin.value(),
            )
        else:
            self.path_points = []
        self._update_plot(update_path=False)

    def _update_plot(self, update_path=True):
        if update_path:
            self._update_path()
            return
        if self.background_item is not None:
            self.background_item.setRect(
                QtCore.QRectF(
                    -self.image_origin_x,
                    self.field_height - self.image_origin_y,
                    self.field_width,
                    -self.field_height,
                )
            )
        if self.control_points:
            points = np.asarray(self.control_points)
            self.control_item.setData(points[:, 0], points[:, 1])
        else:
            self.control_item.setData([], [])
        if self.path_points:
            self.curve_item.setData([p.x for p in self.path_points], [p.y for p in self.path_points])
        else:
            self.curve_item.setData([], [])
        # Allow the view to extend above the field while preserving its aspect ratio.
        self.plot.setLimits(
            xMin=-self.image_origin_x,
            xMax=self.field_width - self.image_origin_x,
            yMin=-self.field_height - self.image_origin_y,
            yMax=self.field_height * 2.0 - self.image_origin_y,
        )
        self._fit_field_to_view()
        self._update_table()
        self.point_count_label.setText(f"路径点: {len(self.path_points)}")

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
        visible_height = max(self.field_height, self.field_width * rect.height() / rect.width())
        # Keep the bottom start areas away from the plot and window edges.
        bottom_margin = visible_height * 0.06
        self.view_box.setRange(
            xRange=(-self.image_origin_x, self.field_width - self.image_origin_x),
            yRange=(
                -self.image_origin_y - bottom_margin,
                visible_height - self.image_origin_y - bottom_margin,
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
        for row, (x, y) in enumerate(self.control_points):
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
            x = value
        else:
            y = value
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


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    editor = PathEditor()
    editor.show()
    QtCore.QTimer.singleShot(0, editor._fit_field_to_view)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
