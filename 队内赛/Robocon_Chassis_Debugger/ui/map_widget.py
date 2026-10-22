"""比赛地图、B-spline 轨迹和机器人实时位置显示。"""
from __future__ import annotations

import math
from pathlib import Path
import numpy as np
from PyQt5 import QtCore, QtWidgets
from matplotlib import rcParams
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import FancyArrowPatch, Polygon, Rectangle

rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Arial Unicode MS", "DejaVu Sans"]
rcParams["axes.unicode_minus"] = False


class MapWidget(QtWidgets.QWidget):
    point_added = QtCore.pyqtSignal(float, float)
    point_removed = QtCore.pyqtSignal(float, float)
    point_moved = QtCore.pyqtSignal(int, float, float)

    def __init__(self, width_m: float, height_m: float, parent=None):
        super().__init__(parent)
        self.width_m, self.height_m = width_m, height_m
        self.figure = Figure(figsize=(8, 5), facecolor="#ffffff")
        self.canvas = FigureCanvas(self.figure)
        self.axes = self.figure.add_subplot(111)
        self.axes.set_facecolor("#ffffff")
        self.canvas.mpl_connect("button_press_event", self._mouse_press)
        self.canvas.mpl_connect("motion_notify_event", self._mouse_move)
        self.canvas.mpl_connect("button_release_event", self._mouse_release)
        self.canvas.mpl_connect("draw_event", self._on_canvas_draw)
        layout = QtWidgets.QVBoxLayout(self); layout.setContentsMargins(0, 0, 0, 0); layout.addWidget(self.canvas)
        self._background = None
        self._trajectory = []
        self._control_points = []
        self._obstacles = []
        self._robot_state = None
        self._robot_patch = None
        self._heading_arrow = None
        self._robot_center = None
        self._canvas_background = None
        self._blitting = False
        self.robot_length_m, self.robot_width_m = 0.7, 0.7
        self._edit_mode = False
        self._drag_index = None
        self._draw()

    def set_field_size(self, width_m: float, height_m: float) -> None:
        self.width_m, self.height_m = width_m, height_m; self._draw()

    def load_background(self, filename: str | Path) -> None:
        image = plt_imread(filename)
        self._background = image; self._draw()

    def set_obstacles(self, obstacles) -> None:
        self._obstacles = obstacles; self._draw()

    def set_trajectory(self, points) -> None:
        self._trajectory = points; self._draw()

    def set_control_points(self, points) -> None:
        self._control_points = points; self._draw()

    def set_robot_state(self, state) -> None:
        self._robot_state = state
        self._blit_robot()

    def set_robot_dimensions(self, length_m: float, width_m: float) -> None:
        self.robot_length_m, self.robot_width_m = length_m, width_m; self._draw()

    def set_edit_mode(self, enabled: bool) -> None:
        self._edit_mode = enabled

    def _mouse_press(self, event) -> None:
        if not self._edit_mode or event.inaxes != self.axes or event.xdata is None or event.ydata is None:
            return
        if event.button == 1:
            self._drag_index = self._nearest_control_point(float(event.xdata), float(event.ydata))
            if self._drag_index is None:
                self.point_added.emit(float(event.xdata), float(event.ydata))
        elif event.button == 3:
            self.point_removed.emit(float(event.xdata), float(event.ydata))

    def _mouse_move(self, event) -> None:
        if self._drag_index is None or event.inaxes != self.axes or event.xdata is None or event.ydata is None:
            return
        x = min(max(0.0, float(event.xdata)), self.width_m)
        y = min(max(0.0, float(event.ydata)), self.height_m)
        self.point_moved.emit(self._drag_index, x, y)

    def _mouse_release(self, event) -> None:
        if event.button == 1:
            self._drag_index = None

    def _nearest_control_point(self, x: float, y: float):
        if not self._control_points:
            return None
        threshold = max(self.width_m, self.height_m) * 0.035
        distances = [math.hypot(px - x, py - y) for px, py in self._control_points]
        index = min(range(len(distances)), key=distances.__getitem__)
        return index if distances[index] <= threshold else None

    def _draw(self) -> None:
        self.axes.clear(); self.axes.set_facecolor("#ffffff")
        self.axes.set_xlim(0, self.width_m); self.axes.set_ylim(0, self.height_m); self.axes.set_aspect("equal", adjustable="box")
        self.axes.set_xlabel("世界坐标 X / m", color="#475569"); self.axes.set_ylabel("世界坐标 Y / m", color="#475569")
        self.axes.tick_params(colors="#475569"); self.axes.grid(color="#d7dee8", alpha=.8, linewidth=.6)
        for spine in self.axes.spines.values(): spine.set_color("#94a3b8")
        if self._background is not None:
            # 地图底图保持原始颜色，轨迹与机器人通过更高 z-order 叠加。
            self.axes.imshow(self._background, extent=(0, self.width_m, 0, self.height_m), origin="upper", alpha=1.0, aspect="equal", zorder=0)
            self.axes.set_aspect("equal", adjustable="box")
        self.axes.add_patch(Rectangle((0, 0), self.width_m, self.height_m, fill=False, edgecolor="#64748b", linewidth=1.2))
        for obstacle in self._obstacles:
            self.axes.add_patch(Rectangle((obstacle.x, obstacle.y), obstacle.width, obstacle.height, facecolor="#7f1d1d", edgecolor="#f87171", alpha=.55, linewidth=1.2))
            self.axes.text(obstacle.x + obstacle.width / 2, obstacle.y + obstacle.height / 2, obstacle.name, ha="center", va="center", color="#fecaca", fontsize=8)
        if self._control_points:
            xs, ys = zip(*self._control_points); self.axes.plot(xs, ys, "o--", color="#f59e0b", markersize=5, linewidth=1, label="控制点")
        if self._trajectory:
            xs = [p.x for p in self._trajectory]; ys = [p.y for p in self._trajectory]
            self.axes.plot(xs, ys, color="#38bdf8", linewidth=2.4, label="B-spline 轨迹")
            self.axes.scatter([xs[0], xs[-1]], [ys[0], ys[-1]], c=["#22c55e", "#fb7185"], s=45, zorder=5)
            self.axes.text(xs[0], ys[0], "  START", color="#86efac", fontsize=8, va="bottom")
            self.axes.text(xs[-1], ys[-1], "  END", color="#fda4af", fontsize=8, va="top")
        self._create_robot_artists()
        handles, labels = self.axes.get_legend_handles_labels()
        if handles:
            self.axes.legend(loc="upper right", facecolor="#ffffff", edgecolor="#cbd5e1", labelcolor="#1f2937", fontsize=8)
        self.figure.tight_layout(pad=.8); self._canvas_background = None; self.canvas.draw()

    def _create_robot_artists(self) -> None:
        self._robot_patch = Polygon(np.zeros((4, 2)), closed=True, facecolor="#2563eb", edgecolor="#1d4ed8", alpha=.9, zorder=8, animated=True, label="机器人")
        self.axes.add_patch(self._robot_patch)
        self._heading_arrow = FancyArrowPatch((0, 0), (0, 0), arrowstyle="-|>", mutation_scale=14, color="#ffffff", linewidth=2, zorder=9, animated=True)
        self.axes.add_patch(self._heading_arrow)
        self._robot_center, = self.axes.plot([], [], marker="o", markersize=3, color="#ffffff", zorder=10, animated=True)
        visible = self._robot_state is not None
        self._robot_patch.set_visible(visible); self._heading_arrow.set_visible(visible); self._robot_center.set_visible(visible)

    def _update_robot_artists(self) -> None:
        if self._robot_state is None or self._robot_patch is None:
            return
        state = self._robot_state
        x, y = state.position_x_m, state.position_y_m
        yaw = math.radians(state.yaw_deg)
        length, width = self.robot_length_m, self.robot_width_m
        corners = np.array([[length / 2, width / 2], [length / 2, -width / 2], [-length / 2, -width / 2], [-length / 2, width / 2]])
        rot = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        poly = corners @ rot.T + [x, y]
        self._robot_patch.set_xy(poly); self._robot_patch.set_visible(True)
        self._heading_arrow.set_positions((x, y), (x + .42 * math.cos(yaw), y + .42 * math.sin(yaw))); self._heading_arrow.set_visible(True)
        self._robot_center.set_data([x], [y]); self._robot_center.set_visible(True)

    def _on_canvas_draw(self, event) -> None:
        if self._blitting:
            return
        self._canvas_background = self.canvas.copy_from_bbox(self.axes.bbox)
        self._blit_robot()

    def _blit_robot(self) -> None:
        if self._robot_state is None or self._canvas_background is None or self._robot_patch is None:
            return
        self._blitting = True
        try:
            self.canvas.restore_region(self._canvas_background)
            self._update_robot_artists()
            self.axes.draw_artist(self._robot_patch)
            self.axes.draw_artist(self._heading_arrow)
            self.axes.draw_artist(self._robot_center)
            self.canvas.blit(self.axes.bbox)
        finally:
            self._blitting = False


def plt_imread(filename):
    from matplotlib.image import imread
    return imread(str(filename))
