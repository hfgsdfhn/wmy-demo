"""实时位置、速度与角速度曲线。"""
from __future__ import annotations

from collections import deque
from PyQt5 import QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class PlotWidget(QtWidgets.QFrame):
    def __init__(self, parent=None):
        super().__init__(parent); layout = QtWidgets.QVBoxLayout(self); layout.setContentsMargins(4, 4, 4, 4)
        self.figure = Figure(figsize=(8, 2.2), facecolor="#ffffff"); self.canvas = FigureCanvas(self.figure); layout.addWidget(self.canvas)
        self.axes = self.figure.subplots(1, 3); self.times = deque(maxlen=500); self.values = {key: deque(maxlen=500) for key in ("x", "y", "speed", "yaw_rate")}
        self._style()

    def _style(self):
        for axis in self.axes:
            axis.set_facecolor("#ffffff"); axis.tick_params(colors="#64748b", labelsize=7); axis.grid(color="#d7dee8", alpha=.8)
            for spine in axis.spines.values(): spine.set_color("#94a3b8")
        self.axes[0].set_title("POSITION", color="#1f2937", fontsize=8); self.axes[0].set_ylabel("m", color="#64748b", fontsize=7)
        self.axes[1].set_title("VELOCITY", color="#1f2937", fontsize=8); self.axes[1].set_ylabel("m/s", color="#64748b", fontsize=7)
        self.axes[2].set_title("YAW RATE", color="#1f2937", fontsize=8); self.axes[2].set_ylabel("deg/s", color="#64748b", fontsize=7)

    def update_state(self, state):
        self.times.append(state.timestamp_ms / 1000.0); self.values["x"].append(state.position_x_m); self.values["y"].append(state.position_y_m); self.values["speed"].append(state.speed_mps); self.values["yaw_rate"].append(state.yaw_rate_degps)
        for axis in self.axes: axis.clear()
        self._style(); t = list(self.times)
        self.axes[0].plot(t, self.values["x"], color="#0284c7", label="X"); self.axes[0].plot(t, self.values["y"], color="#d97706", label="Y"); self.axes[0].legend(fontsize=7, facecolor="#ffffff", labelcolor="#1f2937")
        self.axes[1].plot(t, self.values["speed"], color="#34d399")
        self.axes[2].plot(t, self.values["yaw_rate"], color="#fb7185")
        self.figure.tight_layout(pad=.6); self.canvas.draw_idle()
