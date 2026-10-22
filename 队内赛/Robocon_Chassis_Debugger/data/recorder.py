"""实时数据记录与 CSV 导出。"""
from __future__ import annotations

import csv
from pathlib import Path
from .robot_state import RobotState


class DataRecorder:
    def __init__(self) -> None:
        self.rows: list[dict] = []
        self._start_ms: int | None = None

    def append(self, state: RobotState) -> None:
        if self._start_ms is None:
            self._start_ms = state.timestamp_ms
        self.rows.append({
            "time": (state.timestamp_ms - self._start_ms) / 1000.0,
            "x": state.position_x_m, "y": state.position_y_m, "yaw": state.yaw_deg,
            "vx": state.velocity_x_mps, "vy": state.velocity_y_mps, "omega": state.yaw_rate_degps,
        })

    def clear(self) -> None:
        self.rows.clear()
        self._start_ms = None

    def save_csv(self, filename: str | Path) -> None:
        with Path(filename).open("w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.DictWriter(handle, fieldnames=["time", "x", "y", "yaw", "vx", "vy", "omega"])
            writer.writeheader()
            writer.writerows(self.rows)
