"""轨迹绘图的轻量数据适配器。"""
from __future__ import annotations

from .collision_check import Obstacle
from ..data.trajectory_data import TrajectoryPoint


def bounds(points: list[TrajectoryPoint]) -> tuple[float, float, float, float]:
    if not points:
        return 0.0, 1.0, 0.0, 1.0
    xs = [p.x for p in points]; ys = [p.y for p in points]
    return min(xs), max(xs), min(ys), max(ys)


def obstacle_rectangles(obstacles: list[Obstacle]):
    return [(o.x, o.y, o.width, o.height, o.name) for o in obstacles]
