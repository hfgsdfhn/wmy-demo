"""路径边界与障碍物安全检测。"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from ..data.trajectory_data import TrajectoryPoint
from ..robot.robot_model import RobotModel


@dataclass
class Obstacle:
    x: float
    y: float
    width: float
    height: float
    name: str = "障碍物"


@dataclass
class SafetyReport:
    safe: bool
    message: str
    index: int | None = None
    distance_m: float | None = None


def check_trajectory(points: list[TrajectoryPoint], field_width: float, field_height: float, robot: RobotModel, obstacles: list[Obstacle]) -> SafetyReport:
    for index, point in enumerate(points):
        footprint = robot.footprint(point.x, point.y, point.yaw)
        if np.any(footprint[:, 0] < 0.0) or np.any(footprint[:, 0] > field_width) or np.any(footprint[:, 1] < 0.0) or np.any(footprint[:, 1] > field_height):
            clearance = min(np.min(footprint[:, 0]), field_width - np.max(footprint[:, 0]), np.min(footprint[:, 1]), field_height - np.max(footprint[:, 1]))
            size = f"{robot.length_m * 1000:.0f}×{robot.width_m * 1000:.0f} mm"
            return SafetyReport(False, f"第 {index + 1} 个路径点的 {size} 方形底盘范围超出地图边界，禁止启动", index, float(clearance))
        for obstacle in obstacles:
            obstacle_polygon = np.array([
                [obstacle.x, obstacle.y],
                [obstacle.x + obstacle.width, obstacle.y],
                [obstacle.x + obstacle.width, obstacle.y + obstacle.height],
                [obstacle.x, obstacle.y + obstacle.height],
            ])
            if _polygons_intersect(footprint, obstacle_polygon):
                return SafetyReport(False, f"轨迹危险！第 {index + 1} 个路径点的方形底盘范围与 {obstacle.name} 重叠，禁止启动", index, 0.0)
    return SafetyReport(True, f"安全检测通过 · {len(points)} 个路径点")


def _polygons_intersect(first: np.ndarray, second: np.ndarray) -> bool:
    """使用分离轴定理检测旋转底盘矩形与地图障碍矩形。"""
    for polygon in (first, second):
        for index in range(len(polygon)):
            edge = polygon[(index + 1) % len(polygon)] - polygon[index]
            axis = np.array([-edge[1], edge[0]])
            norm = np.linalg.norm(axis)
            if norm <= 1e-12:
                continue
            axis /= norm
            first_projection = first @ axis
            second_projection = second @ axis
            if first_projection.max() < second_projection.min() or second_projection.max() < first_projection.min():
                return False
    return True
