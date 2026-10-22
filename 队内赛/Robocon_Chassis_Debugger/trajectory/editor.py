"""B-spline 控制点与轨迹采样逻辑，不依赖 UI。"""
from __future__ import annotations

import math
import numpy as np
from ..data.trajectory_data import TrajectoryPoint


def bspline_points(control_points: list[tuple[float, float]], count: int = 220) -> np.ndarray:
    """生成三次 B-spline 的近似采样，四点不足时退化为折线。"""
    if len(control_points) < 2:
        return np.asarray(control_points, dtype=float)
    control = np.asarray(control_points, dtype=float)
    if len(control) < 4:
        return np.vstack([np.linspace(control[i], control[i + 1], max(2, count // (len(control) - 1)), endpoint=False) for i in range(len(control) - 1)] + [control[-1]])
    degree = 3
    knots = np.concatenate((np.zeros(degree), np.arange(len(control) - degree + 1), np.full(degree, len(control) - degree)))
    u_values = np.linspace(0, len(control) - degree, count)

    def de_boor(u: float) -> np.ndarray:
        span = int(np.searchsorted(knots, u, side="right") - 1)
        span = min(len(control) - 1, max(degree, span))
        d = [control[span - degree + j].copy() for j in range(degree + 1)]
        for r in range(1, degree + 1):
            for j in range(degree, r - 1, -1):
                index = span - degree + j
                denominator = knots[index + degree - r + 1] - knots[index]
                alpha = 0.0 if denominator == 0 else (u - knots[index]) / denominator
                d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j]
        return d[degree]

    return np.asarray([de_boor(u) for u in u_values])


def sample_trajectory(control_points: list[tuple[float, float]], count: int = 220, speed: float = 0.8) -> list[TrajectoryPoint]:
    samples = bspline_points(control_points, count)
    if len(samples) < 2:
        return [TrajectoryPoint(float(x), float(y), 0.0, 0.0) for x, y in samples]
    dx = np.gradient(samples[:, 0]); dy = np.gradient(samples[:, 1])
    yaw = np.unwrap(np.arctan2(dy, dx))
    velocity = np.full(len(samples), max(0.0, speed), dtype=float)
    velocity[[0, -1]] = 0.0
    return [TrajectoryPoint(float(x), float(y), float(a), float(v)) for (x, y), a, v in zip(samples, yaw, velocity)]
