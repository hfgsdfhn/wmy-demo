"""Convert sampled XY positions into robot-ready path data."""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np


@dataclass
class PathPoint:
    x: float
    y: float
    theta: float
    curvature: float
    s: float
    velocity: float


def generate_path(
    points: np.ndarray,
    lateral_acceleration: float = 2.0,
    max_speed: float = 2.5,
    speed_scale: float = 1.0,
    acceleration: float = 1.0,
    deceleration: float = 1.0,
) -> list[PathPoint]:
    """Calculate geometry and a speed profile for a sampled path.

    Coordinates and ``s`` are millimetres. Theta is radians, counterclockwise
    from +X; curvature is 1/m; velocity is m/s.  ``lateral_acceleration`` is
    used only for curvature speed limiting.  Acceleration and deceleration
    shape the start and stop speed envelope along the path distance.
    """
    _validate_profile_inputs(
        lateral_acceleration, max_speed, speed_scale, acceleration, deceleration
    )
    values = np.asarray(points, dtype=float)
    if len(values) == 0:
        return []
    if len(values) == 1:
        return [PathPoint(values[0, 0], values[0, 1], 0.0, 0.0, 0.0, 0.0)]

    dx = np.gradient(values[:, 0])
    dy = np.gradient(values[:, 1])
    theta = np.unwrap(np.arctan2(dy, dx))
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    speed_sq = dx * dx + dy * dy
    curvature_per_mm = np.divide(
        dx * ddy - dy * ddx,
        np.power(speed_sq, 1.5),
        out=np.zeros_like(dx),
        where=speed_sq > 1e-12,
    )
    curvature = curvature_per_mm * 1000.0
    segment = np.linalg.norm(np.diff(values, axis=0), axis=1)
    s = np.concatenate(([0.0], np.cumsum(segment)))
    velocity = _plan_speed_profile(
        curvature,
        segment / 1000.0,
        lateral_acceleration,
        max_speed,
        speed_scale,
        acceleration,
        deceleration,
    )
    return [
        PathPoint(float(x), float(y), float(t), float(k), float(d), float(v))
        for (x, y), t, k, d, v in zip(values, theta, curvature, s, velocity)
    ]


def _validate_profile_inputs(
    lateral_acceleration: float,
    max_speed: float,
    speed_scale: float,
    acceleration: float,
    deceleration: float,
) -> None:
    if lateral_acceleration <= 0.0:
        raise ValueError("Lateral acceleration must be positive.")
    if max_speed < 0.0 or speed_scale < 0.0:
        raise ValueError("Speed limits must not be negative.")
    if acceleration <= 0.0 or deceleration <= 0.0:
        raise ValueError("Acceleration and deceleration must be positive.")


def _plan_speed_profile(
    curvature: np.ndarray,
    segment_m: np.ndarray,
    lateral_acceleration: float,
    max_speed: float,
    speed_scale: float,
    acceleration: float,
    deceleration: float,
) -> np.ndarray:
    """Apply curvature, start-up and stopping constraints to every point."""
    abs_curvature = np.abs(curvature)
    curvature_limit = np.full(len(curvature), max_speed)
    turning = abs_curvature > 1e-9
    curvature_limit[turning] = np.sqrt(
        lateral_acceleration / abs_curvature[turning]
    )
    point_limit = np.minimum(max_speed, curvature_limit) * speed_scale

    # Forward pass: the robot starts from rest and can only accelerate over ds.
    velocity = np.zeros(len(curvature))
    for index, ds in enumerate(segment_m, start=1):
        reachable = math.sqrt(velocity[index - 1] ** 2 + 2.0 * acceleration * ds)
        velocity[index] = min(point_limit[index], reachable)

    # Backward pass: reserve enough distance to decelerate to zero at the end.
    velocity[-1] = 0.0
    for index in range(len(velocity) - 2, -1, -1):
        reachable = math.sqrt(velocity[index + 1] ** 2 + 2.0 * deceleration * segment_m[index])
        velocity[index] = min(velocity[index], reachable, point_limit[index])

    return velocity
