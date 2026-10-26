"""Time-domain reference states for MPC controllers.

The path editor normally exports geometry sampled by distance.  An MPC needs a
reference at a fixed control period instead, so this module re-parameterises
the planned speed profile by time.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

from .path_generator import PathPoint


# Default control period of the MPC reference trajectory, in seconds.
DEFAULT_SAMPLE_TIME = 0.01
MIN_SAMPLE_TIME = 0.001
MAX_SAMPLE_TIME = 1.0
MAX_STATE_COUNT = 200_000


@dataclass(frozen=True)
class MpcState:
    """One MPC reference state at a given time.

    ``x``/``y``/``s`` use millimetres and the same robot coordinate frame as
    :class:`~path.path_generator.PathPoint`.  ``theta`` is in radians,
    ``velocity`` in m/s, ``omega`` in rad/s, ``acceleration`` in m/s^2 and
    ``curvature`` in 1/m.
    """

    step: int
    time: float
    x: float
    y: float
    theta: float
    velocity: float
    omega: float
    acceleration: float
    curvature: float
    s: float


def generate_mpc_states(
    path_points: Sequence[PathPoint],
    sample_time: float = DEFAULT_SAMPLE_TIME,
) -> list[MpcState]:
    """Resample a distance-sampled path on a fixed ``sample_time`` grid.

    Every input point must carry a planned speed profile in ``velocity``
    (m/s); velocity may only reach zero at the start and at the end.  Between
    two consecutive points the speed is assumed to change at a constant
    acceleration, so traversing a segment of length ``ds`` takes
    ``2 * ds / (v0 + v1)`` seconds.  This keeps the time axis consistent with
    the acceleration-limited profile produced by
    :func:`~path.path_generator.generate_path`.

    The returned states start at ``t = 0`` and are spaced exactly
    ``sample_time`` seconds apart.  When the total duration is not an integer
    multiple of ``sample_time`` one extra state is appended at the exact path
    end, so its step is shorter than ``sample_time``; this is the only
    non-uniform interval in the result.
    """
    _validate_sample_time(sample_time)
    points = list(path_points)
    if not points:
        return []

    x = np.array([float(point.x) for point in points], dtype=float)
    y = np.array([float(point.y) for point in points], dtype=float)
    theta = np.unwrap(np.array([float(point.theta) for point in points], dtype=float))
    curvature = np.array([float(point.curvature) for point in points], dtype=float)
    distance = np.array([float(point.s) for point in points], dtype=float)
    velocity = np.maximum(
        np.array([float(point.velocity) for point in points], dtype=float), 0.0
    )

    if np.any(np.diff(distance) < -1e-9):
        raise ValueError("路径的累计距离 s 必须单调递增")
    # Duplicate positions (zero distance) cannot be parameterised by time and
    # carry no new geometry, so drop them before integrating the profile.
    keep = np.concatenate(([True], np.diff(distance) > 1e-9))
    if not np.all(keep):
        x, y = x[keep], y[keep]
        theta, curvature = theta[keep], curvature[keep]
        distance, velocity = distance[keep], velocity[keep]

    if len(x) == 1:
        return [
            MpcState(
                step=0,
                time=0.0,
                x=float(x[0]),
                y=float(y[0]),
                theta=float(theta[0]),
                velocity=0.0,
                omega=0.0,
                acceleration=0.0,
                curvature=float(curvature[0]),
                s=float(distance[0]),
            )
        ]
    if np.all(velocity <= 1e-9):
        raise ValueError("速度曲线全为 0，无法按时间采样；请增大“速度比例”")

    segment_length = np.diff(distance) / 1000.0
    start_speed = velocity[:-1]
    end_speed = velocity[1:]
    speed_sum = start_speed + end_speed
    stationary = (speed_sum <= 1e-12) & (segment_length > 1e-12)
    if np.any(stationary):
        raise ValueError("路径中存在速度为 0 的区间，无法按时间采样")

    duration = np.divide(
        2.0 * segment_length,
        speed_sum,
        out=np.zeros_like(segment_length),
        where=speed_sum > 1e-12,
    )
    acceleration = np.divide(
        end_speed - start_speed,
        duration,
        out=np.zeros_like(duration),
        where=duration > 1e-12,
    )

    time_at_point = np.concatenate(([0.0], np.cumsum(duration)))
    total_time = float(time_at_point[-1])
    if not math.isfinite(total_time) or total_time <= 0.0:
        raise ValueError("路径总时长为 0，无法按时间采样")

    grid = np.arange(int(math.floor(total_time / sample_time + 1e-9)) + 1) * sample_time
    if total_time - grid[-1] > 1e-9:
        grid = np.append(grid, total_time)
    if grid.size > MAX_STATE_COUNT:
        raise ValueError(
            f"按 {sample_time:g} s 采样将产生 {grid.size} 个状态，"
            f"超过上限 {MAX_STATE_COUNT}，请增大采样周期"
        )

    # Locate every time sample inside the constant-acceleration segments.
    segment = np.searchsorted(time_at_point, grid, side="right") - 1
    segment = np.clip(segment, 0, len(duration) - 1)
    local_time = np.clip(grid - time_at_point[segment], 0.0, duration[segment])
    segment_acceleration = acceleration[segment]
    travelled = start_speed[segment] * local_time + 0.5 * segment_acceleration * local_time**2
    arc = np.clip(
        distance[segment] + travelled * 1000.0,
        distance[segment],
        distance[segment + 1],
    )
    span = distance[segment + 1] - distance[segment]
    fraction = np.divide(
        arc - distance[segment],
        span,
        out=np.zeros_like(arc),
        where=span > 1e-9,
    )
    positions_x = x[segment] + fraction * (x[segment + 1] - x[segment])
    positions_y = y[segment] + fraction * (y[segment + 1] - y[segment])
    headings = theta[segment] + fraction * (theta[segment + 1] - theta[segment])
    curvatures = curvature[segment] + fraction * (curvature[segment + 1] - curvature[segment])
    speeds = np.maximum(start_speed[segment] + segment_acceleration * local_time, 0.0)

    return [
        MpcState(
            step=index,
            time=float(grid[index]),
            x=float(positions_x[index]),
            y=float(positions_y[index]),
            theta=float(headings[index]),
            velocity=float(speeds[index]),
            omega=float(speeds[index] * curvatures[index]),
            acceleration=float(segment_acceleration[index]),
            curvature=float(curvatures[index]),
            s=float(arc[index]),
        )
        for index in range(grid.size)
    ]


def _validate_sample_time(sample_time: float) -> None:
    """Reject control periods that would make the reference unusable."""
    try:
        value = float(sample_time)
    except (TypeError, ValueError) as error:
        raise ValueError("采样周期必须是数值") from error
    if not math.isfinite(value) or not MIN_SAMPLE_TIME <= value <= MAX_SAMPLE_TIME:
        raise ValueError(
            f"采样周期必须在 {MIN_SAMPLE_TIME:g} s 到 {MAX_SAMPLE_TIME:g} s 之间"
        )
