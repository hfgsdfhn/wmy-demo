"""High-level cubic clamped B-spline curve interface."""
from __future__ import annotations

import numpy as np

from .bspline import clamped_uniform_knots, de_boor


class BSplineCurve:
    def __init__(self, control_points, degree: int = 3):
        self.control_points = np.asarray(control_points, dtype=float)
        self.degree = degree
        self.knots = clamped_uniform_knots(len(self.control_points), degree)

    def evaluate(self, u: float) -> np.ndarray:
        return de_boor(self.control_points, self.knots, self.degree, float(np.clip(u, 0.0, 1.0)))

    def sample_by_distance(self, interval_mm: float) -> np.ndarray:
        """Oversample then resample by approximate arc length in millimetres."""
        dense, distance = self._dense_arc_length()
        if distance[-1] < 1e-8:
            return dense[:1]
        targets = np.arange(0.0, distance[-1], interval_mm)
        if targets.size == 0 or targets[-1] < distance[-1]:
            targets = np.append(targets, distance[-1])
        return self._interpolate_at_distance(dense, distance, targets)

    def sample_by_count(self, point_count: int) -> np.ndarray:
        """Return exactly point_count positions uniformly distributed by arc length."""
        if point_count < 2:
            raise ValueError("point_count must be at least 2")
        dense, distance = self._dense_arc_length()
        if distance[-1] < 1e-8:
            return np.repeat(dense[:1], point_count, axis=0)
        targets = np.linspace(0.0, distance[-1], point_count)
        return self._interpolate_at_distance(dense, distance, targets)

    def _dense_arc_length(self) -> tuple[np.ndarray, np.ndarray]:
        estimate_count = max(1000, len(self.control_points) * 250)
        dense_u = np.linspace(0.0, 1.0, estimate_count)
        dense = np.vstack([self.evaluate(u) for u in dense_u])
        segments = np.linalg.norm(np.diff(dense, axis=0), axis=1)
        distance = np.concatenate(([0.0], np.cumsum(segments)))
        return dense, distance

    @staticmethod
    def _interpolate_at_distance(dense: np.ndarray, distance: np.ndarray, targets: np.ndarray) -> np.ndarray:
        return np.column_stack((np.interp(targets, distance, dense[:, 0]), np.interp(targets, distance, dense[:, 1])))
