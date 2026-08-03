"""Basis function utilities for clamped B-spline curves."""
from __future__ import annotations

import numpy as np


def clamped_uniform_knots(control_count: int, degree: int = 3) -> np.ndarray:
    """Return a normalized clamped uniform knot vector."""
    if control_count < degree + 1:
        raise ValueError(f"degree {degree} needs at least {degree + 1} control points")
    interior_count = control_count - degree - 1
    interior = np.arange(1, interior_count + 1, dtype=float) / (interior_count + 1)
    return np.concatenate((np.zeros(degree + 1), interior, np.ones(degree + 1)))


def find_span(knots: np.ndarray, degree: int, u: float) -> int:
    """Find the knot span containing u."""
    n = len(knots) - degree - 2
    if u >= knots[n + 1]:
        return n
    return int(np.searchsorted(knots, u, side="right") - 1)


def de_boor(control_points: np.ndarray, knots: np.ndarray, degree: int, u: float) -> np.ndarray:
    """Evaluate a B-spline using the stable De Boor recursion."""
    span = find_span(knots, degree, u)
    work = np.array(control_points[span - degree : span + 1], dtype=float)
    for level in range(1, degree + 1):
        for j in range(degree, level - 1, -1):
            i = span - degree + j
            denominator = knots[i + degree - level + 1] - knots[i]
            alpha = 0.0 if denominator == 0 else (u - knots[i]) / denominator
            work[j] = (1.0 - alpha) * work[j - 1] + alpha * work[j]
    return work[degree]
