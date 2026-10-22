"""全向底盘几何模型。"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np


@dataclass
class RobotModel:
    length_m: float = 0.7
    width_m: float = 0.7
    safe_margin_m: float = 0.0

    @property
    def collision_radius(self) -> float:
        return math.hypot(self.length_m / 2, self.width_m / 2) + self.safe_margin_m

    def footprint(self, x: float, y: float, yaw: float) -> np.ndarray:
        corners = np.array([[self.length_m / 2, self.width_m / 2], [self.length_m / 2, -self.width_m / 2], [-self.length_m / 2, -self.width_m / 2], [-self.length_m / 2, self.width_m / 2]])
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        return corners @ rotation.T + np.array([x, y])
