"""轨迹数据模型，保持与 B-spline 编辑器的字段兼容。"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math


@dataclass
class TrajectoryPoint:
    x: float
    y: float
    yaw: float = 0.0
    velocity: float = 0.0

    @classmethod
    def from_mapping(cls, value: dict) -> "TrajectoryPoint":
        yaw = value.get("yaw", value.get("theta", value.get("heading", 0.0)))
        return cls(float(value["x"]), float(value["y"]), float(yaw), float(value.get("velocity", value.get("speed", 0.0))))

    def to_dict(self) -> dict:
        return asdict(self)

    def finite(self) -> bool:
        return all(math.isfinite(v) for v in (self.x, self.y, self.yaw, self.velocity))
