"""场地固定坐标与用户坐标系之间的正交变换。"""
from __future__ import annotations

import math
from dataclasses import dataclass


ORIGINS = ("bottom_left", "top_left", "bottom_right", "top_right")
DIRECTIONS = (
    ("positive_right", "positive_up"),
    ("positive_right", "positive_down"),
    ("positive_left", "positive_up"),
    ("positive_left", "positive_down"),
)


@dataclass(frozen=True)
class CoordinateSystem:
    origin: str = "bottom_left"
    x_direction: str = "positive_right"
    y_direction: str = "positive_up"
    origin_offset: tuple[float, float] = (0.0, 0.0)

    def __post_init__(self):
        if self.origin not in ORIGINS:
            raise ValueError("无效的坐标原点")
        if (self.x_direction, self.y_direction) not in DIRECTIONS:
            raise ValueError("无效的坐标轴方向")
        if len(self.origin_offset) != 2 or not all(math.isfinite(float(value)) for value in self.origin_offset):
            raise ValueError("无效的坐标原点偏移")

    @property
    def signs(self) -> tuple[int, int]:
        return (
            1 if self.x_direction == "positive_right" else -1,
            1 if self.y_direction == "positive_up" else -1,
        )

    def origin_point(self, width: float, height: float) -> tuple[float, float]:
        base_x = width if "right" in self.origin else 0.0
        base_y = height if "top" in self.origin else 0.0
        return base_x + self.origin_offset[0], base_y + self.origin_offset[1]

    def base_origin(self, width: float, height: float) -> tuple[float, float]:
        return (
            width if "right" in self.origin else 0.0,
            height if "top" in self.origin else 0.0,
        )

    def to_display(self, point, width: float, height: float) -> tuple[float, float]:
        """转换到不镜像地图的视图坐标。"""
        ox, oy = self.origin_point(width, height)
        return float(point[0]) - ox, float(point[1]) - oy

    def to_field(self, point, width: float, height: float) -> tuple[float, float]:
        """将视图坐标还原为场地固定坐标。"""
        ox, oy = self.origin_point(width, height)
        return ox + float(point[0]), oy + float(point[1])

    def to_coordinate(self, point, width: float, height: float) -> tuple[float, float]:
        ox, oy = self.origin_point(width, height)
        sx, sy = self.signs
        return sx * (float(point[0]) - ox), sy * (float(point[1]) - oy)

    def from_coordinate(self, point, width: float, height: float) -> tuple[float, float]:
        ox, oy = self.origin_point(width, height)
        sx, sy = self.signs
        return ox + sx * float(point[0]), oy + sy * float(point[1])

    def field_rect(self, width: float, height: float) -> tuple[float, float, float, float]:
        ox, oy = self.origin_point(width, height)
        sx, sy = self.signs
        return sx * (0.0 - ox), sy * (height - oy), sx * width, -sy * height

    def transform_angle(self, theta: float) -> float:
        sx, sy = self.signs
        return math.atan2(sy * math.sin(theta), sx * math.cos(theta))

    def transform_curvature(self, curvature: float) -> float:
        sx, sy = self.signs
        return curvature * sx * sy
