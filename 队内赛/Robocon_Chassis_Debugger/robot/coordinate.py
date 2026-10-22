"""世界坐标与屏幕坐标转换。"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FieldCoordinate:
    width_m: float
    height_m: float

    def clamp(self, x: float, y: float) -> tuple[float, float]:
        return min(max(0.0, x), self.width_m), min(max(0.0, y), self.height_m)

    def pixel_to_world(self, px: float, py: float, image_width: float, image_height: float) -> tuple[float, float]:
        return px / image_width * self.width_m, (1.0 - py / image_height) * self.height_m

    def world_to_pixel(self, x: float, y: float, image_width: float, image_height: float) -> tuple[float, float]:
        return x / self.width_m * image_width, (1.0 - y / self.height_m) * image_height
