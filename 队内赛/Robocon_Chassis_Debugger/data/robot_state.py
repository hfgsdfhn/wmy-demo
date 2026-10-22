"""STM32 实时状态模型。"""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class RobotState:
    timestamp_ms: int = 0
    position_x_m: float = 0.0
    position_y_m: float = 0.0
    yaw_deg: float = 0.0
    velocity_x_mps: float = 0.0
    velocity_y_mps: float = 0.0
    yaw_rate_degps: float = 0.0
    dt35_distance_0_m: float = 0.0
    dt35_distance_1_m: float = 0.0
    status_flags: int = 0
    imu_yaw_deg: float = 0.0
    chassis_yaw_deg: float = 0.0
    fused_yaw_deg: float = 0.0
    fused_yaw_rate_radps: float = 0.0

    @property
    def speed_mps(self) -> float:
        return math.hypot(self.velocity_x_mps, self.velocity_y_mps)

    @property
    def status_text(self) -> str:
        return "FAULT" if self.status_flags & 0x01 else ("RUNNING" if self.status_flags & 0x02 else "NORMAL")
