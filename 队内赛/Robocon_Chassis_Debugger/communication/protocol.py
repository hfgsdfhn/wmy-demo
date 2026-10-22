"""Binary UART protocol shared by the desktop debugger and chassis firmware."""
from __future__ import annotations

import struct
from dataclasses import dataclass

from ..data.robot_state import RobotState
from .crc import crc16


HEADER = b"\xAA\x55"
MAX_PAYLOAD = 64
ID_STATE = 0x10
ID_HEALTH = 0x11
ID_DUAL_MCU_DIAGNOSTIC = 0x12
ID_MANUAL = 0x20
ID_COMMAND = 0x22
ID_MOTION_LIMITS = 0x23
ID_YAW_TARGET = 0x24
ID_YAW_PID_PARAMS = 0x25
ID_YAW_PID_PROFILE = 0x26
ID_DT35_DIRECTION = 0x28
ID_DT35_SEQUENCE = 0x29
ID_STR_PID_PARAMS = 0x2A
ID_DT35_TARGETS = 0x2B
ID_YAW_SWITCH_THRESHOLDS = 0x2C
ID_YAW_PID_PROFILE_PARAMS = 0x2D
ID_YAW_PID_MODE = 0x27
COMMANDS = {"START": 1, "STOP": 2, "RESET": 3, "ESTOP": 4}


@dataclass
class Frame:
    frame_id: int
    payload: bytes


@dataclass
class HealthState:
    timestamp_ms: int
    online_mask: int
    system_events: int
    can_error_mask: int
    pc_command: int = 0
    motor_feedback_mask: int = 0
    motor_tx_ok_count: int = 0
    motor_tx_failed_count: int = 0
    motor_target_erpm: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    motor_measured_speed_mps: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    motor_last_feedback_ms: int = 0

    def online(self, bit: int) -> bool:
        return bool(self.online_mask & (1 << bit))


@dataclass
class DualMcuDiagnostic:
    timestamp_ms: int
    last_valid_rx_ms: int
    rx_byte_count: int
    valid_frame_count: int
    rejected_frame_count: int
    freshness: int
    sequence: int
    control_mode: int
    action_id: int
    flags: int
    joy_left_x: int
    joy_left_y: int
    joy_right_x: int
    joy_right_y: int
    shoulder_left: int
    shoulder_right: int


def encode_frame(frame_id: int, payload: bytes = b"") -> bytes:
    body = struct.pack("<BH", frame_id, len(payload)) + payload
    return HEADER + body + struct.pack("<H", crc16(body))


class FrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[Frame]:
        self.buffer.extend(data)
        frames: list[Frame] = []
        while True:
            start = self.buffer.find(HEADER)
            if start < 0:
                if self.buffer.endswith(HEADER[:1]):
                    self.buffer[:] = HEADER[:1]
                else:
                    self.buffer.clear()
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 5:
                break
            frame_id, length = struct.unpack_from("<BH", self.buffer, 2)
            if length > MAX_PAYLOAD:
                del self.buffer[0]
                continue
            total = 7 + length
            if len(self.buffer) < total:
                break
            body = bytes(self.buffer[2:5 + length])
            received = struct.unpack_from("<H", self.buffer, 5 + length)[0]
            del self.buffer[:total]
            if crc16(body) == received:
                frames.append(Frame(frame_id, body[3:]))
        return frames


def encode_manual(vx: float, vy: float, yaw_rate_degps: float) -> bytes:
    """Encode a manual world-frame velocity command.

    vx and vy are world X/Y in m/s.  The chassis firmware rotates them into
    its body frame using the current yaw; yaw_rate_degps is unchanged.
    """
    return encode_frame(ID_MANUAL, struct.pack("<fff", vx, vy, yaw_rate_degps))


def encode_command(command: str) -> bytes:
    return encode_frame(ID_COMMAND, bytes([COMMANDS[command.upper()]]))


def encode_dt35_direction(direction: int) -> bytes:
    return encode_frame(ID_DT35_DIRECTION, struct.pack("<I", direction))


def encode_dt35_sequence(directions: tuple[int, ...]) -> bytes:
    return encode_frame(ID_DT35_SEQUENCE, struct.pack(
        "<" + "I" * len(directions), *directions))


def encode_motion_limits(max_linear_speed_mps: float, max_angular_speed_degps: float,
                         max_acceleration_mps2: float) -> bytes:
    return encode_frame(ID_MOTION_LIMITS, struct.pack(
        "<fff", max_linear_speed_mps, max_angular_speed_degps, max_acceleration_mps2))


def encode_yaw_target(yaw_deg: float) -> bytes:
    """Set the degree-based YAW PID target."""
    return encode_frame(ID_YAW_TARGET, struct.pack("<f", yaw_deg))


def encode_yaw_pid_params(kp: float, ki: float, kd: float,
                          max_output_degps: float, integral_limit: float,
                          integral_separation_deg: float,
                          deadzone_deg: float) -> bytes:
    """Configure the degree-based single IMU-yaw position PID.

    The wire protocol uses degrees and degrees per second.
    """
    return encode_frame(ID_YAW_PID_PARAMS, struct.pack(
        "<fffffff", kp, ki, kd, max_output_degps, integral_limit,
        integral_separation_deg, deadzone_deg))


def encode_yaw_pid_profile(profile: int) -> bytes:
    """Select a firmware-defined YAW PID profile (0=90, 1=Q/E, 2=180)."""
    if profile not in (0, 1, 2):
        raise ValueError(f"invalid YAW PID profile: {profile}")
    return encode_frame(ID_YAW_PID_PROFILE, bytes((profile,)))


def encode_yaw_pid_profile_params(profile: int, kp: float, ki: float, kd: float,
                                  max_output_degps: float, integral_limit: float,
                                  integral_separation_deg: float,
                                  deadzone_deg: float) -> bytes:
    """Update one persistent YAW PID profile (0=90, 1=heading hold, 2=180)."""
    if profile not in (0, 1, 2):
        raise ValueError(f"invalid YAW PID profile: {profile}")
    return encode_frame(ID_YAW_PID_PROFILE_PARAMS, struct.pack(
        "<Bfffffff", profile, kp, ki, kd, max_output_degps, integral_limit,
        integral_separation_deg, deadzone_deg))


def encode_str_pid_params(kp: float, ki: float, kd: float, max_output: float,
                          integral_limit: float, integral_separation: float,
                          deadzone: float) -> bytes:
    return encode_frame(ID_STR_PID_PARAMS, struct.pack(
        "<fffffff", kp, ki, kd, max_output, integral_limit,
        integral_separation, deadzone))


def encode_dt35_targets(back_to_front_m: float, front_to_back_m: float,
                        right_to_left_m: float, left_to_right_m: float) -> bytes:
    return encode_frame(ID_DT35_TARGETS, struct.pack(
        "<ffff", back_to_front_m, front_to_back_m,
        right_to_left_m, left_to_right_m))


def encode_yaw_switch_thresholds(large_threshold_deg: float,
                                 small_threshold_deg: float) -> bytes:
    return encode_frame(ID_YAW_SWITCH_THRESHOLDS, struct.pack(
        "<ff", large_threshold_deg, small_threshold_deg))


def encode_yaw_pid_mode(enabled: bool) -> bytes:
    """Enable or disable the IMU-yaw position loop."""
    return encode_frame(ID_YAW_PID_MODE, bytes((int(enabled),)))


def decode_state(payload: bytes) -> RobotState:
    legacy_format = "<IffffffI"
    current_format = "<IffffffffI"
    extended_format = "<IffffffffffI"
    fused_format = "<IffffffffffIff"
    if len(payload) == struct.calcsize(legacy_format):
        values = struct.unpack(legacy_format, payload)
        return RobotState(*values, imu_yaw_deg=values[3])
    if len(payload) == struct.calcsize(current_format):
        values = struct.unpack(current_format, payload)
        return RobotState(*values, imu_yaw_deg=values[3])
    if len(payload) == struct.calcsize(fused_format):
        values = struct.unpack(fused_format, payload)
        return RobotState(
            timestamp_ms=values[0], position_x_m=values[1], position_y_m=values[2],
            yaw_deg=values[3], velocity_x_mps=values[4], velocity_y_mps=values[5],
            yaw_rate_degps=values[6], dt35_distance_0_m=values[7],
            dt35_distance_1_m=values[8], imu_yaw_deg=values[9],
            chassis_yaw_deg=values[10], status_flags=values[11],
            fused_yaw_deg=values[12], fused_yaw_rate_radps=values[13])
    if len(payload) != struct.calcsize(extended_format):
        raise ValueError(f"state frame length invalid: {len(payload)}")
    values = struct.unpack(extended_format, payload)
    return RobotState(
        timestamp_ms=values[0], position_x_m=values[1], position_y_m=values[2],
        yaw_deg=values[3], velocity_x_mps=values[4], velocity_y_mps=values[5],
        yaw_rate_degps=values[6], dt35_distance_0_m=values[7],
        dt35_distance_1_m=values[8], imu_yaw_deg=values[9],
        chassis_yaw_deg=values[10], status_flags=values[11])


def decode_health(payload: bytes) -> HealthState:
    legacy_size = struct.calcsize("<IBII")
    if len(payload) == legacy_size:
        return HealthState(*struct.unpack("<IBII", payload))
    previous_format = "<IBIIBBIIffff"
    current_format = "<IBIIBBIIffffI"
    wheel_speed_format = "<IBIIBBIIffffffffI"
    if len(payload) == struct.calcsize(previous_format):
        values = struct.unpack(previous_format, payload)
        return HealthState(*values[:8], tuple(values[8:12]))
    if len(payload) == struct.calcsize(wheel_speed_format):
        values = struct.unpack(wheel_speed_format, payload)
        return HealthState(*values[:8], tuple(values[8:12]), tuple(values[12:16]), values[16])
    if len(payload) != struct.calcsize(current_format):
        raise ValueError(f"health frame length invalid: {len(payload)}")
    values = struct.unpack(current_format, payload)
    return HealthState(*values[:8], tuple(values[8:12]), (0.0, 0.0, 0.0, 0.0), values[12])


def decode_dual_mcu_diagnostic(payload: bytes) -> DualMcuDiagnostic:
    frame_format = "<IIIIIBBBBBhhhhhh"
    if len(payload) != struct.calcsize(frame_format):
        raise ValueError(f"dual MCU diagnostic frame length invalid: {len(payload)}")
    return DualMcuDiagnostic(*struct.unpack(frame_format, payload))
