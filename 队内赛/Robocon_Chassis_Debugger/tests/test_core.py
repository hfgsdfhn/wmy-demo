from __future__ import annotations

import struct
import math
import os
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5 import QtCore, QtWidgets

from Robocon_Chassis_Debugger.communication.protocol import (
    FrameParser, decode_dual_mcu_diagnostic, decode_health, decode_state,
    encode_command, encode_manual, encode_motion_limits, encode_yaw_pid_mode, encode_yaw_pid_params,
    encode_yaw_pid_profile, encode_yaw_target,
)
from Robocon_Chassis_Debugger.data.robot_state import RobotState
from Robocon_Chassis_Debugger.ui.main_window import (
    MainWindow, YAW_PID_PROFILE_90, YAW_PID_PROFILE_QE,
)

_QT_APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
_TEST_WINDOWS: list[MainWindow] = []


class FakeSerial:
    connected = True

    def __init__(self):
        self.frames: list[bytes] = []

    def write(self, data: bytes) -> bool:
        self.frames.append(data)
        return True


def _window() -> MainWindow:
    window = MainWindow(Path("missing-test-config.json"))
    window.serial = FakeSerial()
    window.drive_enabled = True
    _TEST_WINDOWS.append(window)
    return window


def _decode_frames(window: MainWindow):
    parser = FrameParser()
    return [parser.feed(frame)[0] for frame in window.serial.frames]


def test_protocol_round_trip():
    frame = encode_manual(0.4, -0.2, 1.0)
    decoded = FrameParser().feed(frame)
    assert decoded and decoded[0].frame_id == 0x20
    assert all(math.isclose(a, b, rel_tol=1e-6) for a, b in zip(struct.unpack("<fff", decoded[0].payload), (0.4, -0.2, 1.0)))


def test_state_decode():
    payload = struct.pack("<IffffffffI", 100, 1.0, 2.0, .5, .2, .3, .1,
                          0.0, 0.0, 2)
    state = decode_state(payload)
    assert isinstance(state, RobotState)
    assert state.status_text == "RUNNING"


def test_state_decode_uses_imu_yaw():
    payload = struct.pack("<IffffffffI", 100, 1.0, 2.0, .5, .2, .3, .1,
                          0.0, 0.0, 2)
    state = decode_state(payload)
    assert math.isclose(state.yaw_deg, .5)
    assert math.isclose(state.imu_yaw_deg, .5)


def test_extended_state_decode_exposes_all_yaw_sources():
    payload = struct.pack("<IffffffffffI", 100, 1.0, 2.0, 30.0, .2, .3,
                          .1, 0.0, 0.0, 31.0, 29.0, 2)
    state = decode_state(payload)
    assert math.isclose(state.yaw_deg, 30.0)
    assert math.isclose(state.imu_yaw_deg, 31.0)
    assert math.isclose(state.chassis_yaw_deg, 29.0)


def test_fused_state_decode_exposes_ekf_yaw():
    payload = struct.pack("<IffffffffffIff", 100, 1.0, 2.0, 30.0, .2, .3,
                          .1, 0.0, 0.0, 31.0, 29.0, 2, 30.5, .12)
    state = decode_state(payload)
    assert math.isclose(state.fused_yaw_deg, 30.5)
    assert math.isclose(state.fused_yaw_rate_radps, .12, rel_tol=1e-6)


def test_motion_limits_round_trip():
    frame = encode_motion_limits(1.2, 2.4, 3.6)
    decoded = FrameParser().feed(frame)
    assert decoded[0].frame_id == 0x23
    assert all(math.isclose(a, b, rel_tol=1e-6) for a, b in zip(struct.unpack("<fff", decoded[0].payload), (1.2, 2.4, 3.6)))


def test_yaw_pid_frames_round_trip():
    target = FrameParser().feed(encode_yaw_target(90.0))[0]
    assert target.frame_id == 0x24
    assert math.isclose(struct.unpack("<f", target.payload)[0], 90.0, rel_tol=1e-6)
    params = FrameParser().feed(encode_yaw_pid_params(
        2.5, 2.5, .0, 2.6, .5, .349066, .017453))[0]
    assert params.frame_id == 0x25
    assert all(math.isclose(a, b, rel_tol=1e-6) for a, b in zip(
        struct.unpack("<fffffff", params.payload),
        (2.5, 2.5, .0, 2.6, .5, .349066, .017453)))


def test_yaw_pid_mode_round_trip():
    mode = FrameParser().feed(encode_yaw_pid_mode(True))[0]
    assert mode.frame_id == 0x27
    assert mode.payload == b"\x01"


def test_yaw_pid_profile_round_trip():
    profile = FrameParser().feed(encode_yaw_pid_profile(YAW_PID_PROFILE_QE))[0]
    assert profile.frame_id == 0x26
    assert profile.payload == bytes((YAW_PID_PROFILE_QE,))


def test_health_decode():
    health = decode_health(struct.pack("<IBII", 123, 0b1011, 4, 8))
    assert health.timestamp_ms == 123
    assert health.online(0) and not health.online(2)


def test_extended_health_decode():
    payload = struct.pack(
        "<IBIIBBIIffff", 456, 1, 0x11, 0, 1, 0xF, 1200, 3,
        1550.0, 1550.0, 1550.0, 1550.0)
    health = decode_health(payload)
    assert health.pc_command == 1
    assert health.motor_feedback_mask == 0xF
    assert health.motor_tx_ok_count == 1200
    assert health.motor_tx_failed_count == 3
    assert health.motor_target_erpm == (1550.0, 1550.0, 1550.0, 1550.0)


def test_health_decode_with_measured_wheel_speeds():
    payload = struct.pack(
        "<IBIIBBIIffffffffI", 456, 1, 0x11, 0, 1, 0xF, 1200, 3,
        1550.0, 1550.0, 1550.0, 1550.0,
        0.10, -0.20, 0.30, -0.40, 789)
    health = decode_health(payload)
    assert all(math.isclose(actual, expected, rel_tol=1e-6) for actual, expected in zip(
        health.motor_measured_speed_mps, (0.10, -0.20, 0.30, -0.40)))
    assert health.motor_last_feedback_ms == 789


def test_dual_mcu_diagnostic_decode():
    payload = struct.pack(
        "<IIIIIBBBBBhhhhhh", 1000, 950, 12345, 11, 2,
        1, 7, 1, 1, 0x05, -255, 255, -12, 34, 64, -64)
    diagnostic = decode_dual_mcu_diagnostic(payload)
    assert diagnostic.timestamp_ms == 1000
    assert diagnostic.last_valid_rx_ms == 950
    assert diagnostic.rx_byte_count == 12345
    assert diagnostic.valid_frame_count == 11
    assert diagnostic.rejected_frame_count == 2
    assert diagnostic.freshness == 1
    assert diagnostic.sequence == 7
    assert diagnostic.flags == 0x05
    assert (diagnostic.joy_left_x, diagnostic.joy_left_y) == (-255, 255)
    assert (diagnostic.shoulder_left, diagnostic.shoulder_right) == (64, -64)


def test_parser_keeps_split_header():
    parser = FrameParser()
    frame = encode_command("START")
    assert parser.feed(b"noise\xaa") == []
    decoded = parser.feed(frame[1:])
    assert decoded and decoded[0].frame_id == 0x22


def test_parser_recovers_from_invalid_length():
    parser = FrameParser()
    invalid = b"\xaa\x55\x20\xff\xff"
    decoded = parser.feed(invalid + encode_manual(0.1, 0.0, 0.0))
    assert decoded and decoded[0].frame_id == 0x20


def test_relative_90_rotation_sends_preset_before_target():
    window = _window()
    window.latest_yaw_deg = 135.0

    window._turn_relative_90(-90.0)
    frames = _decode_frames(window)

    assert [frame.frame_id for frame in frames] == [0x26, 0x24]
    assert frames[0].payload == bytes((YAW_PID_PROFILE_90,))
    assert math.isclose(struct.unpack("<f", frames[1].payload)[0], 45.0)

    window._turn_relative_90(90.0)
    frames = _decode_frames(window)
    assert [frame.frame_id for frame in frames[2:]] == [0x26, 0x24]
    assert frames[2].payload == bytes((YAW_PID_PROFILE_90,))
    assert math.isclose(struct.unpack("<f", frames[-1].payload)[0], -135.0)


def test_qe_switches_preset_once_and_restores_90_after_release():
    window = _window()

    window._set_control_key(QtCore.Qt.Key_Q, True)
    window._set_control_key(QtCore.Qt.Key_Q, True)
    frames = _decode_frames(window)
    assert [frame.frame_id for frame in frames] == [0x26]
    assert frames[0].payload == bytes((YAW_PID_PROFILE_QE,))

    window._set_control_key(QtCore.Qt.Key_Q, False)
    frames = _decode_frames(window)
    assert [frame.frame_id for frame in frames] == [0x26, 0x26]
    assert frames[1].payload == bytes((YAW_PID_PROFILE_90,))


def test_qe_pid_steps_target_once_and_keepalive_does_not_accumulate():
    window = _window()
    window.yaw_target_deg = 10.0
    window.yaw_target_initialized = True

    window._set_control_key(QtCore.Qt.Key_Q, True)
    window._set_control_key(QtCore.Qt.Key_Q, True)
    window._send_manual_keepalive()
    frames = _decode_frames(window)

    assert [frame.frame_id for frame in frames] == [0x26, 0x24, 0x20]
    assert math.isclose(struct.unpack("<f", frames[1].payload)[0], 15.0)
    assert math.isclose(window.yaw_target_deg, 15.0)


def test_qe_pid_repeats_step_while_held():
    window = _window()
    window.yaw_target_deg = 10.0
    window.yaw_target_initialized = True
    window._set_control_key(QtCore.Qt.Key_Q, True)
    for _ in range(5):
        window._send_manual_keepalive()
    assert math.isclose(window.yaw_target_deg, 20.0)


def test_qe_without_pid_keeps_direct_manual_rotation():
    window = _window()
    window.yaw_pid_enable_check.setChecked(False)
    window._set_control_key(QtCore.Qt.Key_E, True)
    window._send_manual_keepalive()
    frames = _decode_frames(window)

    assert frames[-1].frame_id == 0x20
    assert math.isclose(struct.unpack("<fff", frames[-1].payload)[2], -45.0)


def test_relative_90_rotation_rejects_missing_connection_enable_or_yaw():
    window = _window()
    window.serial.connected = False
    window._turn_relative_90(90.0)
    assert window.serial.frames == []

    window.serial.connected = True
    window.drive_enabled = False
    window._turn_relative_90(90.0)
    assert window.serial.frames == []

    window.drive_enabled = True
    window._turn_relative_90(90.0)
    assert window.serial.frames == []
