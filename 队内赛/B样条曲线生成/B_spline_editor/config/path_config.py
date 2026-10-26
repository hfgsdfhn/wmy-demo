"""Serialization and validation for editable B-spline path configurations."""
from __future__ import annotations

import json
import math
from pathlib import Path

from .coordinate_system import CoordinateSystem, ORIGINS, DIRECTIONS
from path.mpc_generator import DEFAULT_SAMPLE_TIME, MAX_SAMPLE_TIME, MIN_SAMPLE_TIME


CONFIG_FORMAT = "robocon-bspline-path"
CONFIG_VERSION = 3
DEFAULT_CONFIG_DIRECTORY = Path(__file__).resolve().parents[1] / "saved_paths"


class PathConfigError(ValueError):
    """Raised when a path configuration file cannot be used safely."""


def save_path_config(filename: Path, config: dict) -> None:
    """Validate and write an editor configuration as readable JSON."""
    validated = validate_path_config(config)
    filename.parent.mkdir(parents=True, exist_ok=True)
    filename.write_text(json.dumps(validated, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def load_path_config(filename: Path) -> dict:
    """Read and validate a configuration without altering editor state."""
    try:
        data = json.loads(filename.read_text(encoding="utf-8"))
    except OSError as error:
        raise PathConfigError(f"无法读取文件：{error}") from error
    except json.JSONDecodeError as error:
        raise PathConfigError(f"不是有效的 JSON 配置文件：{error.msg}") from error
    return validate_path_config(data)


def validate_path_config(data: object) -> dict:
    """Return a normalized, complete configuration or raise PathConfigError."""
    root = _mapping(data, "配置根对象")
    if root.get("format") != CONFIG_FORMAT:
        raise PathConfigError("不是本编辑器生成的路径配置文件")
    version = root.get("version")
    if version not in (1, 2, CONFIG_VERSION):
        raise PathConfigError("不支持的配置文件版本")
    field = _mapping(root.get("field"), "field")
    width = _number(field.get("width_mm"), "field.width_mm", 100.0, 100000.0)
    height = _number(field.get("height_mm"), "field.height_mm", 100.0, 100000.0)
    sampling = _mapping(root.get("sampling"), "sampling")
    mode = sampling.get("mode")
    if mode not in ("interval", "count"):
        raise PathConfigError("sampling.mode 必须是 interval 或 count")
    interval = _number(sampling.get("interval_mm"), "sampling.interval_mm", 1.0, 1000.0)
    count = _integer(sampling.get("count"), "sampling.count", 2, 10000)
    speed = _mapping(root.get("speed"), "speed")
    lateral_acceleration = _number(speed.get("lateral_acceleration"), "speed.lateral_acceleration", 0.01, 100.0)
    speed_scale = _number(speed.get("speed_scale"), "speed.speed_scale", 0.0, 1.0)
    acceleration = _number(speed.get("acceleration"), "speed.acceleration", 0.01, 100.0)
    deceleration = _number(speed.get("deceleration"), "speed.deceleration", 0.01, 100.0)
    mpc = _mapping(root.get("mpc", {}), "mpc")
    sample_time = _number(
        mpc.get("sample_time", DEFAULT_SAMPLE_TIME),
        "mpc.sample_time",
        MIN_SAMPLE_TIME,
        MAX_SAMPLE_TIME,
    )
    raw_points = root.get("control_points")
    if not isinstance(raw_points, list):
        raise PathConfigError("control_points 必须是数组")
    points = []
    for index, point in enumerate(raw_points, start=1):
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise PathConfigError(f"第 {index} 个控制点必须包含 X、Y 两个数值")
        x = _number(point[0], f"第 {index} 个控制点 X", 0.0, width)
        y = _number(point[1], f"第 {index} 个控制点 Y", 0.0, height)
        points.append([x, y])
    raw_coordinate_system = root.get("coordinate_system", {})
    if version == 1 and "coordinate_system" not in root:
        raw_coordinate_system = {}
    coordinate_system = _mapping(raw_coordinate_system, "coordinate_system")
    origin = coordinate_system.get("origin", "bottom_left")
    raw_offset = coordinate_system.get("origin_offset_mm", [0.0, 0.0])
    if not isinstance(raw_offset, (list, tuple)) or len(raw_offset) != 2:
        raise PathConfigError("coordinate_system.origin_offset_mm 必须包含两个数值")
    offset_x = _number(raw_offset[0], "coordinate_system.origin_offset_mm[0]", -100000.0, 100000.0)
    offset_y = _number(raw_offset[1], "coordinate_system.origin_offset_mm[1]", -100000.0, 100000.0)
    x_direction = coordinate_system.get("x_direction", "positive_right")
    y_direction = coordinate_system.get("y_direction", "positive_up")
    if origin not in ORIGINS:
        raise PathConfigError("coordinate_system.origin 无效")
    if (x_direction, y_direction) not in DIRECTIONS:
        raise PathConfigError("coordinate_system 的坐标轴方向无效")
    system = CoordinateSystem(origin, x_direction, y_direction, (offset_x, offset_y))
    origin_x, origin_y = system.origin_point(width, height)
    if not 0.0 <= origin_x <= width or not 0.0 <= origin_y <= height:
        raise PathConfigError("coordinate_system.origin_offset_mm 使原点超出场地范围")
    return {
        "format": CONFIG_FORMAT, "version": CONFIG_VERSION, "control_points": points,
        "field": {"width_mm": width, "height_mm": height},
        "sampling": {"mode": mode, "interval_mm": interval, "count": count},
        "speed": {"lateral_acceleration": lateral_acceleration, "speed_scale": speed_scale,
                  "acceleration": acceleration, "deceleration": deceleration},
        "mpc": {"sample_time": sample_time},
        "coordinate_system": {
            "origin": origin, "origin_offset_mm": [offset_x, offset_y],
            "x_direction": x_direction, "y_direction": y_direction,
        },
    }


def _mapping(value: object, name: str) -> dict:
    if not isinstance(value, dict):
        raise PathConfigError(f"{name} 必须是对象")
    return value


def _number(value: object, name: str, minimum: float, maximum: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PathConfigError(f"{name} 必须是数值")
    value = float(value)
    if not math.isfinite(value) or not minimum <= value <= maximum:
        raise PathConfigError(f"{name} 必须在 {minimum:g} 到 {maximum:g} 之间")
    return value


def _integer(value: object, name: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise PathConfigError(f"{name} 必须是 {minimum} 到 {maximum} 的整数")
    return value
