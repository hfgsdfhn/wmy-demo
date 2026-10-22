"""兼容 JSON、CSV 以及 B-spline 编辑器导出的 C 头文件。"""
from __future__ import annotations

import csv
import json
from pathlib import Path
import re
from ..data.trajectory_data import TrajectoryPoint


class TrajectoryLoadError(ValueError):
    pass


def _normalize(points: list[TrajectoryPoint]) -> list[TrajectoryPoint]:
    if not points:
        return points
    # 旧编辑器使用 mm；调试器内部统一使用 m。
    if max(max(abs(p.x), abs(p.y)) for p in points) > 20.0:
        return [TrajectoryPoint(p.x / 1000.0, p.y / 1000.0, p.yaw, p.velocity) for p in points]
    return points


def load_trajectory(filename: str | Path) -> list[TrajectoryPoint]:
    path = Path(filename)
    if not path.exists():
        raise TrajectoryLoadError(f"文件不存在：{path}")
    suffix = path.suffix.lower()
    try:
        if suffix == ".json":
            payload = json.loads(path.read_text(encoding="utf-8-sig"))
            values = payload.get("trajectory", payload.get("points", payload)) if isinstance(payload, dict) else payload
            points = [_point_from_item(item) for item in values]
        elif suffix in (".csv", ".txt"):
            with path.open(encoding="utf-8-sig", newline="") as handle:
                points = [TrajectoryPoint.from_mapping(row) for row in csv.DictReader(handle)]
        elif suffix in (".h", ".hpp", ".c"):
            points = _load_c_array(path.read_text(encoding="utf-8", errors="ignore"))
        else:
            raise TrajectoryLoadError("支持 JSON、CSV 和 C 头文件")
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        raise TrajectoryLoadError(f"轨迹格式错误：{error}") from error
    if not points or any(not point.finite() for point in points):
        raise TrajectoryLoadError("轨迹为空或包含无效数字")
    return _normalize(points)


def _point_from_item(item) -> TrajectoryPoint:
    if isinstance(item, dict):
        return TrajectoryPoint.from_mapping(item)
    if isinstance(item, (list, tuple)) and len(item) >= 2:
        return TrajectoryPoint(float(item[0]), float(item[1]), float(item[2]) if len(item) > 2 else 0.0, float(item[3]) if len(item) > 3 else 0.0)
    raise TrajectoryLoadError("轨迹点必须是对象或数组")


def _load_c_array(source: str) -> list[TrajectoryPoint]:
    points: list[TrajectoryPoint] = []
    # B-spline editor 输出 {x, y, theta, curvature, s, velocity}。
    for match in re.finditer(r"\{\s*([^{}]+?)\s*\}", source):
        values = [v.strip().rstrip("fF") for v in match.group(1).split(",")]
        if len(values) < 4:
            continue
        try:
            points.append(TrajectoryPoint(float(values[0]), float(values[1]), float(values[2]), float(values[-1])))
        except ValueError:
            continue
    if not points:
        raise TrajectoryLoadError("未在 C 文件中找到轨迹数组")
    return points


def save_trajectory(filename: str | Path, points: list[TrajectoryPoint]) -> None:
    Path(filename).write_text(json.dumps({"format": "robocon_trajectory_v1", "units": "m", "trajectory": [p.to_dict() for p in points]}, ensure_ascii=False, indent=2), encoding="utf-8")
