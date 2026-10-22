"""C source exporter for STM32 path follower projects."""
from __future__ import annotations


def export_c_array(path_points, array_name: str = "path") -> str:
    lines = [
        "/* theta: rad, curvature: 1/m, s: mm, velocity: m/s */",
        "typedef struct",
        "{",
        "    float x;",
        "    float y;",
        "    float theta;",
        "    float curvature;",
        "    float s;",
        "    float velocity;",
        "} PathPoint;",
        "",
        f"#define PATH_SIZE {len(path_points)}",
        "",
        f"const PathPoint {array_name}[PATH_SIZE] =",
        "{",
    ]
    lines.extend(
        "    "
        f"{{{point.x:.0f}, {point.y:.0f}, {point.theta:.3f}f, "
        f"{_format_significant(point.curvature)}f, {point.s:.0f}, {point.velocity:.3f}f}},"
        for point in path_points
    )
    lines.extend(("};", ""))
    return "\n".join(lines)


def _format_significant(value: float) -> str:
    """Format a C floating literal with three significant digits."""
    if abs(value) < 1e-9:
        return "0.0"
    text = f"{value:.3g}"
    return f"{text}.0" if "e" not in text.lower() and "." not in text else text
