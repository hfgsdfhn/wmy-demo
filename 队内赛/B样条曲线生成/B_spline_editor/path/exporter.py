"""C source exporter for STM32 path follower projects."""
from __future__ import annotations


def export_c_array(path_points, array_name: str = "path") -> str:
    lines = [
        "/* theta: yaw, rad (0 = +Y forward, counter-clockwise positive);",
        " * curvature: 1/m, s: mm, velocity: m/s */",
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


def export_mpc_c_array(states, sample_time: float, array_name: str = "mpc_reference") -> str:
    """Export time-sampled MPC references as a C header."""
    if not states:
        raise ValueError("没有可导出的 MPC 状态")
    lines = [
        "/* MPC reference states resampled by time.",
        f" * sample time: {sample_time:.6f} s; the last state is the path end and",
        " * its interval may be shorter than the sample time.",
        " * theta: yaw, rad, unwrapped (0 = +Y forward, CCW positive);",
        " * t: s, x/y: mm, v: m/s, omega: rad/s,",
        " * accel: m/s^2, kappa: 1/m, s: mm */",
        "typedef struct",
        "{",
        "    float t;",
        "    float x;",
        "    float y;",
        "    float theta;",
        "    float v;",
        "    float omega;",
        "    float accel;",
        "    float kappa;",
        "    float s;",
        "} MpcState;",
        "",
        f"#define MPC_SAMPLE_TIME {sample_time:.6f}f",
        f"#define MPC_STATE_COUNT {len(states)}",
        "",
        f"const MpcState {array_name}[MPC_STATE_COUNT] =",
        "{",
    ]
    lines.extend(
        "    "
        f"{{{state.time:.4f}f, {state.x:.1f}f, {state.y:.1f}f, {state.theta:.4f}f, "
        f"{state.velocity:.3f}f, {state.omega:.3f}f, {state.acceleration:.2f}f, "
        f"{_format_significant(state.curvature)}f, {state.s:.1f}f}},"
        for state in states
    )
    lines.extend(("};", ""))
    return "\n".join(lines)


def export_mpc_csv(states) -> str:
    """Export time-sampled MPC references as CSV for MATLAB/Python analysis."""
    if not states:
        raise ValueError("没有可导出的 MPC 状态")
    lines = [
        "step,time_s,x_mm,y_mm,theta_rad,v_mps,omega_radps,accel_mps2,kappa_1pm,s_mm"
    ]
    lines.extend(
        f"{state.step},{state.time:.6f},{state.x:.3f},{state.y:.3f},"
        f"{state.theta:.6f},{state.velocity:.6f},{state.omega:.6f},"
        f"{state.acceleration:.6f},{state.curvature:.6f},{state.s:.3f}"
        for state in states
    )
    return "\n".join(lines) + "\n"


def _format_significant(value: float) -> str:
    """Format a C floating literal with three significant digits."""
    if abs(value) < 1e-9:
        return "0.0"
    text = f"{value:.3g}"
    return f"{text}.0" if "e" not in text.lower() and "." not in text else text
