from __future__ import annotations

import math
from typing import Sequence

from teleop.transform.calibration import IDENTITY_MATRIX_3X3
from teleop.transform.coordinate_transform import (
    DEFAULT_LEFT_AXIS_MATRIX_FROM_USER,
    DEFAULT_RIGHT_AXIS_MATRIX_FROM_USER,
)


Vec3 = tuple[float, float, float]
Mat3 = tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]


def _to_vec3(name: str, values: Sequence[float]) -> Vec3:
    output = tuple(float(v) for v in values)
    if len(output) != 3:
        raise ValueError(f"{name} must have length 3")
    if not all(math.isfinite(v) for v in output):
        raise ValueError(f"{name} must contain finite values")
    return (output[0], output[1], output[2])


def _to_mat3(name: str, values: Sequence[Sequence[float]]) -> Mat3:
    rows = tuple(tuple(float(v) for v in row) for row in values)
    if len(rows) != 3 or any(len(row) != 3 for row in rows):
        raise ValueError(f"{name} must be 3x3")
    if not all(math.isfinite(v) for row in rows for v in row):
        raise ValueError(f"{name} must contain finite values")
    return (rows[0], rows[1], rows[2])


def _mat3_vec3_mul(m: Mat3, v: Vec3) -> Vec3:
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def check_position_unit_conversion(
    delta_pico_m: Sequence[float],
    expected_delta_robot_mm: Sequence[float],
    tol_mm: float = 1e-9,
) -> bool:
    """Check m->mm conversion for a Cartesian delta vector."""
    delta_m = _to_vec3("delta_pico_m", delta_pico_m)
    expected_mm = _to_vec3("expected_delta_robot_mm", expected_delta_robot_mm)
    converted_mm = (delta_m[0] * 1000.0, delta_m[1] * 1000.0, delta_m[2] * 1000.0)
    return all(abs(a - b) <= float(tol_mm) for a, b in zip(converted_mm, expected_mm))


def check_reference_relative_target(
    pico_ref_m: Sequence[float],
    pico_now_m: Sequence[float],
    robot_ref_mm: Sequence[float],
    axis_matrix: Sequence[Sequence[float]],
    scale: float,
    r_user_from_pico: Sequence[Sequence[float]] = IDENTITY_MATRIX_3X3,
) -> Vec3:
    """Compute reference-relative robot target in mm for one arm."""
    ref_m = _to_vec3("pico_ref_m", pico_ref_m)
    now_m = _to_vec3("pico_now_m", pico_now_m)
    ref_robot_mm = _to_vec3("robot_ref_mm", robot_ref_mm)

    a_arm = _to_mat3("axis_matrix", axis_matrix)
    r_user = _to_mat3("r_user_from_pico", r_user_from_pico)
    scale_value = float(scale)
    if not math.isfinite(scale_value):
        raise ValueError("scale must be finite")

    delta_xr_m = (now_m[0] - ref_m[0], now_m[1] - ref_m[1], now_m[2] - ref_m[2])
    delta_user_m = _mat3_vec3_mul(r_user, delta_xr_m)
    delta_robot_m = _mat3_vec3_mul(a_arm, delta_user_m)
    delta_robot_mm = (
        scale_value * 1000.0 * delta_robot_m[0],
        scale_value * 1000.0 * delta_robot_m[1],
        scale_value * 1000.0 * delta_robot_m[2],
    )

    return (
        ref_robot_mm[0] + delta_robot_mm[0],
        ref_robot_mm[1] + delta_robot_mm[1],
        ref_robot_mm[2] + delta_robot_mm[2],
    )


def check_coordinate_axis_mapping(side: str) -> dict[str, Vec3]:
    """Return expected robot deltas for unit user directions (+X/+Y/+Z)."""
    side_norm = side.strip().lower()
    if side_norm == "right":
        axis_matrix = DEFAULT_RIGHT_AXIS_MATRIX_FROM_USER
    elif side_norm == "left":
        axis_matrix = DEFAULT_LEFT_AXIS_MATRIX_FROM_USER
    else:
        raise ValueError("side must be 'left' or 'right'")

    a_arm = _to_mat3("axis_matrix", axis_matrix)
    return {
        "user_+X": _mat3_vec3_mul(a_arm, (1.0, 0.0, 0.0)),
        "user_+Y": _mat3_vec3_mul(a_arm, (0.0, 1.0, 0.0)),
        "user_+Z": _mat3_vec3_mul(a_arm, (0.0, 0.0, 1.0)),
    }


def validate_ready_pose_config(
    left_ready_q_deg: Sequence[float],
    right_ready_q_deg: Sequence[float],
    left_ik_reference_q_deg: Sequence[float],
    right_ik_reference_q_deg: Sequence[float],
) -> list[str]:
    errors: list[str] = []

    left_ready = tuple(float(v) for v in left_ready_q_deg)
    right_ready = tuple(float(v) for v in right_ready_q_deg)
    left_ik_ref = tuple(float(v) for v in left_ik_reference_q_deg)
    right_ik_ref = tuple(float(v) for v in right_ik_reference_q_deg)

    if len(left_ready) != 7:
        errors.append("left_ready_q_deg must have length 7")
    if len(right_ready) != 7:
        errors.append("right_ready_q_deg must have length 7")

    if len(left_ready) == 7 and all(abs(v) <= 1e-12 for v in left_ready):
        errors.append("left_ready_q_deg must not be all zeros")
    if len(right_ready) == 7 and all(abs(v) <= 1e-12 for v in right_ready):
        errors.append("right_ready_q_deg must not be all zeros")

    if len(left_ready) == 7 and len(left_ik_ref) == 7 and left_ready == left_ik_ref:
        errors.append("left_ready_q_deg must be separate from left_ik_reference_q_deg")
    if len(right_ready) == 7 and len(right_ik_ref) == 7 and right_ready == right_ik_ref:
        errors.append("right_ready_q_deg must be separate from right_ik_reference_q_deg")

    return errors


def validate_command_safety_defaults(command_config: object) -> list[str]:
    errors: list[str] = []

    dry_run = bool(getattr(command_config, "dry_run", False))
    command_enabled = bool(getattr(command_config, "command_enabled", True))
    max_joint_step_deg = float(getattr(command_config, "max_joint_step_deg", 0.0))
    max_joint_velocity_deg_s = float(getattr(command_config, "max_joint_velocity_deg_s", 0.0))
    joint_step_limit_mode = str(getattr(command_config, "joint_step_limit_mode", "")).strip().lower()

    if dry_run is not True:
        errors.append("dry_run should default to True")
    if command_enabled is not False:
        errors.append("command_enabled should default to False")

    if max_joint_step_deg <= 0.0:
        errors.append("max_joint_step_deg must be positive")
    if max_joint_velocity_deg_s <= 0.0:
        errors.append("max_joint_velocity_deg_s must be positive")
    if joint_step_limit_mode != "reject":
        errors.append("joint_step_limit_mode should default to 'reject'")

    return errors


__all__ = [
    "check_position_unit_conversion",
    "check_reference_relative_target",
    "check_coordinate_axis_mapping",
    "validate_ready_pose_config",
    "validate_command_safety_defaults",
]
