from __future__ import annotations

from enum import Enum
import time
from typing import Any

from teleop.core.command_frame import CommandLoopDiagnostics
from teleop.core.robot_frame import DualArmRobotFeedback, DualArmRobotTarget, RobotArmFeedback, RobotArmTarget
from teleop.logging.async_logger import LoggingStats
from teleop.safety.state_machine import SafetyDecision
from teleop.ui.snapshot import (
    ArmVisualizationSnapshot,
    TeleopVisualizationSnapshot,
    compute_error_norm_mm,
)


def build_arm_visualization_snapshot(
    target: RobotArmTarget | None,
    feedback: RobotArmFeedback | None,
    *,
    calibrated: bool = False,
    active: bool = False,
    status: str = "",
) -> ArmVisualizationSnapshot:
    target_xyz = _vec3_or_none(getattr(target, "position_xyz", None))
    feedback_xyz = _vec3_or_none(getattr(feedback, "position_xyz", None))

    target_abc = _vec3_or_none(getattr(target, "orientation_abc", None))
    feedback_abc = _vec3_or_none(getattr(feedback, "orientation_abc", None))

    target_valid = bool(getattr(target, "valid", False)) if target is not None else False
    feedback_valid = bool(getattr(feedback, "valid", False)) if feedback is not None else False

    error_norm_mm = None
    if target_valid and feedback_valid:
        error_norm_mm = compute_error_norm_mm(target_xyz, feedback_xyz)

    return ArmVisualizationSnapshot(
        target_xyz_mm=target_xyz,
        feedback_xyz_mm=feedback_xyz,
        target_abc_deg=target_abc,
        feedback_abc_deg=feedback_abc,
        target_valid=target_valid,
        feedback_valid=feedback_valid,
        calibrated=bool(calibrated),
        active=bool(active),
        error_norm_mm=error_norm_mm,
        status=str(status),
    )


def build_visualization_snapshot(
    *,
    timestamp_ns: int | None = None,
    target: DualArmRobotTarget | None = None,
    feedback: DualArmRobotFeedback | None = None,
    safety_decision: SafetyDecision | None = None,
    command_diagnostics: CommandLoopDiagnostics | None = None,
    logging_stats: LoggingStats | None = None,
    pico_connected: bool = False,
    robot_connected: bool = False,
    pico_frame_age_ms: float | None = None,
    enable_left: bool | None = None,
    enable_right: bool | None = None,
    left_calibrated: bool | None = None,
    right_calibrated: bool | None = None,
    ik_status: str = "",
    sdk_status: str = "",
    teleop_mode: str = "position_only",
    orientation_tracking_enabled: bool = False,
    orientation_relative_mode: str = "",
    left_relative_angle_deg: float | None = None,
    right_relative_angle_deg: float | None = None,
) -> TeleopVisualizationSnapshot:
    ts_ns = int(time.time_ns() if timestamp_ns is None else timestamp_ns)

    left_target = target.left if target is not None else None
    right_target = target.right if target is not None else None
    left_feedback = feedback.left if feedback is not None else None
    right_feedback = feedback.right if feedback is not None else None

    left_status_obj = _extract_side_status(safety_decision, "left")
    right_status_obj = _extract_side_status(safety_decision, "right")

    left_active = bool(getattr(safety_decision, "left_allowed", False)) if safety_decision is not None else False
    right_active = bool(getattr(safety_decision, "right_allowed", False)) if safety_decision is not None else False

    left_status = _side_reason(safety_decision, left_status_obj, "left")
    right_status = _side_reason(safety_decision, right_status_obj, "right")

    left_enable = bool(enable_left) if enable_left is not None else bool(getattr(left_status_obj, "enable", False))
    right_enable = bool(enable_right) if enable_right is not None else bool(getattr(right_status_obj, "enable", False))

    left_is_calibrated = (
        bool(left_calibrated)
        if left_calibrated is not None
        else bool(getattr(left_status_obj, "calibrated", left_target is not None))
    )
    right_is_calibrated = (
        bool(right_calibrated)
        if right_calibrated is not None
        else bool(getattr(right_status_obj, "calibrated", right_target is not None))
    )

    left_snapshot = build_arm_visualization_snapshot(
        left_target,
        left_feedback,
        calibrated=left_is_calibrated,
        active=left_active,
        status=left_status,
    )
    right_snapshot = build_arm_visualization_snapshot(
        right_target,
        right_feedback,
        calibrated=right_is_calibrated,
        active=right_active,
        status=right_status,
    )

    safety_state = "UNKNOWN"
    global_status = ""
    if safety_decision is not None:
        safety_state = _to_text(getattr(safety_decision, "state", "UNKNOWN"))
        global_status = str(getattr(safety_decision, "global_reason", ""))

    command_loop_dt_ms = None
    target_age_ms = None
    if command_diagnostics is not None:
        command_loop_dt_ms = float(getattr(command_diagnostics, "dt_ms", 0.0))
        target_age_value = getattr(command_diagnostics, "target_age_ms", None)
        target_age_ms = float(target_age_value) if target_age_value is not None else None

    logging_enabled = False
    dropped_log_count = 0
    if logging_stats is not None:
        logging_enabled = bool(getattr(logging_stats, "enabled", False))
        dropped_log_count = int(getattr(logging_stats, "records_dropped", 0))

    return TeleopVisualizationSnapshot(
        timestamp_ns=ts_ns,
        left=left_snapshot,
        right=right_snapshot,
        pico_connected=bool(pico_connected),
        robot_connected=bool(robot_connected),
        safety_state=safety_state,
        global_status=global_status,
        enable_left=left_enable,
        enable_right=right_enable,
        pico_frame_age_ms=float(pico_frame_age_ms) if pico_frame_age_ms is not None else None,
        command_loop_dt_ms=command_loop_dt_ms,
        target_age_ms=target_age_ms,
        ik_status=str(ik_status),
        sdk_status=str(sdk_status),
        teleop_mode=str(teleop_mode),
        orientation_tracking_enabled=bool(orientation_tracking_enabled),
        orientation_relative_mode=str(orientation_relative_mode),
        left_relative_angle_deg=float(left_relative_angle_deg) if left_relative_angle_deg is not None else None,
        right_relative_angle_deg=float(right_relative_angle_deg) if right_relative_angle_deg is not None else None,
        logging_enabled=logging_enabled,
        dropped_log_count=dropped_log_count,
    )


def _extract_side_status(decision: SafetyDecision | None, side: str) -> Any:
    if decision is None:
        return None

    attr_name = f"{side}_status"
    if hasattr(decision, attr_name):
        return getattr(decision, attr_name)
    return None


def _side_reason(decision: SafetyDecision | None, side_status_obj: Any, side: str) -> str:
    if side_status_obj is not None and hasattr(side_status_obj, "reason"):
        return str(getattr(side_status_obj, "reason"))

    if decision is not None:
        attr_name = f"{side}_reason"
        if hasattr(decision, attr_name):
            return str(getattr(decision, attr_name))

    return ""


def _to_text(value: Any) -> str:
    if isinstance(value, Enum):
        return str(value.value)
    return str(value)


def _vec3_or_none(value: Any) -> tuple[float, float, float] | None:
    if value is None:
        return None
    try:
        if len(value) != 3:
            return None
        return (float(value[0]), float(value[1]), float(value[2]))
    except (TypeError, ValueError):
        return None


__all__ = [
    "build_arm_visualization_snapshot",
    "build_visualization_snapshot",
]
