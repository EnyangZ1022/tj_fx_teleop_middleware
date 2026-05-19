from __future__ import annotations

from dataclasses import replace
import math
import time

from teleop.core.robot_frame import DualArmRobotTarget, RobotArmTarget
from teleop.core.teleop_frame import TeleopArmInput, TeleopFrame
from teleop.safety.safety_config import SafetyConfig
from teleop.safety.state_machine import ArmSafetyStatus, SafetyDecision, SafetyEvent, SafetyState
from teleop.transform.calibration import DualArmCalibrationState


class TargetSafetyGate:
    """Gate Stage 3 robot targets with Stage 4 safety checks.

    Target positions are interpreted as robot-side millimeters and velocity limits as mm/s.
    """

    def __init__(self, config: SafetyConfig | None = None):
        self._config = config if config is not None else SafetyConfig()
        self._last_safe_target: DualArmRobotTarget | None = None
        self._last_safe_time_ns: dict[str, int | None] = {
            "left": None,
            "right": None,
        }
        self._state: SafetyState = SafetyState.DISCONNECTED
        self._emergency_stop_active: bool = False
        self._emergency_stop_reason: str = SafetyEvent.EMERGENCY_STOP.value
        self._error_active: bool = False
        self._error_reason: str = SafetyEvent.ERROR_ACTIVE.value

    @property
    def state(self) -> SafetyState:
        return self._state

    def reset(self) -> None:
        self._last_safe_target = None
        self._last_safe_time_ns = {
            "left": None,
            "right": None,
        }
        self._state = SafetyState.DISCONNECTED
        self._emergency_stop_active = False
        self._emergency_stop_reason = SafetyEvent.EMERGENCY_STOP.value
        self._error_active = False
        self._error_reason = SafetyEvent.ERROR_ACTIVE.value

    def trigger_emergency_stop(self, reason: str = "emergency_stop") -> None:
        self._emergency_stop_active = True
        self._emergency_stop_reason = str(reason) if reason else SafetyEvent.EMERGENCY_STOP.value
        self._state = SafetyState.EMERGENCY_STOP

    def clear_emergency_stop(self) -> None:
        self._emergency_stop_active = False
        self._emergency_stop_reason = SafetyEvent.EMERGENCY_STOP.value

    def set_error(self, reason: str = "error_active") -> None:
        self._error_active = True
        self._error_reason = str(reason) if reason else SafetyEvent.ERROR_ACTIVE.value
        self._state = SafetyState.ERROR

    def clear_error(self) -> None:
        self._error_active = False
        self._error_reason = SafetyEvent.ERROR_ACTIVE.value

    def evaluate(
        self,
        teleop_frame: TeleopFrame | None,
        robot_target: DualArmRobotTarget | None,
        calibration: DualArmCalibrationState | None,
        now_ns: int | None = None,
    ) -> SafetyDecision:
        curr_ns = int(now_ns if now_ns is not None else time.time_ns())

        if self._emergency_stop_active:
            return self._finalize(
                state=SafetyState.EMERGENCY_STOP,
                left_status=_deny_status("left", SafetyEvent.EMERGENCY_STOP.value),
                right_status=_deny_status("right", SafetyEvent.EMERGENCY_STOP.value),
                global_reason=self._emergency_stop_reason,
                safe_target=None,
            )

        if self._error_active:
            return self._finalize(
                state=SafetyState.ERROR,
                left_status=_deny_status("left", SafetyEvent.ERROR_ACTIVE.value),
                right_status=_deny_status("right", SafetyEvent.ERROR_ACTIVE.value),
                global_reason=self._error_reason,
                safe_target=None,
            )

        if teleop_frame is None or self._is_pico_stale(teleop_frame, curr_ns):
            return self._finalize(
                state=SafetyState.DISCONNECTED,
                left_status=_deny_status("left", SafetyEvent.PICO_TIMEOUT.value),
                right_status=_deny_status("right", SafetyEvent.PICO_TIMEOUT.value),
                global_reason=SafetyEvent.PICO_TIMEOUT.value,
                safe_target=None,
            )

        left_calibrated = calibration is not None and calibration.left is not None
        right_calibrated = calibration is not None and calibration.right is not None
        both_calibrated = left_calibrated and right_calibrated

        if self._config.require_both_arms_calibrated and not both_calibrated:
            return self._finalize(
                state=SafetyState.WAIT_CALIBRATION,
                left_status=_deny_status("left", SafetyEvent.MISSING_CALIBRATION.value),
                right_status=_deny_status("right", SafetyEvent.MISSING_CALIBRATION.value),
                global_reason=SafetyEvent.MISSING_CALIBRATION.value,
                safe_target=None,
            )

        left_status = self._evaluate_arm(
            side="left",
            arm_input=teleop_frame.left,
            arm_target=robot_target.left if robot_target is not None else None,
            calibrated=left_calibrated,
            now_ns=curr_ns,
        )
        right_status = self._evaluate_arm(
            side="right",
            arm_input=teleop_frame.right,
            arm_target=robot_target.right if robot_target is not None else None,
            calibrated=right_calibrated,
            now_ns=curr_ns,
        )

        if not left_calibrated and not right_calibrated:
            return self._finalize(
                state=SafetyState.WAIT_CALIBRATION,
                left_status=left_status,
                right_status=right_status,
                global_reason=SafetyEvent.MISSING_CALIBRATION.value,
                safe_target=None,
            )

        if not left_status.pose_valid and not right_status.pose_valid:
            return self._finalize(
                state=SafetyState.PAUSED,
                left_status=left_status,
                right_status=right_status,
                global_reason=SafetyEvent.INVALID_POSE.value,
                safe_target=None,
            )

        if not self._config.allow_single_arm_motion and (left_status.allowed != right_status.allowed):
            if left_status.allowed:
                left_status = replace(left_status, allowed=False, reason="single_arm_motion_not_allowed")
            if right_status.allowed:
                right_status = replace(right_status, allowed=False, reason="single_arm_motion_not_allowed")

        safe_target = _build_safe_target(
            source=robot_target,
            left_allowed=left_status.allowed,
            right_allowed=right_status.allowed,
        )

        allow_motion = safe_target is not None and (left_status.allowed or right_status.allowed)

        if allow_motion:
            self._update_last_accepted(
                safe_target=safe_target,
                left_allowed=left_status.allowed,
                right_allowed=right_status.allowed,
                now_ns=curr_ns,
            )
            return self._finalize(
                state=SafetyState.TELEOP_ACTIVE,
                left_status=left_status,
                right_status=right_status,
                global_reason=SafetyEvent.OK.value,
                safe_target=safe_target,
            )

        if left_status.calibrated or right_status.calibrated:
            any_ready_without_enable = (
                _is_ready_without_enable(left_status) or _is_ready_without_enable(right_status)
            )
            if any_ready_without_enable:
                return self._finalize(
                    state=SafetyState.TELEOP_READY,
                    left_status=left_status,
                    right_status=right_status,
                    global_reason=SafetyEvent.ENABLE_RELEASED.value,
                    safe_target=None,
                )

            any_transient_pause = _is_transient_pause_reason(left_status.reason) or _is_transient_pause_reason(right_status.reason)
            if any_transient_pause:
                return self._finalize(
                    state=SafetyState.PAUSED,
                    left_status=left_status,
                    right_status=right_status,
                    global_reason=self._dominant_reason(left_status.reason, right_status.reason),
                    safe_target=None,
                )

            return self._finalize(
                state=SafetyState.CALIBRATED,
                left_status=left_status,
                right_status=right_status,
                global_reason=SafetyEvent.ENABLE_RELEASED.value,
                safe_target=None,
            )

        return self._finalize(
            state=SafetyState.PICO_CONNECTED,
            left_status=left_status,
            right_status=right_status,
            global_reason=SafetyEvent.MISSING_CALIBRATION.value,
            safe_target=None,
        )

    def _evaluate_arm(
        self,
        side: str,
        arm_input: TeleopArmInput,
        arm_target: RobotArmTarget | None,
        calibrated: bool,
        now_ns: int,
    ) -> ArmSafetyStatus:
        pose_valid = arm_input.valid and arm_input.pose_pico is not None
        enable = bool(arm_input.enable) and (float(arm_input.grip) >= float(self._config.enable_on_threshold))
        target_valid = arm_target is not None and bool(arm_target.valid)

        if not calibrated:
            return ArmSafetyStatus(
                side=side,
                allowed=False,
                reason=SafetyEvent.MISSING_CALIBRATION.value,
                target_valid=target_valid,
                pose_valid=pose_valid,
                enable=enable,
                calibrated=False,
            )

        if not pose_valid:
            return ArmSafetyStatus(
                side=side,
                allowed=False,
                reason=SafetyEvent.INVALID_POSE.value,
                target_valid=target_valid,
                pose_valid=False,
                enable=enable,
                calibrated=True,
            )

        if not target_valid:
            return ArmSafetyStatus(
                side=side,
                allowed=False,
                reason=SafetyEvent.TARGET_INVALID.value,
                target_valid=False,
                pose_valid=True,
                enable=enable,
                calibrated=True,
            )

        if not enable:
            return ArmSafetyStatus(
                side=side,
                allowed=False,
                reason=SafetyEvent.ENABLE_RELEASED.value,
                target_valid=True,
                pose_valid=True,
                enable=False,
                calibrated=True,
            )

        if self._exceeds_single_step_limit(side, arm_target):
            return ArmSafetyStatus(
                side=side,
                allowed=False,
                reason=SafetyEvent.TARGET_JUMP.value,
                target_valid=True,
                pose_valid=True,
                enable=True,
                calibrated=True,
            )

        if self._exceeds_velocity_limit(side, arm_target, now_ns):
            return ArmSafetyStatus(
                side=side,
                allowed=False,
                reason=SafetyEvent.VELOCITY_LIMIT.value,
                target_valid=True,
                pose_valid=True,
                enable=True,
                calibrated=True,
            )

        return ArmSafetyStatus(
            side=side,
            allowed=True,
            reason=SafetyEvent.OK.value,
            target_valid=True,
            pose_valid=True,
            enable=True,
            calibrated=True,
        )

    def _exceeds_single_step_limit(self, side: str, target: RobotArmTarget) -> bool:
        if self._last_safe_target is None:
            return False

        prev = self._last_safe_target.left if side == "left" else self._last_safe_target.right
        if prev is None:
            return False

        dist = _distance(prev.position_xyz, target.position_xyz)
        return dist > float(self._config.max_single_step_mm)

    def _exceeds_velocity_limit(self, side: str, target: RobotArmTarget, now_ns: int) -> bool:
        if self._last_safe_target is None:
            return False

        prev = self._last_safe_target.left if side == "left" else self._last_safe_target.right
        if prev is None:
            return False

        last_time_ns = self._last_safe_time_ns[side]
        if last_time_ns is None:
            return False

        dt_ns = int(now_ns) - int(last_time_ns)
        if dt_ns <= 0:
            return True

        dt_s = dt_ns / 1_000_000_000.0
        dist = _distance(prev.position_xyz, target.position_xyz)
        velocity = dist / dt_s
        return velocity > float(self._config.max_velocity_mm_s)

    def _is_pico_stale(self, frame: TeleopFrame, now_ns: int) -> bool:
        timeout_ns = int(float(self._config.pico_timeout_ms) * 1_000_000.0)
        frame_ns = int(frame.pc_receive_time_ns)
        if frame_ns <= 0:
            return True
        return (int(now_ns) - frame_ns) > timeout_ns

    def _update_last_accepted(
        self,
        safe_target: DualArmRobotTarget,
        left_allowed: bool,
        right_allowed: bool,
        now_ns: int,
    ) -> None:
        prev_left = self._last_safe_target.left if self._last_safe_target is not None else None
        prev_right = self._last_safe_target.right if self._last_safe_target is not None else None

        if left_allowed and safe_target.left is not None:
            prev_left = safe_target.left
            self._last_safe_time_ns["left"] = int(now_ns)

        if right_allowed and safe_target.right is not None:
            prev_right = safe_target.right
            self._last_safe_time_ns["right"] = int(now_ns)

        if prev_left is None and prev_right is None:
            self._last_safe_target = None
            return

        self._last_safe_target = DualArmRobotTarget(left=prev_left, right=prev_right)

    def _dominant_reason(self, left_reason: str, right_reason: str) -> str:
        if left_reason != SafetyEvent.OK.value:
            return left_reason
        if right_reason != SafetyEvent.OK.value:
            return right_reason
        return SafetyEvent.OK.value

    def _finalize(
        self,
        state: SafetyState,
        left_status: ArmSafetyStatus,
        right_status: ArmSafetyStatus,
        global_reason: str,
        safe_target: DualArmRobotTarget | None,
    ) -> SafetyDecision:
        self._state = state
        return SafetyDecision(
            state=state,
            allow_motion=safe_target is not None and (left_status.allowed or right_status.allowed),
            left_allowed=left_status.allowed,
            right_allowed=right_status.allowed,
            left_reason=left_status.reason,
            right_reason=right_status.reason,
            global_reason=global_reason,
            safe_target=safe_target,
        )


def _deny_status(side: str, reason: str) -> ArmSafetyStatus:
    return ArmSafetyStatus(
        side=side,
        allowed=False,
        reason=reason,
        target_valid=False,
        pose_valid=False,
        enable=False,
        calibrated=False,
    )


def _build_safe_target(
    source: DualArmRobotTarget | None,
    left_allowed: bool,
    right_allowed: bool,
) -> DualArmRobotTarget | None:
    if source is None:
        return None

    left = source.left if left_allowed else None
    right = source.right if right_allowed else None
    if left is None and right is None:
        return None
    return DualArmRobotTarget(left=left, right=right)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    dx = float(a[0]) - float(b[0])
    dy = float(a[1]) - float(b[1])
    dz = float(a[2]) - float(b[2])
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _is_ready_without_enable(status: ArmSafetyStatus) -> bool:
    return status.calibrated and status.pose_valid and status.target_valid and (not status.enable)


def _is_transient_pause_reason(reason: str) -> bool:
    return reason in {
        SafetyEvent.INVALID_POSE.value,
        SafetyEvent.TARGET_INVALID.value,
        SafetyEvent.TARGET_JUMP.value,
        SafetyEvent.VELOCITY_LIMIT.value,
        "single_arm_motion_not_allowed",
    }


__all__ = ["TargetSafetyGate"]