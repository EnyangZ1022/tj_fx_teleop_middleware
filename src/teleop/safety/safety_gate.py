from __future__ import annotations

from dataclasses import replace
import math
import time

from teleop.core.robot_frame import DualArmRobotTarget, RobotArmTarget
from teleop.core.teleop_frame import TeleopArmInput, TeleopFrame
from teleop.safety.safety_config import SafetyConfig
from teleop.safety.state_machine import ArmSafetyStatus, SafetyDecision, SafetyEvent, SafetyState
from teleop.transform.calibration import DualArmCalibrationState


_SIDES = ("left", "right")


class TargetSafetyGate:
    """Gate Stage 3 robot targets with Stage 4 safety checks.

    Target positions are interpreted as robot-side millimeters and velocity limits as mm/s.
    """

    def __init__(self, config: SafetyConfig | None = None):
        self._config = config if config is not None else SafetyConfig()
        self._last_safe_target: DualArmRobotTarget | None = None
        self._last_safe_time_ns: dict[str, int | None] = {"left": None, "right": None}
        self._last_valid_or_safe_time_ns: dict[str, int | None] = {"left": None, "right": None}
        self._invalid_or_no_target_start_ns: dict[str, int | None] = {"left": None, "right": None}
        self._clamp_saturation_start_ns: dict[str, int | None] = {"left": None, "right": None}
        self._reacquire_position_offset_mm: dict[str, tuple[float, float, float]] = {
            "left": (0.0, 0.0, 0.0),
            "right": (0.0, 0.0, 0.0),
        }
        self._reanchor_count: dict[str, int] = {"left": 0, "right": 0}
        self._teleop_mode: str | None = None

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
        self._last_safe_time_ns = {"left": None, "right": None}
        self._last_valid_or_safe_time_ns = {"left": None, "right": None}
        self._invalid_or_no_target_start_ns = {"left": None, "right": None}
        self._clamp_saturation_start_ns = {"left": None, "right": None}
        self._reacquire_position_offset_mm = {
            "left": (0.0, 0.0, 0.0),
            "right": (0.0, 0.0, 0.0),
        }
        self._reanchor_count = {"left": 0, "right": 0}
        self._teleop_mode = None

        self._state = SafetyState.DISCONNECTED
        self._emergency_stop_active = False
        self._emergency_stop_reason = SafetyEvent.EMERGENCY_STOP.value
        self._error_active = False
        self._error_reason = SafetyEvent.ERROR_ACTIVE.value

    def reset_reacquire_offsets(self) -> None:
        self._reacquire_position_offset_mm = {
            "left": (0.0, 0.0, 0.0),
            "right": (0.0, 0.0, 0.0),
        }
        self._invalid_or_no_target_start_ns = {"left": None, "right": None}
        self._clamp_saturation_start_ns = {"left": None, "right": None}

    def set_teleop_mode(self, teleop_mode: str | None) -> None:
        normalized = str(teleop_mode).strip().lower() if teleop_mode is not None else None
        if self._teleop_mode is None:
            self._teleop_mode = normalized
            return
        if normalized != self._teleop_mode:
            self.reset_reacquire_offsets()
            self._teleop_mode = normalized

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
            self._mark_all_blocked(curr_ns)
            return self._finalize(
                state=SafetyState.EMERGENCY_STOP,
                left_status=_deny_status("left", SafetyEvent.EMERGENCY_STOP.value),
                right_status=_deny_status("right", SafetyEvent.EMERGENCY_STOP.value),
                global_reason=self._emergency_stop_reason,
                safe_target=None,
            )

        if self._error_active:
            self._mark_all_blocked(curr_ns)
            return self._finalize(
                state=SafetyState.ERROR,
                left_status=_deny_status("left", SafetyEvent.ERROR_ACTIVE.value),
                right_status=_deny_status("right", SafetyEvent.ERROR_ACTIVE.value),
                global_reason=self._error_reason,
                safe_target=None,
            )

        if teleop_frame is None or self._is_pico_stale(teleop_frame, curr_ns):
            self._mark_all_blocked(curr_ns)
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
            self._mark_all_blocked(curr_ns)
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
            self._mark_all_blocked(curr_ns)
            return self._finalize(
                state=SafetyState.WAIT_CALIBRATION,
                left_status=left_status,
                right_status=right_status,
                global_reason=SafetyEvent.MISSING_CALIBRATION.value,
                safe_target=None,
            )

        if not left_status.pose_valid and not right_status.pose_valid:
            self._mark_all_blocked(curr_ns)
            return self._finalize(
                state=SafetyState.PAUSED,
                left_status=left_status,
                right_status=right_status,
                global_reason=SafetyEvent.INVALID_POSE.value,
                safe_target=None,
            )

        if not self._config.allow_single_arm_motion and (left_status.allowed != right_status.allowed):
            if left_status.allowed:
                left_status = replace(
                    left_status,
                    allowed=False,
                    reason="single_arm_motion_not_allowed",
                    safe_target=None,
                )
            if right_status.allowed:
                right_status = replace(
                    right_status,
                    allowed=False,
                    reason="single_arm_motion_not_allowed",
                    safe_target=None,
                )

        safe_target = _build_safe_target(left_status=left_status, right_status=right_status)
        allow_motion = safe_target is not None and (left_status.allowed or right_status.allowed)

        if allow_motion:
            self._update_last_accepted(
                safe_target=safe_target,
                left_allowed=left_status.allowed,
                right_allowed=right_status.allowed,
                now_ns=curr_ns,
            )
            if left_status.allowed:
                self._mark_side_active("left", curr_ns)
            else:
                self._mark_side_blocked("left", curr_ns)
            if right_status.allowed:
                self._mark_side_active("right", curr_ns)
            else:
                self._mark_side_blocked("right", curr_ns)

            return self._finalize(
                state=SafetyState.TELEOP_ACTIVE,
                left_status=left_status,
                right_status=right_status,
                global_reason=SafetyEvent.OK.value,
                safe_target=safe_target,
            )

        self._mark_side_blocked("left", curr_ns)
        self._mark_side_blocked("right", curr_ns)

        if left_status.calibrated or right_status.calibrated:
            any_ready_without_enable = _is_ready_without_enable(left_status) or _is_ready_without_enable(right_status)
            if any_ready_without_enable:
                return self._finalize(
                    state=SafetyState.TELEOP_READY,
                    left_status=left_status,
                    right_status=right_status,
                    global_reason=SafetyEvent.ENABLE_RELEASED.value,
                    safe_target=None,
                )

            any_transient_pause = _is_transient_pause_reason(left_status.reason) or _is_transient_pause_reason(
                right_status.reason
            )
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
        target_valid = arm_target is not None and bool(arm_target.valid) and _is_finite_arm_target(arm_target)

        if not calibrated:
            self._clear_clamp_saturation(side)
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
            self._clear_clamp_saturation(side)
            return ArmSafetyStatus(
                side=side,
                allowed=False,
                reason=SafetyEvent.INVALID_POSE.value,
                target_valid=target_valid,
                pose_valid=False,
                enable=enable,
                calibrated=True,
            )

        if not target_valid or arm_target is None:
            self._clear_clamp_saturation(side)
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
            self._clear_clamp_saturation(side)
            return ArmSafetyStatus(
                side=side,
                allowed=False,
                reason=SafetyEvent.ENABLE_RELEASED.value,
                target_valid=True,
                pose_valid=True,
                enable=False,
                calibrated=True,
            )

        raw_target = arm_target
        adjusted_target = self._apply_position_offset(side, raw_target)

        reanchored = False
        reanchor_reason = ""
        reanchor_gap_ms: float | None = None
        reanchor_offset_norm_mm: float | None = None

        reanchor_after_gap = self._maybe_reanchor_after_gap(side=side, raw_target=raw_target, now_ns=now_ns)
        if reanchor_after_gap is not None:
            reanchored = True
            reanchor_reason = reanchor_after_gap[0]
            reanchor_gap_ms = reanchor_after_gap[1]
            reanchor_offset_norm_mm = reanchor_after_gap[2]
            adjusted_target = self._apply_position_offset(side, raw_target)

        raw_distance_mm, allowed_distance_mm, step_exceeded, velocity_exceeded, exceeded = self._compute_limit_metrics(
            side=side,
            target=adjusted_target,
            now_ns=now_ns,
        )

        if exceeded:
            if str(self._config.target_limit_mode) == "clamp":
                clamped_target = self._clamp_target_to_allowed_distance(
                    side=side,
                    target=adjusted_target,
                    allowed_distance_mm=allowed_distance_mm,
                )
                clamp_reason = _clamp_reason(step_exceeded=step_exceeded, velocity_exceeded=velocity_exceeded)
                clamp_distance_mm = None
                if raw_distance_mm is not None and allowed_distance_mm is not None:
                    clamp_distance_mm = max(0.0, float(raw_distance_mm - allowed_distance_mm))

                raw_to_safe_error_mm = _distance(raw_target.position_xyz, clamped_target.position_xyz)
                clamp_streak_ms = self._update_clamp_saturation(
                    side=side,
                    raw_to_safe_error_mm=raw_to_safe_error_mm,
                    now_ns=now_ns,
                )

                reanchor_after_clamp = self._maybe_reanchor_after_clamp_saturation(
                    side=side,
                    raw_target=raw_target,
                    now_ns=now_ns,
                    clamp_streak_ms=clamp_streak_ms,
                )
                if reanchor_after_clamp is not None and not reanchored:
                    reanchored = True
                    reanchor_reason = reanchor_after_clamp[0]
                    reanchor_gap_ms = reanchor_after_clamp[1]
                    reanchor_offset_norm_mm = reanchor_after_clamp[2]

                return ArmSafetyStatus(
                    side=side,
                    allowed=True,
                    reason=clamp_reason,
                    target_valid=True,
                    pose_valid=True,
                    enable=True,
                    calibrated=True,
                    safe_target=clamped_target,
                    clamped=True,
                    clamp_reason=clamp_reason,
                    raw_distance_mm=raw_distance_mm,
                    allowed_distance_mm=allowed_distance_mm,
                    clamp_distance_mm=clamp_distance_mm,
                    raw_to_safe_error_mm=raw_to_safe_error_mm,
                    reanchored=reanchored,
                    reanchor_reason=reanchor_reason,
                    reanchor_gap_ms=reanchor_gap_ms,
                    reanchor_offset_norm_mm=reanchor_offset_norm_mm,
                    clamp_streak_ms=clamp_streak_ms,
                )

            self._clear_clamp_saturation(side)
            return ArmSafetyStatus(
                side=side,
                allowed=False,
                reason=_reject_reason(step_exceeded=step_exceeded, velocity_exceeded=velocity_exceeded),
                target_valid=True,
                pose_valid=True,
                enable=True,
                calibrated=True,
                safe_target=None,
                clamped=False,
                clamp_reason="",
                raw_distance_mm=raw_distance_mm,
                allowed_distance_mm=allowed_distance_mm,
                clamp_distance_mm=None,
                raw_to_safe_error_mm=None,
                reanchored=reanchored,
                reanchor_reason=reanchor_reason,
                reanchor_gap_ms=reanchor_gap_ms,
                reanchor_offset_norm_mm=reanchor_offset_norm_mm,
                clamp_streak_ms=None,
            )

        self._clear_clamp_saturation(side)
        raw_to_safe_error_mm = _distance(raw_target.position_xyz, adjusted_target.position_xyz)
        return ArmSafetyStatus(
            side=side,
            allowed=True,
            reason=SafetyEvent.OK.value,
            target_valid=True,
            pose_valid=True,
            enable=True,
            calibrated=True,
            safe_target=adjusted_target,
            clamped=False,
            clamp_reason="",
            raw_distance_mm=raw_distance_mm,
            allowed_distance_mm=allowed_distance_mm,
            clamp_distance_mm=0.0 if raw_distance_mm is not None else None,
            raw_to_safe_error_mm=raw_to_safe_error_mm,
            reanchored=reanchored,
            reanchor_reason=reanchor_reason,
            reanchor_gap_ms=reanchor_gap_ms,
            reanchor_offset_norm_mm=reanchor_offset_norm_mm,
            clamp_streak_ms=None,
        )

    def _compute_limit_metrics(
        self,
        *,
        side: str,
        target: RobotArmTarget,
        now_ns: int,
    ) -> tuple[float | None, float | None, bool, bool, bool]:
        prev = self._last_safe_arm(side)
        if prev is None:
            return None, None, False, False, False

        raw_distance_mm = _distance(prev.position_xyz, target.position_xyz)
        max_step_mm = max(0.0, float(self._config.max_single_step_mm))

        dt_s = self._safe_dt_seconds(side=side, now_ns=now_ns)
        if dt_s is None:
            allowed_by_velocity_mm = max_step_mm
        else:
            allowed_by_velocity_mm = max(0.0, float(self._config.max_velocity_mm_s) * dt_s)

        allowed_distance_mm = min(max_step_mm, allowed_by_velocity_mm)
        if not math.isfinite(allowed_distance_mm) or allowed_distance_mm < 0.0:
            allowed_distance_mm = 0.0

        step_exceeded = raw_distance_mm > max_step_mm
        velocity_exceeded = raw_distance_mm > allowed_by_velocity_mm
        exceeded = raw_distance_mm > allowed_distance_mm
        return raw_distance_mm, allowed_distance_mm, step_exceeded, velocity_exceeded, exceeded

    def _safe_dt_seconds(self, *, side: str, now_ns: int) -> float | None:
        last_time_ns = self._last_safe_time_ns.get(side)
        if last_time_ns is None:
            return None

        dt_ns = int(now_ns) - int(last_time_ns)
        if dt_ns <= 0:
            return None

        dt_s = float(dt_ns) / 1_000_000_000.0
        if not math.isfinite(dt_s) or dt_s <= 0.0:
            return None
        return dt_s

    def _clamp_target_to_allowed_distance(
        self,
        *,
        side: str,
        target: RobotArmTarget,
        allowed_distance_mm: float | None,
    ) -> RobotArmTarget:
        prev = self._last_safe_arm(side)
        if prev is None:
            return target

        distance = _distance(prev.position_xyz, target.position_xyz)
        max_distance = float(allowed_distance_mm) if allowed_distance_mm is not None else 0.0
        if not math.isfinite(max_distance) or max_distance < 0.0:
            max_distance = 0.0

        if distance <= max_distance:
            return target

        if distance <= 1e-12 or max_distance <= 0.0:
            clamped_position = prev.position_xyz
        else:
            direction = _normalize(_subtract(target.position_xyz, prev.position_xyz))
            clamped_position = _add(prev.position_xyz, _scale(direction, max_distance))

        return replace(target, position_xyz=clamped_position)

    def _apply_position_offset(self, side: str, target: RobotArmTarget) -> RobotArmTarget:
        offset = self._reacquire_position_offset_mm.get(side, (0.0, 0.0, 0.0))
        if _norm(offset) <= 0.0:
            return target

        adjusted_position = _add(target.position_xyz, offset)
        if not _is_finite_vec3(adjusted_position):
            return target

        return replace(target, position_xyz=adjusted_position)

    def _maybe_reanchor_after_gap(
        self,
        *,
        side: str,
        raw_target: RobotArmTarget,
        now_ns: int,
    ) -> tuple[str, float, float] | None:
        if str(self._config.reacquire_mode) != "position_offset":
            return None

        last_safe = self._last_safe_arm(side)
        if last_safe is None:
            return None

        gap_start_ns = self._invalid_or_no_target_start_ns.get(side)
        if gap_start_ns is None:
            return None

        gap_ms = max(0.0, float(int(now_ns) - int(gap_start_ns)) / 1_000_000.0)
        if gap_ms < float(self._config.reacquire_after_ms):
            return None

        raw_error_mm = _distance(raw_target.position_xyz, last_safe.position_xyz)
        if raw_error_mm <= float(self._config.reacquire_error_mm):
            return None

        offset_norm_mm = self._set_reanchor_offset_from_raw(side=side, raw_target=raw_target)
        if offset_norm_mm is None:
            return None

        self._invalid_or_no_target_start_ns[side] = None
        self._clamp_saturation_start_ns[side] = None
        return SafetyEvent.REANCHORED_AFTER_GAP.value, gap_ms, offset_norm_mm

    def _maybe_reanchor_after_clamp_saturation(
        self,
        *,
        side: str,
        raw_target: RobotArmTarget,
        now_ns: int,
        clamp_streak_ms: float | None,
    ) -> tuple[str, float, float] | None:
        if str(self._config.reacquire_mode) != "position_offset":
            return None
        if str(self._config.target_limit_mode) != "clamp":
            return None
        if clamp_streak_ms is None:
            return None
        if clamp_streak_ms < float(self._config.clamp_error_reanchor_ms):
            return None

        last_safe = self._last_safe_arm(side)
        if last_safe is None:
            return None

        raw_error_mm = _distance(raw_target.position_xyz, last_safe.position_xyz)
        if raw_error_mm <= float(self._config.reacquire_error_mm):
            return None

        offset_norm_mm = self._set_reanchor_offset_from_raw(side=side, raw_target=raw_target)
        if offset_norm_mm is None:
            return None

        self._invalid_or_no_target_start_ns[side] = None
        self._clamp_saturation_start_ns[side] = None
        return SafetyEvent.REANCHORED_AFTER_CLAMP_SATURATION.value, float(clamp_streak_ms), offset_norm_mm

    def _set_reanchor_offset_from_raw(self, *, side: str, raw_target: RobotArmTarget) -> float | None:
        last_safe = self._last_safe_arm(side)
        if last_safe is None:
            return None

        offset = _subtract(last_safe.position_xyz, raw_target.position_xyz)
        if not _is_finite_vec3(offset):
            return None

        self._reacquire_position_offset_mm[side] = offset
        self._reanchor_count[side] = int(self._reanchor_count.get(side, 0)) + 1
        return _norm(offset)

    def _last_safe_arm(self, side: str) -> RobotArmTarget | None:
        if self._last_safe_target is None:
            return None
        return self._last_safe_target.left if side == "left" else self._last_safe_target.right

    def _update_clamp_saturation(
        self,
        *,
        side: str,
        raw_to_safe_error_mm: float | None,
        now_ns: int,
    ) -> float | None:
        if raw_to_safe_error_mm is None or raw_to_safe_error_mm <= float(self._config.reacquire_error_mm):
            self._clamp_saturation_start_ns[side] = None
            return None

        start_ns = self._clamp_saturation_start_ns.get(side)
        if start_ns is None:
            start_ns = int(now_ns)
            self._clamp_saturation_start_ns[side] = start_ns

        return max(0.0, float(int(now_ns) - int(start_ns)) / 1_000_000.0)

    def _clear_clamp_saturation(self, side: str) -> None:
        self._clamp_saturation_start_ns[side] = None

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
            self._last_valid_or_safe_time_ns["left"] = int(now_ns)

        if right_allowed and safe_target.right is not None:
            prev_right = safe_target.right
            self._last_safe_time_ns["right"] = int(now_ns)
            self._last_valid_or_safe_time_ns["right"] = int(now_ns)

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

    def _mark_side_blocked(self, side: str, now_ns: int) -> None:
        if self._invalid_or_no_target_start_ns.get(side) is None:
            self._invalid_or_no_target_start_ns[side] = int(now_ns)

    def _mark_side_active(self, side: str, now_ns: int) -> None:
        self._invalid_or_no_target_start_ns[side] = None
        self._last_valid_or_safe_time_ns[side] = int(now_ns)

    def _mark_all_blocked(self, now_ns: int) -> None:
        for side in _SIDES:
            self._mark_side_blocked(side, now_ns)
            self._clear_clamp_saturation(side)

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
            safety_target_limit_mode=str(self._config.target_limit_mode),
            safety_reacquire_mode=str(self._config.reacquire_mode),
            left_clamped=bool(left_status.clamped),
            right_clamped=bool(right_status.clamped),
            left_clamp_reason=str(left_status.clamp_reason),
            right_clamp_reason=str(right_status.clamp_reason),
            left_raw_distance_mm=left_status.raw_distance_mm,
            right_raw_distance_mm=right_status.raw_distance_mm,
            left_allowed_distance_mm=left_status.allowed_distance_mm,
            right_allowed_distance_mm=right_status.allowed_distance_mm,
            left_clamp_distance_mm=left_status.clamp_distance_mm,
            right_clamp_distance_mm=right_status.clamp_distance_mm,
            left_raw_to_safe_error_mm=left_status.raw_to_safe_error_mm,
            right_raw_to_safe_error_mm=right_status.raw_to_safe_error_mm,
            left_reanchored=bool(left_status.reanchored),
            right_reanchored=bool(right_status.reanchored),
            left_reanchor_reason=str(left_status.reanchor_reason),
            right_reanchor_reason=str(right_status.reanchor_reason),
            left_reanchor_gap_ms=left_status.reanchor_gap_ms,
            right_reanchor_gap_ms=right_status.reanchor_gap_ms,
            left_reanchor_offset_norm_mm=left_status.reanchor_offset_norm_mm,
            right_reanchor_offset_norm_mm=right_status.reanchor_offset_norm_mm,
            left_clamp_streak_ms=left_status.clamp_streak_ms,
            right_clamp_streak_ms=right_status.clamp_streak_ms,
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


def _build_safe_target(*, left_status: ArmSafetyStatus, right_status: ArmSafetyStatus) -> DualArmRobotTarget | None:
    left = left_status.safe_target if left_status.allowed else None
    right = right_status.safe_target if right_status.allowed else None
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


def _reject_reason(*, step_exceeded: bool, velocity_exceeded: bool) -> str:
    if step_exceeded:
        return SafetyEvent.TARGET_JUMP.value
    if velocity_exceeded:
        return SafetyEvent.VELOCITY_LIMIT.value
    return SafetyEvent.TARGET_JUMP.value


def _clamp_reason(*, step_exceeded: bool, velocity_exceeded: bool) -> str:
    if step_exceeded and velocity_exceeded:
        return SafetyEvent.TARGET_AND_VELOCITY_CLAMPED.value
    if step_exceeded:
        return SafetyEvent.TARGET_JUMP_CLAMPED.value
    if velocity_exceeded:
        return SafetyEvent.VELOCITY_LIMIT_CLAMPED.value
    return SafetyEvent.TARGET_JUMP_CLAMPED.value


def _is_finite_vec3(vec: tuple[float, float, float]) -> bool:
    return all(math.isfinite(float(value)) for value in vec)


def _is_finite_arm_target(target: RobotArmTarget) -> bool:
    return _is_finite_vec3(target.position_xyz) and _is_finite_vec3(target.orientation_abc)


def _subtract(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (float(a[0]) - float(b[0]), float(a[1]) - float(b[1]), float(a[2]) - float(b[2]))


def _add(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (float(a[0]) + float(b[0]), float(a[1]) + float(b[1]), float(a[2]) + float(b[2]))


def _scale(vec: tuple[float, float, float], scalar: float) -> tuple[float, float, float]:
    value = float(scalar)
    return (float(vec[0]) * value, float(vec[1]) * value, float(vec[2]) * value)


def _norm(vec: tuple[float, float, float]) -> float:
    return math.sqrt(float(vec[0]) * float(vec[0]) + float(vec[1]) * float(vec[1]) + float(vec[2]) * float(vec[2]))


def _normalize(vec: tuple[float, float, float]) -> tuple[float, float, float]:
    norm = _norm(vec)
    if norm <= 1e-12:
        return (0.0, 0.0, 0.0)
    return (float(vec[0]) / norm, float(vec[1]) / norm, float(vec[2]) / norm)


__all__ = ["TargetSafetyGate"]
