from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from teleop.core.robot_frame import DualArmRobotTarget, RobotArmTarget


class SafetyState(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    PICO_CONNECTED = "PICO_CONNECTED"
    WAIT_CALIBRATION = "WAIT_CALIBRATION"
    CALIBRATED = "CALIBRATED"
    TELEOP_READY = "TELEOP_READY"
    TELEOP_ACTIVE = "TELEOP_ACTIVE"
    PAUSED = "PAUSED"
    ERROR = "ERROR"
    EMERGENCY_STOP = "EMERGENCY_STOP"


class SafetyEvent(str, Enum):
    PICO_TIMEOUT = "pico_timeout"
    MISSING_CALIBRATION = "missing_calibration"
    INVALID_POSE = "invalid_pose"
    ENABLE_RELEASED = "enable_released"
    TARGET_JUMP = "target_jump"
    VELOCITY_LIMIT = "velocity_limit"
    TARGET_JUMP_CLAMPED = "target_jump_clamped"
    VELOCITY_LIMIT_CLAMPED = "velocity_limit_clamped"
    TARGET_AND_VELOCITY_CLAMPED = "target_and_velocity_clamped"
    REANCHORED_AFTER_GAP = "reanchored_after_gap"
    REANCHORED_AFTER_CLAMP_SATURATION = "reanchored_after_clamp_saturation"
    TARGET_INVALID = "target_invalid"
    EMERGENCY_STOP = "emergency_stop"
    ERROR_ACTIVE = "error_active"
    OK = "ok"


@dataclass(frozen=True)
class ArmSafetyStatus:
    side: str
    allowed: bool
    reason: str
    target_valid: bool
    pose_valid: bool
    enable: bool
    calibrated: bool
    safe_target: RobotArmTarget | None = None
    clamped: bool = False
    clamp_reason: str = ""
    raw_distance_mm: float | None = None
    allowed_distance_mm: float | None = None
    clamp_distance_mm: float | None = None
    raw_to_safe_error_mm: float | None = None
    reanchored: bool = False
    reanchor_reason: str = ""
    reanchor_gap_ms: float | None = None
    reanchor_offset_norm_mm: float | None = None
    clamp_streak_ms: float | None = None


@dataclass(frozen=True)
class SafetyDecision:
    state: SafetyState
    allow_motion: bool
    left_allowed: bool
    right_allowed: bool
    left_reason: str
    right_reason: str
    global_reason: str
    safe_target: DualArmRobotTarget | None
    safety_target_limit_mode: str = "reject"
    safety_reacquire_mode: str = "none"
    left_clamped: bool = False
    right_clamped: bool = False
    left_clamp_reason: str = ""
    right_clamp_reason: str = ""
    left_raw_distance_mm: float | None = None
    right_raw_distance_mm: float | None = None
    left_allowed_distance_mm: float | None = None
    right_allowed_distance_mm: float | None = None
    left_clamp_distance_mm: float | None = None
    right_clamp_distance_mm: float | None = None
    left_raw_to_safe_error_mm: float | None = None
    right_raw_to_safe_error_mm: float | None = None
    left_reanchored: bool = False
    right_reanchored: bool = False
    left_reanchor_reason: str = ""
    right_reanchor_reason: str = ""
    left_reanchor_gap_ms: float | None = None
    right_reanchor_gap_ms: float | None = None
    left_reanchor_offset_norm_mm: float | None = None
    right_reanchor_offset_norm_mm: float | None = None
    left_clamp_streak_ms: float | None = None
    right_clamp_streak_ms: float | None = None


class TeleopSafetyStateMachine:
    """Optional thin wrapper that tracks latest high-level safety state."""

    def __init__(self) -> None:
        self._state: SafetyState = SafetyState.DISCONNECTED
        self._error_latched: bool = False
        self._emergency_stop_latched: bool = False

    @property
    def state(self) -> SafetyState:
        return self._state

    def update(self, decision: SafetyDecision) -> SafetyState:
        if self._emergency_stop_latched:
            self._state = SafetyState.EMERGENCY_STOP
            return self._state

        if self._error_latched:
            self._state = SafetyState.ERROR
            return self._state

        self._state = decision.state
        return self._state

    def reset_error(self) -> None:
        self._error_latched = False
        if self._state == SafetyState.ERROR:
            self._state = SafetyState.PAUSED

    def trigger_emergency_stop(self) -> None:
        self._emergency_stop_latched = True
        self._state = SafetyState.EMERGENCY_STOP

    def clear_emergency_stop(self) -> None:
        self._emergency_stop_latched = False
        if self._state == SafetyState.EMERGENCY_STOP:
            self._state = SafetyState.PAUSED

    def trigger_error(self) -> None:
        self._error_latched = True
        if not self._emergency_stop_latched:
            self._state = SafetyState.ERROR


__all__ = [
    "SafetyState",
    "SafetyEvent",
    "ArmSafetyStatus",
    "SafetyDecision",
    "TeleopSafetyStateMachine",
]