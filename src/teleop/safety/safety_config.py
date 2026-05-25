from __future__ import annotations

from dataclasses import dataclass


_ALLOWED_TARGET_LIMIT_MODES = {"reject", "clamp"}
_ALLOWED_REACQUIRE_MODES = {"none", "position_offset"}


@dataclass(frozen=True, init=False)
class SafetyConfig:
    pico_timeout_ms: float = 300.0
    enable_on_threshold: float = 0.85
    enable_off_threshold: float = 0.45
    max_single_step_mm: float = 100.0
    max_velocity_mm_s: float = 1000.0
    allow_single_arm_motion: bool = True
    require_both_arms_calibrated: bool = False
    target_limit_mode: str = "reject"
    reacquire_mode: str = "none"
    reacquire_after_ms: float = 1000.0
    reacquire_error_mm: float = 150.0
    clamp_error_reanchor_ms: float = 1000.0

    def __init__(
        self,
        *,
        pico_timeout_ms: float = 300.0,
        enable_on_threshold: float = 0.85,
        enable_off_threshold: float = 0.45,
        max_single_step_mm: float = 50.0,
        max_velocity_mm_s: float = 500.0,
        allow_single_arm_motion: bool = True,
        require_both_arms_calibrated: bool = False,
        target_limit_mode: str = "reject",
        reacquire_mode: str = "none",
        reacquire_after_ms: float = 1000.0,
        reacquire_error_mm: float = 150.0,
        clamp_error_reanchor_ms: float = 1000.0,
        # Optional legacy aliases (meters and meters/second).
        max_single_step_m: float | None = None,
        max_velocity_mps: float | None = None,
    ) -> None:
        if max_single_step_m is not None:
            if max_single_step_mm != 50.0:
                raise ValueError("Provide max_single_step_mm or max_single_step_m, not both")
            max_single_step_mm = float(max_single_step_m) * 1000.0

        if max_velocity_mps is not None:
            if max_velocity_mm_s != 500.0:
                raise ValueError("Provide max_velocity_mm_s or max_velocity_mps, not both")
            max_velocity_mm_s = float(max_velocity_mps) * 1000.0

        object.__setattr__(self, "pico_timeout_ms", float(pico_timeout_ms))
        object.__setattr__(self, "enable_on_threshold", float(enable_on_threshold))
        object.__setattr__(self, "enable_off_threshold", float(enable_off_threshold))
        object.__setattr__(self, "max_single_step_mm", float(max_single_step_mm))
        object.__setattr__(self, "max_velocity_mm_s", float(max_velocity_mm_s))
        object.__setattr__(self, "allow_single_arm_motion", bool(allow_single_arm_motion))
        object.__setattr__(self, "require_both_arms_calibrated", bool(require_both_arms_calibrated))

        normalized_target_limit_mode = str(target_limit_mode).strip().lower()
        if normalized_target_limit_mode not in _ALLOWED_TARGET_LIMIT_MODES:
            raise ValueError(
                f"target_limit_mode must be one of {sorted(_ALLOWED_TARGET_LIMIT_MODES)}, "
                f"got {target_limit_mode!r}"
            )
        object.__setattr__(self, "target_limit_mode", normalized_target_limit_mode)

        normalized_reacquire_mode = str(reacquire_mode).strip().lower()
        if normalized_reacquire_mode not in _ALLOWED_REACQUIRE_MODES:
            raise ValueError(
                f"reacquire_mode must be one of {sorted(_ALLOWED_REACQUIRE_MODES)}, "
                f"got {reacquire_mode!r}"
            )
        object.__setattr__(self, "reacquire_mode", normalized_reacquire_mode)
        object.__setattr__(self, "reacquire_after_ms", float(reacquire_after_ms))
        object.__setattr__(self, "reacquire_error_mm", float(reacquire_error_mm))
        object.__setattr__(self, "clamp_error_reanchor_ms", float(clamp_error_reanchor_ms))

        if self.pico_timeout_ms <= 0.0:
            raise ValueError("pico_timeout_ms must be positive")
        if self.max_single_step_mm <= 0.0:
            raise ValueError("max_single_step_mm must be positive")
        if self.max_velocity_mm_s <= 0.0:
            raise ValueError("max_velocity_mm_s must be positive")
        if self.reacquire_after_ms <= 0.0:
            raise ValueError("reacquire_after_ms must be positive")
        if self.reacquire_error_mm <= 0.0:
            raise ValueError("reacquire_error_mm must be positive")
        if self.clamp_error_reanchor_ms <= 0.0:
            raise ValueError("clamp_error_reanchor_ms must be positive")
        if self.enable_on_threshold < 0.0 or self.enable_on_threshold > 1.0:
            raise ValueError("enable_on_threshold must be in [0, 1]")
        if self.enable_off_threshold < 0.0 or self.enable_off_threshold > 1.0:
            raise ValueError("enable_off_threshold must be in [0, 1]")


__all__ = ["SafetyConfig"]