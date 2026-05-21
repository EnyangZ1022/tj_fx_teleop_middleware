from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, init=False)
class SafetyConfig:
    pico_timeout_ms: float = 300.0
    enable_on_threshold: float = 0.85
    enable_off_threshold: float = 0.45
    max_single_step_mm: float = 50.0
    max_velocity_mm_s: float = 500.0
    allow_single_arm_motion: bool = True
    require_both_arms_calibrated: bool = False

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

        if self.pico_timeout_ms <= 0.0:
            raise ValueError("pico_timeout_ms must be positive")
        if self.max_single_step_mm <= 0.0:
            raise ValueError("max_single_step_mm must be positive")
        if self.max_velocity_mm_s <= 0.0:
            raise ValueError("max_velocity_mm_s must be positive")
        if self.enable_on_threshold < 0.0 or self.enable_on_threshold > 1.0:
            raise ValueError("enable_on_threshold must be in [0, 1]")
        if self.enable_off_threshold < 0.0 or self.enable_off_threshold > 1.0:
            raise ValueError("enable_off_threshold must be in [0, 1]")


__all__ = ["SafetyConfig"]