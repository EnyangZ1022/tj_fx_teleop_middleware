from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SafetyConfig:
    pico_timeout_ms: float = 300.0
    enable_on_threshold: float = 0.85
    enable_off_threshold: float = 0.65
    max_single_step_m: float = 0.05
    max_velocity_mps: float = 0.5
    allow_single_arm_motion: bool = True
    require_both_arms_calibrated: bool = False


__all__ = ["SafetyConfig"]