from __future__ import annotations

from dataclasses import dataclass, field, replace

from teleop.filtering import OrientationFilterConfig
from teleop.core.teleop_mode import TeleopMode, normalize_teleop_mode
from teleop.transform.orientation_transform import OrientationTrackingConfig


_ALLOWED_SINGLE_ARM_MODES = {None, "left", "right"}
_ALLOWED_CONTROL_MODES = {"joint_position", "joint_impedance"}
_ALLOWED_PICO_RESAMPLE_MODES = {"latest", "predictive"}


@dataclass(frozen=True)
class FullTeleopAppConfig:
    robot_ip: str = "192.168.1.190"
    connect_pico: bool = True
    connect_robot: bool = True
    move_to_ready: bool = False
    enable_send: bool = False
    dry_run: bool = True
    require_confirmation: bool = True
    command_rate_hz: float = 100.0
    spin_threshold_s: float = 0.0005
    ui_enabled: bool = False
    logging_enabled: bool = False
    teleop_mode: str = TeleopMode.POSITION_ONLY.value
    control_mode: str = "joint_position"
    orientation_tracking: OrientationTrackingConfig = field(default_factory=OrientationTrackingConfig)
    orientation_filter: OrientationFilterConfig = field(default_factory=OrientationFilterConfig)
    single_arm_mode: str | None = None
    pico_resample_mode: str = "latest"
    pico_extrapolation_horizon_ms: float = 15.0
    pico_prediction_max_frame_age_ms: float = 50.0
    pico_velocity_filter_beta: float = 0.5
    pico_max_predicted_step_mm: float = 5.0
    max_runtime_s: float | None = None
    startup_wait_s: float = 2.0

    def __post_init__(self) -> None:
        mode = self.single_arm_mode
        if mode is not None:
            mode = str(mode).strip().lower()
            if mode == "both":
                mode = None
        if mode not in _ALLOWED_SINGLE_ARM_MODES:
            raise ValueError("single_arm_mode must be one of: None, 'left', 'right'")
        object.__setattr__(self, "single_arm_mode", mode)

        pico_resample_mode = str(self.pico_resample_mode).strip().lower()
        if pico_resample_mode not in _ALLOWED_PICO_RESAMPLE_MODES:
            raise ValueError(
                f"pico_resample_mode must be one of {sorted(_ALLOWED_PICO_RESAMPLE_MODES)}, "
                f"got {self.pico_resample_mode!r}"
            )
        object.__setattr__(self, "pico_resample_mode", pico_resample_mode)

        teleop_mode = normalize_teleop_mode(self.teleop_mode)
        object.__setattr__(self, "teleop_mode", teleop_mode)

        control_mode = str(self.control_mode).strip().lower()
        if control_mode not in _ALLOWED_CONTROL_MODES:
            raise ValueError(
                f"control_mode must be one of {sorted(_ALLOWED_CONTROL_MODES)}, got {self.control_mode!r}"
            )
        object.__setattr__(self, "control_mode", control_mode)

        if not isinstance(self.orientation_tracking, OrientationTrackingConfig):
            raise ValueError("orientation_tracking must be an OrientationTrackingConfig instance")
        if not isinstance(self.orientation_filter, OrientationFilterConfig):
            raise ValueError("orientation_filter must be an OrientationFilterConfig instance")

        orientation_cfg = self.orientation_tracking
        if teleop_mode == TeleopMode.POSITION_ONLY.value and orientation_cfg.enabled:
            orientation_cfg = replace(orientation_cfg, enabled=False)
        elif teleop_mode == TeleopMode.POSITION_ORIENTATION.value and not orientation_cfg.enabled:
            orientation_cfg = replace(orientation_cfg, enabled=True)
        object.__setattr__(self, "orientation_tracking", orientation_cfg)

        orientation_filter_cfg = self.orientation_filter
        if teleop_mode == TeleopMode.POSITION_ONLY.value and orientation_filter_cfg.enabled:
            orientation_filter_cfg = replace(orientation_filter_cfg, enabled=False)
        object.__setattr__(self, "orientation_filter", orientation_filter_cfg)

        if not str(self.robot_ip).strip():
            raise ValueError("robot_ip must not be empty")
        if float(self.command_rate_hz) <= 0.0:
            raise ValueError("command_rate_hz must be positive")
        if float(self.spin_threshold_s) < 0.0:
            raise ValueError("spin_threshold_s must be >= 0")
        if float(self.pico_extrapolation_horizon_ms) <= 0.0:
            raise ValueError("pico_extrapolation_horizon_ms must be > 0")
        if float(self.pico_prediction_max_frame_age_ms) <= 0.0:
            raise ValueError("pico_prediction_max_frame_age_ms must be > 0")
        beta = float(self.pico_velocity_filter_beta)
        if beta < 0.0 or beta > 1.0:
            raise ValueError("pico_velocity_filter_beta must be within [0, 1]")
        if float(self.pico_max_predicted_step_mm) <= 0.0:
            raise ValueError("pico_max_predicted_step_mm must be > 0")
        if self.max_runtime_s is not None and float(self.max_runtime_s) <= 0.0:
            raise ValueError("max_runtime_s must be positive when provided")
        if float(self.startup_wait_s) < 0.0:
            raise ValueError("startup_wait_s must be >= 0")

        # Real send always implies non-dry operation.
        if bool(self.enable_send) and bool(self.dry_run):
            object.__setattr__(self, "dry_run", False)


__all__ = ["FullTeleopAppConfig"]
