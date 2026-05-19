from __future__ import annotations

from dataclasses import dataclass


_ALLOWED_SINGLE_ARM_MODES = {None, "left", "right"}


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
    ui_enabled: bool = False
    logging_enabled: bool = False
    single_arm_mode: str | None = None
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

        if not str(self.robot_ip).strip():
            raise ValueError("robot_ip must not be empty")
        if float(self.command_rate_hz) <= 0.0:
            raise ValueError("command_rate_hz must be positive")
        if self.max_runtime_s is not None and float(self.max_runtime_s) <= 0.0:
            raise ValueError("max_runtime_s must be positive when provided")
        if float(self.startup_wait_s) < 0.0:
            raise ValueError("startup_wait_s must be >= 0")

        # Real send always implies non-dry operation.
        if bool(self.enable_send) and bool(self.dry_run):
            object.__setattr__(self, "dry_run", False)


__all__ = ["FullTeleopAppConfig"]
