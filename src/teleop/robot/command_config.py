from __future__ import annotations

from dataclasses import dataclass, field

from teleop.robot.ik_config import IKSolverConfig


_ALLOWED_CONTROL_MODES = {"joint_position", "joint_impedance"}


def _normalize_tuple7(name: str, values: tuple[float, ...]) -> tuple[float, ...]:
    output = tuple(float(v) for v in values)
    if len(output) != 7:
        raise ValueError(f"{name} must contain exactly 7 values")
    return output


@dataclass(frozen=True)
class RobotCommandConfig:
    dry_run: bool = True
    command_enabled: bool = False
    control_mode: str = "joint_position"
    ctrl_hz: int = 100

    max_joint_step_deg: float = 5.0
    max_joint_velocity_deg_s: float = 180.0

    send_left: bool = True
    send_right: bool = True

    joint_k: tuple[float, ...] = (6, 6, 6, 5, 4, 3, 3)
    joint_d: tuple[float, ...] = (0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2)
    vel_ratio: int = 100
    acc_ratio: int = 100
    ik_solver: IKSolverConfig = field(default_factory=IKSolverConfig)

    def __post_init__(self) -> None:
        mode = str(self.control_mode).strip().lower()
        if mode not in _ALLOWED_CONTROL_MODES:
            raise ValueError(
                f"control_mode must be one of {_ALLOWED_CONTROL_MODES}, got {self.control_mode!r}"
            )
        object.__setattr__(self, "control_mode", mode)

        if int(self.ctrl_hz) <= 0:
            raise ValueError("ctrl_hz must be positive")
        if float(self.max_joint_step_deg) <= 0.0:
            raise ValueError("max_joint_step_deg must be positive")
        if float(self.max_joint_velocity_deg_s) <= 0.0:
            raise ValueError("max_joint_velocity_deg_s must be positive")

        if int(self.vel_ratio) <= 0:
            raise ValueError("vel_ratio must be positive")
        if int(self.acc_ratio) <= 0:
            raise ValueError("acc_ratio must be positive")

        object.__setattr__(self, "joint_k", _normalize_tuple7("joint_k", self.joint_k))
        object.__setattr__(self, "joint_d", _normalize_tuple7("joint_d", self.joint_d))
        if not isinstance(self.ik_solver, IKSolverConfig):
            raise ValueError("ik_solver must be an IKSolverConfig instance")


__all__ = ["RobotCommandConfig"]
