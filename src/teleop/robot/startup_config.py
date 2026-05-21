from __future__ import annotations

from dataclasses import dataclass


def _normalize_joint_tuple(name: str, values: tuple[float, ...]) -> tuple[float, ...]:
    joints = tuple(float(v) for v in values)
    if len(joints) != 7:
        raise ValueError(f"{name} must contain exactly 7 joint angles in degree")
    return joints


@dataclass(frozen=True)
class RobotStartupConfig:
    vel_ratio: int = 20
    acc_ratio: int = 20

    left_ready_q_deg: tuple[float, ...] = (90, -60, -90, -90, -30, 0, 0)
    right_ready_q_deg: tuple[float, ...] = (90, 60, -90, -90, 30, 0, 0)

    home_send_hz: float = 100.0
    home_timeout_s: float = 20.0
    home_tol_deg: float = 1.0
    home_stable_samples: int = 20
    check_period_s: float = 0.01
    pre_wait_s: float = 2.0

    def __post_init__(self) -> None:
        if int(self.vel_ratio) <= 0:
            raise ValueError("vel_ratio must be positive")
        if int(self.acc_ratio) <= 0:
            raise ValueError("acc_ratio must be positive")

        object.__setattr__(self, "left_ready_q_deg", _normalize_joint_tuple("left_ready_q_deg", self.left_ready_q_deg))
        object.__setattr__(self, "right_ready_q_deg", _normalize_joint_tuple("right_ready_q_deg", self.right_ready_q_deg))

        if float(self.home_send_hz) <= 0.0:
            raise ValueError("home_send_hz must be positive")
        if float(self.home_timeout_s) <= 0.0:
            raise ValueError("home_timeout_s must be positive")
        if float(self.home_tol_deg) <= 0.0:
            raise ValueError("home_tol_deg must be positive")
        if int(self.home_stable_samples) <= 0:
            raise ValueError("home_stable_samples must be positive")
        if float(self.check_period_s) <= 0.0:
            raise ValueError("check_period_s must be positive")
        if float(self.pre_wait_s) < 0.0:
            raise ValueError("pre_wait_s must be >= 0")


__all__ = ["RobotStartupConfig"]
