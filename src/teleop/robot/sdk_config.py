from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RobotSDKConfig:
    robot_ip: str = "192.168.1.190"
    left_arm: str = "A"
    right_arm: str = "B"
    kine_cfg: str = "assets/kinematics/ccs_m6_40.MvKDCfg"
    vel_ratio: int = 20
    acc_ratio: int = 20
    connect_check_samples: int = 5
    connect_check_interval_s: float = 0.1
    connect_settle_s: float = 0.5
    disable_sdk_logs: bool = True

    left_ik_reference_q_deg: tuple[float, ...] = (90, -90, -90, -90, 0, 0, 0)
    right_ik_reference_q_deg: tuple[float, ...] = (90, 90, -90, -90, 0, 0, 0)


__all__ = ["RobotSDKConfig"]
