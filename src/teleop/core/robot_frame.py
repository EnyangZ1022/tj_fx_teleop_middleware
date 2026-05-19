from __future__ import annotations

from dataclasses import dataclass


Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class RobotArmFeedback:
    """Current end-effector feedback in SDK units (position mm, orientation deg)."""

    position_xyz: Vec3
    orientation_abc: Vec3
    valid: bool = True


@dataclass(frozen=True)
class DualArmRobotFeedback:
    """Robot feedback container for both arms."""

    left: RobotArmFeedback | None
    right: RobotArmFeedback | None


@dataclass(frozen=True)
class RobotArmTarget:
    """Target for one arm in SDK units (position mm, orientation deg)."""

    position_xyz: Vec3
    orientation_abc: Vec3
    valid: bool
    reason: str = ""


@dataclass(frozen=True)
class DualArmRobotTarget:
    """Target command container for both arms."""

    left: RobotArmTarget | None
    right: RobotArmTarget | None


__all__ = [
    "RobotArmFeedback",
    "DualArmRobotFeedback",
    "RobotArmTarget",
    "DualArmRobotTarget",
]