from __future__ import annotations

from dataclasses import dataclass


Vec3 = tuple[float, float, float]
IKReference7 = tuple[float, float, float, float, float, float, float]


@dataclass(frozen=True)
class ArmCommandTarget:
    """Future SDK-facing Cartesian command target in mm/deg units."""

    position_xyz_mm: Vec3
    orientation_abc_deg: Vec3
    ik_reference_q_deg: IKReference7
    valid: bool = True
    reason: str = ""


@dataclass(frozen=True)
class DualArmCommandTarget:
    left: ArmCommandTarget | None
    right: ArmCommandTarget | None


@dataclass(frozen=True)
class CommandLoopDiagnostics:
    loop_rate_hz: float
    dt_ms: float
    target_age_ms: float | None
    used_zero_order_hold: bool
    limited: bool
    limit_reason: str
    sequence_id: int


__all__ = [
    "ArmCommandTarget",
    "DualArmCommandTarget",
    "CommandLoopDiagnostics",
]
