from __future__ import annotations

from dataclasses import dataclass
import math

from teleop.core.command_frame import ArmCommandTarget, DualArmCommandTarget


@dataclass(frozen=True)
class TargetLimiterConfig:
    max_single_step_mm: float = 10.0
    max_cartesian_velocity_mm_s: float = 350.0
    clip_instead_of_reject: bool = True


class TargetLimiter:
    """Limit scheduled command targets based on previous scheduled output."""

    def __init__(self, config: TargetLimiterConfig | None = None) -> None:
        self._config = config if config is not None else TargetLimiterConfig()
        self._last_left: ArmCommandTarget | None = None
        self._last_right: ArmCommandTarget | None = None

    def reset(self) -> None:
        self._last_left = None
        self._last_right = None

    def limit(self, target: DualArmCommandTarget, dt_s: float) -> tuple[DualArmCommandTarget | None, bool, str]:
        left_out, left_next, left_limited, left_reason = self._limit_side(
            side="left",
            current=target.left,
            previous=self._last_left,
            dt_s=dt_s,
        )
        right_out, right_next, right_limited, right_reason = self._limit_side(
            side="right",
            current=target.right,
            previous=self._last_right,
            dt_s=dt_s,
        )

        self._last_left = left_next
        self._last_right = right_next

        limited = left_limited or right_limited
        reasons = [r for r in (left_reason, right_reason) if r]
        limit_reason = ";".join(reasons)

        if left_out is None and right_out is None:
            return None, limited, limit_reason or "all_sides_rejected"

        return DualArmCommandTarget(left=left_out, right=right_out), limited, limit_reason

    def _limit_side(
        self,
        side: str,
        current: ArmCommandTarget | None,
        previous: ArmCommandTarget | None,
        dt_s: float,
    ) -> tuple[ArmCommandTarget | None, ArmCommandTarget | None, bool, str]:
        if current is None:
            return None, previous, False, ""

        if not current.valid:
            return None, previous, True, f"{side}:invalid_target"

        if previous is None:
            return current, current, False, ""

        allowed_step = min(
            float(self._config.max_single_step_mm),
            max(0.0, float(self._config.max_cartesian_velocity_mm_s) * max(0.0, float(dt_s))),
        )

        dist = _distance(previous.position_xyz_mm, current.position_xyz_mm)
        if dist <= allowed_step + 1e-12:
            return current, current, False, ""

        if not self._config.clip_instead_of_reject:
            return None, previous, True, f"{side}:rejected_limit"

        clipped_pos = _clip_towards(
            start=previous.position_xyz_mm,
            end=current.position_xyz_mm,
            max_step=allowed_step,
        )
        clipped = ArmCommandTarget(
            position_xyz_mm=clipped_pos,
            orientation_abc_deg=current.orientation_abc_deg,
            ik_reference_q_deg=current.ik_reference_q_deg,
            valid=current.valid,
            reason=current.reason,
        )
        return clipped, clipped, True, f"{side}:clipped_limit"


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    dx = float(b[0]) - float(a[0])
    dy = float(b[1]) - float(a[1])
    dz = float(b[2]) - float(a[2])
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _clip_towards(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    max_step: float,
) -> tuple[float, float, float]:
    dist = _distance(start, end)
    if dist <= 1e-12 or max_step <= 0.0:
        return (float(start[0]), float(start[1]), float(start[2]))

    ratio = min(1.0, float(max_step) / dist)
    return (
        float(start[0]) + (float(end[0]) - float(start[0])) * ratio,
        float(start[1]) + (float(end[1]) - float(start[1])) * ratio,
        float(start[2]) + (float(end[2]) - float(start[2])) * ratio,
    )


__all__ = [
    "TargetLimiterConfig",
    "TargetLimiter",
]
