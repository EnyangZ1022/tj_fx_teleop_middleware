from __future__ import annotations

from dataclasses import dataclass
import math

from teleop.control.target_buffer import TargetBuffer
from teleop.control.target_limiter import TargetLimiter
from teleop.core.command_frame import ArmCommandTarget, CommandLoopDiagnostics, DualArmCommandTarget
from teleop.core.robot_frame import DualArmRobotTarget, RobotArmTarget


@dataclass(frozen=True)
class CommandSchedulerConfig:
    rate_hz: float = 100.0
    fallback_rate_hz: float = 50.0
    controller_inner_loop_hz: float = 1000.0
    target_max_age_ms: float = 300.0
    left_ik_reference_q_deg: tuple[float, ...] = (90, -90, -90, -90, 0, 0, 0)
    right_ik_reference_q_deg: tuple[float, ...] = (90, 90, -90, -90, 0, 0, 0)

    def __post_init__(self) -> None:
        _validate_positive(self.rate_hz, "rate_hz")
        _validate_positive(self.fallback_rate_hz, "fallback_rate_hz")
        _validate_positive(self.controller_inner_loop_hz, "controller_inner_loop_hz")
        _validate_positive(self.target_max_age_ms, "target_max_age_ms")

        _validate_rate_divides_loop(
            loop_hz=self.controller_inner_loop_hz,
            rate_hz=self.rate_hz,
            name="rate_hz",
        )
        _validate_rate_divides_loop(
            loop_hz=self.controller_inner_loop_hz,
            rate_hz=self.fallback_rate_hz,
            name="fallback_rate_hz",
        )

        if len(self.left_ik_reference_q_deg) != 7:
            raise ValueError("left_ik_reference_q_deg must have exactly 7 elements")
        if len(self.right_ik_reference_q_deg) != 7:
            raise ValueError("right_ik_reference_q_deg must have exactly 7 elements")


class FixedRateCommandScheduler:
    """Prepare fixed-rate command targets from buffered safe robot targets."""

    def __init__(
        self,
        target_buffer: TargetBuffer,
        config: CommandSchedulerConfig | None = None,
        limiter: TargetLimiter | None = None,
    ) -> None:
        self._target_buffer = target_buffer
        self._config = config if config is not None else CommandSchedulerConfig()
        self._limiter = limiter if limiter is not None else TargetLimiter()
        self._sequence_id = 0
        self._last_step_ns: int | None = None
        self._last_buffer_timestamp_ns: int | None = None

    def reset(self) -> None:
        self._sequence_id = 0
        self._last_step_ns = None
        self._last_buffer_timestamp_ns = None
        self._limiter.reset()

    def period_s(self) -> float:
        return 1.0 / float(self._config.rate_hz)

    def period_ns(self) -> int:
        return int(round(self.period_s() * 1_000_000_000.0))

    def step(self, now_ns: int) -> tuple[DualArmCommandTarget | None, CommandLoopDiagnostics]:
        curr_ns = int(now_ns)
        self._sequence_id += 1

        if self._last_step_ns is None:
            dt_s = self.period_s()
        else:
            dt_s = max(0.0, float(curr_ns - self._last_step_ns) / 1_000_000_000.0)
        self._last_step_ns = curr_ns

        safe_target, target_age_ms = self._target_buffer.get_latest(curr_ns)
        target_timestamp_ns = self._target_buffer.latest_timestamp_ns()

        if safe_target is None or target_age_ms is None or target_timestamp_ns is None:
            return None, CommandLoopDiagnostics(
                loop_rate_hz=float(self._config.rate_hz),
                dt_ms=dt_s * 1000.0,
                target_age_ms=None,
                used_zero_order_hold=False,
                limited=False,
                limit_reason="no_target",
                sequence_id=self._sequence_id,
            )

        if target_age_ms > float(self._config.target_max_age_ms):
            return None, CommandLoopDiagnostics(
                loop_rate_hz=float(self._config.rate_hz),
                dt_ms=dt_s * 1000.0,
                target_age_ms=float(target_age_ms),
                used_zero_order_hold=False,
                limited=False,
                limit_reason="stale_target",
                sequence_id=self._sequence_id,
            )

        used_zoh = self._last_buffer_timestamp_ns is not None and target_timestamp_ns == self._last_buffer_timestamp_ns
        self._last_buffer_timestamp_ns = target_timestamp_ns

        command_target = self._to_command_target(safe_target)
        limited_target, limited, limit_reason = self._limiter.limit(command_target, dt_s)

        diagnostics = CommandLoopDiagnostics(
            loop_rate_hz=float(self._config.rate_hz),
            dt_ms=dt_s * 1000.0,
            target_age_ms=float(target_age_ms),
            used_zero_order_hold=used_zoh,
            limited=limited,
            limit_reason=limit_reason,
            sequence_id=self._sequence_id,
        )

        return limited_target, diagnostics

    def _to_command_target(self, target: DualArmRobotTarget) -> DualArmCommandTarget:
        left = self._arm_to_command(target.left, self._config.left_ik_reference_q_deg)
        right = self._arm_to_command(target.right, self._config.right_ik_reference_q_deg)
        return DualArmCommandTarget(left=left, right=right)

    @staticmethod
    def _arm_to_command(source: RobotArmTarget | None, ik_reference: tuple[float, ...]) -> ArmCommandTarget | None:
        if source is None:
            return None

        return ArmCommandTarget(
            position_xyz_mm=(
                float(source.position_xyz[0]),
                float(source.position_xyz[1]),
                float(source.position_xyz[2]),
            ),
            orientation_abc_deg=(
                float(source.orientation_abc[0]),
                float(source.orientation_abc[1]),
                float(source.orientation_abc[2]),
            ),
            ik_reference_q_deg=tuple(float(v) for v in ik_reference),  # type: ignore[arg-type]
            valid=bool(source.valid),
            reason=str(source.reason),
        )


def _validate_positive(value: float, name: str) -> None:
    if not math.isfinite(float(value)) or float(value) <= 0.0:
        raise ValueError(f"{name} must be a positive finite number")


def _validate_rate_divides_loop(loop_hz: float, rate_hz: float, name: str) -> None:
    ratio = float(loop_hz) / float(rate_hz)
    nearest = round(ratio)
    if abs(ratio - nearest) > 1e-6:
        raise ValueError(
            f"{name}={rate_hz} is not compatible with controller_inner_loop_hz={loop_hz} (ratio={ratio})"
        )


__all__ = [
    "CommandSchedulerConfig",
    "FixedRateCommandScheduler",
]
