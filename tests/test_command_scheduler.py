import pytest

from teleop.control.command_scheduler import CommandSchedulerConfig, FixedRateCommandScheduler
from teleop.control.target_buffer import TargetBuffer
from teleop.control.target_limiter import TargetLimiter, TargetLimiterConfig
from teleop.core.command_frame import ArmCommandTarget, DualArmCommandTarget
from teleop.core.robot_frame import DualArmRobotTarget, RobotArmTarget
from teleop.core.units import meters_to_mm, position_m_to_mm


def _robot_arm_target(
    x_mm: float,
    y_mm: float,
    z_mm: float,
    *,
    orientation: tuple[float, float, float] = (10.0, 20.0, 30.0),
    valid: bool = True,
) -> RobotArmTarget:
    return RobotArmTarget(
        position_xyz=(x_mm, y_mm, z_mm),
        orientation_abc=orientation,
        valid=valid,
    )


def _robot_target(
    *,
    left: RobotArmTarget | None = None,
    right: RobotArmTarget | None = None,
) -> DualArmRobotTarget:
    return DualArmRobotTarget(left=left, right=right)


def _command_arm_target(
    x_mm: float,
    y_mm: float,
    z_mm: float,
    *,
    orientation: tuple[float, float, float] = (10.0, 20.0, 30.0),
    valid: bool = True,
) -> ArmCommandTarget:
    return ArmCommandTarget(
        position_xyz_mm=(x_mm, y_mm, z_mm),
        orientation_abc_deg=orientation,
        ik_reference_q_deg=(90.0, 90.0, -90.0, -90.0, 0.0, 0.0, 0.0),
        valid=valid,
    )


def _command_target(*, left: ArmCommandTarget | None = None, right: ArmCommandTarget | None = None) -> DualArmCommandTarget:
    return DualArmCommandTarget(left=left, right=right)


def test_unit_conversion_helpers() -> None:
    assert meters_to_mm(1.0) == 1000.0
    assert position_m_to_mm((0.1, 0.2, 0.3)) == pytest.approx((100.0, 200.0, 300.0))


def test_scheduler_config_validation_accepts_supported_rates() -> None:
    cfg = CommandSchedulerConfig(rate_hz=100.0, fallback_rate_hz=50.0, controller_inner_loop_hz=1000.0)
    assert cfg.rate_hz == 100.0
    assert cfg.fallback_rate_hz == 50.0


def test_scheduler_config_validation_rejects_unsupported_rate() -> None:
    with pytest.raises(ValueError):
        CommandSchedulerConfig(rate_hz=83.0, fallback_rate_hz=50.0, controller_inner_loop_hz=1000.0)


def test_target_buffer_update_get_and_clear() -> None:
    buffer = TargetBuffer()
    target = _robot_target(right=_robot_arm_target(1000.0, 2000.0, 3000.0))

    buffer.update(target, timestamp_ns=1_000_000_000)
    latest, age_ms = buffer.get_latest(now_ns=1_050_000_000)

    assert latest is not None
    assert age_ms == pytest.approx(50.0)

    buffer.clear()
    latest_after_clear, age_after_clear = buffer.get_latest(now_ns=1_060_000_000)
    assert latest_after_clear is None
    assert age_after_clear is None


def test_scheduler_attaches_fixed_ik_reference_and_keeps_it_fixed() -> None:
    buffer = TargetBuffer()
    scheduler = FixedRateCommandScheduler(buffer)

    t0 = 1_000_000_000
    buffer.update(
        _robot_target(
            left=_robot_arm_target(1000.0, 2000.0, 3000.0),
            right=_robot_arm_target(4000.0, 5000.0, 6000.0),
        ),
        timestamp_ns=t0,
    )

    cmd1, diag1 = scheduler.step(now_ns=t0)
    cmd2, diag2 = scheduler.step(now_ns=t0 + scheduler.period_ns())

    assert cmd1 is not None
    assert cmd2 is not None
    assert cmd1.left is not None and cmd1.right is not None
    assert cmd2.left is not None and cmd2.right is not None

    assert cmd1.left.ik_reference_q_deg == (90.0, -60.0, -90.0, -90.0, -30.0, 0.0, 0.0)
    assert cmd1.right.ik_reference_q_deg == (90.0, 60.0, -90.0, -90.0, 30.0, 0.0, 0.0)
    assert cmd2.left.ik_reference_q_deg == (90.0, -60.0, -90.0, -90.0, -30.0, 0.0, 0.0)
    assert cmd2.right.ik_reference_q_deg == (90.0, 60.0, -90.0, -90.0, 30.0, 0.0, 0.0)

    assert diag1.used_zero_order_hold is False
    assert diag2.used_zero_order_hold is True


def test_scheduler_returns_none_for_stale_target() -> None:
    buffer = TargetBuffer()
    scheduler = FixedRateCommandScheduler(
        buffer,
        config=CommandSchedulerConfig(target_max_age_ms=300.0),
    )

    buffer.update(_robot_target(right=_robot_arm_target(1000.0, 2000.0, 3000.0)), timestamp_ns=1_000_000_000)
    command, diag = scheduler.step(now_ns=1_500_000_000)

    assert command is None
    assert diag.limit_reason == "stale_target"
    assert diag.target_age_ms == pytest.approx(500.0)


def test_zero_order_hold_when_target_is_fresh_and_not_updated() -> None:
    buffer = TargetBuffer()
    scheduler = FixedRateCommandScheduler(buffer)

    t0 = 1_000_000_000
    buffer.update(_robot_target(right=_robot_arm_target(1000.0, 2000.0, 3000.0)), timestamp_ns=t0)

    cmd1, diag1 = scheduler.step(now_ns=t0)
    cmd2, diag2 = scheduler.step(now_ns=t0 + scheduler.period_ns())

    assert cmd1 is not None
    assert cmd2 is not None
    assert diag1.used_zero_order_hold is False
    assert diag2.used_zero_order_hold is True


def test_target_limiter_single_step_clip_mode() -> None:
    limiter = TargetLimiter(
        TargetLimiterConfig(
            max_single_step_mm=5.0,
            max_cartesian_velocity_mm_s=10_000.0,
            clip_instead_of_reject=True,
        )
    )

    first, _, _ = limiter.limit(_command_target(right=_command_arm_target(0.0, 0.0, 0.0)), dt_s=0.01)
    assert first is not None

    limited, is_limited, reason = limiter.limit(
        _command_target(right=_command_arm_target(20.0, 0.0, 0.0)),
        dt_s=0.01,
    )

    assert limited is not None
    assert limited.right is not None
    assert limited.right.position_xyz_mm == pytest.approx((5.0, 0.0, 0.0))
    assert is_limited is True
    assert "right:clipped_limit" in reason


def test_target_limiter_single_step_reject_mode() -> None:
    limiter = TargetLimiter(
        TargetLimiterConfig(
            max_single_step_mm=5.0,
            max_cartesian_velocity_mm_s=10_000.0,
            clip_instead_of_reject=False,
        )
    )

    first, _, _ = limiter.limit(_command_target(right=_command_arm_target(0.0, 0.0, 0.0)), dt_s=0.01)
    assert first is not None

    rejected, is_limited, reason = limiter.limit(
        _command_target(right=_command_arm_target(20.0, 0.0, 0.0)),
        dt_s=0.01,
    )

    assert rejected is None
    assert is_limited is True
    assert "right:rejected_limit" in reason


def test_target_limiter_velocity_limit_clip_and_reject() -> None:
    clip_limiter = TargetLimiter(
        TargetLimiterConfig(
            max_single_step_mm=1000.0,
            max_cartesian_velocity_mm_s=200.0,
            clip_instead_of_reject=True,
        )
    )
    clip_limiter.limit(_command_target(right=_command_arm_target(0.0, 0.0, 0.0)), dt_s=0.1)
    clipped, clipped_flag, _ = clip_limiter.limit(
        _command_target(right=_command_arm_target(50.0, 0.0, 0.0)),
        dt_s=0.1,
    )

    assert clipped is not None
    assert clipped.right is not None
    assert clipped.right.position_xyz_mm == pytest.approx((20.0, 0.0, 0.0))
    assert clipped_flag is True

    reject_limiter = TargetLimiter(
        TargetLimiterConfig(
            max_single_step_mm=1000.0,
            max_cartesian_velocity_mm_s=200.0,
            clip_instead_of_reject=False,
        )
    )
    reject_limiter.limit(_command_target(right=_command_arm_target(0.0, 0.0, 0.0)), dt_s=0.1)
    rejected, rejected_flag, rejected_reason = reject_limiter.limit(
        _command_target(right=_command_arm_target(50.0, 0.0, 0.0)),
        dt_s=0.1,
    )

    assert rejected is None
    assert rejected_flag is True
    assert "right:rejected_limit" in rejected_reason


def test_scheduler_handles_missing_side_targets() -> None:
    buffer = TargetBuffer()
    scheduler = FixedRateCommandScheduler(buffer)
    t0 = 1_000_000_000

    buffer.update(_robot_target(right=_robot_arm_target(1000.0, 2000.0, 3000.0)), timestamp_ns=t0)
    cmd_right, _ = scheduler.step(now_ns=t0)
    assert cmd_right is not None
    assert cmd_right.left is None
    assert cmd_right.right is not None

    buffer.update(_robot_target(left=_robot_arm_target(4000.0, 5000.0, 6000.0)), timestamp_ns=t0 + scheduler.period_ns())
    cmd_left, _ = scheduler.step(now_ns=t0 + scheduler.period_ns())
    assert cmd_left is not None
    assert cmd_left.left is not None
    assert cmd_left.right is None


def test_limiter_keeps_orientation_unchanged_when_clipping() -> None:
    limiter = TargetLimiter(
        TargetLimiterConfig(
            max_single_step_mm=5.0,
            max_cartesian_velocity_mm_s=10_000.0,
            clip_instead_of_reject=True,
        )
    )

    limiter.limit(
        _command_target(right=_command_arm_target(0.0, 0.0, 0.0, orientation=(1.0, 2.0, 3.0))),
        dt_s=0.01,
    )
    limited, _, _ = limiter.limit(
        _command_target(right=_command_arm_target(20.0, 0.0, 0.0, orientation=(9.0, 8.0, 7.0))),
        dt_s=0.01,
    )

    assert limited is not None
    assert limited.right is not None
    assert limited.right.orientation_abc_deg == pytest.approx((9.0, 8.0, 7.0))


def test_scheduler_module_import_has_no_sdk_dependency() -> None:
    import teleop.control.command_scheduler as scheduler_module

    assert hasattr(scheduler_module, "FixedRateCommandScheduler")
