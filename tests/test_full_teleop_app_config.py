from __future__ import annotations

import pytest

from teleop.app import FullTeleopAppConfig
from teleop.core.pose import Pose7
from teleop.core.robot_frame import DualArmRobotTarget, RobotArmTarget
from teleop.core.teleop_frame import TeleopArmInput, TeleopFrame
from teleop.filtering import OrientationFilterConfig
from teleop.safety import SafetyConfig, TargetSafetyGate
from teleop.safety.state_machine import SafetyState
from teleop.transform.calibration import ArmCalibrationAnchor, DualArmCalibrationState
from teleop.transform.orientation_transform import OrientationTrackingConfig


def _pose(x: float, y: float, z: float) -> Pose7:
    return Pose7.from_tuple((x, y, z, 0.0, 0.0, 0.0, 1.0))


def _arm_input(*, grip: float = 0.9) -> TeleopArmInput:
    return TeleopArmInput(
        pose_pico=_pose(0.0, 0.0, 0.0),
        valid=True,
        enable=grip >= 0.8,
        gripper_position=0.5,
        gripper_changed=False,
        trigger=0.1,
        grip=grip,
        axis_x=0.0,
        axis_y=0.0,
        axis_click=False,
    )


def _frame(now_ns: int, grip: float = 0.9) -> TeleopFrame:
    return TeleopFrame(
        frame_id=1,
        source_device_id="pico",
        source_timestamp_ns=now_ns,
        pc_receive_time_ns=now_ns,
        left=_arm_input(grip=grip),
        right=_arm_input(grip=grip),
        start_pause_requested=False,
        cancel_requested=False,
        calibration_requested=False,
    )


def _target(x_mm: float) -> DualArmRobotTarget:
    arm = RobotArmTarget(
        position_xyz=(x_mm, 0.0, 0.0),
        orientation_abc=(10.0, 20.0, 30.0),
        valid=True,
    )
    return DualArmRobotTarget(left=arm, right=arm)


def _calibration() -> DualArmCalibrationState:
    anchor = ArmCalibrationAnchor(
        pico_anchor_xyz=(0.0, 0.0, 0.0),
        robot_anchor_xyz=(0.0, 0.0, 0.0),
        robot_anchor_abc=(10.0, 20.0, 30.0),
        source_frame_id=1,
    )
    return DualArmCalibrationState(left=anchor, right=anchor)


def test_default_app_config_safe_defaults() -> None:
    cfg = FullTeleopAppConfig()

    assert cfg.dry_run is True
    assert cfg.enable_send is False
    assert cfg.logging_enabled is False
    assert cfg.ui_enabled is False
    assert cfg.teleop_mode == "position_only"
    assert cfg.control_mode == "joint_position"
    assert cfg.orientation_tracking.enabled is False
    assert cfg.orientation_tracking.orientation_algorithm == "absolute_matrix"
    assert cfg.orientation_filter.enabled is False
    assert cfg.orientation_filter.tau_s == 0.02
    assert cfg.orientation_filter.fallback_dt_s == 0.01
    assert cfg.spin_threshold_s == 0.0005
    assert cfg.pico_resample_mode == "latest"
    assert cfg.pico_extrapolation_horizon_ms == 15.0
    assert cfg.pico_prediction_max_frame_age_ms == 50.0
    assert cfg.pico_velocity_filter_beta == 0.5
    assert cfg.pico_max_predicted_step_mm == 5.0


def test_negative_spin_threshold_raises() -> None:
    with pytest.raises(ValueError):
        FullTeleopAppConfig(spin_threshold_s=-0.0001)


def test_invalid_pico_resample_mode_raises() -> None:
    with pytest.raises(ValueError):
        FullTeleopAppConfig(pico_resample_mode="invalid")


def test_invalid_pico_velocity_filter_beta_raises() -> None:
    with pytest.raises(ValueError):
        FullTeleopAppConfig(pico_velocity_filter_beta=1.2)


def test_non_positive_pico_max_predicted_step_mm_raises() -> None:
    with pytest.raises(ValueError):
        FullTeleopAppConfig(pico_max_predicted_step_mm=0.0)


def test_position_orientation_mode_enables_orientation_tracking() -> None:
    cfg = FullTeleopAppConfig(
        teleop_mode="position_orientation",
        orientation_tracking=OrientationTrackingConfig(enabled=False),
        orientation_filter=OrientationFilterConfig(enabled=True),
    )

    assert cfg.teleop_mode == "position_orientation"
    assert cfg.orientation_tracking.enabled is True
    assert cfg.orientation_filter.enabled is True


def test_position_orientation_mode_respects_orientation_filter_disable() -> None:
    cfg = FullTeleopAppConfig(
        teleop_mode="position_orientation",
        orientation_filter=OrientationFilterConfig(enabled=False),
    )

    assert cfg.teleop_mode == "position_orientation"
    assert cfg.orientation_filter.enabled is False


def test_invalid_teleop_mode_raises() -> None:
    with pytest.raises(ValueError):
        FullTeleopAppConfig(teleop_mode="invalid_mode")


def test_invalid_control_mode_raises() -> None:
    with pytest.raises(ValueError):
        FullTeleopAppConfig(control_mode="invalid_mode")


def test_enable_send_forces_non_dry_run() -> None:
    cfg = FullTeleopAppConfig(enable_send=True, dry_run=True)

    assert cfg.enable_send is True
    assert cfg.dry_run is False


def test_safety_config_uses_mm_field_names_and_defaults() -> None:
    cfg = SafetyConfig()

    assert cfg.max_single_step_mm == 220.0
    assert cfg.max_velocity_mm_s == 2200.0
    assert cfg.target_limit_mode == "reject"
    assert cfg.reacquire_mode == "none"
    assert cfg.reacquire_after_ms == 1000.0
    assert cfg.reacquire_error_mm == 150.0
    assert cfg.clamp_error_reanchor_ms == 1000.0


def test_safety_config_accepts_patch_d_modes() -> None:
    cfg = SafetyConfig(
        target_limit_mode="clamp",
        reacquire_mode="position_offset",
        reacquire_after_ms=1200.0,
        reacquire_error_mm=180.0,
        clamp_error_reanchor_ms=900.0,
    )

    assert cfg.target_limit_mode == "clamp"
    assert cfg.reacquire_mode == "position_offset"
    assert cfg.reacquire_after_ms == 1200.0
    assert cfg.reacquire_error_mm == 180.0
    assert cfg.clamp_error_reanchor_ms == 900.0


def test_safety_gate_step_limits_use_millimeters() -> None:
    gate = TargetSafetyGate(SafetyConfig(max_single_step_mm=50.0, max_velocity_mm_s=10_000.0))
    frame = _frame(now_ns=1_000_000_000)
    calibration = _calibration()

    first = gate.evaluate(frame, _target(0.0), calibration, now_ns=1_000_000_100)
    assert first.allow_motion is True

    small_step = gate.evaluate(frame, _target(10.0), calibration, now_ns=1_100_000_100)
    assert small_step.state == SafetyState.TELEOP_ACTIVE
    assert small_step.allow_motion is True

    large_step = gate.evaluate(frame, _target(100.0), calibration, now_ns=1_200_000_100)
    assert large_step.state == SafetyState.PAUSED
    assert large_step.allow_motion is False
    assert large_step.left_reason == "target_jump"
    assert large_step.right_reason == "target_jump"
