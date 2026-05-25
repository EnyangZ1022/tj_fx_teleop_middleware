from __future__ import annotations

import pytest

from teleop.core.pose import Pose7
from teleop.core.teleop_frame import TeleopArmInput, TeleopFrame
from teleop.input.pico_resampler import PicoCausalResampler, PicoResamplerConfig


def _pose(x: float, y: float, z: float) -> Pose7:
    return Pose7.from_tuple((x, y, z, 0.0, 0.0, 0.0, 1.0))


def _arm_input(x: float, y: float, z: float, *, enable: bool = True) -> TeleopArmInput:
    return TeleopArmInput(
        pose_pico=_pose(x, y, z),
        valid=True,
        enable=enable,
        gripper_position=0.5,
        gripper_changed=False,
        trigger=0.1,
        grip=0.9 if enable else 0.0,
        axis_x=0.0,
        axis_y=0.0,
        axis_click=False,
    )


def _frame(
    *,
    frame_id: int,
    now_ns: int,
    x_m: float,
    receiver_seq: int,
    enable: bool = True,
    source_ts_ns: int | None = None,
    pc_receive_ns: int | None = None,
) -> TeleopFrame:
    source_ns = now_ns if source_ts_ns is None else int(source_ts_ns)
    pc_ns = now_ns if pc_receive_ns is None else int(pc_receive_ns)
    return TeleopFrame(
        frame_id=frame_id,
        source_device_id="pico_test",
        source_timestamp_ns=source_ns,
        pc_receive_time_ns=pc_ns,
        left=_arm_input(x_m, 0.0, 0.0, enable=enable),
        right=_arm_input(x_m, 0.0, 0.0, enable=enable),
        start_pause_requested=False,
        cancel_requested=False,
        calibration_requested=False,
        receiver_seq=receiver_seq,
    )


def test_latest_mode_passthrough() -> None:
    resampler = PicoCausalResampler(PicoResamplerConfig(mode="latest"))
    frame = _frame(frame_id=1, now_ns=1_000_000_000, x_m=0.0, receiver_seq=10)

    result = resampler.process(latest_actual_frame=frame, now_ns=1_000_000_000, teleop_mode="position_only")

    assert result.frame is frame
    assert result.mode == "latest"
    assert result.prediction_used is False
    assert result.prediction_reason == "latest_mode"


def test_predictive_mode_short_gap_prediction_clamps_step_and_preserves_timestamp() -> None:
    resampler = PicoCausalResampler(
        PicoResamplerConfig(
            mode="predictive",
            extrapolation_horizon_ms=15.0,
            prediction_max_frame_age_ms=50.0,
            velocity_filter_beta=1.0,
            max_predicted_step_mm=5.0,
        )
    )

    t0 = 1_000_000_000
    frame1 = _frame(frame_id=1, now_ns=t0, x_m=0.000, receiver_seq=100)
    frame2 = _frame(frame_id=2, now_ns=t0 + 10_000_000, x_m=0.010, receiver_seq=101)

    _ = resampler.process(latest_actual_frame=frame1, now_ns=t0, teleop_mode="position_only")
    _ = resampler.process(latest_actual_frame=frame2, now_ns=t0 + 10_000_000, teleop_mode="position_only")
    predicted = resampler.process(
        latest_actual_frame=frame2,
        now_ns=t0 + 20_000_000,
        teleop_mode="position_only",
    )

    assert predicted.frame is not None
    assert predicted.prediction_used is True
    assert predicted.prediction_clamped is True
    assert predicted.prediction_reason == "step_clamped"
    assert predicted.frame.pc_receive_time_ns == frame2.pc_receive_time_ns
    assert predicted.frame.source_timestamp_ns == frame2.source_timestamp_ns
    assert predicted.frame.left.enable == frame2.left.enable
    assert predicted.frame.right.enable == frame2.right.enable
    assert predicted.frame.left.pose_pico is not None
    assert predicted.frame.left.pose_pico.x == pytest.approx(0.015, abs=1e-9)
    assert predicted.predicted_left_pos_step_mm == pytest.approx(5.0, abs=1e-6)
    assert predicted.predicted_right_pos_step_mm == pytest.approx(5.0, abs=1e-6)


def test_predictive_mode_frame_too_old_returns_latest_hold() -> None:
    resampler = PicoCausalResampler(
        PicoResamplerConfig(
            mode="predictive",
            extrapolation_horizon_ms=15.0,
            prediction_max_frame_age_ms=50.0,
            velocity_filter_beta=1.0,
            max_predicted_step_mm=5.0,
        )
    )

    t0 = 1_000_000_000
    frame1 = _frame(frame_id=1, now_ns=t0, x_m=0.000, receiver_seq=100)
    frame2 = _frame(frame_id=2, now_ns=t0 + 10_000_000, x_m=0.010, receiver_seq=101)

    _ = resampler.process(latest_actual_frame=frame1, now_ns=t0, teleop_mode="position_only")
    _ = resampler.process(latest_actual_frame=frame2, now_ns=t0 + 10_000_000, teleop_mode="position_only")
    old = resampler.process(
        latest_actual_frame=frame2,
        now_ns=t0 + 80_000_000,
        teleop_mode="position_only",
    )

    assert old.frame is frame2
    assert old.prediction_used is False
    assert old.prediction_reason == "frame_too_old"


def test_predictive_mode_invalid_dt_disables_velocity_update() -> None:
    resampler = PicoCausalResampler(
        PicoResamplerConfig(
            mode="predictive",
            extrapolation_horizon_ms=15.0,
            prediction_max_frame_age_ms=50.0,
            velocity_filter_beta=0.5,
            max_predicted_step_mm=5.0,
        )
    )

    t0 = 1_000_000_000
    frame1 = _frame(
        frame_id=1,
        now_ns=t0,
        x_m=0.000,
        receiver_seq=1,
        source_ts_ns=t0,
        pc_receive_ns=t0,
    )
    # Keep source and pc timestamps equal to force invalid dt in both clock domains.
    frame2 = _frame(
        frame_id=2,
        now_ns=t0 + 10_000_000,
        x_m=0.010,
        receiver_seq=2,
        source_ts_ns=t0,
        pc_receive_ns=t0,
    )

    _ = resampler.process(latest_actual_frame=frame1, now_ns=t0, teleop_mode="position_only")
    second = resampler.process(latest_actual_frame=frame2, now_ns=t0 + 10_000_000, teleop_mode="position_only")
    stalled = resampler.process(latest_actual_frame=frame2, now_ns=t0 + 20_000_000, teleop_mode="position_only")

    assert second.prediction_used is False
    assert second.prediction_reason == "invalid_dt"
    assert stalled.prediction_used is False
    assert stalled.prediction_reason == "no_previous_frame"


def test_predictive_mode_change_resets_state() -> None:
    resampler = PicoCausalResampler(PicoResamplerConfig(mode="predictive"))
    frame = _frame(frame_id=1, now_ns=1_000_000_000, x_m=0.0, receiver_seq=10)

    _ = resampler.process(latest_actual_frame=frame, now_ns=1_000_000_000, teleop_mode="position_only")
    changed = resampler.process(
        latest_actual_frame=frame,
        now_ns=1_010_000_000,
        teleop_mode="position_orientation",
    )

    assert changed.prediction_used is False
    assert changed.prediction_reason == "reset"
