from __future__ import annotations

from pathlib import Path

import pytest

from teleop.analysis.log_reader import TeleopStepLog
from teleop.analysis.teleop_jump_analysis import (
    ReadyPoseConfig,
    analyze_side_jump,
    compute_feedback_baseline,
    find_rejects,
    plot_jump_summary,
    write_jump_report_md,
    write_jump_timeseries_csv,
)


def _step(
    *,
    t_s: float,
    frame_id: int,
    sequence_id: int | None = None,
    safety_state: str | None = None,
    allow_motion: bool | None = None,
    command_ready: bool | None = None,
    feedback_left_xyz_mm: tuple[float, float, float] | None = None,
    feedback_left_abc_deg: tuple[float, float, float] | None = None,
    feedback_right_xyz_mm: tuple[float, float, float] | None = None,
    feedback_right_abc_deg: tuple[float, float, float] | None = None,
    command_left_q_deg: tuple[float, ...] | None = None,
    command_right_q_deg: tuple[float, ...] | None = None,
    command_left_sent: bool | None = None,
    command_right_sent: bool | None = None,
    command_left_reason: str | None = None,
    command_right_reason: str | None = None,
) -> TeleopStepLog:
    return TeleopStepLog(
        timestamp_ns=int(round(t_s * 1_000_000_000.0)),
        t_s=t_s,
        sequence_id=sequence_id,
        frame_id=frame_id,
        safety_state=safety_state,
        allow_motion=allow_motion,
        command_ready=command_ready,
        feedback_left_xyz_mm=feedback_left_xyz_mm,
        feedback_left_abc_deg=feedback_left_abc_deg,
        feedback_right_xyz_mm=feedback_right_xyz_mm,
        feedback_right_abc_deg=feedback_right_abc_deg,
        command_left_q_deg=command_left_q_deg,
        command_right_q_deg=command_right_q_deg,
        command_left_sent=command_left_sent,
        command_right_sent=command_right_sent,
        command_left_reason=command_left_reason,
        command_right_reason=command_right_reason,
        raw_payload={},
    )


def _baseline() -> dict[str, tuple[float, ...] | None]:
    return {
        "left_xyz": (0.0, 0.0, 0.0),
        "left_abc": (0.0, 0.0, 0.0),
        "right_xyz": (0.0, 0.0, 0.0),
        "right_abc": (0.0, 0.0, 0.0),
    }


def test_baseline_median_is_computed_with_preferred_frames() -> None:
    steps = [
        _step(
            t_s=0.0,
            frame_id=1,
            allow_motion=False,
            command_ready=False,
            feedback_left_xyz_mm=(0.0, 0.0, 0.0),
            feedback_left_abc_deg=(0.0, 0.0, 0.0),
            feedback_right_xyz_mm=(0.0, 0.0, 0.0),
            feedback_right_abc_deg=(0.0, 0.0, 0.0),
        ),
        _step(
            t_s=0.1,
            frame_id=2,
            allow_motion=False,
            command_ready=False,
            feedback_left_xyz_mm=(10.0, 0.0, 0.0),
            feedback_left_abc_deg=(10.0, 0.0, 0.0),
            feedback_right_xyz_mm=(20.0, 0.0, 0.0),
            feedback_right_abc_deg=(20.0, 0.0, 0.0),
        ),
        _step(
            t_s=0.2,
            frame_id=3,
            allow_motion=True,
            command_ready=True,
            feedback_left_xyz_mm=(100.0, 0.0, 0.0),
            feedback_left_abc_deg=(100.0, 0.0, 0.0),
            feedback_right_xyz_mm=(100.0, 0.0, 0.0),
            feedback_right_abc_deg=(100.0, 0.0, 0.0),
        ),
    ]

    baseline = compute_feedback_baseline(steps, baseline_n=2)

    assert baseline["left_xyz"] == pytest.approx((5.0, 0.0, 0.0))
    assert baseline["left_abc"] == pytest.approx((5.0, 0.0, 0.0))
    assert baseline["right_xyz"] == pytest.approx((10.0, 0.0, 0.0))
    assert baseline["right_abc"] == pytest.approx((10.0, 0.0, 0.0))


def test_first_command_q_and_first_sent_q_are_found() -> None:
    ready = ReadyPoseConfig()
    steps = [
        _step(t_s=0.0, frame_id=1),
        _step(t_s=0.1, frame_id=2, command_left_q_deg=ready.left_q_deg, command_left_sent=False, command_left_reason="ok"),
        _step(
            t_s=0.2,
            frame_id=3,
            command_left_q_deg=(90.0, -60.0, -90.0, -90.0, 5.0, 0.0, 0.0),
            command_left_sent=True,
            command_left_reason="sent",
        ),
    ]

    summary = analyze_side_jump(
        steps=steps,
        side="left",
        ready_q_deg=ready.left_q_deg,
        baseline=_baseline(),
    )

    assert summary.first_command_frame_id == 2
    assert summary.first_sent_frame_id == 3
    assert summary.first_sent_q_deg is not None
    assert summary.first_sent_max_abs_dq_deg == pytest.approx(5.0)


def test_q_jump_from_ready_is_detected() -> None:
    ready = ReadyPoseConfig()
    steps = [
        _step(t_s=0.0, frame_id=1, command_left_q_deg=ready.left_q_deg),
        _step(
            t_s=0.2,
            frame_id=2,
            command_left_q_deg=(90.0, -60.0, -90.0, -90.0, 20.0, 0.0, 0.0),
            command_left_sent=False,
        ),
    ]

    summary = analyze_side_jump(
        steps=steps,
        side="left",
        ready_q_deg=ready.left_q_deg,
        baseline=_baseline(),
        jump_threshold_deg=15.0,
    )

    assert summary.first_jump_frame_id == 2
    assert summary.first_jump_max_abs_dq_deg == pytest.approx(20.0)


def test_rejects_are_extracted() -> None:
    steps = [
        _step(t_s=0.0, frame_id=1, command_left_reason="sent", command_left_sent=True, sequence_id=11),
        _step(t_s=0.1, frame_id=2, command_left_reason="joint_step_limit", command_left_sent=False, sequence_id=12),
        _step(t_s=0.2, frame_id=3, command_left_reason="target_jump", command_left_sent=False, sequence_id=13),
    ]

    rejects = find_rejects(steps, side="left", limit=3)

    assert len(rejects) == 2
    assert rejects[0]["reason"] == "joint_step_limit"
    assert rejects[1]["reason"] == "target_jump"
    assert rejects[0]["sequence_id"] == 12


def test_feedback_threshold_crossing_works() -> None:
    ready = ReadyPoseConfig()
    steps = [
        _step(t_s=0.0, frame_id=1, feedback_left_xyz_mm=(0.0, 0.0, 0.0), command_left_q_deg=ready.left_q_deg),
        _step(t_s=0.1, frame_id=2, feedback_left_xyz_mm=(2.0, 0.0, 0.0), command_left_q_deg=ready.left_q_deg),
        _step(t_s=0.2, frame_id=3, feedback_left_xyz_mm=(6.0, 0.0, 0.0), command_left_q_deg=ready.left_q_deg),
    ]

    summary = analyze_side_jump(
        steps=steps,
        side="left",
        ready_q_deg=ready.left_q_deg,
        baseline=_baseline(),
        feedback_thresholds_mm=(1.0, 5.0),
    )

    assert summary.feedback_threshold_crossings[1.0] is not None
    assert summary.feedback_threshold_crossings[5.0] is not None
    assert summary.feedback_threshold_crossings[1.0]["frame_id"] == 2
    assert summary.feedback_threshold_crossings[5.0]["frame_id"] == 3


def test_catchup_estimate_works_on_synthetic_data() -> None:
    ready = ReadyPoseConfig()
    steps = [
        _step(t_s=0.0, frame_id=1, feedback_left_xyz_mm=(0.0, 0.0, 0.0), command_left_q_deg=ready.left_q_deg),
        _step(
            t_s=1.0,
            frame_id=2,
            feedback_left_xyz_mm=(0.0, 0.0, 0.0),
            command_left_q_deg=ready.left_q_deg,
            command_left_sent=True,
            command_left_reason="sent",
        ),
        _step(t_s=2.0, frame_id=3, feedback_left_xyz_mm=(2.0, 0.0, 0.0), command_left_q_deg=ready.left_q_deg),
        _step(t_s=3.0, frame_id=4, feedback_left_xyz_mm=(10.0, 0.0, 0.0), command_left_q_deg=ready.left_q_deg),
    ]

    summary = analyze_side_jump(
        steps=steps,
        side="left",
        ready_q_deg=ready.left_q_deg,
        baseline=_baseline(),
        catch_ratio=0.9,
    )

    assert summary.catchup_info is not None
    assert summary.catchup_info["max_disp_mm"] == pytest.approx(10.0)
    assert summary.catchup_info["target_disp_mm"] == pytest.approx(9.0)
    assert summary.catchup_info["reach_frame_id"] == 4
    assert summary.catchup_info["delay_s"] == pytest.approx(2.0)


def test_csv_writer_creates_file(tmp_path: Path) -> None:
    ready = ReadyPoseConfig()
    steps = [
        _step(
            t_s=0.0,
            frame_id=1,
            safety_state="paused",
            allow_motion=False,
            command_ready=False,
            feedback_left_xyz_mm=(0.0, 0.0, 0.0),
            feedback_right_xyz_mm=(0.0, 0.0, 0.0),
            command_left_q_deg=ready.left_q_deg,
            command_right_q_deg=ready.right_q_deg,
            command_left_sent=False,
            command_right_sent=False,
            command_left_reason="ok",
            command_right_reason="ok",
        )
    ]

    out_path = write_jump_timeseries_csv(
        path=tmp_path / "jump_timeseries.csv",
        steps=steps,
        baseline=_baseline(),
        ready_config=ready,
    )

    assert out_path.exists()
    text = out_path.read_text(encoding="utf-8")
    assert "left_feedback_disp_mm" in text
    assert "left_command_max_abs_dq_from_ready_deg" in text


def test_markdown_writer_creates_file(tmp_path: Path) -> None:
    ready = ReadyPoseConfig()
    steps = [
        _step(t_s=0.0, frame_id=1, command_left_q_deg=ready.left_q_deg, command_right_q_deg=ready.right_q_deg)
    ]

    baseline = _baseline()
    left_summary = analyze_side_jump(steps, "left", ready.left_q_deg, baseline)
    right_summary = analyze_side_jump(steps, "right", ready.right_q_deg, baseline)

    out_path = write_jump_report_md(
        path=tmp_path / "jump_report.md",
        input_path=tmp_path / "teleop_session.jsonl",
        steps=steps,
        baseline=baseline,
        left_summary=left_summary,
        right_summary=right_summary,
    )

    assert out_path.exists()
    text = out_path.read_text(encoding="utf-8")
    assert "does not recompute IK" in text


def test_plot_function_can_be_skipped() -> None:
    ready = ReadyPoseConfig()
    steps = [_step(t_s=0.0, frame_id=1)]

    paths = plot_jump_summary(
        output_dir=Path("."),
        steps=steps,
        baseline=_baseline(),
        ready_config=ready,
        reject_markers=[],
        no_plots=True,
    )

    assert paths == []
