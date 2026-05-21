from __future__ import annotations

from pathlib import Path

import pytest

from teleop.analysis.ik_replay_analysis import (
    IKReplaySummary,
    build_ik_replay_summary,
    build_q_series_from_recorded_commands,
    find_reject_markers,
    plot_ik_replay_summary,
    summarize_q_jumps,
    write_ik_replay_report,
    write_reject_markers_csv,
    write_replay_q_timeseries_csv,
)
from teleop.analysis.log_reader import TeleopStepLog


def _step(
    *,
    t_s: float,
    frame_id: int,
    left_q: tuple[float, ...] | None = None,
    right_q: tuple[float, ...] | None = None,
    left_reason: str | None = None,
    right_reason: str | None = None,
    left_sent: bool | None = None,
    right_sent: bool | None = None,
    raw_payload: dict | None = None,
) -> TeleopStepLog:
    return TeleopStepLog(
        timestamp_ns=int(round(t_s * 1_000_000_000.0)),
        t_s=t_s,
        sequence_id=frame_id,
        frame_id=frame_id,
        safety_state=None,
        allow_motion=None,
        command_ready=None,
        feedback_left_xyz_mm=None,
        feedback_left_abc_deg=None,
        feedback_right_xyz_mm=None,
        feedback_right_abc_deg=None,
        command_left_q_deg=left_q,
        command_right_q_deg=right_q,
        command_left_sent=left_sent,
        command_right_sent=right_sent,
        command_left_reason=left_reason,
        command_right_reason=right_reason,
        raw_payload=raw_payload or {},
    )


def _left_ready() -> tuple[float, ...]:
    return (90.0, -60.0, -90.0, -90.0, 0.0, 0.0, 0.0)


def _right_ready() -> tuple[float, ...]:
    return (90.0, 60.0, -90.0, -90.0, 0.0, 0.0, 0.0)


def test_builds_q_series_from_synthetic_steps() -> None:
    steps = [
        _step(t_s=0.0, frame_id=1, left_q=_left_ready()),
        _step(t_s=0.1, frame_id=2, left_q=(90.0, -60.0, -90.0, -89.0, 0.0, 0.0, 0.0)),
    ]

    series = build_q_series_from_recorded_commands(steps, side="left")

    assert series.side == "left"
    assert series.t_s == pytest.approx([0.0, 0.1])
    assert series.frame_id == [1, 2]
    assert series.q_deg[0] == pytest.approx(_left_ready())


def test_computes_max_step_correctly() -> None:
    steps = [
        _step(t_s=0.0, frame_id=1, left_q=_left_ready()),
        _step(t_s=0.1, frame_id=2, left_q=(90.0, -60.0, -90.0, -85.0, 0.0, 0.0, 0.0)),
    ]

    series = build_q_series_from_recorded_commands(steps, side="left")

    assert series.max_step_deg[0] is None
    assert series.max_step_deg[1] == pytest.approx(5.0)


def test_computes_velocity_using_dt_correctly() -> None:
    steps = [
        _step(t_s=1.0, frame_id=1, left_q=_left_ready()),
        _step(t_s=1.5, frame_id=2, left_q=(90.0, -60.0, -90.0, -80.0, 0.0, 0.0, 0.0)),
    ]

    series = build_q_series_from_recorded_commands(steps, side="left")

    assert series.max_velocity_deg_s[0] is None
    assert series.max_velocity_deg_s[1] == pytest.approx(20.0)


def test_handles_missing_q() -> None:
    steps = [
        _step(t_s=0.0, frame_id=1, left_q=None),
        _step(t_s=0.1, frame_id=2, left_q=_left_ready()),
    ]

    series = build_q_series_from_recorded_commands(steps, side="left")

    assert series.q_deg[0] is None
    assert series.max_step_deg[0] is None
    assert series.max_step_deg[1] is None


def test_finds_first_reject_markers_per_side() -> None:
    steps = [
        _step(t_s=0.0, frame_id=1, left_reason="sent", right_reason="dry_run"),
        _step(t_s=0.1, frame_id=2, left_reason="joint_step_limit", right_reason="ok"),
        _step(t_s=0.2, frame_id=3, left_reason="target_jump", right_reason="joint_velocity_limit"),
    ]

    markers = find_reject_markers(steps, limit_per_side=1)

    assert len(markers) == 2
    assert markers[0].side == "left"
    assert markers[0].reason == "joint_step_limit"
    assert markers[1].side == "right"
    assert markers[1].reason == "joint_velocity_limit"


def test_summarizes_first_valid_q_distance_from_ready_q() -> None:
    ready = _left_ready()
    steps = [
        _step(t_s=0.0, frame_id=1, left_q=(91.0, -60.0, -90.0, -90.0, 0.0, 0.0, 0.0), left_sent=False),
        _step(t_s=0.1, frame_id=2, left_q=(92.0, -60.0, -90.0, -90.0, 0.0, 0.0, 0.0), left_sent=True),
    ]

    series = build_q_series_from_recorded_commands(steps, side="left")
    stats = summarize_q_jumps(series, ready)

    assert stats["first_valid_max_abs_dq_deg"] == pytest.approx(1.0)
    assert stats["first_sent_max_abs_dq_deg"] == pytest.approx(2.0)


def test_writes_report_and_csv_files(tmp_path: Path) -> None:
    steps = [
        _step(t_s=0.0, frame_id=1, left_q=_left_ready(), right_q=_right_ready(), left_reason="ok", right_reason="ok"),
        _step(
            t_s=0.1,
            frame_id=2,
            left_q=(90.0, -60.0, -90.0, -85.0, 0.0, 0.0, 0.0),
            right_q=(90.0, 60.0, -90.0, -85.0, 0.0, 0.0, 0.0),
            left_reason="joint_step_limit",
            right_reason="ok",
            left_sent=False,
            right_sent=False,
        ),
    ]

    summary = build_ik_replay_summary(steps=steps, mode="recorded", side="both", limit_per_side=3)

    q_csv = tmp_path / "replay_q_timeseries.csv"
    reject_csv = tmp_path / "reject_markers.csv"
    report = tmp_path / "ik_replay_report.md"

    write_replay_q_timeseries_csv(path=q_csv, summary=summary)
    write_reject_markers_csv(path=reject_csv, reject_markers=summary.reject_markers)
    write_ik_replay_report(
        path=report,
        input_path=tmp_path / "teleop_session.jsonl",
        summary=summary,
        ready_left_q=_left_ready(),
        ready_right_q=_right_ready(),
    )

    assert q_csv.exists()
    assert reject_csv.exists()
    assert report.exists()

    assert "left_q1" in q_csv.read_text(encoding="utf-8")
    assert "reason" in reject_csv.read_text(encoding="utf-8")
    assert "recomputed IK used" in report.read_text(encoding="utf-8")


def test_plot_function_can_be_skipped() -> None:
    empty_series = build_q_series_from_recorded_commands([], side="left")
    summary = IKReplaySummary(
        mode="recorded",
        left=empty_series,
        right=build_q_series_from_recorded_commands([], side="right"),
        reject_markers=[],
        notes=[],
    )

    paths = plot_ik_replay_summary(output_dir=Path("."), summary=summary, no_plots=True)

    assert paths == []


def test_auto_mode_falls_back_to_recorded_when_targets_missing() -> None:
    steps = [
        _step(t_s=0.0, frame_id=1, left_q=_left_ready(), right_q=_right_ready(), raw_payload={}),
        _step(t_s=0.1, frame_id=2, left_q=_left_ready(), right_q=_right_ready(), raw_payload={}),
    ]

    summary = build_ik_replay_summary(steps=steps, mode="auto", side="both", limit_per_side=2)

    assert summary.mode == "recorded"
    assert any("target xyzabc fields not found" in note for note in summary.notes)


def test_recompute_mode_raises_clear_error_when_targets_missing() -> None:
    steps = [
        _step(t_s=0.0, frame_id=1, left_q=_left_ready(), right_q=_right_ready(), raw_payload={}),
        _step(t_s=0.1, frame_id=2, left_q=_left_ready(), right_q=_right_ready(), raw_payload={}),
    ]

    with pytest.raises(RuntimeError, match="target xyzabc fields not found"):
        _ = build_ik_replay_summary(steps=steps, mode="recompute", side="both", limit_per_side=2)
