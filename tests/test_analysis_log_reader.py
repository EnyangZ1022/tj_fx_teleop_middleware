from __future__ import annotations

import json
from pathlib import Path
import sys

import pytest

from teleop.analysis.log_reader import (
    LogRecord,
    as_bool_or_none,
    as_float_tuple,
    as_int_or_none,
    as_str_or_none,
    extract_events,
    extract_teleop_steps,
    filter_records,
    read_jsonl_records,
)


def _write_jsonl(path: Path, lines: list[str]) -> None:
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def test_reads_valid_jsonl_records(tmp_path: Path) -> None:
    records = [
        {
            "record_type": "event",
            "timestamp_ns": 1_000,
            "level": "INFO",
            "event": "start",
            "payload": {"ok": True},
            "sequence_id": 1,
        },
        {
            "record_type": "frame",
            "timestamp_ns": 1_500,
            "level": "DEBUG",
            "event": "teleop_step",
            "payload": {"frame_id": 2},
            "sequence_id": 2,
        },
    ]

    path = tmp_path / "valid.jsonl"
    _write_jsonl(path, [json.dumps(one) for one in records])
    parsed = read_jsonl_records(path)

    assert len(parsed) == 2
    assert parsed[0].record_type == "event"
    assert parsed[1].event == "teleop_step"
    assert parsed[0].payload == {"ok": True}


def test_skips_malformed_lines_when_non_strict(tmp_path: Path) -> None:
    path = tmp_path / "mixed.jsonl"
    _write_jsonl(
        path,
        [
            json.dumps({"record_type": "event", "timestamp_ns": 100, "event": "a", "payload": {}}),
            "{bad_json",
            json.dumps({"record_type": "frame", "timestamp_ns": 200, "event": "b", "payload": {}}),
            json.dumps([1, 2, 3]),
        ],
    )

    parsed = read_jsonl_records(path, strict=False)

    assert len(parsed) == 2
    assert [r.line_no for r in parsed] == [1, 3]


def test_raises_on_malformed_line_when_strict(tmp_path: Path) -> None:
    path = tmp_path / "bad.jsonl"
    _write_jsonl(path, [json.dumps({"record_type": "event"}), "{bad_line"])

    with pytest.raises(ValueError, match="Invalid JSON line at 2"):
        read_jsonl_records(path, strict=True)


def test_computes_relative_ts_correctly(tmp_path: Path) -> None:
    path = tmp_path / "relative_ts.jsonl"
    _write_jsonl(
        path,
        [
            json.dumps({"record_type": "event", "timestamp_ns": 1_000_000_000, "event": "start", "payload": {}}),
            json.dumps({"record_type": "frame", "timestamp_ns": 1_500_000_000, "event": "teleop_step", "payload": {}}),
            json.dumps({"record_type": "performance", "timestamp_ns": 2_000_000_000, "event": "loop", "payload": {}}),
        ],
    )

    parsed = read_jsonl_records(path)

    assert parsed[0].t_s == pytest.approx(0.0)
    assert parsed[1].t_s == pytest.approx(0.5)
    assert parsed[2].t_s == pytest.approx(1.0)


def test_filter_records_by_type_and_event(tmp_path: Path) -> None:
    path = tmp_path / "filter.jsonl"
    _write_jsonl(
        path,
        [
            json.dumps({"record_type": "event", "timestamp_ns": 1, "event": "start", "payload": {}}),
            json.dumps({"record_type": "frame", "timestamp_ns": 2, "event": "teleop_step", "payload": {}}),
            json.dumps({"record_type": "event", "timestamp_ns": 3, "event": "stop", "payload": {}}),
        ],
    )

    records = read_jsonl_records(path)

    only_events = filter_records(records, record_type="event")
    only_start = filter_records(records, event="start")

    assert len(only_events) == 2
    assert len(only_start) == 1
    assert only_start[0].event == "start"


def test_extracts_teleop_step_records_and_converts_fields() -> None:
    records = [
        LogRecord(
            record_type="frame",
            timestamp_ns=1_000,
            level="DEBUG",
            event="teleop_step",
            payload={
                "frame_id": 10,
                "safety_state": "teleop_active",
                "allow_motion": True,
                "command_ready": True,
                "feedback_left_xyz_mm": [1, 2, 3],
                "feedback_left_abc_deg": [4, 5, 6],
                "feedback_right_xyz_mm": [7.0, 8.0, 9.0],
                "feedback_right_abc_deg": [10, 11, 12],
                "command_left_q_deg": [1, 2, 3, 4, 5, 6, 7],
                "command_right_q_deg": [7, 6, 5, 4, 3, 2, 1],
                "command_left_sent": True,
                "command_right_sent": False,
                "command_left_reason": "sent",
                "command_right_reason": "dry_run",
            },
            sequence_id=3,
            line_no=1,
            t_s=0.0,
        ),
        LogRecord(
            record_type="frame",
            timestamp_ns=2_000,
            level="DEBUG",
            event="teleop_step",
            payload={},
            sequence_id=4,
            line_no=2,
            t_s=0.1,
        ),
        LogRecord(
            record_type="frame",
            timestamp_ns=3_000,
            level="DEBUG",
            event="teleop_step",
            payload={
                "feedback_left_xyz_mm": [1, 2],
                "feedback_right_abc_deg": [1, 2, 3, 4],
                "command_left_q_deg": [1, 2, 3],
                "command_right_q_deg": [1, 2, 3, 4, 5, 6],
            },
            sequence_id=5,
            line_no=3,
            t_s=0.2,
        ),
        LogRecord(
            record_type="frame",
            timestamp_ns=4_000,
            level="DEBUG",
            event="other_frame",
            payload={"frame_id": 99},
            sequence_id=6,
            line_no=4,
            t_s=0.3,
        ),
    ]

    steps = extract_teleop_steps(records)

    assert len(steps) == 3

    first = steps[0]
    assert first.frame_id == 10
    assert first.feedback_left_xyz_mm == pytest.approx((1.0, 2.0, 3.0))
    assert first.feedback_left_abc_deg == pytest.approx((4.0, 5.0, 6.0))
    assert first.feedback_right_xyz_mm == pytest.approx((7.0, 8.0, 9.0))
    assert first.feedback_right_abc_deg == pytest.approx((10.0, 11.0, 12.0))
    assert first.command_left_q_deg == pytest.approx((1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0))
    assert first.command_right_q_deg == pytest.approx((7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0))
    assert first.command_left_sent is True
    assert first.command_right_sent is False
    assert first.command_left_reason == "sent"
    assert first.command_right_reason == "dry_run"

    second = steps[1]
    assert second.frame_id is None
    assert second.safety_state is None
    assert second.allow_motion is None
    assert second.command_ready is None
    assert second.feedback_left_xyz_mm is None
    assert second.command_left_q_deg is None

    third = steps[2]
    assert third.feedback_left_xyz_mm is None
    assert third.feedback_right_abc_deg is None
    assert third.command_left_q_deg is None
    assert third.command_right_q_deg is None


def test_extract_events_returns_event_records(tmp_path: Path) -> None:
    path = tmp_path / "events.jsonl"
    _write_jsonl(
        path,
        [
            json.dumps({"record_type": "event", "timestamp_ns": 10, "event": "start", "payload": {}}),
            json.dumps({"record_type": "frame", "timestamp_ns": 11, "event": "teleop_step", "payload": {}}),
            json.dumps({"record_type": "event", "timestamp_ns": 12, "event": "stop", "payload": {}}),
        ],
    )

    records = read_jsonl_records(path)
    events = extract_events(records)

    assert [r.event for r in events] == ["start", "stop"]


def test_helper_converters() -> None:
    assert as_float_tuple([1, 2, 3], 3) == pytest.approx((1.0, 2.0, 3.0))
    assert as_float_tuple([1, 2], 3) is None

    assert as_bool_or_none(True) is True
    assert as_bool_or_none("false") is False
    assert as_bool_or_none(1) is True
    assert as_bool_or_none(7) is None

    assert as_int_or_none(5) == 5
    assert as_int_or_none("6") == 6
    assert as_int_or_none("6.0") == 6
    assert as_int_or_none("6.2") is None

    assert as_str_or_none(None) is None
    assert as_str_or_none("x") == "x"
    assert as_str_or_none(123) == "123"


def test_import_does_not_require_sdk_modules() -> None:
    before = set(sys.modules.keys())
    from teleop.analysis import log_reader as imported_module

    after = set(sys.modules.keys())
    newly_imported = after - before

    assert callable(imported_module.read_jsonl_records)
    assert "python_sdk.fx_robot" not in newly_imported
