from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys


def _load_analyze_module():
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "analyze_timing_log.py"
    spec = importlib.util.spec_from_file_location("analyze_timing_log", script_path)
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_jsonl(path: Path, records: list[dict]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        for record in records:
            handle.write(json.dumps(record, ensure_ascii=False) + "\n")


def _timing_record(payload: dict, ts: int) -> dict:
    return {
        "record_type": "timing",
        "timestamp_ns": ts,
        "level": "DEBUG",
        "event": "teleop_timing",
        "payload": payload,
        "sequence_id": None,
    }


def _receiver_timing_record(payload: dict, ts: int) -> dict:
    return {
        "record_type": "receiver_timing",
        "timestamp_ns": ts,
        "level": "DEBUG",
        "event": "pico_receiver_timing",
        "payload": payload,
        "sequence_id": None,
    }


def test_default_prints_all_and_active_summary(tmp_path: Path, capsys) -> None:
    module = _load_analyze_module()

    timing_path = tmp_path / "teleop_timing.jsonl"
    records = [
        _timing_record(
            {
                "loop_wall_ns": 1_000_000_000,
                "loop_perf_ns": 100,
                "loop_dt_ms": 10.0,
                "loop_total_ms": 1.0,
                "deadline_late_ms": 0.2,
                "safety_state": "DISCONNECTED",
                "pico_frame_new": False,
                "left_sent": False,
                "right_sent": False,
                "left_reason": "not_sent",
                "right_reason": "not_sent",
            },
            1_000_000_000,
        ),
        _timing_record(
            {
                "loop_wall_ns": 1_010_000_000,
                "loop_perf_ns": 200,
                "loop_dt_ms": 10.0,
                "loop_total_ms": 1.1,
                "deadline_late_ms": 0.1,
                "safety_state": "TELEOP_ACTIVE",
                "pico_frame_new": True,
                "left_sent": True,
                "right_sent": True,
                "left_reason": "sent",
                "right_reason": "sent",
            },
            1_010_000_000,
        ),
        _timing_record(
            {
                "loop_wall_ns": 1_020_000_000,
                "loop_perf_ns": 300,
                "loop_dt_ms": 10.0,
                "loop_total_ms": 1.2,
                "deadline_late_ms": 0.1,
                "safety_state": "TELEOP_ACTIVE",
                "pico_frame_new": True,
                "left_sent": True,
                "right_sent": True,
                "left_reason": "sent",
                "right_reason": "sent",
            },
            1_020_000_000,
        ),
    ]
    _write_jsonl(timing_path, records)

    old_argv = list(sys.argv)
    try:
        sys.argv = ["analyze_timing_log.py", "--input", str(timing_path)]
        module.main()
    finally:
        sys.argv = old_argv

    output = capsys.readouterr().out
    assert "Main Timing Summary: all_rows" in output
    assert "Main Timing Summary: TELEOP_ACTIVE" in output
    assert "pico_receiver_seq_delta" in output
    assert "skipped receiver frames" in output


def test_state_filter_prints_only_selected_subset(tmp_path: Path, capsys) -> None:
    module = _load_analyze_module()

    timing_path = tmp_path / "teleop_timing.jsonl"
    records = [
        _timing_record({"loop_wall_ns": 1_000_000_000, "safety_state": "DISCONNECTED"}, 1_000_000_000),
        _timing_record({"loop_wall_ns": 1_010_000_000, "safety_state": "TELEOP_ACTIVE"}, 1_010_000_000),
        _timing_record({"loop_wall_ns": 1_020_000_000, "safety_state": "TELEOP_ACTIVE"}, 1_020_000_000),
    ]
    _write_jsonl(timing_path, records)

    old_argv = list(sys.argv)
    try:
        sys.argv = [
            "analyze_timing_log.py",
            "--input",
            str(timing_path),
            "--state",
            "TELEOP_ACTIVE",
        ]
        module.main()
    finally:
        sys.argv = old_argv

    output = capsys.readouterr().out
    assert "Main Timing Summary: state=TELEOP_ACTIVE" in output
    assert "Main Timing Summary: all_rows" not in output


def test_receiver_input_prints_receiver_summary_and_comparison(tmp_path: Path, capsys) -> None:
    module = _load_analyze_module()

    timing_path = tmp_path / "teleop_timing.jsonl"
    timing_records = [
        _timing_record(
            {
                "loop_wall_ns": 1_000_000_000,
                "safety_state": "TELEOP_ACTIVE",
                "pico_frame_new": True,
            },
            1_000_000_000,
        ),
        _timing_record(
            {
                "loop_wall_ns": 1_020_000_000,
                "safety_state": "TELEOP_ACTIVE",
                "pico_frame_new": True,
            },
            1_020_000_000,
        ),
    ]
    _write_jsonl(timing_path, timing_records)

    receiver_path = tmp_path / "teleop_receiver_timing.jsonl"
    receiver_records = [
        _receiver_timing_record(
            {
                "receiver_seq": 1,
                "pc_receive_perf_ns": 100,
                "pc_receive_wall_ns": 1_005_000_000,
                "pico_source_timestamp_ns": 10,
                "frame_seq": 1,
                "parse_duration_ms": 0.2,
                "json_size_bytes": 512,
            },
            1_005_000_000,
        ),
        _receiver_timing_record(
            {
                "receiver_seq": 2,
                "pc_receive_perf_ns": 200,
                "pc_receive_wall_ns": 1_015_000_000,
                "pico_source_timestamp_ns": 20,
                "frame_seq": 2,
                "parse_duration_ms": 0.3,
                "json_size_bytes": 520,
            },
            1_015_000_000,
        ),
    ]
    _write_jsonl(receiver_path, receiver_records)

    old_argv = list(sys.argv)
    try:
        sys.argv = [
            "analyze_timing_log.py",
            "--input",
            str(timing_path),
            "--receiver-input",
            str(receiver_path),
        ]
        module.main()
    finally:
        sys.argv = old_argv

    output = capsys.readouterr().out
    assert "Receiver Timing Summary" in output
    assert "Active Window Comparison" in output
