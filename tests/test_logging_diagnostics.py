from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

from teleop.logging.diagnostics import compute_dt_stats, count_events, summarize_session


def test_compute_dt_stats_and_summarize_session() -> None:
    timestamps = [1_000_000_000, 1_010_000_000, 1_020_000_000, 1_050_000_000]
    dt_stats = compute_dt_stats(timestamps)

    assert dt_stats["count"] == 3
    assert dt_stats["min_ms"] == pytest.approx(10.0)
    assert dt_stats["max_ms"] == pytest.approx(30.0)
    assert dt_stats["mean_ms"] == pytest.approx((10.0 + 10.0 + 30.0) / 3.0)

    records = [
        {"record_type": "event", "event": "start", "timestamp_ns": 100},
        {"record_type": "frame", "event": "frame", "timestamp_ns": 200},
        {"record_type": "performance", "event": "perf", "timestamp_ns": 300},
        {"record_type": "error", "event": "fault", "timestamp_ns": 400},
        {
            "record_type": "event",
            "event": "logger_stats",
            "timestamp_ns": 500,
            "payload": {"records_dropped": 3},
        },
    ]

    events = count_events(records)
    assert events["start"] == 1
    assert events["frame"] == 1

    summary = summarize_session(records)
    assert summary["total_records"] == 5
    assert summary["record_type_counts"]["event"] == 2
    assert summary["record_type_counts"]["error"] == 1
    assert summary["duration_s"] == pytest.approx(4e-7)
    assert summary["dropped_log_count"] == 3


def test_stage7_scripts_import_without_side_effects() -> None:
    root = Path(__file__).resolve().parents[1]

    scripts = [
        root / "scripts" / "logging_dry_run.py",
        root / "scripts" / "analyze_teleop_log.py",
        root / "scripts" / "analyze_timing_log.py",
        root / "scripts" / "replay_teleop_log.py",
    ]

    for script in scripts:
        spec = importlib.util.spec_from_file_location(script.stem, script)
        assert spec is not None
        assert spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        assert hasattr(module, "main")
