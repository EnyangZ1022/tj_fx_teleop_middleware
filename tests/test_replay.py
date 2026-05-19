from __future__ import annotations

import json
from pathlib import Path

import pytest

from teleop.logging.replay import filter_records, load_session_summary, read_jsonl_records


def test_read_jsonl_and_filter_and_summary(tmp_path: Path) -> None:
    path = tmp_path / "session.jsonl"
    lines = [
        json.dumps({"record_type": "event", "event": "start", "timestamp_ns": 100}),
        "{bad_json_line",
        json.dumps({"record_type": "frame", "event": "tick", "timestamp_ns": 200}),
        json.dumps({"record_type": "event", "event": "stop", "timestamp_ns": 300}),
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    records = list(read_jsonl_records(path, strict=False))
    assert len(records) == 3

    only_events = list(filter_records(records, record_type="event"))
    assert len(only_events) == 2

    only_start = list(filter_records(records, event="start"))
    assert len(only_start) == 1
    assert only_start[0]["event"] == "start"

    summary = load_session_summary(path)
    assert summary["total_records"] == 3
    assert summary["record_type_counts"]["event"] == 2
    assert summary["record_type_counts"]["frame"] == 1
    assert summary["duration_s"] == pytest.approx(2e-7)


def test_read_jsonl_strict_raises_on_bad_line(tmp_path: Path) -> None:
    path = tmp_path / "bad.jsonl"
    path.write_text('{"record_type":"event"}\n{bad_line\n', encoding="utf-8")

    with pytest.raises(ValueError):
        list(read_jsonl_records(path, strict=True))
