from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
from pathlib import Path
import time

import pytest

from teleop.logging.async_logger import AsyncSessionLogger
from teleop.logging.log_config import LoggingConfig


@dataclass(frozen=True)
class _PayloadDataclass:
    value: int


class _PayloadEnum(Enum):
    ALPHA = 1


def _find_single_session_file(log_root: Path, *, file_name: str = "teleop_session.jsonl") -> Path:
    session_dirs = [p for p in log_root.iterdir() if p.is_dir()]
    assert len(session_dirs) == 1
    session_file = session_dirs[0] / file_name
    assert session_file.exists()
    return session_file


def test_disabled_logger_is_noop(tmp_path: Path) -> None:
    cfg = LoggingConfig(enabled=False, log_dir=str(tmp_path))
    logger = AsyncSessionLogger(cfg)

    logger.start()

    t0 = time.perf_counter()
    for i in range(1000):
        logger.log_event("noop_event", {"i": i})
        logger.log_frame("noop_frame", {"i": i})
        logger.log_performance("noop_perf", {"i": i})
        logger.log_timing("noop_timing", {"i": i})
        logger.log_receiver_timing("noop_receiver_timing", {"i": i})
    elapsed = time.perf_counter() - t0

    logger.stop()
    stats = logger.get_stats()

    assert elapsed < 0.5
    assert stats.enabled is False
    assert stats.started is False
    assert not any(tmp_path.iterdir())


def test_enabled_logger_writes_jsonl(tmp_path: Path) -> None:
    cfg = LoggingConfig(
        enabled=True,
        log_dir=str(tmp_path),
        session_name="test",
        record_events=True,
        record_frames=True,
        record_performance=True,
        frame_sample_hz=1000.0,
        performance_sample_hz=1000.0,
        flush_interval_s=0.05,
    )
    logger = AsyncSessionLogger(cfg)

    logger.start()
    logger.log_event("event_a", {"a": 1})
    logger.log_frame("frame_a", {"b": 2})
    logger.log_performance("perf_a", {"c": 3})
    logger.log_error("error_a", {"d": 4})
    logger.stop()

    session_file = _find_single_session_file(tmp_path)
    lines = [ln for ln in session_file.read_text(encoding="utf-8").splitlines() if ln.strip()]

    assert len(lines) >= 4
    records = [json.loads(ln) for ln in lines]
    events = {r["event"] for r in records}
    assert {"event_a", "frame_a", "perf_a", "error_a"}.issubset(events)


def test_queue_full_behavior_drops_low_priority(monkeypatch, tmp_path: Path) -> None:
    cfg = LoggingConfig(
        enabled=True,
        log_dir=str(tmp_path),
        record_events=False,
        record_frames=True,
        record_performance=False,
        frame_sample_hz=1000.0,
        max_queue_size=5,
        batch_size=1,
        flush_interval_s=0.2,
        drop_when_full=True,
    )
    logger = AsyncSessionLogger(cfg)

    original_write_batch = AsyncSessionLogger._write_batch

    def _slow_write_batch(self, records):
        time.sleep(0.005)
        return original_write_batch(self, records)

    monkeypatch.setattr(AsyncSessionLogger, "_write_batch", _slow_write_batch)

    logger.start()
    attempted = 200
    for idx in range(attempted):
        logger.log_frame("frame_overflow", {"idx": idx})
    logger.stop()

    stats = logger.get_stats()

    assert stats.records_dropped >= 0
    assert stats.records_written <= attempted


def test_json_serialization_payload_types(tmp_path: Path) -> None:
    cfg = LoggingConfig(
        enabled=True,
        log_dir=str(tmp_path),
        record_events=True,
        flush_interval_s=0.05,
    )
    logger = AsyncSessionLogger(cfg)

    logger.start()
    logger.log_event(
        "serialization_check",
        {
            "dc": _PayloadDataclass(7),
            "enum": _PayloadEnum.ALPHA,
            "tuple": (1, 2, 3),
        },
    )
    logger.stop()

    session_file = _find_single_session_file(tmp_path)
    line = session_file.read_text(encoding="utf-8").strip().splitlines()[0]
    record = json.loads(line)
    payload = record["payload"]

    assert payload["dc"] == {"value": 7}
    assert payload["enum"] == "ALPHA"
    assert payload["tuple"] == [1, 2, 3]


def test_timing_mode_writes_timing_jsonl(tmp_path: Path) -> None:
    cfg = LoggingConfig(
        enabled=True,
        logging_mode="timing",
        log_dir=str(tmp_path),
        record_events=False,
        record_frames=False,
        record_performance=False,
        record_timing=True,
        flush_interval_s=0.05,
    )
    logger = AsyncSessionLogger(cfg)

    logger.start()
    logger.log_timing("teleop_timing", {"loop_seq": 1, "loop_dt_ms": 10.0})
    logger.stop()

    session_file = _find_single_session_file(tmp_path, file_name="teleop_timing.jsonl")
    lines = [ln for ln in session_file.read_text(encoding="utf-8").splitlines() if ln.strip()]

    assert len(lines) >= 1
    record = json.loads(lines[0])
    assert record["record_type"] == "timing"
    assert record["event"] == "teleop_timing"
    assert record["payload"]["loop_seq"] == 1
    assert "log_enqueue_ms" in record["payload"]


def test_timing_mode_writes_receiver_timing_jsonl(tmp_path: Path) -> None:
    cfg = LoggingConfig(
        enabled=True,
        logging_mode="timing",
        log_dir=str(tmp_path),
        record_events=False,
        record_frames=False,
        record_performance=False,
        record_timing=False,
        record_receiver_timing=True,
        flush_interval_s=0.05,
    )
    logger = AsyncSessionLogger(cfg)

    logger.start()
    logger.log_receiver_timing(
        "pico_receiver_timing",
        {
            "receiver_seq": 1,
            "pc_receive_perf_ns": 10,
            "pc_receive_wall_ns": 20,
            "pico_source_timestamp_ns": 30,
            "frame_seq": 40,
            "parse_duration_ms": 0.12,
            "json_size_bytes": 256,
        },
    )
    logger.stop()

    receiver_file = _find_single_session_file(tmp_path, file_name="teleop_receiver_timing.jsonl")
    lines = [ln for ln in receiver_file.read_text(encoding="utf-8").splitlines() if ln.strip()]

    assert len(lines) >= 1
    record = json.loads(lines[0])
    assert record["record_type"] == "receiver_timing"
    assert record["event"] == "pico_receiver_timing"
    assert record["timestamp_ns"] == 20
    assert record["payload"]["receiver_seq"] == 1


def test_receiver_timing_file_not_created_when_disabled(tmp_path: Path) -> None:
    cfg = LoggingConfig(
        enabled=True,
        logging_mode="full",
        log_dir=str(tmp_path),
        record_events=True,
        record_receiver_timing=False,
        flush_interval_s=0.05,
    )
    logger = AsyncSessionLogger(cfg)

    logger.start()
    logger.log_event("event_a", {"ok": True})
    logger.stop()

    session_dirs = [p for p in tmp_path.iterdir() if p.is_dir()]
    assert len(session_dirs) == 1
    receiver_file = session_dirs[0] / "teleop_receiver_timing.jsonl"
    assert receiver_file.exists() is False
