from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable, Iterator


def read_jsonl_records(path: str | Path, strict: bool = False) -> Iterator[dict]:
    file_path = Path(path)
    with file_path.open("r", encoding="utf-8") as handle:
        for line_no, line in enumerate(handle, start=1):
            text = line.strip()
            if not text:
                continue
            try:
                obj = json.loads(text)
            except json.JSONDecodeError:
                if strict:
                    raise ValueError(f"Invalid JSON line at {line_no} in {file_path}")
                continue

            if isinstance(obj, dict):
                yield obj
            elif strict:
                raise ValueError(f"JSON line {line_no} in {file_path} is not an object")


def filter_records(
    records: Iterable[dict],
    record_type: str | None = None,
    event: str | None = None,
) -> Iterator[dict]:
    for record in records:
        if record_type is not None and str(record.get("record_type", "")) != record_type:
            continue
        if event is not None and str(record.get("event", "")) != event:
            continue
        yield record


def load_session_summary(path: str | Path) -> dict:
    total_records = 0
    record_type_counts: dict[str, int] = {}
    event_counts: dict[str, int] = {}

    min_ts: int | None = None
    max_ts: int | None = None

    for record in read_jsonl_records(path=path, strict=False):
        total_records += 1

        rtype = str(record.get("record_type", "unknown"))
        record_type_counts[rtype] = record_type_counts.get(rtype, 0) + 1

        event = str(record.get("event", ""))
        if event:
            event_counts[event] = event_counts.get(event, 0) + 1

        ts_value = record.get("timestamp_ns")
        if isinstance(ts_value, int):
            if min_ts is None or ts_value < min_ts:
                min_ts = ts_value
            if max_ts is None or ts_value > max_ts:
                max_ts = ts_value

    duration_s = 0.0
    if min_ts is not None and max_ts is not None and max_ts >= min_ts:
        duration_s = float(max_ts - min_ts) / 1_000_000_000.0

    return {
        "total_records": total_records,
        "record_type_counts": record_type_counts,
        "event_counts": event_counts,
        "first_timestamp_ns": min_ts,
        "last_timestamp_ns": max_ts,
        "duration_s": duration_s,
    }


__all__ = ["read_jsonl_records", "filter_records", "load_session_summary"]
