from __future__ import annotations

import math
from typing import Iterable, Sequence


def compute_dt_stats(timestamp_ns_list: Sequence[int]) -> dict:
    timestamps = [int(v) for v in timestamp_ns_list]
    if len(timestamps) < 2:
        return {
            "count": 0,
            "mean_ms": 0.0,
            "min_ms": 0.0,
            "max_ms": 0.0,
            "p95_ms": 0.0,
        }

    dts_ms = []
    for prev, curr in zip(timestamps[:-1], timestamps[1:]):
        dt_ms = float(curr - prev) / 1_000_000.0
        if math.isfinite(dt_ms):
            dts_ms.append(dt_ms)

    if not dts_ms:
        return {
            "count": 0,
            "mean_ms": 0.0,
            "min_ms": 0.0,
            "max_ms": 0.0,
            "p95_ms": 0.0,
        }

    sorted_dt = sorted(dts_ms)
    n = len(sorted_dt)
    p95_index = max(0, min(n - 1, int(math.ceil(0.95 * n)) - 1))

    return {
        "count": n,
        "mean_ms": sum(sorted_dt) / float(n),
        "min_ms": sorted_dt[0],
        "max_ms": sorted_dt[-1],
        "p95_ms": sorted_dt[p95_index],
    }


def count_events(records: Iterable[dict]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for record in records:
        event = str(record.get("event", "")).strip()
        if not event:
            continue
        counts[event] = counts.get(event, 0) + 1
    return counts


def summarize_session(records: Iterable[dict]) -> dict:
    cache = list(records)
    record_type_counts: dict[str, int] = {}

    min_ts: int | None = None
    max_ts: int | None = None
    dropped_logs = 0

    for record in cache:
        rtype = str(record.get("record_type", "unknown"))
        record_type_counts[rtype] = record_type_counts.get(rtype, 0) + 1

        ts_value = record.get("timestamp_ns")
        if isinstance(ts_value, int):
            if min_ts is None or ts_value < min_ts:
                min_ts = ts_value
            if max_ts is None or ts_value > max_ts:
                max_ts = ts_value

        payload = record.get("payload")
        if isinstance(payload, dict):
            dropped_value = payload.get("records_dropped")
            if isinstance(dropped_value, int):
                dropped_logs += dropped_value

    duration_s = 0.0
    if min_ts is not None and max_ts is not None and max_ts >= min_ts:
        duration_s = float(max_ts - min_ts) / 1_000_000_000.0

    return {
        "total_records": len(cache),
        "record_type_counts": record_type_counts,
        "duration_s": duration_s,
        "event_counts": count_events(cache),
        "dropped_log_count": dropped_logs,
    }


__all__ = ["compute_dt_stats", "count_events", "summarize_session"]
