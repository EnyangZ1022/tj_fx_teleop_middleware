from __future__ import annotations

import argparse
import csv
from collections import Counter
import math
from pathlib import Path
import statistics
import sys
from typing import Any

# Allow running this script directly without installing the package.
ROOT_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from teleop.logging.replay import read_jsonl_records


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze teleop timing log (JSONL timing records or flat CSV)."
    )
    parser.add_argument("--input", required=True, help="Path to teleop_timing.jsonl or timing CSV")
    return parser.parse_args()


def _as_float(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def _as_int(value: Any) -> int | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _as_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        if float(value) == 1.0:
            return True
        if float(value) == 0.0:
            return False
    if isinstance(value, str):
        text = value.strip().lower()
        if text in {"true", "1", "yes", "y"}:
            return True
        if text in {"false", "0", "no", "n"}:
            return False
    return None


def _percentile(values: list[float], p: float) -> float:
    if not values:
        return float("nan")
    if len(values) == 1:
        return float(values[0])

    q = max(0.0, min(100.0, float(p))) / 100.0
    sorted_values = sorted(values)
    rank = q * (len(sorted_values) - 1)
    low = int(math.floor(rank))
    high = int(math.ceil(rank))
    if low == high:
        return float(sorted_values[low])

    weight = rank - low
    return float(sorted_values[low] * (1.0 - weight) + sorted_values[high] * weight)


def _stats(values: list[float]) -> dict[str, float | int]:
    if not values:
        return {
            "count": 0,
            "mean": float("nan"),
            "p50": float("nan"),
            "p95": float("nan"),
            "p99": float("nan"),
            "max": float("nan"),
        }

    return {
        "count": len(values),
        "mean": float(statistics.fmean(values)),
        "p50": _percentile(values, 50.0),
        "p95": _percentile(values, 95.0),
        "p99": _percentile(values, 99.0),
        "max": float(max(values)),
    }


def _load_rows(path: Path) -> list[dict[str, Any]]:
    suffix = path.suffix.lower()

    if suffix == ".csv":
        with path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            return [dict(row) for row in reader]

    rows: list[dict[str, Any]] = []
    for record in read_jsonl_records(path, strict=False):
        if str(record.get("record_type", "")) != "timing":
            continue

        payload = record.get("payload")
        if not isinstance(payload, dict):
            continue

        row = dict(payload)
        row.setdefault("timestamp_ns", record.get("timestamp_ns"))
        rows.append(row)

    return rows


def _collect_numeric(rows: list[dict[str, Any]], key: str) -> list[float]:
    out: list[float] = []
    for row in rows:
        value = _as_float(row.get(key))
        if value is not None:
            out.append(value)
    return out


def _collect_send_interval_ms(rows: list[dict[str, Any]]) -> list[float]:
    sent_wall_ns: list[int] = []
    for row in rows:
        left_sent = _as_bool(row.get("left_sent")) is True
        right_sent = _as_bool(row.get("right_sent")) is True
        if not (left_sent or right_sent):
            continue

        wall_ns = _as_int(row.get("loop_wall_ns"))
        if wall_ns is not None:
            sent_wall_ns.append(wall_ns)

    if len(sent_wall_ns) < 2:
        return []

    sent_wall_ns.sort()
    intervals_ms: list[float] = []
    for i in range(1, len(sent_wall_ns)):
        dt_ns = sent_wall_ns[i] - sent_wall_ns[i - 1]
        if dt_ns >= 0:
            intervals_ms.append(float(dt_ns) / 1_000_000.0)
    return intervals_ms


def main() -> None:
    args = parse_args()
    input_path = Path(args.input)

    rows = _load_rows(input_path)
    if not rows:
        print(f"Input: {input_path}")
        print("No timing rows found.")
        return

    loop_dt_stats = _stats(_collect_numeric(rows, "loop_dt_ms"))
    loop_total_stats = _stats(_collect_numeric(rows, "loop_total_ms"))
    feedback_stats = _stats(_collect_numeric(rows, "read_feedback_ms"))
    send_stats = _stats(_collect_numeric(rows, "send_command_ms"))
    frame_age_stats = _stats(_collect_numeric(rows, "frame_age_ms"))
    send_interval_stats = _stats(_collect_send_interval_ms(rows))

    overrun_count = sum(1 for row in rows if _as_bool(row.get("overrun")) is True)
    overrun_ratio = float(overrun_count) / float(len(rows)) if rows else 0.0

    no_target_count = sum(1 for row in rows if _as_bool(row.get("no_target")) is True)
    not_sent_count = sum(
        1
        for row in rows
        if (_as_bool(row.get("left_sent")) is not True and _as_bool(row.get("right_sent")) is not True)
    )
    send_failed_count = sum(1 for row in rows if _as_bool(row.get("send_failed")) is True)

    left_reasons = Counter(str(row.get("left_reason", "")) for row in rows)
    right_reasons = Counter(str(row.get("right_reason", "")) for row in rows)

    reject_markers = ("limit", "reject", "ik_failed", "invalid", "stale")
    rejected_count = 0
    for row in rows:
        left_reason = str(row.get("left_reason", "")).lower()
        right_reason = str(row.get("right_reason", "")).lower()
        if any(marker in left_reason for marker in reject_markers) or any(
            marker in right_reason for marker in reject_markers
        ):
            rejected_count += 1

    print(f"Input: {input_path}")
    print(f"Count: {len(rows)}")
    print(
        "loop_dt_ms stats: "
        f"mean={loop_dt_stats['mean']:.3f}, p50={loop_dt_stats['p50']:.3f}, "
        f"p95={loop_dt_stats['p95']:.3f}, p99={loop_dt_stats['p99']:.3f}, max={loop_dt_stats['max']:.3f}"
    )
    print(
        "loop_total_ms stats: "
        f"mean={loop_total_stats['mean']:.3f}, p50={loop_total_stats['p50']:.3f}, "
        f"p95={loop_total_stats['p95']:.3f}, p99={loop_total_stats['p99']:.3f}, max={loop_total_stats['max']:.3f}"
    )
    print(f"Overrun: count={overrun_count}, ratio={overrun_ratio:.3%}")

    if send_interval_stats["count"]:
        print(
            "send_interval_ms stats: "
            f"mean={send_interval_stats['mean']:.3f}, p50={send_interval_stats['p50']:.3f}, "
            f"p95={send_interval_stats['p95']:.3f}, p99={send_interval_stats['p99']:.3f}, "
            f"max={send_interval_stats['max']:.3f}"
        )
    else:
        print("send_interval_ms stats: insufficient sent samples")

    print(
        "read_feedback_ms stats: "
        f"mean={feedback_stats['mean']:.3f}, p95={feedback_stats['p95']:.3f}, "
        f"p99={feedback_stats['p99']:.3f}, max={feedback_stats['max']:.3f}"
    )
    print(
        "send_command_ms stats: "
        f"mean={send_stats['mean']:.3f}, p95={send_stats['p95']:.3f}, "
        f"p99={send_stats['p99']:.3f}, max={send_stats['max']:.3f}"
    )
    print(
        "frame_age_ms stats: "
        f"mean={frame_age_stats['mean']:.3f}, p95={frame_age_stats['p95']:.3f}, "
        f"p99={frame_age_stats['p99']:.3f}, max={frame_age_stats['max']:.3f}"
    )

    print(
        "Events: "
        f"no_target={no_target_count}, not_sent={not_sent_count}, "
        f"send_failed={send_failed_count}, rejected={rejected_count}"
    )

    print(f"Top left reasons: {left_reasons.most_common(5)}")
    print(f"Top right reasons: {right_reasons.most_common(5)}")


if __name__ == "__main__":
    main()
