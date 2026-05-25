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
        description="Analyze teleop timing logs and optional receiver timing logs."
    )
    parser.add_argument("--input", required=True, help="Path to teleop_timing.jsonl or timing CSV")
    parser.add_argument(
        "--receiver-input",
        default=None,
        help="Optional path to teleop_receiver_timing.jsonl",
    )
    parser.add_argument(
        "--state",
        default=None,
        help="Filter main timing summary by safety_state (for example: TELEOP_ACTIVE)",
    )
    parser.add_argument(
        "--all-states",
        action="store_true",
        help="Print compact per-state row counts when safety_state exists",
    )
    parser.add_argument(
        "--overrun-threshold-ms",
        type=float,
        default=1.0,
        help="Threshold for real_overrun metric (deadline_late_ms > threshold)",
    )
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


def _as_str(value: Any) -> str | None:
    if value is None:
        return None
    if isinstance(value, str):
        return value
    return str(value)


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


def _fmt(value: float | int | None, digits: int = 3) -> str:
    if value is None:
        return "N/A"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "N/A"
    if not math.isfinite(number):
        return "N/A"
    return f"{number:.{digits}f}"


def _print_stats(label: str, values: list[float]) -> None:
    stats = _stats(values)
    if int(stats["count"]) == 0:
        print(f"{label}: N/A")
        return
    print(
        f"{label}: mean={_fmt(stats['mean'])}, p50={_fmt(stats['p50'])}, "
        f"p95={_fmt(stats['p95'])}, p99={_fmt(stats['p99'])}, max={_fmt(stats['max'])}"
    )


def _print_stats_no_p50(label: str, values: list[float]) -> None:
    stats = _stats(values)
    if int(stats["count"]) == 0:
        print(f"{label}: N/A")
        return
    print(
        f"{label}: mean={_fmt(stats['mean'])}, p95={_fmt(stats['p95'])}, "
        f"p99={_fmt(stats['p99'])}, max={_fmt(stats['max'])}"
    )


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


def _load_receiver_rows(path: Path) -> list[dict[str, Any]]:
    suffix = path.suffix.lower()

    if suffix == ".csv":
        with path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            return [dict(row) for row in reader]

    rows: list[dict[str, Any]] = []
    for record in read_jsonl_records(path, strict=False):
        if str(record.get("record_type", "")) != "receiver_timing":
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


def _collect_ints(rows: list[dict[str, Any]], key: str) -> list[int]:
    out: list[int] = []
    for row in rows:
        value = _as_int(row.get(key))
        if value is not None:
            out.append(value)
    return out


def _pick_time_field(rows: list[dict[str, Any]], candidates: tuple[str, ...]) -> str | None:
    for field in candidates:
        count = 0
        for row in rows:
            if _as_int(row.get(field)) is not None:
                count += 1
            if count >= 2:
                return field
    return None


def _duration_and_rate(rows: list[dict[str, Any]], candidates: tuple[str, ...]) -> tuple[float | None, float | None, str | None]:
    field = _pick_time_field(rows, candidates)
    if field is None:
        return None, None, None

    values = _collect_ints(rows, field)
    if len(values) < 2:
        return None, None, field

    duration_ns = max(values) - min(values)
    if duration_ns <= 0:
        return 0.0, None, field

    duration_s = float(duration_ns) / 1_000_000_000.0
    effective_hz = float(len(values)) / duration_s if duration_s > 0.0 else None
    return duration_s, effective_hz, field


def _collect_intervals_ms(rows: list[dict[str, Any]], field: str) -> list[float]:
    values = _collect_ints(rows, field)
    if len(values) < 2:
        return []

    intervals: list[float] = []
    for idx in range(1, len(values)):
        dt_ns = values[idx] - values[idx - 1]
        if dt_ns >= 0:
            intervals.append(float(dt_ns) / 1_000_000.0)
    return intervals


def _collect_gaps_ms(
    rows: list[dict[str, Any]],
    *,
    time_field: str,
    predicate: callable,
) -> list[float]:
    values: list[int] = []
    for row in rows:
        if not predicate(row):
            continue
        ts = _as_int(row.get(time_field))
        if ts is not None:
            values.append(ts)

    if len(values) < 2:
        return []

    gaps: list[float] = []
    for idx in range(1, len(values)):
        dt_ns = values[idx] - values[idx - 1]
        if dt_ns >= 0:
            gaps.append(float(dt_ns) / 1_000_000.0)
    return gaps


def _threshold_counts(values: list[float], thresholds_ms: tuple[float, ...]) -> dict[float, int]:
    result: dict[float, int] = {}
    for threshold in thresholds_ms:
        result[threshold] = sum(1 for value in values if value > float(threshold))
    return result


def _print_threshold_counts(label: str, values: list[float], thresholds_ms: tuple[float, ...]) -> None:
    if not values:
        print(f"{label}: N/A")
        return

    counts = _threshold_counts(values, thresholds_ms)
    total = len(values)
    parts = [
        f">{int(th)}ms={counts[th]} ({(float(counts[th]) / float(total)):.2%})"
        for th in thresholds_ms
    ]
    print(f"{label}: " + ", ".join(parts))


def _collect_send_interval_ms(rows: list[dict[str, Any]]) -> list[float]:
    time_field = _pick_time_field(rows, ("loop_wall_ns", "loop_perf_ns"))
    if time_field is None:
        return []

    sent_wall_ns: list[int] = []
    for row in rows:
        left_sent = _as_bool(row.get("left_sent")) is True
        right_sent = _as_bool(row.get("right_sent")) is True
        if not (left_sent or right_sent):
            continue

        wall_ns = _as_int(row.get(time_field))
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


def _is_rejected_reason(text: str) -> bool:
    reason = text.strip().lower()
    if not reason:
        return False
    reject_markers = ("limit", "reject", "ik_failed", "invalid", "stale")
    return any(marker in reason for marker in reject_markers)


def _rows_for_state(rows: list[dict[str, Any]], state: str) -> list[dict[str, Any]]:
    selected: list[dict[str, Any]] = []
    for row in rows:
        value = _as_str(row.get("safety_state"))
        if value == state:
            selected.append(row)
    return selected


def _summarize_predictive_fields(rows: list[dict[str, Any]]) -> None:
    mode_counts: Counter[str] = Counter()
    for row in rows:
        mode = _as_str(row.get("pico_resample_mode"))
        if mode is not None and mode != "":
            mode_counts[mode] += 1

    if mode_counts:
        print(f"pico_resample_mode counts: {mode_counts.most_common()}")
    else:
        print("pico_resample_mode counts: N/A")

    prediction_used_true = 0
    prediction_used_false = 0
    for row in rows:
        used_value = _as_bool(row.get("pico_prediction_used"))
        if used_value is True:
            prediction_used_true += 1
        elif used_value is False:
            prediction_used_false += 1

    prediction_used_known = prediction_used_true + prediction_used_false
    prediction_used_ratio = (
        float(prediction_used_true) / float(prediction_used_known)
        if prediction_used_known > 0
        else None
    )
    print(
        "prediction_used: "
        f"true={prediction_used_true}, false={prediction_used_false}, ratio={_fmt(prediction_used_ratio, digits=4)}"
    )

    _print_stats("pico_prediction_h_ms", _collect_numeric(rows, "pico_prediction_h_ms"))
    _print_stats_no_p50(
        "pico_prediction_frame_age_ms",
        _collect_numeric(rows, "pico_prediction_frame_age_ms"),
    )

    reason_counts: Counter[str] = Counter()
    for row in rows:
        reason = _as_str(row.get("pico_prediction_reason"))
        if reason is not None and reason != "":
            reason_counts[reason] += 1

    if reason_counts:
        print(f"prediction reasons: {reason_counts.most_common(8)}")
    else:
        print("prediction reasons: N/A")

    prediction_clamped_true = 0
    prediction_clamped_false = 0
    for row in rows:
        clamped_value = _as_bool(row.get("pico_prediction_clamped"))
        if clamped_value is True:
            prediction_clamped_true += 1
        elif clamped_value is False:
            prediction_clamped_false += 1

    prediction_clamped_known = prediction_clamped_true + prediction_clamped_false
    prediction_clamped_ratio = (
        float(prediction_clamped_true) / float(prediction_clamped_known)
        if prediction_clamped_known > 0
        else None
    )
    print(
        "prediction_clamped: "
        f"true={prediction_clamped_true}, false={prediction_clamped_false}, "
        f"ratio={_fmt(prediction_clamped_ratio, digits=4)}"
    )

    _print_stats("latest_left_input_speed_mm_s", _collect_numeric(rows, "latest_left_input_speed_mm_s"))
    _print_stats("latest_right_input_speed_mm_s", _collect_numeric(rows, "latest_right_input_speed_mm_s"))
    _print_stats("predicted_left_pos_step_mm", _collect_numeric(rows, "predicted_left_pos_step_mm"))
    _print_stats("predicted_right_pos_step_mm", _collect_numeric(rows, "predicted_right_pos_step_mm"))


def _bool_count_and_ratio(rows: list[dict[str, Any]], key: str) -> tuple[int, int, float | None]:
    true_count = 0
    known_count = 0
    for row in rows:
        value = _as_bool(row.get(key))
        if value is None:
            continue
        known_count += 1
        if value:
            true_count += 1
    ratio = (float(true_count) / float(known_count)) if known_count > 0 else None
    return true_count, known_count, ratio


def _summarize_safety_fields(rows: list[dict[str, Any]]) -> None:
    left_reason_counts: Counter[str] = Counter()
    right_reason_counts: Counter[str] = Counter()
    for row in rows:
        left_reason = _as_str(row.get("safety_left_reason"))
        right_reason = _as_str(row.get("safety_right_reason"))
        if left_reason:
            left_reason_counts[left_reason] += 1
        if right_reason:
            right_reason_counts[right_reason] += 1

    if left_reason_counts:
        print(f"safety_left_reason counts: {left_reason_counts.most_common(8)}")
    else:
        print("safety_left_reason counts: N/A")

    if right_reason_counts:
        print(f"safety_right_reason counts: {right_reason_counts.most_common(8)}")
    else:
        print("safety_right_reason counts: N/A")

    left_clamped_count, left_clamped_known, left_clamped_ratio = _bool_count_and_ratio(rows, "safety_left_clamped")
    right_clamped_count, right_clamped_known, right_clamped_ratio = _bool_count_and_ratio(rows, "safety_right_clamped")
    print(
        "safety_left_clamped: "
        f"count={left_clamped_count}, known={left_clamped_known}, ratio={_fmt(left_clamped_ratio, digits=4)}"
    )
    print(
        "safety_right_clamped: "
        f"count={right_clamped_count}, known={right_clamped_known}, ratio={_fmt(right_clamped_ratio, digits=4)}"
    )

    left_reanchored_count, left_reanchored_known, left_reanchored_ratio = _bool_count_and_ratio(
        rows,
        "safety_left_reanchored",
    )
    right_reanchored_count, right_reanchored_known, right_reanchored_ratio = _bool_count_and_ratio(
        rows,
        "safety_right_reanchored",
    )
    print(
        "safety_left_reanchored: "
        f"count={left_reanchored_count}, known={left_reanchored_known}, ratio={_fmt(left_reanchored_ratio, digits=4)}"
    )
    print(
        "safety_right_reanchored: "
        f"count={right_reanchored_count}, known={right_reanchored_known}, ratio={_fmt(right_reanchored_ratio, digits=4)}"
    )

    clamp_distance_values = _collect_numeric(rows, "safety_left_clamp_distance_mm") + _collect_numeric(
        rows,
        "safety_right_clamp_distance_mm",
    )
    raw_to_safe_values = _collect_numeric(rows, "safety_left_raw_to_safe_error_mm") + _collect_numeric(
        rows,
        "safety_right_raw_to_safe_error_mm",
    )
    reanchor_offset_values = _collect_numeric(rows, "safety_left_reanchor_offset_norm_mm") + _collect_numeric(
        rows,
        "safety_right_reanchor_offset_norm_mm",
    )
    reanchor_gap_values = _collect_numeric(rows, "safety_left_reanchor_gap_ms") + _collect_numeric(
        rows,
        "safety_right_reanchor_gap_ms",
    )
    clamp_streak_values = _collect_numeric(rows, "safety_left_clamp_streak_ms") + _collect_numeric(
        rows,
        "safety_right_clamp_streak_ms",
    )

    _print_stats("safety_clamp_distance_mm", clamp_distance_values)
    _print_stats("safety_raw_to_safe_error_mm", raw_to_safe_values)
    _print_stats("safety_reanchor_offset_norm_mm", reanchor_offset_values)
    _print_stats("safety_reanchor_gap_ms", reanchor_gap_values)
    _print_stats("safety_clamp_streak_ms", clamp_streak_values)


def _print_state_count_table(rows: list[dict[str, Any]]) -> None:
    counter: Counter[str] = Counter()
    for row in rows:
        state = _as_str(row.get("safety_state"))
        counter[state if state is not None and state != "" else "<missing>"] += 1

    if not counter:
        print("Safety state counts: N/A")
        return

    print("Safety state counts:")
    for state, count in counter.most_common():
        print(f"  {state}: {count}")


def _summarize_main_subset(
    rows: list[dict[str, Any]],
    *,
    title: str,
    overrun_threshold_ms: float,
) -> None:
    print(f"\n=== Main Timing Summary: {title} ===")
    if not rows:
        print("count: 0")
        print("No rows in this subset.")
        return

    duration_s, effective_loop_hz, duration_field = _duration_and_rate(rows, ("loop_wall_ns", "loop_perf_ns"))
    print(f"count: {len(rows)}")
    print(f"duration_s ({duration_field or 'N/A'}): {_fmt(duration_s)}")
    print(f"effective_loop_hz: {_fmt(effective_loop_hz)}")

    _print_stats("loop_dt_ms", _collect_numeric(rows, "loop_dt_ms"))
    _print_stats("loop_total_ms", _collect_numeric(rows, "loop_total_ms"))
    _print_stats("deadline_late_ms", _collect_numeric(rows, "deadline_late_ms"))

    deadline_late_values = _collect_numeric(rows, "deadline_late_ms")
    real_overrun_count = sum(1 for value in deadline_late_values if value > float(overrun_threshold_ms))
    real_overrun_ratio = float(real_overrun_count) / float(len(rows)) if rows else 0.0
    print(
        f"real_overrun (deadline_late_ms > {overrun_threshold_ms:.3f}ms): "
        f"count={real_overrun_count}, ratio={real_overrun_ratio:.3%}"
    )

    send_intervals = _collect_send_interval_ms(rows)
    _print_stats("send_interval_ms", send_intervals)
    _print_threshold_counts("send gap counts", send_intervals, (12.0, 15.0, 20.0, 30.0))

    not_sent_count = sum(
        1
        for row in rows
        if (_as_bool(row.get("left_sent")) is not True and _as_bool(row.get("right_sent")) is not True)
    )
    no_target_count = sum(1 for row in rows if _as_bool(row.get("no_target")) is True)
    send_failed_count = sum(1 for row in rows if _as_bool(row.get("send_failed")) is True)
    rejected_count = 0

    left_reasons = Counter()
    right_reasons = Counter()
    for row in rows:
        left_reason = _as_str(row.get("left_reason")) or ""
        right_reason = _as_str(row.get("right_reason")) or ""
        if left_reason:
            left_reasons[left_reason] += 1
        if right_reason:
            right_reasons[right_reason] += 1
        if _is_rejected_reason(left_reason) or _is_rejected_reason(right_reason):
            rejected_count += 1

    print(
        "command events: "
        f"not_sent={not_sent_count}, no_target={no_target_count}, "
        f"send_failed={send_failed_count}, rejected={rejected_count}"
    )
    print(f"top left reasons: {left_reasons.most_common(5)}")
    print(f"top right reasons: {right_reasons.most_common(5)}")
    _summarize_safety_fields(rows)

    _print_stats_no_p50("read_feedback_ms", _collect_numeric(rows, "read_feedback_ms"))
    _print_stats_no_p50("send_command_ms", _collect_numeric(rows, "send_command_ms"))

    frame_age_values = _collect_numeric(rows, "frame_age_ms")
    _print_stats("frame_age_ms", frame_age_values)
    _print_threshold_counts("frame_age thresholds", frame_age_values, (20.0, 30.0, 50.0, 100.0))

    pico_new_true = 0
    pico_new_false = 0
    for row in rows:
        value = _as_bool(row.get("pico_frame_new"))
        if value is True:
            pico_new_true += 1
        elif value is False:
            pico_new_false += 1

    total_pico_new_known = pico_new_true + pico_new_false
    pico_new_ratio = (
        (float(pico_new_true) / float(total_pico_new_known))
        if total_pico_new_known > 0
        else None
    )
    effective_pico_new_hz = (
        (float(pico_new_true) / float(duration_s))
        if duration_s is not None and duration_s > 0.0
        else None
    )
    print(
        "pico_frame_new stats: "
        f"true={pico_new_true}, false={pico_new_false}, "
        f"ratio={_fmt(pico_new_ratio, digits=4)}, effective_pico_new_frame_hz={_fmt(effective_pico_new_hz)}"
    )

    receiver_seq_delta_values = _collect_numeric(rows, "pico_receiver_seq_delta")
    _print_stats("pico_receiver_seq_delta", receiver_seq_delta_values)

    total_skipped_frames = 0
    rows_with_skipped_frames = 0
    for row in rows:
        skipped_value = _as_int(row.get("pico_skipped_receiver_frames"))
        if skipped_value is None:
            continue
        total_skipped_frames += max(0, int(skipped_value))
        if skipped_value > 0:
            rows_with_skipped_frames += 1

    skipped_rows_ratio = float(rows_with_skipped_frames) / float(len(rows)) if rows else 0.0
    skipped_frames_per_second = (
        float(total_skipped_frames) / float(duration_s)
        if duration_s is not None and duration_s > 0.0
        else None
    )
    print(
        "skipped receiver frames: "
        f"total_pico_skipped_receiver_frames={total_skipped_frames}, "
        f"rows_with_skipped_receiver_frames={rows_with_skipped_frames}, "
        f"skipped_rows_ratio={skipped_rows_ratio:.3%}, "
        f"skipped_frames_per_second={_fmt(skipped_frames_per_second)}"
    )

    receiver_delta_distribution = {
        "delta==0": 0,
        "delta==1": 0,
        "delta==2": 0,
        "delta==3": 0,
        "delta>=4": 0,
    }
    receiver_delta_known_count = 0
    for row in rows:
        delta_value = _as_int(row.get("pico_receiver_seq_delta"))
        if delta_value is None:
            continue
        receiver_delta_known_count += 1
        if delta_value == 0:
            receiver_delta_distribution["delta==0"] += 1
        elif delta_value == 1:
            receiver_delta_distribution["delta==1"] += 1
        elif delta_value == 2:
            receiver_delta_distribution["delta==2"] += 1
        elif delta_value == 3:
            receiver_delta_distribution["delta==3"] += 1
        elif delta_value >= 4:
            receiver_delta_distribution["delta>=4"] += 1

    if receiver_delta_known_count == 0:
        print("receiver_seq_delta distribution: N/A")
    else:
        print(
            "receiver_seq_delta distribution: "
            + ", ".join(
                f"{key}={value}" for key, value in receiver_delta_distribution.items()
            )
        )

    receiver_seq_values = _collect_ints(rows, "pico_receiver_seq")
    effective_receiver_seq_hz_seen_by_main: float | None = None
    if receiver_seq_values and duration_s is not None and duration_s > 0.0:
        receiver_seq_range = max(receiver_seq_values) - min(receiver_seq_values)
        if receiver_seq_range >= 0:
            effective_receiver_seq_hz_seen_by_main = float(receiver_seq_range) / float(duration_s)
    print(
        "effective_receiver_seq_hz_seen_by_main: "
        f"{_fmt(effective_receiver_seq_hz_seen_by_main)}"
    )

    _summarize_predictive_fields(rows)

    gap_time_field = _pick_time_field(rows, ("loop_wall_ns", "loop_perf_ns"))
    pico_new_gaps: list[float] = []
    if gap_time_field is not None:
        pico_new_gaps = _collect_gaps_ms(
            rows,
            time_field=gap_time_field,
            predicate=lambda row: _as_bool(row.get("pico_frame_new")) is True,
        )
    _print_stats("pico_new_frame_gap_ms", pico_new_gaps)
    _print_threshold_counts("pico new-frame gap counts", pico_new_gaps, (20.0, 30.0, 50.0, 100.0))


def _summarize_receiver_rows(rows: list[dict[str, Any]]) -> None:
    print("\n=== Receiver Timing Summary ===")
    if not rows:
        print("receiver_count: 0")
        print("No receiver timing rows found.")
        return

    duration_s, effective_receiver_hz, duration_field = _duration_and_rate(
        rows,
        ("pc_receive_wall_ns", "pc_receive_perf_ns"),
    )
    print(f"receiver_count: {len(rows)}")
    print(f"receiver_duration_s ({duration_field or 'N/A'}): {_fmt(duration_s)}")
    print(f"effective_receiver_hz: {_fmt(effective_receiver_hz)}")

    interval_field = _pick_time_field(rows, ("pc_receive_perf_ns", "pc_receive_wall_ns"))
    receiver_pc_intervals = _collect_intervals_ms(rows, interval_field) if interval_field is not None else []
    _print_stats("receiver_pc_interval_ms", receiver_pc_intervals)
    _print_threshold_counts("receiver pc gap counts", receiver_pc_intervals, (20.0, 30.0, 50.0, 100.0))

    positive_pico_rows: list[dict[str, Any]] = []
    for row in rows:
        ts = _as_int(row.get("pico_source_timestamp_ns"))
        if ts is not None and ts > 0:
            positive_pico_rows.append(row)
    receiver_pico_internal_intervals = _collect_intervals_ms(positive_pico_rows, "pico_source_timestamp_ns")
    _print_stats("receiver_pico_internal_interval_ms", receiver_pico_internal_intervals)
    _print_threshold_counts(
        "receiver pico internal gap counts",
        receiver_pico_internal_intervals,
        (20.0, 30.0, 50.0, 100.0),
    )

    _print_stats_no_p50("parse_duration_ms", _collect_numeric(rows, "parse_duration_ms"))
    _print_stats_no_p50("json_size_bytes", _collect_numeric(rows, "json_size_bytes"))

    frame_seq_values = _collect_ints(rows, "frame_seq")
    if not frame_seq_values:
        print("frame_seq integrity: N/A")
        return

    duplicate_count = 0
    skipped_count = 0
    out_of_order_count = 0
    seen: set[int] = set()
    prev: int | None = None

    for seq in frame_seq_values:
        if seq in seen:
            duplicate_count += 1
        else:
            seen.add(seq)

        if prev is not None:
            if seq < prev:
                out_of_order_count += 1
            elif seq > prev + 1:
                skipped_count += int(seq - prev - 1)
        prev = seq

    print(
        "frame_seq integrity: "
        f"duplicate={duplicate_count}, skipped={skipped_count}, out_of_order={out_of_order_count}"
    )


def _receiver_rows_during_active_window(
    receiver_rows: list[dict[str, Any]],
    *,
    active_start_wall_ns: int,
    active_end_wall_ns: int,
) -> list[dict[str, Any]]:
    selected: list[dict[str, Any]] = []
    for row in receiver_rows:
        wall_ns = _as_int(row.get("pc_receive_wall_ns"))
        if wall_ns is None:
            continue
        if active_start_wall_ns <= wall_ns <= active_end_wall_ns:
            selected.append(row)
    return selected


def _main_active_window(rows: list[dict[str, Any]]) -> tuple[int, int] | None:
    active_rows = _rows_for_state(rows, "TELEOP_ACTIVE")
    active_times = _collect_ints(active_rows, "loop_wall_ns")
    if len(active_times) < 2:
        return None
    return (min(active_times), max(active_times))


def _effective_pico_new_hz(rows: list[dict[str, Any]]) -> float | None:
    duration_s, _, _ = _duration_and_rate(rows, ("loop_wall_ns", "loop_perf_ns"))
    if duration_s is None or duration_s <= 0.0:
        return None

    new_count = sum(1 for row in rows if _as_bool(row.get("pico_frame_new")) is True)
    return float(new_count) / float(duration_s)


def _print_active_receiver_comparison(main_rows: list[dict[str, Any]], receiver_rows: list[dict[str, Any]]) -> None:
    window = _main_active_window(main_rows)
    if window is None:
        print("\n=== Active Window Comparison ===")
        print("Active window unavailable (missing TELEOP_ACTIVE loop_wall_ns data).")
        return

    active_start_wall_ns, active_end_wall_ns = window
    active_duration_s = float(active_end_wall_ns - active_start_wall_ns) / 1_000_000_000.0
    if active_duration_s <= 0.0:
        print("\n=== Active Window Comparison ===")
        print("Active window duration is non-positive.")
        return

    receiver_active_rows = _receiver_rows_during_active_window(
        receiver_rows,
        active_start_wall_ns=active_start_wall_ns,
        active_end_wall_ns=active_end_wall_ns,
    )
    receiver_count_active = len(receiver_active_rows)
    receiver_hz_active = float(receiver_count_active) / active_duration_s

    main_active_rows = _rows_for_state(main_rows, "TELEOP_ACTIVE")
    main_pico_new_hz_active = _effective_pico_new_hz(main_active_rows)
    ratio = None
    if main_pico_new_hz_active is not None and main_pico_new_hz_active > 0.0:
        ratio = receiver_hz_active / main_pico_new_hz_active

    print("\n=== Active Window Comparison ===")
    print(f"active_start_wall_ns: {active_start_wall_ns}")
    print(f"active_end_wall_ns: {active_end_wall_ns}")
    print(f"active_duration_s: {_fmt(active_duration_s)}")
    print(f"receiver_count_during_active: {receiver_count_active}")
    print(f"receiver_hz_during_active: {_fmt(receiver_hz_active)}")
    print(f"main_pico_new_frame_hz_during_active: {_fmt(main_pico_new_hz_active)}")
    print(f"receiver_vs_main_new_frame_ratio: {_fmt(ratio)}")
    print("Interpretation hint:")
    print(
        "  - If receiver_hz_during_active is also low or has large gaps, the issue is likely upstream of or inside PicoReceiver input."
    )
    print(
        "  - If receiver_hz_during_active is stable near 100Hz but main_pico_new_frame_hz is much lower, the issue is likely between receiver latest-frame update and main-loop consumption."
    )
    print(
        "  - If both are stable but stutter remains, the issue may be target/q content discontinuity rather than timing frequency."
    )


def main() -> None:
    args = parse_args()
    input_path = Path(args.input)

    rows = _load_rows(input_path)
    if not rows:
        print(f"Input: {input_path}")
        print("No timing rows found.")
        return
    print(f"Input: {input_path}")

    if args.all_states:
        _print_state_count_table(rows)

    if args.state is not None:
        filtered_rows = _rows_for_state(rows, args.state)
        _summarize_main_subset(
            filtered_rows,
            title=f"state={args.state}",
            overrun_threshold_ms=float(args.overrun_threshold_ms),
        )
    else:
        _summarize_main_subset(
            rows,
            title="all_rows",
            overrun_threshold_ms=float(args.overrun_threshold_ms),
        )

        has_safety_state = any(_as_str(row.get("safety_state")) is not None for row in rows)
        active_rows = _rows_for_state(rows, "TELEOP_ACTIVE") if has_safety_state else []
        if active_rows:
            _summarize_main_subset(
                active_rows,
                title="TELEOP_ACTIVE",
                overrun_threshold_ms=float(args.overrun_threshold_ms),
            )

    if args.receiver_input is not None:
        receiver_path = Path(args.receiver_input)
        receiver_rows = _load_receiver_rows(receiver_path)
        print(f"\nReceiver input: {receiver_path}")
        _summarize_receiver_rows(receiver_rows)
        _print_active_receiver_comparison(rows, receiver_rows)


if __name__ == "__main__":
    main()
