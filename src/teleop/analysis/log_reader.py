from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Sequence


@dataclass
class LogRecord:
    record_type: str
    timestamp_ns: int
    level: str
    event: str
    payload: dict[str, Any]
    sequence_id: int | None
    line_no: int
    t_s: float


@dataclass
class TeleopStepLog:
    timestamp_ns: int
    t_s: float
    sequence_id: int | None
    frame_id: int | None

    safety_state: str | None
    allow_motion: bool | None
    command_ready: bool | None

    feedback_left_xyz_mm: tuple[float, float, float] | None
    feedback_left_abc_deg: tuple[float, float, float] | None
    feedback_right_xyz_mm: tuple[float, float, float] | None
    feedback_right_abc_deg: tuple[float, float, float] | None

    command_left_q_deg: tuple[float, ...] | None
    command_right_q_deg: tuple[float, ...] | None
    command_left_sent: bool | None
    command_right_sent: bool | None
    command_left_reason: str | None
    command_right_reason: str | None

    raw_payload: dict[str, Any]


def read_jsonl_records(path: str | Path, strict: bool = False) -> list[LogRecord]:
    file_path = Path(path)
    parsed_lines: list[tuple[int, dict[str, Any]]] = []

    with file_path.open("r", encoding="utf-8") as handle:
        for line_no, line in enumerate(handle, start=1):
            text = line.strip()
            if not text:
                continue

            try:
                obj = json.loads(text)
            except json.JSONDecodeError as exc:
                if strict:
                    raise ValueError(f"Invalid JSON line at {line_no} in {file_path}: {exc.msg}") from exc
                continue

            if not isinstance(obj, dict):
                if strict:
                    raise ValueError(f"JSON line {line_no} in {file_path} is not an object")
                continue

            parsed_lines.append((line_no, obj))

    first_valid_timestamp_ns = _first_valid_timestamp(parsed_lines)
    baseline_ns = first_valid_timestamp_ns if first_valid_timestamp_ns is not None else 0

    records: list[LogRecord] = []
    for line_no, obj in parsed_lines:
        timestamp_ns = as_int_or_none(obj.get("timestamp_ns"))
        if timestamp_ns is None:
            timestamp_ns = baseline_ns

        payload_value = obj.get("payload")
        payload = dict(payload_value) if isinstance(payload_value, dict) else {}

        t_s = 0.0
        if first_valid_timestamp_ns is not None:
            t_s = float(timestamp_ns - baseline_ns) / 1_000_000_000.0

        records.append(
            LogRecord(
                record_type=as_str_or_none(obj.get("record_type")) or "",
                timestamp_ns=int(timestamp_ns),
                level=as_str_or_none(obj.get("level")) or "",
                event=as_str_or_none(obj.get("event")) or "",
                payload=payload,
                sequence_id=as_int_or_none(obj.get("sequence_id")),
                line_no=line_no,
                t_s=t_s,
            )
        )

    return records


def filter_records(
    records: Sequence[LogRecord],
    record_type: str | None = None,
    event: str | None = None,
) -> list[LogRecord]:
    filtered: list[LogRecord] = []
    for record in records:
        if record_type is not None and record.record_type != record_type:
            continue
        if event is not None and record.event != event:
            continue
        filtered.append(record)
    return filtered


def extract_teleop_steps(records: Sequence[LogRecord]) -> list[TeleopStepLog]:
    steps: list[TeleopStepLog] = []

    for record in records:
        if record.record_type != "frame" or record.event != "teleop_step":
            continue

        payload = dict(record.payload) if isinstance(record.payload, dict) else {}

        steps.append(
            TeleopStepLog(
                timestamp_ns=record.timestamp_ns,
                t_s=record.t_s,
                sequence_id=record.sequence_id,
                frame_id=as_int_or_none(payload.get("frame_id")),
                safety_state=as_str_or_none(payload.get("safety_state")),
                allow_motion=as_bool_or_none(payload.get("allow_motion")),
                command_ready=as_bool_or_none(payload.get("command_ready")),
                feedback_left_xyz_mm=_as_float_tuple3(payload.get("feedback_left_xyz_mm")),
                feedback_left_abc_deg=_as_float_tuple3(payload.get("feedback_left_abc_deg")),
                feedback_right_xyz_mm=_as_float_tuple3(payload.get("feedback_right_xyz_mm")),
                feedback_right_abc_deg=_as_float_tuple3(payload.get("feedback_right_abc_deg")),
                command_left_q_deg=as_float_tuple(payload.get("command_left_q_deg"), length=7),
                command_right_q_deg=as_float_tuple(payload.get("command_right_q_deg"), length=7),
                command_left_sent=as_bool_or_none(payload.get("command_left_sent")),
                command_right_sent=as_bool_or_none(payload.get("command_right_sent")),
                command_left_reason=as_str_or_none(payload.get("command_left_reason")),
                command_right_reason=as_str_or_none(payload.get("command_right_reason")),
                raw_payload=payload,
            )
        )

    return steps


def extract_events(records: Sequence[LogRecord]) -> list[LogRecord]:
    return filter_records(records, record_type="event")


def as_float_tuple(value: Any, length: int) -> tuple[float, ...] | None:
    if isinstance(value, (str, bytes)):
        return None
    if not isinstance(value, Sequence):
        return None
    if len(value) != int(length):
        return None

    result: list[float] = []
    for one in value:
        try:
            number = float(one)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(number):
            return None
        result.append(number)
    return tuple(result)


def as_bool_or_none(value: Any) -> bool | None:
    if value is None:
        return None
    if isinstance(value, bool):
        return value

    if isinstance(value, (int, float)) and not isinstance(value, bool):
        if float(value) == 1.0:
            return True
        if float(value) == 0.0:
            return False
        return None

    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "1", "yes", "y"}:
            return True
        if normalized in {"false", "0", "no", "n"}:
            return False

    return None


def as_int_or_none(value: Any) -> int | None:
    if value is None or isinstance(value, bool):
        return None

    if isinstance(value, int):
        return int(value)

    if isinstance(value, float):
        if not math.isfinite(value) or not float(value).is_integer():
            return None
        return int(value)

    if isinstance(value, str):
        text = value.strip()
        if not text:
            return None

        try:
            if any(marker in text.lower() for marker in (".", "e")):
                as_float = float(text)
                if not math.isfinite(as_float) or not as_float.is_integer():
                    return None
                return int(as_float)
            return int(text)
        except ValueError:
            return None

    return None


def as_str_or_none(value: Any) -> str | None:
    if value is None:
        return None
    if isinstance(value, str):
        return value
    return str(value)


def _as_float_tuple3(value: Any) -> tuple[float, float, float] | None:
    parsed = as_float_tuple(value, length=3)
    if parsed is None:
        return None
    return (parsed[0], parsed[1], parsed[2])


def _first_valid_timestamp(parsed_lines: Sequence[tuple[int, dict[str, Any]]]) -> int | None:
    for _, obj in parsed_lines:
        ts = as_int_or_none(obj.get("timestamp_ns"))
        if ts is not None:
            return ts
    return None


__all__ = [
    "LogRecord",
    "TeleopStepLog",
    "read_jsonl_records",
    "filter_records",
    "extract_teleop_steps",
    "extract_events",
    "as_float_tuple",
    "as_bool_or_none",
    "as_int_or_none",
    "as_str_or_none",
]
