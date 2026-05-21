from __future__ import annotations

from dataclasses import dataclass
import csv
import importlib.util
import math
from pathlib import Path
from typing import Any, Sequence

from teleop.analysis.log_reader import TeleopStepLog


@dataclass
class SideQSeries:
    side: str
    t_s: list[float]
    frame_id: list[int | None]
    q_deg: list[tuple[float, ...] | None]
    max_step_deg: list[float | None]
    max_velocity_deg_s: list[float | None]
    reasons: list[str | None]
    sent_flags: list[bool | None]


@dataclass
class RejectMarker:
    side: str
    t_s: float
    frame_id: int | None
    reason: str
    index: int


@dataclass
class IKReplaySummary:
    mode: str
    left: SideQSeries
    right: SideQSeries
    reject_markers: list[RejectMarker]
    notes: list[str]


def build_q_series_from_recorded_commands(
    steps: Sequence[TeleopStepLog],
    side: str,
) -> SideQSeries:
    side_norm = _normalize_side(side)

    t_values: list[float] = []
    frame_ids: list[int | None] = []
    q_values: list[tuple[float, ...] | None] = []
    max_steps: list[float | None] = []
    max_velocities: list[float | None] = []
    reasons: list[str | None] = []
    sent_flags: list[bool | None] = []

    prev_valid_q: tuple[float, ...] | None = None
    prev_valid_t_s: float | None = None

    q_attr = "command_left_q_deg" if side_norm == "left" else "command_right_q_deg"
    reason_attr = "command_left_reason" if side_norm == "left" else "command_right_reason"
    sent_attr = "command_left_sent" if side_norm == "left" else "command_right_sent"

    for step in steps:
        t_values.append(float(step.t_s))
        frame_ids.append(step.frame_id)
        reasons.append(getattr(step, reason_attr))
        sent_flags.append(getattr(step, sent_attr))

        q_raw = getattr(step, q_attr)
        q_curr = _as_finite_float_tuple(q_raw)
        q_values.append(q_curr)

        if q_curr is None:
            max_steps.append(None)
            max_velocities.append(None)
            continue

        if prev_valid_q is None:
            max_steps.append(None)
            max_velocities.append(None)
            prev_valid_q = q_curr
            prev_valid_t_s = float(step.t_s)
            continue

        step_deg: float | None = None
        if len(prev_valid_q) == len(q_curr):
            step_deg = _max_abs(vec_sub(q_curr, prev_valid_q))
        max_steps.append(step_deg)

        if step_deg is None or prev_valid_t_s is None:
            max_velocities.append(None)
        else:
            dt_s = float(step.t_s) - float(prev_valid_t_s)
            if dt_s <= 0.0:
                max_velocities.append(None)
            else:
                max_velocities.append(float(step_deg) / dt_s)

        prev_valid_q = q_curr
        prev_valid_t_s = float(step.t_s)

    return SideQSeries(
        side=side_norm,
        t_s=t_values,
        frame_id=frame_ids,
        q_deg=q_values,
        max_step_deg=max_steps,
        max_velocity_deg_s=max_velocities,
        reasons=reasons,
        sent_flags=sent_flags,
    )


def find_reject_markers(
    steps: Sequence[TeleopStepLog],
    limit_per_side: int = 3,
) -> list[RejectMarker]:
    limit = max(0, int(limit_per_side))
    markers: list[RejectMarker] = []
    counts = {"left": 0, "right": 0}

    for index, step in enumerate(steps):
        left_reason = step.command_left_reason
        if counts["left"] < limit and is_reject_reason(left_reason):
            markers.append(
                RejectMarker(
                    side="left",
                    t_s=float(step.t_s),
                    frame_id=step.frame_id,
                    reason=str(left_reason),
                    index=index,
                )
            )
            counts["left"] += 1

        right_reason = step.command_right_reason
        if counts["right"] < limit and is_reject_reason(right_reason):
            markers.append(
                RejectMarker(
                    side="right",
                    t_s=float(step.t_s),
                    frame_id=step.frame_id,
                    reason=str(right_reason),
                    index=index,
                )
            )
            counts["right"] += 1

        if counts["left"] >= limit and counts["right"] >= limit:
            break

    return markers


def summarize_q_jumps(
    series: SideQSeries,
    ready_q: Sequence[float],
) -> dict[str, Any]:
    ready = tuple(float(v) for v in ready_q)

    valid_indices = [idx for idx, q in enumerate(series.q_deg) if _same_joint_len(q, ready)]
    sent_indices = [
        idx
        for idx, q in enumerate(series.q_deg)
        if _same_joint_len(q, ready) and series.sent_flags[idx] is True
    ]

    summary: dict[str, Any] = {
        "side": series.side,
        "valid_q_samples": len(valid_indices),
        "sent_samples": sum(1 for flag in series.sent_flags if flag is True),
        "first_valid_index": None,
        "first_valid_t_s": None,
        "first_valid_frame_id": None,
        "first_valid_q_deg": None,
        "first_valid_max_abs_dq_deg": None,
        "first_valid_norm_dq_deg": None,
        "first_sent_index": None,
        "first_sent_t_s": None,
        "first_sent_frame_id": None,
        "first_sent_q_deg": None,
        "first_sent_max_abs_dq_deg": None,
        "first_sent_norm_dq_deg": None,
        "max_step_deg": None,
        "max_step_index": None,
        "max_step_t_s": None,
        "max_step_frame_id": None,
        "max_velocity_deg_s": None,
        "max_velocity_index": None,
        "max_velocity_t_s": None,
        "max_velocity_frame_id": None,
    }

    if valid_indices:
        idx = valid_indices[0]
        q0 = series.q_deg[idx]
        assert q0 is not None
        dq0 = vec_sub(q0, ready)
        summary["first_valid_index"] = idx
        summary["first_valid_t_s"] = series.t_s[idx]
        summary["first_valid_frame_id"] = series.frame_id[idx]
        summary["first_valid_q_deg"] = q0
        summary["first_valid_max_abs_dq_deg"] = _max_abs(dq0)
        summary["first_valid_norm_dq_deg"] = vec_norm(dq0)

    if sent_indices:
        idx = sent_indices[0]
        q1 = series.q_deg[idx]
        assert q1 is not None
        dq1 = vec_sub(q1, ready)
        summary["first_sent_index"] = idx
        summary["first_sent_t_s"] = series.t_s[idx]
        summary["first_sent_frame_id"] = series.frame_id[idx]
        summary["first_sent_q_deg"] = q1
        summary["first_sent_max_abs_dq_deg"] = _max_abs(dq1)
        summary["first_sent_norm_dq_deg"] = vec_norm(dq1)

    step_candidates = [
        (idx, value)
        for idx, value in enumerate(series.max_step_deg)
        if value is not None and math.isfinite(float(value))
    ]
    if step_candidates:
        idx, value = max(step_candidates, key=lambda item: float(item[1]))
        summary["max_step_deg"] = float(value)
        summary["max_step_index"] = idx
        summary["max_step_t_s"] = series.t_s[idx]
        summary["max_step_frame_id"] = series.frame_id[idx]

    velocity_candidates = [
        (idx, value)
        for idx, value in enumerate(series.max_velocity_deg_s)
        if value is not None and math.isfinite(float(value))
    ]
    if velocity_candidates:
        idx, value = max(velocity_candidates, key=lambda item: float(item[1]))
        summary["max_velocity_deg_s"] = float(value)
        summary["max_velocity_index"] = idx
        summary["max_velocity_t_s"] = series.t_s[idx]
        summary["max_velocity_frame_id"] = series.frame_id[idx]

    return summary


def try_replay_ik_from_targets(
    steps: Sequence[TeleopStepLog],
    side: str,
    require_sdk: bool = False,
) -> tuple[SideQSeries | None, list[str]]:
    side_norm = _normalize_side(side)

    xyz_keys, abc_keys = _target_keys_for_side(side_norm)
    with_targets = 0

    for step in steps:
        payload = dict(step.raw_payload) if isinstance(step.raw_payload, dict) else {}
        has_xyz = _payload_has_vector(payload, xyz_keys, length=3)
        has_abc = _payload_has_vector(payload, abc_keys, length=3)
        if has_xyz and has_abc:
            with_targets += 1

    total = len(steps)
    ratio = float(with_targets) / float(total) if total > 0 else 0.0

    if with_targets == 0 or ratio < 0.5:
        message = "target xyzabc fields not found; falling back to recorded command q"
        if bool(require_sdk):
            raise RuntimeError(message)
        return None, [message]

    sdk_spec = importlib.util.find_spec("python_sdk.fx_kine")
    if sdk_spec is None:
        message = "target xyzabc fields found but SDK IK module is unavailable; falling back to recorded command q"
        if bool(require_sdk):
            raise RuntimeError(message)
        return None, [message]

    message = "IK replay from target xyzabc is not implemented yet; falling back to recorded command q"
    if bool(require_sdk):
        raise RuntimeError(message)
    return None, [message]


def build_ik_replay_summary(
    steps: Sequence[TeleopStepLog],
    mode: str = "auto",
    side: str = "both",
    limit_per_side: int = 3,
    require_sdk: bool = False,
) -> IKReplaySummary:
    mode_norm = _normalize_mode(mode)
    selected_sides = _selected_sides(side)

    left_series = build_q_series_from_recorded_commands(steps, side="left")
    right_series = build_q_series_from_recorded_commands(steps, side="right")

    notes: list[str] = []
    actual_mode = "recorded"

    if mode_norm in {"auto", "recompute"}:
        recomputed: dict[str, SideQSeries] = {}

        for one_side in selected_sides:
            replay_series, replay_notes = try_replay_ik_from_targets(
                steps=steps,
                side=one_side,
                require_sdk=(mode_norm == "recompute") or bool(require_sdk),
            )
            notes.extend(replay_notes)
            if replay_series is not None:
                recomputed[one_side] = replay_series

        if mode_norm == "recompute":
            if len(recomputed) != len(selected_sides):
                raise RuntimeError("recompute mode requested but IK replay result is unavailable")
            actual_mode = "recompute"
            if "left" in recomputed:
                left_series = recomputed["left"]
            if "right" in recomputed:
                right_series = recomputed["right"]
        elif len(recomputed) == len(selected_sides) and selected_sides:
            actual_mode = "recompute"
            if "left" in recomputed:
                left_series = recomputed["left"]
            if "right" in recomputed:
                right_series = recomputed["right"]

    reject_markers = [
        marker
        for marker in find_reject_markers(steps=steps, limit_per_side=limit_per_side)
        if marker.side in selected_sides
    ]

    return IKReplaySummary(
        mode=actual_mode,
        left=left_series,
        right=right_series,
        reject_markers=reject_markers,
        notes=notes,
    )


def write_replay_q_timeseries_csv(path: Path, summary: IKReplaySummary) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    columns = [
        "t_s",
        "frame_id",
        "left_q1",
        "left_q2",
        "left_q3",
        "left_q4",
        "left_q5",
        "left_q6",
        "left_q7",
        "right_q1",
        "right_q2",
        "right_q3",
        "right_q4",
        "right_q5",
        "right_q6",
        "right_q7",
        "left_max_step_deg",
        "right_max_step_deg",
        "left_max_velocity_deg_s",
        "right_max_velocity_deg_s",
        "left_reason",
        "right_reason",
        "left_sent",
        "right_sent",
    ]

    n = max(len(summary.left.t_s), len(summary.right.t_s))

    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()

        for idx in range(n):
            left_q = _series_item(summary.left.q_deg, idx)
            right_q = _series_item(summary.right.q_deg, idx)
            row = {
                "t_s": _choose_time(summary.left.t_s, summary.right.t_s, idx),
                "frame_id": _choose_frame(summary.left.frame_id, summary.right.frame_id, idx),
                "left_q1": _joint_or_none(left_q, 0),
                "left_q2": _joint_or_none(left_q, 1),
                "left_q3": _joint_or_none(left_q, 2),
                "left_q4": _joint_or_none(left_q, 3),
                "left_q5": _joint_or_none(left_q, 4),
                "left_q6": _joint_or_none(left_q, 5),
                "left_q7": _joint_or_none(left_q, 6),
                "right_q1": _joint_or_none(right_q, 0),
                "right_q2": _joint_or_none(right_q, 1),
                "right_q3": _joint_or_none(right_q, 2),
                "right_q4": _joint_or_none(right_q, 3),
                "right_q5": _joint_or_none(right_q, 4),
                "right_q6": _joint_or_none(right_q, 5),
                "right_q7": _joint_or_none(right_q, 6),
                "left_max_step_deg": _series_item(summary.left.max_step_deg, idx),
                "right_max_step_deg": _series_item(summary.right.max_step_deg, idx),
                "left_max_velocity_deg_s": _series_item(summary.left.max_velocity_deg_s, idx),
                "right_max_velocity_deg_s": _series_item(summary.right.max_velocity_deg_s, idx),
                "left_reason": _series_item(summary.left.reasons, idx),
                "right_reason": _series_item(summary.right.reasons, idx),
                "left_sent": _series_item(summary.left.sent_flags, idx),
                "right_sent": _series_item(summary.right.sent_flags, idx),
            }
            writer.writerow(row)


def write_reject_markers_csv(path: Path, reject_markers: Sequence[RejectMarker]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    columns = ["side", "t_s", "frame_id", "reason", "index"]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for marker in reject_markers:
            writer.writerow(
                {
                    "side": marker.side,
                    "t_s": marker.t_s,
                    "frame_id": marker.frame_id,
                    "reason": marker.reason,
                    "index": marker.index,
                }
            )


def write_ik_replay_report(
    path: Path,
    input_path: Path,
    summary: IKReplaySummary,
    ready_left_q: Sequence[float],
    ready_right_q: Sequence[float],
) -> None:
    left_stats = summarize_q_jumps(summary.left, ready_left_q)
    right_stats = summarize_q_jumps(summary.right, ready_right_q)

    selected_mode = _note_value(summary.notes, key="selected_mode")
    if selected_mode is None:
        selected_mode = "unknown"

    lines: list[str] = []
    lines.append("# IK Replay / Q-Series Analysis Report")
    lines.append("")
    lines.append(f"- input file: {input_path}")
    lines.append(f"- selected mode: {selected_mode}")
    lines.append(f"- actual mode used: {summary.mode}")
    lines.append(f"- recomputed IK used: {'yes' if summary.mode == 'recompute' else 'no'}")
    lines.append("")

    lines.append("## Ready Q")
    lines.append("")
    lines.append(f"- left ready q: {tuple(float(v) for v in ready_left_q)}")
    lines.append(f"- right ready q: {tuple(float(v) for v in ready_right_q)}")
    lines.append("")

    lines.extend(_stats_lines("left", left_stats))
    lines.append("")
    lines.extend(_stats_lines("right", right_stats))
    lines.append("")

    lines.append("## First Reject Markers")
    lines.append("")
    if summary.reject_markers:
        for marker in summary.reject_markers:
            lines.append(
                f"- side={marker.side}, t_s={marker.t_s:.6f}, frame_id={marker.frame_id}, "
                f"reason={marker.reason}, index={marker.index}"
            )
    else:
        lines.append("- none")
    lines.append("")

    lines.append("## Notes")
    lines.append("")
    if summary.notes:
        for note in summary.notes:
            lines.append(f"- {note}")
    else:
        lines.append("- none")
    lines.append("")

    lines.append("## Interpretation Hints")
    lines.append("")
    lines.append("- A q step spike before reject usually suggests IK or target discontinuity.")
    lines.append("- Smooth q with frequent reject often suggests threshold or configuration issues.")
    lines.append(
        "- If target xyzabc fields are missing, enable richer command target logging for future IK replay recomputation."
    )
    lines.append("")

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def plot_ik_replay_summary(
    output_dir: Path,
    summary: IKReplaySummary,
    no_plots: bool = False,
) -> list[Path]:
    if bool(no_plots):
        return []

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return []

    output_dir.mkdir(parents=True, exist_ok=True)

    generated: list[Path] = []

    left_reject_times = [marker.t_s for marker in summary.reject_markers if marker.side == "left"]
    right_reject_times = [marker.t_s for marker in summary.reject_markers if marker.side == "right"]
    all_reject_times = [marker.t_s for marker in summary.reject_markers]

    # Left q joints.
    fig_left, ax_left = plt.subplots(figsize=(10, 5))
    for joint_idx in range(7):
        ax_left.plot(
            summary.left.t_s,
            _joint_series_values(summary.left.q_deg, joint_idx),
            label=f"q{joint_idx + 1}",
        )
    _draw_markers(ax_left, left_reject_times, color="tab:red", label="left reject")
    ax_left.set_title("Left Arm Command Q Joints")
    ax_left.set_xlabel("t (s)")
    ax_left.set_ylabel("deg")
    ax_left.grid(True, alpha=0.3)
    ax_left.legend(loc="best", ncol=2)
    left_path = output_dir / "left_q_joints.png"
    fig_left.tight_layout()
    fig_left.savefig(left_path)
    plt.close(fig_left)
    generated.append(left_path)

    # Right q joints.
    fig_right, ax_right = plt.subplots(figsize=(10, 5))
    for joint_idx in range(7):
        ax_right.plot(
            summary.right.t_s,
            _joint_series_values(summary.right.q_deg, joint_idx),
            label=f"q{joint_idx + 1}",
        )
    _draw_markers(ax_right, right_reject_times, color="tab:red", label="right reject")
    ax_right.set_title("Right Arm Command Q Joints")
    ax_right.set_xlabel("t (s)")
    ax_right.set_ylabel("deg")
    ax_right.grid(True, alpha=0.3)
    ax_right.legend(loc="best", ncol=2)
    right_path = output_dir / "right_q_joints.png"
    fig_right.tight_layout()
    fig_right.savefig(right_path)
    plt.close(fig_right)
    generated.append(right_path)

    # Max q step.
    fig_step, ax_step = plt.subplots(figsize=(10, 4))
    ax_step.plot(summary.left.t_s, _float_values(summary.left.max_step_deg), label="left max q step")
    ax_step.plot(summary.right.t_s, _float_values(summary.right.max_step_deg), label="right max q step")
    _draw_markers(ax_step, all_reject_times, color="tab:red", label="reject")
    ax_step.set_title("Max Joint Step per Frame")
    ax_step.set_xlabel("t (s)")
    ax_step.set_ylabel("deg")
    ax_step.grid(True, alpha=0.3)
    ax_step.legend(loc="best")
    step_path = output_dir / "q_step_deg.png"
    fig_step.tight_layout()
    fig_step.savefig(step_path)
    plt.close(fig_step)
    generated.append(step_path)

    # Max q velocity.
    fig_vel, ax_vel = plt.subplots(figsize=(10, 4))
    ax_vel.plot(summary.left.t_s, _float_values(summary.left.max_velocity_deg_s), label="left max q velocity")
    ax_vel.plot(summary.right.t_s, _float_values(summary.right.max_velocity_deg_s), label="right max q velocity")
    _draw_markers(ax_vel, all_reject_times, color="tab:red", label="reject")
    ax_vel.set_title("Max Joint Velocity")
    ax_vel.set_xlabel("t (s)")
    ax_vel.set_ylabel("deg/s")
    ax_vel.grid(True, alpha=0.3)
    ax_vel.legend(loc="best")
    vel_path = output_dir / "q_velocity_deg_s.png"
    fig_vel.tight_layout()
    fig_vel.savefig(vel_path)
    plt.close(fig_vel)
    generated.append(vel_path)

    return generated


def vec_sub(a: Sequence[float], b: Sequence[float]) -> tuple[float, ...]:
    if len(a) != len(b):
        raise ValueError("vector length mismatch")
    return tuple(float(x) - float(y) for x, y in zip(a, b))


def vec_norm(v: Sequence[float]) -> float:
    return math.sqrt(sum(float(one) * float(one) for one in v))


def is_reject_reason(reason: str | None) -> bool:
    if reason is None:
        return False
    normalized = str(reason).strip().lower()
    return normalized not in {"", "ok", "sent", "dry_run"}


def _normalize_side(side: str) -> str:
    side_norm = str(side).strip().lower()
    if side_norm not in {"left", "right"}:
        raise ValueError("side must be 'left' or 'right'")
    return side_norm


def _normalize_mode(mode: str) -> str:
    mode_norm = str(mode).strip().lower()
    if mode_norm not in {"auto", "recorded", "recompute"}:
        raise ValueError("mode must be one of: auto, recorded, recompute")
    return mode_norm


def _selected_sides(side: str) -> set[str]:
    side_norm = str(side).strip().lower()
    if side_norm == "both":
        return {"left", "right"}
    if side_norm in {"left", "right"}:
        return {side_norm}
    raise ValueError("side must be one of: left, right, both")


def _max_abs(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    return max(abs(float(v)) for v in values)


def _same_joint_len(q: tuple[float, ...] | None, ready_q: Sequence[float]) -> bool:
    return q is not None and len(q) == len(ready_q)


def _as_finite_float_tuple(value: Any) -> tuple[float, ...] | None:
    if value is None or isinstance(value, (str, bytes)):
        return None
    if not isinstance(value, Sequence):
        return None

    out: list[float] = []
    for one in value:
        try:
            val = float(one)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(val):
            return None
        out.append(val)

    if not out:
        return None
    return tuple(out)


def _target_keys_for_side(side: str) -> tuple[tuple[str, ...], tuple[str, ...]]:
    if side == "left":
        return (
            (
                "target_left_xyz_mm",
                "command_left_target_xyz_mm",
                "target_left_xyz",
                "command_left_xyz_mm",
            ),
            (
                "target_left_abc_deg",
                "command_left_target_abc_deg",
                "target_left_abc",
                "command_left_abc_deg",
            ),
        )

    return (
        (
            "target_right_xyz_mm",
            "command_right_target_xyz_mm",
            "target_right_xyz",
            "command_right_xyz_mm",
        ),
        (
            "target_right_abc_deg",
            "command_right_target_abc_deg",
            "target_right_abc",
            "command_right_abc_deg",
        ),
    )


def _payload_has_vector(payload: dict[str, Any], keys: Sequence[str], length: int) -> bool:
    for key in keys:
        vec = _as_finite_float_tuple(payload.get(key))
        if vec is not None and len(vec) == int(length):
            return True
    return False


def _series_item(series: Sequence[Any], index: int) -> Any:
    if 0 <= int(index) < len(series):
        return series[index]
    return None


def _joint_or_none(values: tuple[float, ...] | None, joint_index: int) -> float | None:
    if values is None:
        return None
    if 0 <= joint_index < len(values):
        return float(values[joint_index])
    return None


def _choose_time(left_t: Sequence[float], right_t: Sequence[float], index: int) -> float | None:
    left_value = _series_item(left_t, index)
    if left_value is not None:
        return float(left_value)
    right_value = _series_item(right_t, index)
    if right_value is not None:
        return float(right_value)
    return None


def _choose_frame(left_f: Sequence[int | None], right_f: Sequence[int | None], index: int) -> int | None:
    left_value = _series_item(left_f, index)
    if left_value is not None:
        return left_value
    right_value = _series_item(right_f, index)
    if right_value is not None:
        return right_value
    return None


def _joint_series_values(q_values: Sequence[tuple[float, ...] | None], joint_idx: int) -> list[float]:
    out: list[float] = []
    for q in q_values:
        if q is None or joint_idx >= len(q):
            out.append(float("nan"))
        else:
            out.append(float(q[joint_idx]))
    return out


def _float_values(values: Sequence[float | None]) -> list[float]:
    out: list[float] = []
    for one in values:
        if one is None:
            out.append(float("nan"))
        else:
            out.append(float(one))
    return out


def _draw_markers(axis: Any, times: Sequence[float], color: str, label: str) -> None:
    for idx, one in enumerate(times):
        axis.axvline(float(one), color=color, linestyle="--", alpha=0.35, label=(label if idx == 0 else None))


def _note_value(notes: Sequence[str], key: str) -> str | None:
    prefix = f"{key}="
    for note in notes:
        if str(note).startswith(prefix):
            return str(note)[len(prefix) :]
    return None


def _stats_lines(side: str, stats: dict[str, Any]) -> list[str]:
    lines: list[str] = []
    lines.append(f"## {side.capitalize()} Side")
    lines.append("")
    lines.append(f"- valid q sample count: {stats.get('valid_q_samples')}")
    lines.append(f"- sent sample count: {stats.get('sent_samples')}")
    lines.append(f"- first valid q: {stats.get('first_valid_q_deg')}")
    lines.append(f"- first valid q max |dq-ready|: {stats.get('first_valid_max_abs_dq_deg')}")
    lines.append(f"- first valid q ||dq-ready||: {stats.get('first_valid_norm_dq_deg')}")
    lines.append(f"- first sent q: {stats.get('first_sent_q_deg')}")
    lines.append(f"- first sent q max |dq-ready|: {stats.get('first_sent_max_abs_dq_deg')}")
    lines.append(f"- first sent q ||dq-ready||: {stats.get('first_sent_norm_dq_deg')}")
    lines.append(f"- max q step: {stats.get('max_step_deg')} at t={stats.get('max_step_t_s')}")
    lines.append(f"- max q velocity: {stats.get('max_velocity_deg_s')} at t={stats.get('max_velocity_t_s')}")
    return lines


__all__ = [
    "SideQSeries",
    "RejectMarker",
    "IKReplaySummary",
    "build_q_series_from_recorded_commands",
    "find_reject_markers",
    "summarize_q_jumps",
    "try_replay_ik_from_targets",
    "build_ik_replay_summary",
    "write_replay_q_timeseries_csv",
    "write_reject_markers_csv",
    "write_ik_replay_report",
    "plot_ik_replay_summary",
]
