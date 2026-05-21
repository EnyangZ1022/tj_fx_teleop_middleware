from __future__ import annotations

from dataclasses import dataclass
import csv
import math
from pathlib import Path
from statistics import median
from typing import Any, Sequence

from teleop.analysis.log_reader import TeleopStepLog


@dataclass
class ReadyPoseConfig:
    left_q_deg: tuple[float, ...] = (90, -60, -90, -90, 0, 0, 0)
    right_q_deg: tuple[float, ...] = (90, 60, -90, -90, 0, 0, 0)


@dataclass
class SideJumpSummary:
    side: str
    baseline_xyz_mm: tuple[float, float, float] | None
    baseline_abc_deg: tuple[float, float, float] | None

    first_command_t_s: float | None
    first_command_frame_id: int | None
    first_sent_t_s: float | None
    first_sent_frame_id: int | None
    first_sent_q_deg: tuple[float, ...] | None
    first_sent_max_abs_dq_deg: float | None
    first_sent_norm_dq_deg: float | None
    first_sent_reason: str | None

    first_jump_t_s: float | None
    first_jump_frame_id: int | None
    first_jump_max_abs_dq_deg: float | None

    first_rejects: list[dict[str, Any]]
    feedback_threshold_crossings: dict[float, dict[str, Any] | None]
    catchup_info: dict[str, Any] | None


def vec_sub(a: Sequence[float], b: Sequence[float]) -> tuple[float, ...]:
    if len(a) != len(b):
        raise ValueError("vector lengths must match")
    return tuple(float(va) - float(vb) for va, vb in zip(a, b))


def vec_norm(v: Sequence[float]) -> float:
    return math.sqrt(sum(float(one) * float(one) for one in v))


def max_abs(v: Sequence[float]) -> float:
    if not v:
        return 0.0
    return max(abs(float(one)) for one in v)


def median_vector(vectors: Sequence[Sequence[float]]) -> tuple[float, ...] | None:
    if not vectors:
        return None

    dim = len(vectors[0])
    if dim == 0:
        return tuple()

    for vec in vectors:
        if len(vec) != dim:
            return None

    out: list[float] = []
    for idx in range(dim):
        out.append(float(median(float(vec[idx]) for vec in vectors)))
    return tuple(out)


def is_reject_reason(reason: str | None) -> bool:
    if reason is None:
        return False
    normalized = str(reason).strip().lower()
    return normalized not in {"", "ok", "sent", "dry_run"}


def compute_feedback_baseline(
    steps: Sequence[TeleopStepLog],
    baseline_n: int = 30,
) -> dict[str, tuple[float, ...] | None]:
    n = max(1, int(baseline_n))

    preferred: dict[str, list[tuple[float, ...]]] = {
        "left_xyz": [],
        "left_abc": [],
        "right_xyz": [],
        "right_abc": [],
    }
    fallback: dict[str, list[tuple[float, ...]]] = {
        "left_xyz": [],
        "left_abc": [],
        "right_xyz": [],
        "right_abc": [],
    }

    for step in steps:
        is_preferred = (step.command_ready is False) and (step.allow_motion is False)
        target = preferred if is_preferred else fallback

        _append_if_present(target["left_xyz"], step.feedback_left_xyz_mm)
        _append_if_present(target["left_abc"], step.feedback_left_abc_deg)
        _append_if_present(target["right_xyz"], step.feedback_right_xyz_mm)
        _append_if_present(target["right_abc"], step.feedback_right_abc_deg)

    baseline: dict[str, tuple[float, ...] | None] = {}
    for key in ("left_xyz", "left_abc", "right_xyz", "right_abc"):
        vectors = list(preferred[key][:n])
        if len(vectors) < n:
            vectors.extend(fallback[key][: n - len(vectors)])
        baseline[key] = median_vector(vectors)

    return baseline


def find_rejects(
    steps: Sequence[TeleopStepLog],
    side: str,
    limit: int = 3,
) -> list[dict[str, Any]]:
    side_norm = _normalize_side(side)
    n = max(0, int(limit))

    reason_attr = _reason_attr(side_norm)
    sent_attr = _sent_attr(side_norm)

    rejects: list[dict[str, Any]] = []
    for step in steps:
        reason = getattr(step, reason_attr)
        if not is_reject_reason(reason):
            continue

        rejects.append(
            {
                "side": side_norm,
                "t_s": step.t_s,
                "frame_id": step.frame_id,
                "reason": reason,
                "sequence_id": step.sequence_id,
                "sent": getattr(step, sent_attr),
            }
        )
        if len(rejects) >= n:
            break

    return rejects


def analyze_side_jump(
    steps: Sequence[TeleopStepLog],
    side: str,
    ready_q_deg: Sequence[float],
    baseline: dict[str, tuple[float, ...] | None],
    jump_threshold_deg: float = 15.0,
    feedback_thresholds_mm: Sequence[float] = (1, 5, 10, 20, 50),
    catch_ratio: float = 0.9,
    first_rejects: int = 3,
) -> SideJumpSummary:
    side_norm = _normalize_side(side)
    ready_q = tuple(float(v) for v in ready_q_deg)

    q_attr = _command_q_attr(side_norm)
    sent_attr = _sent_attr(side_norm)
    reason_attr = _reason_attr(side_norm)
    feedback_xyz_attr = _feedback_xyz_attr(side_norm)

    baseline_xyz = _as_xyz_tuple(baseline.get(f"{side_norm}_xyz"))
    baseline_abc = _as_xyz_tuple(baseline.get(f"{side_norm}_abc"))

    first_command_t_s: float | None = None
    first_command_frame_id: int | None = None

    first_sent_t_s: float | None = None
    first_sent_frame_id: int | None = None
    first_sent_q_deg: tuple[float, ...] | None = None
    first_sent_max_abs_dq_deg: float | None = None
    first_sent_norm_dq_deg: float | None = None
    first_sent_reason: str | None = None

    first_jump_t_s: float | None = None
    first_jump_frame_id: int | None = None
    first_jump_max_abs_dq_deg: float | None = None

    feedback_threshold_crossings: dict[float, dict[str, Any] | None] = {
        float(th): None for th in feedback_thresholds_mm
    }

    post_sent_feedback: list[tuple[TeleopStepLog, float]] = []

    for step in steps:
        q = getattr(step, q_attr)
        sent = getattr(step, sent_attr)
        reason = getattr(step, reason_attr)

        if first_command_t_s is None and _is_valid_q(q, ready_q):
            first_command_t_s = step.t_s
            first_command_frame_id = step.frame_id

        if _is_valid_q(q, ready_q):
            dq = vec_sub(q, ready_q)
            dq_max_abs = max_abs(dq)

            if first_jump_t_s is None and dq_max_abs >= float(jump_threshold_deg):
                first_jump_t_s = step.t_s
                first_jump_frame_id = step.frame_id
                first_jump_max_abs_dq_deg = dq_max_abs

            if first_sent_t_s is None and sent is True:
                first_sent_t_s = step.t_s
                first_sent_frame_id = step.frame_id
                first_sent_q_deg = tuple(float(v) for v in q)
                first_sent_max_abs_dq_deg = dq_max_abs
                first_sent_norm_dq_deg = vec_norm(dq)
                first_sent_reason = reason

        feedback_xyz = getattr(step, feedback_xyz_attr)
        disp_mm = _feedback_disp_mm(feedback_xyz, baseline_xyz)

        if disp_mm is not None:
            for threshold in list(feedback_threshold_crossings.keys()):
                if feedback_threshold_crossings[threshold] is None and disp_mm >= threshold:
                    feedback_threshold_crossings[threshold] = {
                        "side": side_norm,
                        "t_s": step.t_s,
                        "frame_id": step.frame_id,
                        "disp_mm": disp_mm,
                        "sequence_id": step.sequence_id,
                    }

            if first_sent_t_s is not None and step.t_s >= first_sent_t_s:
                post_sent_feedback.append((step, disp_mm))

    catchup_info = _compute_catchup_info(
        post_sent_feedback=post_sent_feedback,
        first_sent_t_s=first_sent_t_s,
        catch_ratio=float(catch_ratio),
    )

    summary = SideJumpSummary(
        side=side_norm,
        baseline_xyz_mm=baseline_xyz,
        baseline_abc_deg=baseline_abc,
        first_command_t_s=first_command_t_s,
        first_command_frame_id=first_command_frame_id,
        first_sent_t_s=first_sent_t_s,
        first_sent_frame_id=first_sent_frame_id,
        first_sent_q_deg=first_sent_q_deg,
        first_sent_max_abs_dq_deg=first_sent_max_abs_dq_deg,
        first_sent_norm_dq_deg=first_sent_norm_dq_deg,
        first_sent_reason=first_sent_reason,
        first_jump_t_s=first_jump_t_s,
        first_jump_frame_id=first_jump_frame_id,
        first_jump_max_abs_dq_deg=first_jump_max_abs_dq_deg,
        first_rejects=find_rejects(steps=steps, side=side_norm, limit=first_rejects),
        feedback_threshold_crossings=feedback_threshold_crossings,
        catchup_info=catchup_info,
    )
    return summary


def write_jump_timeseries_csv(
    path: str | Path,
    steps: Sequence[TeleopStepLog],
    baseline: dict[str, tuple[float, ...] | None],
    ready_config: ReadyPoseConfig,
) -> Path:
    out_path = Path(path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    fieldnames = [
        "t_s",
        "frame_id",
        "safety_state",
        "allow_motion",
        "command_ready",
        "left_feedback_disp_mm",
        "right_feedback_disp_mm",
        "left_command_max_abs_dq_from_ready_deg",
        "right_command_max_abs_dq_from_ready_deg",
        "left_sent",
        "right_sent",
        "left_reason",
        "right_reason",
    ]

    with out_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()

        for step in steps:
            writer.writerow(
                {
                    "t_s": step.t_s,
                    "frame_id": step.frame_id,
                    "safety_state": step.safety_state,
                    "allow_motion": step.allow_motion,
                    "command_ready": step.command_ready,
                    "left_feedback_disp_mm": _feedback_disp_mm(step.feedback_left_xyz_mm, _as_xyz_tuple(baseline.get("left_xyz"))),
                    "right_feedback_disp_mm": _feedback_disp_mm(step.feedback_right_xyz_mm, _as_xyz_tuple(baseline.get("right_xyz"))),
                    "left_command_max_abs_dq_from_ready_deg": _command_max_abs_dq(step.command_left_q_deg, ready_config.left_q_deg),
                    "right_command_max_abs_dq_from_ready_deg": _command_max_abs_dq(step.command_right_q_deg, ready_config.right_q_deg),
                    "left_sent": step.command_left_sent,
                    "right_sent": step.command_right_sent,
                    "left_reason": step.command_left_reason,
                    "right_reason": step.command_right_reason,
                }
            )

    return out_path


def write_jump_report_md(
    path: str | Path,
    input_path: str | Path,
    steps: Sequence[TeleopStepLog],
    baseline: dict[str, tuple[float, ...] | None],
    left_summary: SideJumpSummary,
    right_summary: SideJumpSummary,
) -> Path:
    out_path = Path(path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    lines: list[str] = []
    lines.append("# Teleop Jump Analysis Report")
    lines.append("")
    lines.append(f"- Input file: {Path(input_path)}")
    lines.append(f"- Teleop step frames: {len(steps)}")
    lines.append("")

    lines.append("## Feedback Baseline")
    lines.append("")
    lines.append(f"- left_xyz_mm: {baseline.get('left_xyz')}")
    lines.append(f"- left_abc_deg: {baseline.get('left_abc')}")
    lines.append(f"- right_xyz_mm: {baseline.get('right_xyz')}")
    lines.append(f"- right_abc_deg: {baseline.get('right_abc')}")
    lines.append("")

    lines.extend(_summary_lines_for_side(left_summary))
    lines.append("")
    lines.extend(_summary_lines_for_side(right_summary))
    lines.append("")

    lines.append("## Limitation")
    lines.append("")
    lines.append(
        "This tool analyzes recorded command q and feedback xyzabc only; "
        "it does not recompute IK and does not connect to robot or PICO."
    )
    lines.append("")

    out_path.write_text("\n".join(lines), encoding="utf-8")
    return out_path


def plot_jump_summary(
    output_dir: str | Path,
    steps: Sequence[TeleopStepLog],
    baseline: dict[str, tuple[float, ...] | None],
    ready_config: ReadyPoseConfig,
    reject_markers: Sequence[dict[str, Any]],
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

    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    t_values = [float(step.t_s) for step in steps]
    left_q_jump = [_command_max_abs_dq(step.command_left_q_deg, ready_config.left_q_deg) for step in steps]
    right_q_jump = [_command_max_abs_dq(step.command_right_q_deg, ready_config.right_q_deg) for step in steps]

    left_feedback_disp = [
        _feedback_disp_mm(step.feedback_left_xyz_mm, _as_xyz_tuple(baseline.get("left_xyz"))) for step in steps
    ]
    right_feedback_disp = [
        _feedback_disp_mm(step.feedback_right_xyz_mm, _as_xyz_tuple(baseline.get("right_xyz"))) for step in steps
    ]

    reject_times = [float(marker.get("t_s")) for marker in reject_markers if marker.get("t_s") is not None]

    first_left_sent_t = _first_sent_time(steps, side="left")
    first_right_sent_t = _first_sent_time(steps, side="right")

    generated: list[Path] = []

    fig1, ax1 = plt.subplots(figsize=(10, 4))
    ax1.plot(t_values, _to_plot_values(left_q_jump), label="left max |q-ready| (deg)")
    ax1.plot(t_values, _to_plot_values(right_q_jump), label="right max |q-ready| (deg)")
    _draw_reject_markers(ax1, reject_times)
    _draw_sent_markers(ax1, first_left_sent_t, first_right_sent_t)
    ax1.set_xlabel("t (s)")
    ax1.set_ylabel("deg")
    ax1.set_title("Command Q Jump from Ready Pose")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="best")
    path1 = out_dir / "command_q_jump_deg.png"
    fig1.tight_layout()
    fig1.savefig(path1)
    plt.close(fig1)
    generated.append(path1)

    fig2, ax2 = plt.subplots(figsize=(10, 4))
    ax2.plot(t_values, _to_plot_values(left_feedback_disp), label="left feedback displacement (mm)")
    ax2.plot(t_values, _to_plot_values(right_feedback_disp), label="right feedback displacement (mm)")
    _draw_reject_markers(ax2, reject_times)
    _draw_sent_markers(ax2, first_left_sent_t, first_right_sent_t)
    ax2.set_xlabel("t (s)")
    ax2.set_ylabel("mm")
    ax2.set_title("Feedback Displacement from Baseline")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="best")
    path2 = out_dir / "feedback_displacement_mm.png"
    fig2.tight_layout()
    fig2.savefig(path2)
    plt.close(fig2)
    generated.append(path2)

    return generated


def _summary_lines_for_side(summary: SideJumpSummary) -> list[str]:
    lines: list[str] = []
    lines.append(f"## {summary.side.capitalize()} Side")
    lines.append("")
    lines.append(f"- baseline_xyz_mm: {summary.baseline_xyz_mm}")
    lines.append(f"- baseline_abc_deg: {summary.baseline_abc_deg}")
    lines.append(f"- first_command_t_s: {summary.first_command_t_s}")
    lines.append(f"- first_command_frame_id: {summary.first_command_frame_id}")
    lines.append(f"- first_sent_t_s: {summary.first_sent_t_s}")
    lines.append(f"- first_sent_frame_id: {summary.first_sent_frame_id}")
    lines.append(f"- first_sent_q_deg: {summary.first_sent_q_deg}")
    lines.append(f"- first_sent_max_abs_dq_deg: {summary.first_sent_max_abs_dq_deg}")
    lines.append(f"- first_sent_norm_dq_deg: {summary.first_sent_norm_dq_deg}")
    lines.append(f"- first_sent_reason: {summary.first_sent_reason}")
    lines.append(f"- first_jump_t_s: {summary.first_jump_t_s}")
    lines.append(f"- first_jump_frame_id: {summary.first_jump_frame_id}")
    lines.append(f"- first_jump_max_abs_dq_deg: {summary.first_jump_max_abs_dq_deg}")
    lines.append("")

    lines.append("### First Rejects")
    lines.append("")
    if summary.first_rejects:
        for item in summary.first_rejects:
            lines.append(f"- {item}")
    else:
        lines.append("- none")
    lines.append("")

    lines.append("### Feedback Threshold Crossings")
    lines.append("")
    for threshold, data in summary.feedback_threshold_crossings.items():
        lines.append(f"- {threshold} mm: {data}")
    lines.append("")

    lines.append("### Catch-up Estimate")
    lines.append("")
    lines.append(f"- {summary.catchup_info}")
    return lines


def _append_if_present(store: list[tuple[float, ...]], value: tuple[float, ...] | None) -> None:
    if value is None:
        return
    store.append(tuple(float(v) for v in value))


def _normalize_side(side: str) -> str:
    side_norm = str(side).strip().lower()
    if side_norm not in {"left", "right"}:
        raise ValueError("side must be 'left' or 'right'")
    return side_norm


def _command_q_attr(side: str) -> str:
    return "command_left_q_deg" if side == "left" else "command_right_q_deg"


def _sent_attr(side: str) -> str:
    return "command_left_sent" if side == "left" else "command_right_sent"


def _reason_attr(side: str) -> str:
    return "command_left_reason" if side == "left" else "command_right_reason"


def _feedback_xyz_attr(side: str) -> str:
    return "feedback_left_xyz_mm" if side == "left" else "feedback_right_xyz_mm"


def _as_xyz_tuple(value: tuple[float, ...] | None) -> tuple[float, float, float] | None:
    if value is None or len(value) != 3:
        return None
    return (float(value[0]), float(value[1]), float(value[2]))


def _is_valid_q(q: tuple[float, ...] | None, ready_q: Sequence[float]) -> bool:
    return q is not None and len(q) == len(ready_q)


def _feedback_disp_mm(
    feedback_xyz: tuple[float, float, float] | None,
    baseline_xyz: tuple[float, float, float] | None,
) -> float | None:
    if feedback_xyz is None or baseline_xyz is None:
        return None
    return vec_norm(vec_sub(feedback_xyz, baseline_xyz))


def _command_max_abs_dq(command_q: tuple[float, ...] | None, ready_q: Sequence[float]) -> float | None:
    if command_q is None or len(command_q) != len(ready_q):
        return None
    return max_abs(vec_sub(command_q, ready_q))


def _compute_catchup_info(
    post_sent_feedback: Sequence[tuple[TeleopStepLog, float]],
    first_sent_t_s: float | None,
    catch_ratio: float,
) -> dict[str, Any] | None:
    if first_sent_t_s is None or not post_sent_feedback:
        return None

    ratio = float(catch_ratio)
    ratio = min(max(ratio, 0.0), 1.0)

    max_disp_mm = max(disp for _, disp in post_sent_feedback)
    target_disp_mm = ratio * max_disp_mm

    reached_step: TeleopStepLog | None = None
    reached_disp_mm: float | None = None

    for step, disp in post_sent_feedback:
        if disp >= target_disp_mm:
            reached_step = step
            reached_disp_mm = disp
            break

    return {
        "max_disp_mm": max_disp_mm,
        "catch_ratio": ratio,
        "target_disp_mm": target_disp_mm,
        "reached": reached_step is not None,
        "reach_t_s": reached_step.t_s if reached_step is not None else None,
        "reach_frame_id": reached_step.frame_id if reached_step is not None else None,
        "reach_disp_mm": reached_disp_mm,
        "delay_s": (reached_step.t_s - first_sent_t_s) if reached_step is not None else None,
    }


def _first_sent_time(steps: Sequence[TeleopStepLog], side: str) -> float | None:
    sent_attr = _sent_attr(_normalize_side(side))
    for step in steps:
        if getattr(step, sent_attr) is True:
            return float(step.t_s)
    return None


def _to_plot_values(values: Sequence[float | None]) -> list[float]:
    out: list[float] = []
    for value in values:
        if value is None:
            out.append(float("nan"))
        else:
            out.append(float(value))
    return out


def _draw_reject_markers(axis: Any, reject_times: Sequence[float]) -> None:
    for idx, t_value in enumerate(reject_times):
        label = "reject" if idx == 0 else None
        axis.axvline(float(t_value), color="tab:red", linestyle="--", alpha=0.35, label=label)


def _draw_sent_markers(axis: Any, left_sent_t: float | None, right_sent_t: float | None) -> None:
    if left_sent_t is not None:
        axis.axvline(float(left_sent_t), color="tab:green", linestyle=":", alpha=0.4, label="left first sent")
    if right_sent_t is not None:
        axis.axvline(float(right_sent_t), color="tab:purple", linestyle=":", alpha=0.4, label="right first sent")


__all__ = [
    "ReadyPoseConfig",
    "SideJumpSummary",
    "vec_sub",
    "vec_norm",
    "max_abs",
    "median_vector",
    "is_reject_reason",
    "compute_feedback_baseline",
    "find_rejects",
    "analyze_side_jump",
    "write_jump_timeseries_csv",
    "write_jump_report_md",
    "plot_jump_summary",
]
