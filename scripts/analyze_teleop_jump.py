from __future__ import annotations

import argparse
from pathlib import Path
import sys

# Allow running this script directly without installing the package.
ROOT_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from teleop.analysis.log_reader import extract_teleop_steps, read_jsonl_records
from teleop.analysis.teleop_jump_analysis import (
    ReadyPoseConfig,
    analyze_side_jump,
    compute_feedback_baseline,
    plot_jump_summary,
    write_jump_report_md,
    write_jump_timeseries_csv,
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Offline teleop jump analysis from teleop_session.jsonl")
    parser.add_argument("--input", default="teleop_session.jsonl", help="Path to teleop_session.jsonl")
    parser.add_argument("--output-dir", default="teleop_jump_analysis", help="Output directory")
    parser.add_argument("--baseline-n", type=int, default=30, help="Number of frames for baseline median")
    parser.add_argument("--jump-th-deg", type=float, default=15.0, help="Jump threshold for max |q-ready| (deg)")
    parser.add_argument(
        "--move-thresholds-mm",
        default="1,5,10,20,50",
        help="Comma-separated feedback displacement thresholds in mm",
    )
    parser.add_argument("--catch-ratio", type=float, default=0.9, help="Catch-up ratio relative to max displacement")
    parser.add_argument("--first-rejects", type=int, default=3, help="How many reject markers to capture")
    parser.add_argument("--no-plots", action="store_true", help="Skip generating PNG plots")
    return parser.parse_args(argv)


def _parse_thresholds(value: str) -> tuple[float, ...]:
    parts = [part.strip() for part in str(value).split(",") if part.strip()]
    if not parts:
        raise ValueError("move thresholds cannot be empty")

    thresholds: list[float] = []
    for part in parts:
        threshold = float(part)
        if threshold <= 0.0:
            raise ValueError(f"threshold must be positive, got {threshold}")
        thresholds.append(threshold)
    return tuple(thresholds)


def _reject_reasons(summary_rejects: list[dict]) -> list[str]:
    reasons: list[str] = []
    for item in summary_rejects:
        reason = item.get("reason")
        reasons.append(str(reason) if reason is not None else "")
    return reasons


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    input_path = Path(args.input)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    thresholds = _parse_thresholds(args.move_thresholds_mm)

    records = read_jsonl_records(input_path, strict=False)
    steps = extract_teleop_steps(records)

    baseline = compute_feedback_baseline(steps=steps, baseline_n=int(args.baseline_n))

    ready = ReadyPoseConfig()
    left_summary = analyze_side_jump(
        steps=steps,
        side="left",
        ready_q_deg=ready.left_q_deg,
        baseline=baseline,
        jump_threshold_deg=float(args.jump_th_deg),
        feedback_thresholds_mm=thresholds,
        catch_ratio=float(args.catch_ratio),
        first_rejects=int(args.first_rejects),
    )
    right_summary = analyze_side_jump(
        steps=steps,
        side="right",
        ready_q_deg=ready.right_q_deg,
        baseline=baseline,
        jump_threshold_deg=float(args.jump_th_deg),
        feedback_thresholds_mm=thresholds,
        catch_ratio=float(args.catch_ratio),
        first_rejects=int(args.first_rejects),
    )

    csv_path = write_jump_timeseries_csv(
        path=output_dir / "jump_timeseries.csv",
        steps=steps,
        baseline=baseline,
        ready_config=ready,
    )

    report_path = write_jump_report_md(
        path=output_dir / "jump_report.md",
        input_path=input_path,
        steps=steps,
        baseline=baseline,
        left_summary=left_summary,
        right_summary=right_summary,
    )

    reject_markers = list(left_summary.first_rejects) + list(right_summary.first_rejects)
    plot_paths = plot_jump_summary(
        output_dir=output_dir,
        steps=steps,
        baseline=baseline,
        ready_config=ready,
        reject_markers=reject_markers,
        no_plots=bool(args.no_plots),
    )

    generated_files = [csv_path, report_path, *plot_paths]

    print(f"Input: {input_path}")
    print(f"Teleop steps: {len(steps)}")
    print(f"Output directory: {output_dir}")
    print(
        "Left first sent: "
        f"t_s={left_summary.first_sent_t_s}, "
        f"max_abs_dq={left_summary.first_sent_max_abs_dq_deg}"
    )
    print(
        "Right first sent: "
        f"t_s={right_summary.first_sent_t_s}, "
        f"max_abs_dq={right_summary.first_sent_max_abs_dq_deg}"
    )
    print(f"Left first reject reasons: {_reject_reasons(left_summary.first_rejects)}")
    print(f"Right first reject reasons: {_reject_reasons(right_summary.first_rejects)}")
    print("Generated files:")
    for one in generated_files:
        print(f"- {one}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
