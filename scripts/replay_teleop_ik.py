from __future__ import annotations

import argparse
from pathlib import Path
import sys

# Allow running this script directly without installing the package.
ROOT_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from teleop.analysis.ik_replay_analysis import (
    build_ik_replay_summary,
    plot_ik_replay_summary,
    summarize_q_jumps,
    write_ik_replay_report,
    write_reject_markers_csv,
    write_replay_q_timeseries_csv,
)
from teleop.analysis.log_reader import extract_teleop_steps, read_jsonl_records


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Offline IK replay / q-series analysis for teleop JSONL logs")
    parser.add_argument("--input", default="teleop_session.jsonl", help="Path to teleop_session.jsonl")
    parser.add_argument("--output-dir", default="teleop_ik_replay_analysis", help="Output directory")
    parser.add_argument("--side", choices=["left", "right", "both"], default="both", help="Analyze side")
    parser.add_argument("--mode", choices=["auto", "recorded", "recompute"], default="auto", help="Replay mode")
    parser.add_argument("--first-rejects", type=int, default=3, help="Max reject markers per side")
    parser.add_argument("--ready-left", default="90,-60,-90,-90,0,0,0", help="Left ready q CSV")
    parser.add_argument("--ready-right", default="90,60,-90,-90,0,0,0", help="Right ready q CSV")
    parser.add_argument("--require-sdk", action="store_true", help="Require SDK for recompute path")
    parser.add_argument("--no-plots", action="store_true", help="Skip plot generation")
    return parser.parse_args(argv)


def _parse_ready_q(name: str, value: str) -> tuple[float, ...]:
    items = [part.strip() for part in str(value).split(",") if part.strip()]
    if len(items) != 7:
        raise ValueError(f"{name} must provide exactly 7 values")

    result: list[float] = []
    for part in items:
        result.append(float(part))
    return tuple(result)


def _reject_reasons_text(summary_rejects: list) -> list[str]:
    reasons: list[str] = []
    for marker in summary_rejects:
        reasons.append(str(marker.reason))
    return reasons


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    input_path = Path(args.input)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    try:
        ready_left_q = _parse_ready_q("--ready-left", args.ready_left)
        ready_right_q = _parse_ready_q("--ready-right", args.ready_right)

        records = read_jsonl_records(path=input_path, strict=False)
        steps = extract_teleop_steps(records)

        summary = build_ik_replay_summary(
            steps=steps,
            mode=str(args.mode),
            side=str(args.side),
            limit_per_side=int(args.first_rejects),
            require_sdk=bool(args.require_sdk),
        )
        summary.notes.insert(0, f"selected_mode={str(args.mode)}")

        left_stats = summarize_q_jumps(summary.left, ready_left_q)
        right_stats = summarize_q_jumps(summary.right, ready_right_q)

        q_csv_path = output_dir / "replay_q_timeseries.csv"
        write_replay_q_timeseries_csv(path=q_csv_path, summary=summary)

        rejects_csv_path = output_dir / "reject_markers.csv"
        write_reject_markers_csv(path=rejects_csv_path, reject_markers=summary.reject_markers)

        report_path = output_dir / "ik_replay_report.md"
        write_ik_replay_report(
            path=report_path,
            input_path=input_path,
            summary=summary,
            ready_left_q=ready_left_q,
            ready_right_q=ready_right_q,
        )

        plot_paths = plot_ik_replay_summary(
            output_dir=output_dir,
            summary=summary,
            no_plots=bool(args.no_plots),
        )

        left_rejects = [marker for marker in summary.reject_markers if marker.side == "left"]
        right_rejects = [marker for marker in summary.reject_markers if marker.side == "right"]

        print(f"Input file: {input_path}")
        print(f"Selected mode: {args.mode}")
        print(f"Actual mode used: {summary.mode}")
        print(f"Output directory: {output_dir}")
        print(f"Left valid q samples: {left_stats.get('valid_q_samples')}")
        print(f"Right valid q samples: {right_stats.get('valid_q_samples')}")
        print(f"Left max step deg: {left_stats.get('max_step_deg')}")
        print(f"Right max step deg: {right_stats.get('max_step_deg')}")
        print(f"Left max velocity deg/s: {left_stats.get('max_velocity_deg_s')}")
        print(f"Right max velocity deg/s: {right_stats.get('max_velocity_deg_s')}")
        print(f"Left first reject reasons: {_reject_reasons_text(left_rejects)}")
        print(f"Right first reject reasons: {_reject_reasons_text(right_rejects)}")
        print("Generated files:")
        print(f"- {q_csv_path}")
        print(f"- {rejects_csv_path}")
        print(f"- {report_path}")
        for one in plot_paths:
            print(f"- {one}")

        return 0
    except Exception as exc:
        print(f"IK replay analysis failed: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
