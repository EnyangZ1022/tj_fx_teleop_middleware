from __future__ import annotations

import argparse
from pathlib import Path
import sys

# Allow running this script directly without installing the package.
ROOT_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from teleop.logging.diagnostics import compute_dt_stats, summarize_session
from teleop.logging.replay import read_jsonl_records


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze teleop JSONL session log.")
    parser.add_argument("--input", required=True, help="Path to teleop_session.jsonl")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    input_path = Path(args.input)

    records = list(read_jsonl_records(input_path, strict=False))
    summary = summarize_session(records)

    timestamps = [int(r["timestamp_ns"]) for r in records if isinstance(r.get("timestamp_ns"), int)]
    dt_stats = compute_dt_stats(timestamps)

    print(f"Input: {input_path}")
    print(f"Total records: {summary['total_records']}")
    print(f"Record type counts: {summary['record_type_counts']}")
    print(f"Event counts: {summary['event_counts']}")
    print(f"Duration (s): {summary['duration_s']:.6f}")
    print(f"Dropped log count (from payload summaries): {summary['dropped_log_count']}")
    print(f"Timestamp dt stats (ms): {dt_stats}")


if __name__ == "__main__":
    main()
