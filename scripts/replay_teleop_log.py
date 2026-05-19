from __future__ import annotations

import argparse
from pathlib import Path
import sys

# Allow running this script directly without installing the package.
ROOT_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from teleop.logging.replay import filter_records, read_jsonl_records


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replay teleop log records for offline analysis only.")
    parser.add_argument("--input", required=True, help="Path to teleop_session.jsonl")
    parser.add_argument("--record-type", default="frame", help="Record type filter (default: frame)")
    parser.add_argument("--event", default=None, help="Optional event filter")
    parser.add_argument("--limit", type=int, default=20, help="Maximum records to print")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    input_path = Path(args.input)

    print("Stage 7 replay is analysis-only.")
    print("No SDK commands are sent by this script.")

    records = read_jsonl_records(input_path, strict=False)
    filtered = filter_records(records, record_type=args.record_type, event=args.event)

    printed = 0
    for record in filtered:
        print(record)
        printed += 1
        if printed >= max(1, int(args.limit)):
            break

    print(f"Printed records: {printed}")


if __name__ == "__main__":
    main()
