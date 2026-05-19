from __future__ import annotations

from pathlib import Path
import time
import sys

# Allow running this script directly without installing the package.
ROOT_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from teleop.logging import AsyncSessionLogger, LoggingConfig


def main() -> None:
    config = LoggingConfig(
        enabled=True,
        log_dir="logs",
        session_name="dry_run",
        record_events=True,
        record_frames=True,
        record_performance=True,
        frame_sample_hz=50.0,
        performance_sample_hz=50.0,
        max_queue_size=1000,
        batch_size=50,
        flush_interval_s=0.2,
    )

    logger = AsyncSessionLogger(config=config)
    logger.start()

    for idx in range(5):
        logger.log_event("stage7_demo_event", payload={"index": idx, "message": "demo"})
        logger.log_frame(
            "stage7_demo_frame",
            payload={
                "index": idx,
                "target_xyz_mm": [1000.0 + idx, 2000.0, 3000.0],
                "orientation_abc_deg": [10.0, 20.0, 30.0],
            },
        )
        logger.log_performance("stage7_demo_performance", payload={"loop_dt_ms": 10.0 + idx})
        time.sleep(0.01)

    logger.log_error("stage7_demo_error", payload={"code": "DEMO", "detail": "synthetic error log"})

    logger.stop()
    stats = logger.get_stats()

    print("Logging dry-run completed.")
    print(f"Session directory: {logger.session_dir}")
    print(f"Session file: {logger.session_file}")
    print(f"Stats: {stats}")


if __name__ == "__main__":
    main()
