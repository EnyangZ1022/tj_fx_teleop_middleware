from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import threading
import time

# Allow running this script directly without installing the package.
ROOT_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from teleop.ui.snapshot import ArmVisualizationSnapshot, LatestSnapshotStore, TeleopVisualizationSnapshot, compute_error_norm_mm
from teleop.ui.ui_config import UIConfig


class MockSnapshotProducer:
    def __init__(self, snapshot_store: LatestSnapshotStore, update_hz: float):
        self._snapshot_store = snapshot_store
        self._period_s = 1.0 / float(update_hz)
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, name="mock-ui-snapshot-producer", daemon=True)
        self._start_t = time.perf_counter()

        self._left_feedback = [-120.0, 40.0, 30.0]
        self._right_feedback = [120.0, 40.0, -30.0]

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()

    def join(self, timeout: float = 1.0) -> None:
        self._thread.join(timeout=timeout)

    def _run(self) -> None:
        next_t = time.perf_counter()

        while not self._stop_event.is_set():
            elapsed_s = time.perf_counter() - self._start_t
            snapshot = self._build_snapshot(elapsed_s)
            self._snapshot_store.set(snapshot)

            next_t += self._period_s
            sleep_s = next_t - time.perf_counter()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_t = time.perf_counter()

    def _build_snapshot(self, t_s: float) -> TeleopVisualizationSnapshot:
        left_target = (
            -120.0 + 45.0 * math.cos(0.7 * t_s),
            45.0 + 25.0 * math.sin(1.1 * t_s),
            25.0 + 20.0 * math.sin(0.45 * t_s),
        )
        right_target = (
            120.0 + 40.0 * math.sin(0.8 * t_s + 0.8),
            42.0 + 22.0 * math.cos(1.0 * t_s + 0.5),
            -28.0 + 20.0 * math.sin(0.55 * t_s + 1.1),
        )

        # A small low-pass lag gives visible target-feedback error lines.
        lag_gain = 0.16
        for i in range(3):
            self._left_feedback[i] += lag_gain * (left_target[i] - self._left_feedback[i])
            self._right_feedback[i] += lag_gain * (right_target[i] - self._right_feedback[i])

        cycle_s = t_s % 16.0
        if cycle_s < 3.0:
            safety_state = "WAIT_CALIBRATION"
            global_status = "Calibrate both arms"
            left_calibrated = False
            right_calibrated = False
            enable_left = False
            enable_right = False
            left_active = False
            right_active = False
        elif cycle_s < 6.0:
            safety_state = "TELEOP_READY"
            global_status = "Enable held low"
            left_calibrated = True
            right_calibrated = True
            enable_left = False
            enable_right = False
            left_active = False
            right_active = False
        elif cycle_s < 13.0:
            safety_state = "TELEOP_ACTIVE"
            global_status = "Teleoperation active"
            left_calibrated = True
            right_calibrated = True
            enable_left = True
            enable_right = True
            left_active = True
            right_active = True
        else:
            safety_state = "PAUSED"
            global_status = "Enable released"
            left_calibrated = True
            right_calibrated = True
            enable_left = False
            enable_right = False
            left_active = False
            right_active = False

        left_error = compute_error_norm_mm(left_target, tuple(self._left_feedback))
        right_error = compute_error_norm_mm(right_target, tuple(self._right_feedback))

        left_snapshot = ArmVisualizationSnapshot(
            target_xyz_mm=left_target,
            feedback_xyz_mm=(self._left_feedback[0], self._left_feedback[1], self._left_feedback[2]),
            target_abc_deg=(0.0, 0.0, 0.0),
            feedback_abc_deg=(0.0, 0.0, 0.0),
            target_valid=True,
            feedback_valid=True,
            calibrated=left_calibrated,
            active=left_active,
            error_norm_mm=left_error,
            status="ok" if left_active else "idle",
        )
        right_snapshot = ArmVisualizationSnapshot(
            target_xyz_mm=right_target,
            feedback_xyz_mm=(self._right_feedback[0], self._right_feedback[1], self._right_feedback[2]),
            target_abc_deg=(0.0, 0.0, 0.0),
            feedback_abc_deg=(0.0, 0.0, 0.0),
            target_valid=True,
            feedback_valid=True,
            calibrated=right_calibrated,
            active=right_active,
            error_norm_mm=right_error,
            status="ok" if right_active else "idle",
        )

        pico_frame_age_ms = 8.0 + 4.0 * (0.5 + 0.5 * math.sin(2.3 * t_s))
        command_loop_dt_ms = 10.0 + 0.6 * math.sin(6.0 * t_s)
        target_age_ms = 18.0 + 5.0 * (0.5 + 0.5 * math.cos(3.4 * t_s))

        dropped_count = int(max(0.0, (t_s - 10.0) // 2.0))

        return TeleopVisualizationSnapshot(
            timestamp_ns=time.time_ns(),
            left=left_snapshot,
            right=right_snapshot,
            pico_connected=True,
            robot_connected=True,
            safety_state=safety_state,
            global_status=global_status,
            enable_left=enable_left,
            enable_right=enable_right,
            pico_frame_age_ms=pico_frame_age_ms,
            command_loop_dt_ms=command_loop_dt_ms,
            target_age_ms=target_age_ms,
            ik_status="ok",
            sdk_status="connected",
            logging_enabled=False,
            dropped_log_count=dropped_count,
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run Stage 8 mock diagnostic UI (no hardware required).")
    parser.add_argument("--update-hz", type=float, default=20.0, help="UI and mock snapshot update rate")
    parser.add_argument("--duration-s", type=float, default=None, help="Optional auto-exit duration")
    parser.add_argument("--no-3d", action="store_true", help="Disable 3D scene and show status panel only")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if float(args.update_hz) <= 0.0:
        raise ValueError("--update-hz must be positive")

    from PySide6.QtCore import QTimer
    from PySide6.QtWidgets import QApplication
    from teleop.ui.main_window import TeleopDiagnosticWindow

    snapshot_store = LatestSnapshotStore()
    config = UIConfig(
        enabled=True,
        update_hz=float(args.update_hz),
        show_3d_view=not bool(args.no_3d),
    )

    producer = MockSnapshotProducer(snapshot_store=snapshot_store, update_hz=float(args.update_hz))
    producer.start()

    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)

    window = TeleopDiagnosticWindow(snapshot_store=snapshot_store, config=config)
    window.show()

    if args.duration_s is not None and float(args.duration_s) > 0.0:
        QTimer.singleShot(int(round(float(args.duration_s) * 1000.0)), app.quit)

    try:
        return int(app.exec())
    finally:
        producer.stop()
        producer.join(timeout=1.0)


if __name__ == "__main__":
    raise SystemExit(main())
