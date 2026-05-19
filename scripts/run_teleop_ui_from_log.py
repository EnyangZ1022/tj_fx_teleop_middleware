from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

# Allow running this script directly without installing the package.
ROOT_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from teleop.logging.replay import read_jsonl_records
from teleop.ui.snapshot import ArmVisualizationSnapshot, LatestSnapshotStore, TeleopVisualizationSnapshot, compute_error_norm_mm
from teleop.ui.ui_config import UIConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replay Stage 7 logs in Stage 8 diagnostic UI (analysis-only).")
    parser.add_argument("--input", required=True, help="Path to teleop_session.jsonl")
    parser.add_argument("--update-hz", type=float, default=20.0, help="Replay/UI refresh rate")
    parser.add_argument("--no-3d", action="store_true", help="Disable 3D scene and show status panel only")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    print("Stage 8 log replay is analysis-only.")
    print("No robot command is sent by this script.")

    records = list(read_jsonl_records(args.input, strict=False))
    snapshots = [s for s in (_record_to_snapshot(r) for r in records) if s is not None]

    if not snapshots:
        print("No visualization snapshots were found in the input log.")
        print("Expected payload keys such as visualization_snapshot or left/right target+feedback xyz fields.")
        return 0

    from PySide6.QtCore import QTimer
    from PySide6.QtWidgets import QApplication

    from teleop.ui.main_window import TeleopDiagnosticWindow

    snapshot_store = LatestSnapshotStore()
    snapshot_store.set(snapshots[0])

    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)

    config = UIConfig(
        enabled=True,
        update_hz=float(args.update_hz),
        window_title="TJ-FX Teleop Diagnostic UI (Log Replay)",
        show_3d_view=not bool(args.no_3d),
    )

    window = TeleopDiagnosticWindow(snapshot_store=snapshot_store, config=config)
    window.show()

    interval_ms = max(1, int(round(1000.0 / float(args.update_hz))))
    timer = QTimer()
    timer.setInterval(interval_ms)

    state = {"index": 0}

    def _advance() -> None:
        idx = state["index"]
        if idx >= len(snapshots):
            timer.stop()
            return
        snapshot_store.set(snapshots[idx])
        state["index"] = idx + 1

    timer.timeout.connect(_advance)
    timer.start()

    return int(app.exec())


def _record_to_snapshot(record: dict) -> TeleopVisualizationSnapshot | None:
    payload = record.get("payload")
    if not isinstance(payload, dict):
        return None

    ts_ns = int(record.get("timestamp_ns", 0) or 0)
    if ts_ns <= 0:
        ts_ns = 1

    vis = payload.get("visualization_snapshot")
    if isinstance(vis, dict):
        return _snapshot_from_nested_dict(vis, ts_ns)

    left_target = _vec3(payload.get("left_target_xyz_mm"))
    left_feedback = _vec3(payload.get("left_feedback_xyz_mm"))
    right_target = _vec3(payload.get("right_target_xyz_mm"))
    right_feedback = _vec3(payload.get("right_feedback_xyz_mm"))

    if all(v is None for v in (left_target, left_feedback, right_target, right_feedback)):
        return None

    left_snapshot = ArmVisualizationSnapshot(
        target_xyz_mm=left_target,
        feedback_xyz_mm=left_feedback,
        target_valid=left_target is not None,
        feedback_valid=left_feedback is not None,
        calibrated=bool(payload.get("left_calibrated", False)),
        active=bool(payload.get("left_active", False)),
        error_norm_mm=compute_error_norm_mm(left_target, left_feedback),
        status=str(payload.get("left_status", "")),
    )
    right_snapshot = ArmVisualizationSnapshot(
        target_xyz_mm=right_target,
        feedback_xyz_mm=right_feedback,
        target_valid=right_target is not None,
        feedback_valid=right_feedback is not None,
        calibrated=bool(payload.get("right_calibrated", False)),
        active=bool(payload.get("right_active", False)),
        error_norm_mm=compute_error_norm_mm(right_target, right_feedback),
        status=str(payload.get("right_status", "")),
    )

    return TeleopVisualizationSnapshot(
        timestamp_ns=ts_ns,
        left=left_snapshot,
        right=right_snapshot,
        pico_connected=bool(payload.get("pico_connected", False)),
        robot_connected=bool(payload.get("robot_connected", False)),
        safety_state=str(payload.get("safety_state", "UNKNOWN")),
        global_status=str(payload.get("global_status", "")),
        enable_left=bool(payload.get("enable_left", False)),
        enable_right=bool(payload.get("enable_right", False)),
        pico_frame_age_ms=_float_or_none(payload.get("pico_frame_age_ms")),
        command_loop_dt_ms=_float_or_none(payload.get("command_loop_dt_ms")),
        target_age_ms=_float_or_none(payload.get("target_age_ms")),
        ik_status=str(payload.get("ik_status", "")),
        sdk_status=str(payload.get("sdk_status", "")),
        logging_enabled=bool(payload.get("logging_enabled", False)),
        dropped_log_count=int(payload.get("dropped_log_count", 0) or 0),
    )


def _snapshot_from_nested_dict(source: dict, default_ts_ns: int) -> TeleopVisualizationSnapshot | None:
    left = source.get("left")
    right = source.get("right")
    if not isinstance(left, dict) or not isinstance(right, dict):
        return None

    ts_ns = int(source.get("timestamp_ns", default_ts_ns) or default_ts_ns)
    if ts_ns <= 0:
        ts_ns = default_ts_ns

    left_target = _vec3(left.get("target_xyz_mm"))
    left_feedback = _vec3(left.get("feedback_xyz_mm"))
    right_target = _vec3(right.get("target_xyz_mm"))
    right_feedback = _vec3(right.get("feedback_xyz_mm"))

    left_snapshot = ArmVisualizationSnapshot(
        target_xyz_mm=left_target,
        feedback_xyz_mm=left_feedback,
        target_abc_deg=_vec3(left.get("target_abc_deg")),
        feedback_abc_deg=_vec3(left.get("feedback_abc_deg")),
        target_valid=bool(left.get("target_valid", left_target is not None)),
        feedback_valid=bool(left.get("feedback_valid", left_feedback is not None)),
        calibrated=bool(left.get("calibrated", False)),
        active=bool(left.get("active", False)),
        error_norm_mm=_float_or_none(left.get("error_norm_mm"))
        if left.get("error_norm_mm") is not None
        else compute_error_norm_mm(left_target, left_feedback),
        status=str(left.get("status", "")),
    )
    right_snapshot = ArmVisualizationSnapshot(
        target_xyz_mm=right_target,
        feedback_xyz_mm=right_feedback,
        target_abc_deg=_vec3(right.get("target_abc_deg")),
        feedback_abc_deg=_vec3(right.get("feedback_abc_deg")),
        target_valid=bool(right.get("target_valid", right_target is not None)),
        feedback_valid=bool(right.get("feedback_valid", right_feedback is not None)),
        calibrated=bool(right.get("calibrated", False)),
        active=bool(right.get("active", False)),
        error_norm_mm=_float_or_none(right.get("error_norm_mm"))
        if right.get("error_norm_mm") is not None
        else compute_error_norm_mm(right_target, right_feedback),
        status=str(right.get("status", "")),
    )

    return TeleopVisualizationSnapshot(
        timestamp_ns=ts_ns,
        left=left_snapshot,
        right=right_snapshot,
        pico_connected=bool(source.get("pico_connected", False)),
        robot_connected=bool(source.get("robot_connected", False)),
        safety_state=str(source.get("safety_state", "UNKNOWN")),
        global_status=str(source.get("global_status", "")),
        enable_left=bool(source.get("enable_left", False)),
        enable_right=bool(source.get("enable_right", False)),
        pico_frame_age_ms=_float_or_none(source.get("pico_frame_age_ms")),
        command_loop_dt_ms=_float_or_none(source.get("command_loop_dt_ms")),
        target_age_ms=_float_or_none(source.get("target_age_ms")),
        ik_status=str(source.get("ik_status", "")),
        sdk_status=str(source.get("sdk_status", "")),
        logging_enabled=bool(source.get("logging_enabled", False)),
        dropped_log_count=int(source.get("dropped_log_count", 0) or 0),
    )


def _vec3(value: object) -> tuple[float, float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None

    try:
        x = float(value[0])
        y = float(value[1])
        z = float(value[2])
    except (TypeError, ValueError):
        return None

    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
        return None
    return (x, y, z)


def _float_or_none(value: object) -> float | None:
    if value is None:
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(parsed):
        return None
    return parsed


if __name__ == "__main__":
    raise SystemExit(main())
