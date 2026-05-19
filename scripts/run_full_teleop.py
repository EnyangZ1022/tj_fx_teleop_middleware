from __future__ import annotations

import argparse
from pathlib import Path
import sys
import threading

# Allow running this script directly without installing the package.
ROOT_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from teleop.app import FullTeleopApp, FullTeleopAppConfig
from teleop.logging import LoggingConfig
from teleop.ui.snapshot import LatestSnapshotStore
from teleop.ui.ui_config import UIConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run full teleoperation orchestration pipeline with safe defaults. "
            "Dry-run is default; real command sending requires explicit opt-in and confirmation."
        )
    )
    parser.add_argument("--robot-ip", default="192.168.1.190", help="Robot controller IP")
    parser.add_argument("--no-pico", action="store_true", help="Disable Pico input connection")
    parser.add_argument("--no-robot", action="store_true", help="Disable robot SDK connection")
    parser.add_argument("--move-to-ready", action="store_true", help="Run ready-pose startup sequence")
    parser.add_argument("--dry-run", action="store_true", help="Keep dry-run mode enabled (default behavior)")
    parser.add_argument("--enable-send", action="store_true", help="Enable real robot command send")
    parser.add_argument("--ui", action="store_true", help="Enable diagnostic UI")
    parser.add_argument("--logging", action="store_true", help="Enable async logging")
    parser.add_argument("--rate-hz", type=float, default=100.0, help="Command scheduler rate in Hz")
    parser.add_argument("--side", choices=["left", "right", "both"], default="both", help="Single-arm mode")
    parser.add_argument("--max-runtime-s", type=float, default=None, help="Optional runtime cap in seconds")
    parser.add_argument(
        "--confirm",
        action="store_true",
        help="Required alongside --enable-send to unlock interactive YES confirmation",
    )
    return parser.parse_args()


def _confirm_real_send(args: argparse.Namespace) -> bool:
    print("Real send requested. Review before proceeding:")
    print(f"  robot_ip: {args.robot_ip}")
    print(f"  rate_hz: {float(args.rate_hz):.1f}")
    print(f"  move_to_ready: {bool(args.move_to_ready)}")
    print("Safety reminders:")
    print("  - keep emergency stop reachable")
    print("  - ensure workspace is clear")
    print("  - verify calibration and deadman behavior")
    text = input("Type YES to continue with real send: ").strip()
    return text == "YES"


def _build_app_config(args: argparse.Namespace) -> FullTeleopAppConfig:
    dry_run = True
    if bool(args.enable_send):
        dry_run = False
    elif bool(args.dry_run):
        dry_run = True

    side_mode = None if args.side == "both" else args.side

    return FullTeleopAppConfig(
        robot_ip=str(args.robot_ip),
        connect_pico=not bool(args.no_pico),
        connect_robot=not bool(args.no_robot),
        move_to_ready=bool(args.move_to_ready),
        enable_send=bool(args.enable_send),
        dry_run=dry_run,
        require_confirmation=True,
        command_rate_hz=float(args.rate_hz),
        ui_enabled=bool(args.ui),
        logging_enabled=bool(args.logging),
        single_arm_mode=side_mode,
        max_runtime_s=float(args.max_runtime_s) if args.max_runtime_s is not None else None,
    )


def main() -> int:
    args = parse_args()

    if bool(args.enable_send) and bool(args.no_robot):
        print("--enable-send cannot be combined with --no-robot")
        return 2

    if bool(args.enable_send) and not bool(args.confirm):
        print("Real send requires --confirm and interactive YES confirmation.")
        return 2

    if args.max_runtime_s is not None and float(args.max_runtime_s) <= 0.0:
        print("--max-runtime-s must be positive when provided")
        return 2

    if bool(args.enable_send):
        if not _confirm_real_send(args):
            print("Confirmation failed or canceled. Exiting without send.")
            return 1

    app_config = _build_app_config(args)

    ui_config = UIConfig(
        enabled=bool(args.ui),
        update_hz=20.0,
        window_title="TJ-FX Teleop Diagnostic UI",
    )
    logging_config = LoggingConfig(enabled=bool(args.logging))

    snapshot_store = LatestSnapshotStore() if bool(args.ui) else None

    app = FullTeleopApp(
        config=app_config,
        logging_config=logging_config,
        ui_config=ui_config,
        snapshot_store=snapshot_store,
    )

    if bool(args.ui):
        from teleop.ui.app import run_ui

        app.initialize()
        thread = threading.Thread(target=app.run, name="full-teleop-loop", daemon=True)
        thread.start()

        try:
            return run_ui(
                snapshot_store=app.snapshot_store if app.snapshot_store is not None else LatestSnapshotStore(),
                config=ui_config,
            )
        finally:
            app.request_stop()
            app.shutdown()
            thread.join(timeout=2.0)

    try:
        app.run()
        return 0
    except KeyboardInterrupt:
        app.request_stop()
        app.shutdown()
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
