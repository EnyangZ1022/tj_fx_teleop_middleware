from __future__ import annotations

import argparse
import ctypes
from contextlib import contextmanager
from pathlib import Path
import platform
import sys
import threading

# Allow running this script directly without installing the package.
ROOT_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from teleop.app import FullTeleopApp, FullTeleopAppConfig
from teleop.core.teleop_mode import TeleopMode
from teleop.filtering import OrientationFilterConfig
from teleop.logging import LoggingConfig
from teleop.robot import RobotCommandConfig
from teleop.transform.orientation_transform import OrientationTrackingConfig
from teleop.ui.snapshot import LatestSnapshotStore
from teleop.ui.ui_config import UIConfig


@contextmanager
def _windows_high_res_timer(enable: bool, period_ms: int):
    """Temporarily request a high-resolution Windows timer.

    On non-Windows platforms, this is a no-op.
    """
    if not enable:
        yield
        return

    if platform.system() != "Windows":
        print("Windows high-resolution timer requested on non-Windows platform; no-op.")
        yield
        return

    try:
        if int(period_ms) <= 0:
            raise ValueError("period_ms must be positive")

        winmm = ctypes.WinDLL("winmm")
        begin = winmm.timeBeginPeriod
        end = winmm.timeEndPeriod

        begin.argtypes = [ctypes.c_uint]
        begin.restype = ctypes.c_uint
        end.argtypes = [ctypes.c_uint]
        end.restype = ctypes.c_uint

        result = begin(int(period_ms))
    except Exception as exc:
        print(f"Warning: high-resolution timer setup failed: {exc}")
        yield
        return

    if result != 0:
        print(
            f"Warning: timeBeginPeriod({period_ms}) failed with code {result}; "
            "continuing without high-resolution timer."
        )
        yield
        return

    print(f"Windows high-resolution timer enabled: {period_ms} ms")
    try:
        yield
    finally:
        try:
            end_result = end(int(period_ms))
            if end_result != 0:
                print(f"Warning: timeEndPeriod({period_ms}) failed with code {end_result}")
            else:
                print("Windows high-resolution timer restored.")
        except Exception as exc:
            print(f"Warning: timeEndPeriod({period_ms}) raised exception: {exc}")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
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
    parser.add_argument(
        "--teleop-mode",
        choices=[TeleopMode.POSITION_ONLY.value, TeleopMode.POSITION_ORIENTATION.value],
        default=TeleopMode.POSITION_ONLY.value,
        help="Teleoperation mode (position-only default; orientation tracking is experimental)",
    )
    parser.add_argument(
        "--enable-orientation",
        action="store_true",
        help="Shorthand for --teleop-mode position_orientation",
    )
    parser.add_argument(
        "--control-mode",
        choices=["joint_position", "joint_impedance"],
        default="joint_position",
        help="Robot command control mode (joint_position default; joint_impedance requires extra safety checks)",
    )
    parser.add_argument(
        "--orientation-algorithm",
        choices=["absolute_matrix", "relative_rotvec"],
        default="absolute_matrix",
        help="Orientation tracking algorithm selector (absolute_matrix default)",
    )
    parser.add_argument(
        "--enable-orientation-filter",
        action="store_true",
        help="Enable quaternion Slerp low-pass orientation filtering in position_orientation mode",
    )
    parser.add_argument(
        "--disable-orientation-filter",
        action="store_true",
        help="Disable quaternion Slerp low-pass orientation filtering in position_orientation mode",
    )
    parser.add_argument(
        "--orientation-filter-tau",
        type=float,
        default=None,
        help="Override orientation filter time constant tau in seconds (default 0.02)",
    )
    parser.add_argument(
        "--orientation-filter-fallback-dt",
        type=float,
        default=None,
        help="Override orientation filter fallback dt in seconds (default 0.01)",
    )
    parser.add_argument(
        "--joint-limit-mode",
        choices=["reject", "ramp"],
        default="reject",
        help="Unified joint limit handling mode (reject default, ramp experimental)",
    )
    parser.add_argument(
        "--max-joint-step-deg",
        type=float,
        default=None,
        help="Override max joint step limit in degrees (keeps config default when omitted)",
    )
    parser.add_argument(
        "--max-joint-velocity-deg-s",
        type=float,
        default=None,
        help="Override max joint velocity limit in deg/s (keeps config default when omitted)",
    )
    parser.add_argument(
        "--enable-win-high-res-timer",
        action="store_true",
        help=(
            "Enable Windows high-resolution timer using timeBeginPeriod. "
            "No-op on non-Windows platforms."
        ),
    )
    parser.add_argument(
        "--win-high-res-timer-ms",
        type=int,
        default=1,
        help="Requested Windows timer period in milliseconds. Default: 1.",
    )
    parser.add_argument(
        "--spin-threshold-ms",
        type=float,
        default=0.5,
        help=(
            "Final busy-spin window in milliseconds near scheduler deadline. "
            "Set 0 to disable spin and use pure sleep."
        ),
    )
    parser.add_argument("--rate-hz", type=float, default=100.0, help="Command scheduler rate in Hz")
    parser.add_argument("--side", choices=["left", "right", "both"], default="both", help="Single-arm mode")
    parser.add_argument("--max-runtime-s", type=float, default=None, help="Optional runtime cap in seconds")
    parser.add_argument(
        "--confirm",
        action="store_true",
        help="Required alongside --enable-send to unlock interactive YES confirmation",
    )
    args = parser.parse_args(argv)
    if int(args.win_high_res_timer_ms) <= 0:
        parser.error("--win-high-res-timer-ms must be positive")
    if float(args.spin_threshold_ms) < 0.0:
        parser.error("--spin-threshold-ms must be >= 0")
    return args


def _confirm_real_send(args: argparse.Namespace) -> bool:
    print("Real send requested. Review before proceeding:")
    print(f"  robot_ip: {args.robot_ip}")
    print(f"  rate_hz: {float(args.rate_hz):.1f}")
    print(f"  control_mode: {str(args.control_mode)}")
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
    teleop_mode = str(args.teleop_mode)
    if bool(args.enable_orientation):
        teleop_mode = TeleopMode.POSITION_ORIENTATION.value

    orientation_tracking = OrientationTrackingConfig(
        enabled=(teleop_mode == TeleopMode.POSITION_ORIENTATION.value),
        orientation_algorithm=str(args.orientation_algorithm),
    )

    orientation_filter_enabled = teleop_mode == TeleopMode.POSITION_ORIENTATION.value
    if bool(args.disable_orientation_filter):
        orientation_filter_enabled = False
    if bool(args.enable_orientation_filter):
        orientation_filter_enabled = True

    orientation_filter = OrientationFilterConfig(
        enabled=orientation_filter_enabled,
        tau_s=float(args.orientation_filter_tau) if args.orientation_filter_tau is not None else 0.02,
        fallback_dt_s=(
            float(args.orientation_filter_fallback_dt)
            if args.orientation_filter_fallback_dt is not None
            else 0.01
        ),
    )

    return FullTeleopAppConfig(
        robot_ip=str(args.robot_ip),
        connect_pico=not bool(args.no_pico),
        connect_robot=not bool(args.no_robot),
        move_to_ready=bool(args.move_to_ready),
        enable_send=bool(args.enable_send),
        dry_run=dry_run,
        require_confirmation=True,
        command_rate_hz=float(args.rate_hz),
        spin_threshold_s=float(args.spin_threshold_ms) / 1000.0,
        ui_enabled=bool(args.ui),
        logging_enabled=bool(args.logging),
        teleop_mode=teleop_mode,
        orientation_tracking=orientation_tracking,
        orientation_filter=orientation_filter,
        control_mode=str(args.control_mode),
        single_arm_mode=side_mode,
        max_runtime_s=float(args.max_runtime_s) if args.max_runtime_s is not None else None,
    )


def _build_robot_command_config(args: argparse.Namespace) -> RobotCommandConfig:
    overrides: dict[str, float | str] = {
        "joint_limit_mode": str(args.joint_limit_mode),
    }
    if args.max_joint_step_deg is not None:
        overrides["max_joint_step_deg"] = float(args.max_joint_step_deg)
    if args.max_joint_velocity_deg_s is not None:
        overrides["max_joint_velocity_deg_s"] = float(args.max_joint_velocity_deg_s)
    return RobotCommandConfig(**overrides)


def _validate_runtime_args(args: argparse.Namespace) -> str | None:
    if bool(args.enable_orientation_filter) and bool(args.disable_orientation_filter):
        return "--enable-orientation-filter and --disable-orientation-filter cannot be used together"

    if bool(args.enable_send) and bool(args.no_robot):
        return "--enable-send cannot be combined with --no-robot"

    if bool(args.enable_send) and not bool(args.confirm):
        return "Real send requires --confirm and interactive YES confirmation."

    if (
        bool(args.enable_send)
        and str(args.control_mode).strip().lower() == "joint_impedance"
        and not bool(args.move_to_ready)
    ):
        return "joint_impedance mode requires --move-to-ready for real robot sending in this MVP."

    if args.max_runtime_s is not None and float(args.max_runtime_s) <= 0.0:
        return "--max-runtime-s must be positive when provided"

    if args.orientation_filter_tau is not None and float(args.orientation_filter_tau) <= 0.0:
        return "--orientation-filter-tau must be > 0"

    if args.orientation_filter_fallback_dt is not None and float(args.orientation_filter_fallback_dt) <= 0.0:
        return "--orientation-filter-fallback-dt must be > 0"

    if args.max_joint_step_deg is not None and float(args.max_joint_step_deg) <= 0.0:
        return "--max-joint-step-deg must be > 0"

    if args.max_joint_velocity_deg_s is not None and float(args.max_joint_velocity_deg_s) <= 0.0:
        return "--max-joint-velocity-deg-s must be > 0"

    return None


def main() -> int:
    args = parse_args()

    if int(args.win_high_res_timer_ms) > 15:
        print(
            "Warning: --win-high-res-timer-ms is greater than 15 ms; "
            "this may not improve timing precision."
        )

    validation_error = _validate_runtime_args(args)
    if validation_error is not None:
        print(validation_error)
        return 2

    if bool(args.enable_send):
        if not _confirm_real_send(args):
            print("Confirmation failed or canceled. Exiting without send.")
            return 1

    with _windows_high_res_timer(
        enable=bool(args.enable_win_high_res_timer),
        period_ms=int(args.win_high_res_timer_ms),
    ):
        app_config = _build_app_config(args)
        robot_command_config = _build_robot_command_config(args)

        print(f"Control mode: {app_config.control_mode}")
        print(f"Teleop mode: {app_config.teleop_mode}")
        print(f"joint_limit_mode: {robot_command_config.joint_limit_mode}")
        print(f"max_joint_step_deg: {float(robot_command_config.max_joint_step_deg):.3f}")
        print(f"max_joint_velocity_deg_s: {float(robot_command_config.max_joint_velocity_deg_s):.3f}")
        print(f"spin_threshold_ms: {float(args.spin_threshold_ms):.3f}")
        print(f"win_high_res_timer: {'enabled' if bool(args.enable_win_high_res_timer) else 'disabled'}")
        print(f"win_high_res_timer_ms: {int(args.win_high_res_timer_ms)}")
        if app_config.teleop_mode == TeleopMode.POSITION_ORIENTATION.value:
            print(f"Orientation filter: {'enabled' if bool(app_config.orientation_filter.enabled) else 'disabled'}")
            print(f"orientation_filter_tau_s: {float(app_config.orientation_filter.tau_s):.5f}")
            print(f"orientation_filter_fallback_dt_s: {float(app_config.orientation_filter.fallback_dt_s):.5f}")
            print(f"reset_on_calibration: {bool(app_config.orientation_filter.reset_on_calibration)}")
        if bool(app_config.orientation_tracking.enabled):
            print("Orientation tracking: enabled")
            print(f"  orientation_algorithm={app_config.orientation_tracking.orientation_algorithm}")
            print(f"  use_calibration_offset={bool(app_config.orientation_tracking.use_calibration_offset)}")
            print(f"  rotation_scale={float(app_config.orientation_tracking.rotation_scale):.3f}")
            print(f"  max_total_angle_deg={float(app_config.orientation_tracking.max_total_angle_deg):.2f}")
            print(f"  max_step_angle_deg={float(app_config.orientation_tracking.max_step_angle_deg):.2f}")
            print(f"  relative_mode={app_config.orientation_tracking.relative_mode}")
        else:
            print("Orientation tracking: disabled")

        ui_config = UIConfig(
            enabled=bool(args.ui),
            update_hz=20.0,
            window_title="TJ-FX Teleop Diagnostic UI",
        )
        logging_config = LoggingConfig(enabled=bool(args.logging))
        if bool(args.logging):
            logging_config = LoggingConfig(
                enabled=True,
                record_events=True,
                record_frames=True,
                record_performance=True,
                frame_sample_hz=float(args.rate_hz),
                performance_sample_hz=min(float(args.rate_hz), 50.0),
            )

        snapshot_store = LatestSnapshotStore() if bool(args.ui) else None

        app = FullTeleopApp(
            config=app_config,
            robot_command_config=robot_command_config,
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
