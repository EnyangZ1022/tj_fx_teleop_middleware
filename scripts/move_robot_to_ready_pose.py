from __future__ import annotations

import argparse
from pathlib import Path
import sys

# Allow running this script directly without installing the package.
ROOT_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from teleop.robot import RobotSDKConfig, RobotSDKReadOnlyAdapter
from teleop.robot.startup import (
    RobotStartupAdapter,
    RobotStartupConfig,
    get_current_joints,
    max_joint_abs_error_deg,
)


def _parse_joint_list(text: str) -> tuple[float, ...]:
    pieces = [p.strip() for p in text.split(",") if p.strip()]
    if len(pieces) != 7:
        raise argparse.ArgumentTypeError("joint list must contain exactly 7 comma-separated values")
    try:
        return tuple(float(v) for v in pieces)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("joint list values must be numeric") from exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Hardware-dependent startup script: connect safely, clear errors, verify feedback, "
            "move both arms to ready pose at low speed, then stop. This is NOT teleoperation."
        )
    )
    parser.add_argument("--robot-ip", default="192.168.1.190", help="Robot controller IP")
    parser.add_argument("--left-arm", default="A", help="SDK arm label for project left side")
    parser.add_argument("--right-arm", default="B", help="SDK arm label for project right side")
    parser.add_argument("--vel-ratio", type=int, default=20, help="Low-speed startup velocity ratio")
    parser.add_argument("--acc-ratio", type=int, default=20, help="Low-speed startup acceleration ratio")
    parser.add_argument(
        "--left-ready",
        type=_parse_joint_list,
        default=(90.0, -60.0, -90.0, -90.0, 0.0, 0.0, 0.0),
        help="Left ready joints in degree, comma-separated",
    )
    parser.add_argument(
        "--right-ready",
        type=_parse_joint_list,
        default=(90.0, 60.0, -90.0, -90.0, 0.0, 0.0, 0.0),
        help="Right ready joints in degree, comma-separated",
    )
    parser.add_argument("--dry-run", action="store_true", help="Preview only, do not connect or send motion commands")
    return parser.parse_args()


def _print_plan(args: argparse.Namespace, startup_config: RobotStartupConfig) -> None:
    print("Stage 6B-pre startup plan (NOT teleoperation):")
    print(f"  robot_ip: {args.robot_ip}")
    print(f"  arm mapping: left->{args.left_arm}, right->{args.right_arm}")
    print(f"  vel_ratio: {startup_config.vel_ratio}")
    print(f"  acc_ratio: {startup_config.acc_ratio}")
    print(f"  left_ready_q_deg: {startup_config.left_ready_q_deg}")
    print(f"  right_ready_q_deg: {startup_config.right_ready_q_deg}")


def main() -> None:
    args = parse_args()

    startup_config = RobotStartupConfig(
        vel_ratio=args.vel_ratio,
        acc_ratio=args.acc_ratio,
        left_ready_q_deg=tuple(args.left_ready),
        right_ready_q_deg=tuple(args.right_ready),
    )

    _print_plan(args, startup_config)

    sdk_config = RobotSDKConfig(
        robot_ip=args.robot_ip,
        left_arm=args.left_arm,
        right_arm=args.right_arm,
    )
    sdk_adapter = RobotSDKReadOnlyAdapter(config=sdk_config)

    if args.dry_run:
        print("Dry-run enabled: connecting and validating feedback stream only. No joint command will be sent.")
        try:
            sdk_adapter.connect()
            print("Connected and feedback stream verified.")
            print(f"Would move left arm {sdk_config.left_arm} to: {startup_config.left_ready_q_deg}")
            print(f"Would move right arm {sdk_config.right_arm} to: {startup_config.right_ready_q_deg}")
            print("Would use position mode with low-speed ratios:")
            print(f"  vel_ratio={startup_config.vel_ratio}, acc_ratio={startup_config.acc_ratio}")
        finally:
            sdk_adapter.disconnect()
            print("Robot connection released.")
        return

    confirm = input("Type YES to execute startup motion: ").strip()
    if confirm != "YES":
        print("Canceled by user.")
        return

    startup_adapter = RobotStartupAdapter(sdk_adapter=sdk_adapter, startup_config=startup_config)

    print("Connecting robot and executing startup ready-pose movement...")
    try:
        startup_adapter.move_to_ready_pose(dry_run=False)

        assert sdk_adapter.robot is not None
        assert sdk_adapter.dcss is not None

        left_current = get_current_joints(sdk_adapter.robot, sdk_adapter.dcss, sdk_config.left_arm)
        right_current = get_current_joints(sdk_adapter.robot, sdk_adapter.dcss, sdk_config.right_arm)

        left_err = max_joint_abs_error_deg(left_current, startup_config.left_ready_q_deg)
        right_err = max_joint_abs_error_deg(right_current, startup_config.right_ready_q_deg)

        print(f"Left final max joint error: {left_err:.3f} deg")
        print(f"Right final max joint error: {right_err:.3f} deg")
        print("Startup move completed. Teleoperation mode is not started.")
    finally:
        sdk_adapter.disconnect()
        print("Robot connection released.")


if __name__ == "__main__":
    main()
