from __future__ import annotations

import argparse
from pathlib import Path
import sys

# Allow running this script directly without installing the package.
ROOT_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from teleop.core.command_frame import ArmCommandTarget, DualArmCommandTarget
from teleop.robot import RobotSDKConfig, RobotSDKReadOnlyAdapter, RobotStartupAdapter, RobotStartupConfig
from teleop.robot.command_adapter import RobotCommandAdapter
from teleop.robot.command_config import RobotCommandConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Hardware-dependent minimal Stage 6B flow: connect, move ready pose (Stage 6B-pre), "
            "build one tiny command, optionally send once, then pause/stop/release."
        )
    )
    parser.add_argument("--robot-ip", default="192.168.1.190", help="Robot controller IP")
    parser.add_argument("--dry-run", action="store_true", help="Force dry-run mode")
    parser.add_argument("--enable-send", action="store_true", help="Allow one real send after confirmation")
    parser.add_argument("--delta-mm", type=float, default=2.0, help="Small x-axis delta in mm")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.enable_send and abs(float(args.delta_mm)) > 5.0:
        raise ValueError("--delta-mm must be <= 5.0 when --enable-send is used")

    dry_run = True
    if args.enable_send and not args.dry_run:
        dry_run = False

    sdk_adapter = RobotSDKReadOnlyAdapter(config=RobotSDKConfig(robot_ip=args.robot_ip))
    startup_adapter = RobotStartupAdapter(sdk_adapter=sdk_adapter, startup_config=RobotStartupConfig())

    cmd_adapter = RobotCommandAdapter(
        sdk_adapter=sdk_adapter,
        config=RobotCommandConfig(
            dry_run=dry_run,
            command_enabled=False,
            control_mode="joint_position",
            send_left=True,
            send_right=False,
        ),
    )

    try:
        startup_adapter.move_to_ready_pose(dry_run=dry_run)

        cmd_adapter.prepare()

        left_fb = sdk_adapter.get_arm_feedback("left")
        left_target = ArmCommandTarget(
            position_xyz_mm=(
                float(left_fb.position_xyz[0]) + float(args.delta_mm),
                float(left_fb.position_xyz[1]),
                float(left_fb.position_xyz[2]),
            ),
            orientation_abc_deg=(
                float(left_fb.orientation_abc[0]),
                float(left_fb.orientation_abc[1]),
                float(left_fb.orientation_abc[2]),
            ),
            ik_reference_q_deg=tuple(float(v) for v in sdk_adapter._config.left_ik_reference_q_deg),
            valid=True,
        )
        command = DualArmCommandTarget(left=left_target, right=None)

        print("Stage 6B minimal one-shot command")
        print(f"  dry_run={dry_run}")
        print(f"  target={left_target}")

        if not dry_run:
            confirm = input("Type YES to send one command after ready pose: ").strip()
            if confirm != "YES":
                print("Canceled by user.")
                return
            cmd_adapter.enter_command_mode()
            cmd_adapter.enable_commands()

        result = cmd_adapter.send_command(command)
        print("send_command result:")
        print(result)
    finally:
        cmd_adapter.pause()
        cmd_adapter.stop()


if __name__ == "__main__":
    main()
