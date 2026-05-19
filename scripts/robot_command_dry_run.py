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
from teleop.robot import RobotSDKConfig, RobotSDKReadOnlyAdapter
from teleop.robot.command_adapter import RobotCommandAdapter
from teleop.robot.command_config import RobotCommandConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Hardware-dependent Stage 6B script: build a tiny command target near current pose, "
            "run IK, and optionally send once with explicit confirmation."
        )
    )
    parser.add_argument("--robot-ip", default="192.168.1.190", help="Robot controller IP")
    parser.add_argument("--dry-run", action="store_true", help="Force dry-run preview mode")
    parser.add_argument("--enable-send", action="store_true", help="Allow one real SDK send after confirmation")
    parser.add_argument("--side", choices=["left", "right", "both"], default="both", help="Which side to process")
    parser.add_argument("--delta-mm", type=float, default=2.0, help="Small position delta in mm for mock command")
    return parser.parse_args()


def _build_target(
    sdk_adapter: RobotSDKReadOnlyAdapter,
    side: str,
    delta_mm: float,
) -> ArmCommandTarget:
    feedback = sdk_adapter.get_arm_feedback(side)
    if side == "left":
        ik_ref = tuple(float(v) for v in sdk_adapter._config.left_ik_reference_q_deg)
    else:
        ik_ref = tuple(float(v) for v in sdk_adapter._config.right_ik_reference_q_deg)

    target_xyz = (
        float(feedback.position_xyz[0]) + float(delta_mm),
        float(feedback.position_xyz[1]),
        float(feedback.position_xyz[2]),
    )

    return ArmCommandTarget(
        position_xyz_mm=target_xyz,
        orientation_abc_deg=(
            float(feedback.orientation_abc[0]),
            float(feedback.orientation_abc[1]),
            float(feedback.orientation_abc[2]),
        ),
        ik_reference_q_deg=ik_ref,
        valid=True,
    )


def main() -> None:
    args = parse_args()

    if args.enable_send and abs(float(args.delta_mm)) > 5.0:
        raise ValueError("--delta-mm must be <= 5.0 when --enable-send is used")

    dry_run = True
    if args.enable_send and not args.dry_run:
        dry_run = False

    sdk_config = RobotSDKConfig(robot_ip=args.robot_ip)
    sdk_adapter = RobotSDKReadOnlyAdapter(config=sdk_config)

    cmd_config = RobotCommandConfig(
        dry_run=dry_run,
        command_enabled=False,
        control_mode="joint_position",
        send_left=args.side in {"left", "both"},
        send_right=args.side in {"right", "both"},
    )
    cmd_adapter = RobotCommandAdapter(sdk_adapter=sdk_adapter, config=cmd_config)

    try:
        sdk_adapter.connect()
        cmd_adapter.prepare()

        left_target = _build_target(sdk_adapter, "left", args.delta_mm) if args.side in {"left", "both"} else None
        right_target = _build_target(sdk_adapter, "right", args.delta_mm) if args.side in {"right", "both"} else None
        command = DualArmCommandTarget(left=left_target, right=right_target)

        print("Stage 6B command dry-run/send preview")
        print(f"  dry_run={dry_run}")
        print(f"  side={args.side}")
        print(f"  delta_mm={args.delta_mm}")
        print(f"  left_target={left_target}")
        print(f"  right_target={right_target}")

        if not dry_run:
            print("Real send requested. Safety checks:")
            print("  - robot must already be in safe ready posture")
            print("  - area must be clear")
            print("  - emergency stop must be reachable")
            confirm = input("Type YES to send one command: ").strip()
            if confirm != "YES":
                print("Canceled by user.")
                return

            cmd_adapter.enter_command_mode()
            cmd_adapter.enable_commands()

        result = cmd_adapter.send_command(command)
        print("send_command result:")
        print(result)
    finally:
        cmd_adapter.stop()


if __name__ == "__main__":
    main()
