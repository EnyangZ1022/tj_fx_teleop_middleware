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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Hardware-dependent diagnostic: connect robot, read joint feedback, convert to FK xyzabc, print once. "
            "No motion command is sent."
        )
    )
    parser.add_argument("--robot-ip", default="192.168.1.190", help="Robot controller IP")
    parser.add_argument("--kine-cfg", default="assets/kinematics/ccs_m6_40.MvKDCfg", help="Kinematics config path")
    parser.add_argument("--left-arm", default="A", help="SDK arm label for project left side")
    parser.add_argument("--right-arm", default="B", help="SDK arm label for project right side")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    config = RobotSDKConfig(
        robot_ip=args.robot_ip,
        kine_cfg=args.kine_cfg,
        left_arm=args.left_arm,
        right_arm=args.right_arm,
    )

    adapter = RobotSDKReadOnlyAdapter(config=config)
    print("Stage 6A read-only hardware diagnostic (no motion command).")

    try:
        adapter.connect()
        dual_feedback = adapter.get_dual_arm_feedback()
        print("DualArmRobotFeedback:")
        print(dual_feedback)
    finally:
        adapter.disconnect()


if __name__ == "__main__":
    main()
