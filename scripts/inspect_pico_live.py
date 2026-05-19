from __future__ import annotations

from pathlib import Path
import sys
import time
from dataclasses import dataclass

# Allow running this script directly without installing the package.
ROOT_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from teleop.core.pose import Pose7
from teleop.core.pico_frame import PicoControllerState, PicoRawFrame
from teleop.input.pico_receiver import PicoReceiver


@dataclass
class Baseline:
    head: Pose7 | None = None
    left: Pose7 | None = None
    right: Pose7 | None = None


def fmt_pose(p: Pose7 | None) -> str:
    if p is None:
        return "None"
    return (
        f"pos=({p.x:+.4f}, {p.y:+.4f}, {p.z:+.4f}) "
        f"quat=({p.qx:+.3f}, {p.qy:+.3f}, {p.qz:+.3f}, {p.qw:+.3f})"
    )


def fmt_delta(p: Pose7 | None, b: Pose7 | None) -> str:
    if p is None or b is None:
        return "d=(n/a)"
    return f"d=({p.x - b.x:+.4f}, {p.y - b.y:+.4f}, {p.z - b.z:+.4f})"


def valid_pose(p: Pose7 | None) -> Pose7 | None:
    if p is None:
        return None
    return p if p.is_valid() else None


def fmt_ctrl(side: str, ctrl: PicoControllerState, baseline_pose: Pose7 | None, valid: bool) -> str:
    pose = ctrl.pose if valid else None
    return (
        f"{side:<5} valid={valid} "
        f"{fmt_pose(pose)} {fmt_delta(pose, baseline_pose)} | "
        f"trigger={ctrl.trigger:.3f} grip={ctrl.grip:.3f} "
        f"axis=({ctrl.axis_x:+.3f},{ctrl.axis_y:+.3f}) "
        f"axisClick={int(ctrl.axis_click)} "
        f"primary={int(ctrl.primary_button)} "
        f"secondary={int(ctrl.secondary_button)} "
        f"menu={int(ctrl.menu_button)}"
    )


def main() -> None:
    receiver = PicoReceiver()
    receiver.start()

    baseline = Baseline()
    last_frame_id = -1

    print("Pico live inspector started.")
    print("Suggested test order:")
    print("  1) Keep headset/controllers still for 2 seconds.")
    print("  2) Move one controller +right / +up / +forward, one direction at a time.")
    print("  3) Hold each button for at least 2 seconds to observe field changes.")
    print("  4) Press Ctrl+C to stop.")
    print()

    try:
        while True:
            frame = receiver.get_latest_frame()

            if frame is None:
                print("No frame received yet.")
                time.sleep(1.0)
                continue

            # Set baseline once when valid poses first appear.
            if baseline.head is None and frame.head_pose.is_valid():
                baseline.head = frame.head_pose
            if baseline.left is None and frame.left_valid:
                baseline.left = frame.left_ctrl.pose
            if baseline.right is None and frame.right_valid:
                baseline.right = frame.right_ctrl.pose

            new_frame_flag = "" if frame.frame_id != last_frame_id else " [NO NEW FRAME]"
            last_frame_id = frame.frame_id

            print("=" * 120)
            print(
                f"frame_id={frame.frame_id}{new_frame_flag} "
                f"device={frame.device_id} "
                f"pico_ts={frame.pico_timestamp_ns} "
                f"pc_ts={frame.pc_receive_time_ns}"
            )

            head_pose = frame.head_pose if frame.head_pose.is_valid() else None
            print(f"head  valid={head_pose is not None} {fmt_pose(head_pose)} {fmt_delta(head_pose, baseline.head)}")
            print(fmt_ctrl("left", frame.left_ctrl, baseline.left, frame.left_valid))
            print(fmt_ctrl("right", frame.right_ctrl, baseline.right, frame.right_valid))

            time.sleep(1.0)

    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()
        print("Pico live inspector stopped.")


if __name__ == "__main__":
    main()