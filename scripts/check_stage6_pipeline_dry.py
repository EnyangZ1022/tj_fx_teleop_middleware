from __future__ import annotations

from pathlib import Path
import sys

# Allow running this script directly without installing the package.
ROOT_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from teleop.control.command_scheduler import CommandSchedulerConfig, FixedRateCommandScheduler
from teleop.control.target_buffer import TargetBuffer
from teleop.core.pose import Pose7
from teleop.core.robot_frame import DualArmRobotFeedback, RobotArmFeedback
from teleop.core.teleop_frame import TeleopArmInput, TeleopFrame
from teleop.safety.safety_gate import TargetSafetyGate
from teleop.transform.coordinate_transform import PositionOnlyCoordinateTransformer


def _arm_input(*, xyz_m: tuple[float, float, float], grip: float, valid: bool = True) -> TeleopArmInput:
    pose = Pose7.from_tuple((xyz_m[0], xyz_m[1], xyz_m[2], 0.0, 0.0, 0.0, 1.0)) if valid else None
    return TeleopArmInput(
        pose_pico=pose,
        valid=valid,
        enable=grip >= 0.85,
        gripper_position=0.5,
        gripper_changed=False,
        trigger=0.1,
        grip=grip,
        axis_x=0.0,
        axis_y=0.0,
        axis_click=False,
    )


def _teleop_frame(*, frame_id: int, now_ns: int, left_xyz_m: tuple[float, float, float], right_xyz_m: tuple[float, float, float], grip: float) -> TeleopFrame:
    return TeleopFrame(
        frame_id=frame_id,
        source_device_id="synthetic_pico",
        source_timestamp_ns=now_ns,
        pc_receive_time_ns=now_ns,
        left=_arm_input(xyz_m=left_xyz_m, grip=grip),
        right=_arm_input(xyz_m=right_xyz_m, grip=grip),
        start_pause_requested=False,
        cancel_requested=False,
        calibration_requested=False,
    )


def _feedback() -> DualArmRobotFeedback:
    return DualArmRobotFeedback(
        left=RobotArmFeedback(
            position_xyz=(500.0, 1000.0, 1500.0),
            orientation_abc=(10.0, 20.0, 30.0),
            valid=True,
        ),
        right=RobotArmFeedback(
            position_xyz=(1000.0, 2000.0, 3000.0),
            orientation_abc=(40.0, 50.0, 60.0),
            valid=True,
        ),
    )


def main() -> None:
    transformer = PositionOnlyCoordinateTransformer()
    safety_gate = TargetSafetyGate()
    target_buffer = TargetBuffer()
    scheduler = FixedRateCommandScheduler(target_buffer=target_buffer, config=CommandSchedulerConfig())

    now_ns = 1_000_000_000
    ref_frame = _teleop_frame(
        frame_id=1,
        now_ns=now_ns,
        left_xyz_m=(0.0, 0.0, 0.0),
        right_xyz_m=(0.0, 0.0, 0.0),
        grip=0.9,
    )

    feedback = _feedback()
    calibration = transformer.create_calibration(ref_frame, feedback)

    moved_frame_enabled = _teleop_frame(
        frame_id=2,
        now_ns=now_ns + 10_000_000,
        left_xyz_m=(0.0, 0.0, 0.0),
        right_xyz_m=(0.0, 0.0, 0.1),
        grip=0.9,
    )
    moved_target = transformer.make_target(moved_frame_enabled, feedback, calibration)

    print("Stage 6C synthetic dry pipeline")
    print("1) Transform output check")
    if moved_target.right is None:
        raise RuntimeError("right target is None")
    print(f"   right target xyz mm: {moved_target.right.position_xyz}")
    print(f"   right target abc deg: {moved_target.right.orientation_abc}")

    expected_right_x = 1100.0
    if abs(moved_target.right.position_xyz[0] - expected_right_x) > 1e-6:
        raise RuntimeError("Expected right +Z 0.1 m to map to +100 mm on robot +X")

    if moved_target.right.orientation_abc != feedback.right.orientation_abc:
        raise RuntimeError("Orientation is not frozen in position-only mode")

    print("2) Safety gate blocks when enable is false")
    moved_frame_disabled = _teleop_frame(
        frame_id=3,
        now_ns=now_ns + 20_000_000,
        left_xyz_m=(0.0, 0.0, 0.0),
        right_xyz_m=(0.0, 0.0, 0.1),
        grip=0.1,
    )
    blocked_decision = safety_gate.evaluate(
        moved_frame_disabled,
        moved_target,
        calibration,
        now_ns=now_ns + 20_000_000,
    )
    print(
        f"   state={blocked_decision.state.value} allow_motion={blocked_decision.allow_motion} "
        f"global_reason={blocked_decision.global_reason}"
    )
    if blocked_decision.allow_motion:
        raise RuntimeError("Safety gate should block motion when enable/deadman is false")

    print("3) Safety pass and scheduler attach fixed IK references")
    safe_decision = safety_gate.evaluate(
        moved_frame_enabled,
        moved_target,
        calibration,
        now_ns=now_ns + 30_000_000,
    )
    if safe_decision.safe_target is None:
        raise RuntimeError("Expected safe_target from safety gate")

    target_buffer.update(safe_decision.safe_target, timestamp_ns=now_ns + 30_000_000)
    command, diag = scheduler.step(now_ns=now_ns + 40_000_000)
    print(
        f"   scheduler sequence={diag.sequence_id} age_ms={diag.target_age_ms} "
        f"zoh={diag.used_zero_order_hold} limited={diag.limited}"
    )

    if command is None or command.right is None:
        raise RuntimeError("Expected right command target after scheduling")

    print(f"   right command xyz mm: {command.right.position_xyz_mm}")
    print(f"   right command abc deg: {command.right.orientation_abc_deg}")
    print(f"   right fixed ik ref q deg: {command.right.ik_reference_q_deg}")

    if command.left is not None:
        print(f"   left fixed ik ref q deg: {command.left.ik_reference_q_deg}")

    expected_left_ref = (90.0, -90.0, -90.0, -90.0, 0.0, 0.0, 0.0)
    expected_right_ref = (90.0, 90.0, -90.0, -90.0, 0.0, 0.0, 0.0)

    if command.right.ik_reference_q_deg != expected_right_ref:
        raise RuntimeError("Right IK reference mismatch")
    if command.left is not None and command.left.ik_reference_q_deg != expected_left_ref:
        raise RuntimeError("Left IK reference mismatch")

    print("All Stage 6C synthetic dry checks passed.")


if __name__ == "__main__":
    main()
