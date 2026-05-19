from teleop.core.pose import Pose7
from teleop.core.robot_frame import DualArmRobotTarget, RobotArmTarget
from teleop.core.teleop_frame import TeleopArmInput, TeleopFrame
from teleop.safety.safety_config import SafetyConfig
from teleop.safety.safety_gate import TargetSafetyGate
from teleop.safety.state_machine import SafetyState
from teleop.transform.calibration import ArmCalibrationAnchor, DualArmCalibrationState


def _pose(x: float, y: float, z: float) -> Pose7:
    return Pose7.from_tuple((x, y, z, 0.0, 0.0, 0.0, 1.0))


def _arm_input(
    *,
    valid: bool = True,
    grip: float = 0.9,
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> TeleopArmInput:
    return TeleopArmInput(
        pose_pico=_pose(*xyz) if valid else None,
        valid=valid,
        enable=grip >= 0.8,
        gripper_position=0.5,
        gripper_changed=False,
        trigger=0.5,
        grip=grip,
        axis_x=0.0,
        axis_y=0.0,
        axis_click=False,
    )


def _teleop_frame(
    *,
    pc_receive_time_ns: int,
    left_valid: bool = True,
    right_valid: bool = True,
    left_grip: float = 0.9,
    right_grip: float = 0.9,
) -> TeleopFrame:
    return TeleopFrame(
        frame_id=1,
        source_device_id="pico",
        source_timestamp_ns=pc_receive_time_ns,
        pc_receive_time_ns=pc_receive_time_ns,
        left=_arm_input(valid=left_valid, grip=left_grip),
        right=_arm_input(valid=right_valid, grip=right_grip),
        start_pause_requested=False,
        cancel_requested=False,
        calibration_requested=False,
    )


def _target(
    *,
    left: RobotArmTarget | None = None,
    right: RobotArmTarget | None = None,
) -> DualArmRobotTarget:
    return DualArmRobotTarget(
        left=left,
        right=right,
    )


def _arm_target(
    x: float,
    y: float,
    z: float,
    *,
    valid: bool = True,
) -> RobotArmTarget:
    return RobotArmTarget(
        position_xyz=(x, y, z),
        orientation_abc=(10.0, 20.0, 30.0),
        valid=valid,
    )


def _calibration(*, left: bool = True, right: bool = True) -> DualArmCalibrationState:
    left_anchor = (
        ArmCalibrationAnchor(
            pico_anchor_xyz=(0.0, 0.0, 0.0),
            robot_anchor_xyz=(0.0, 0.0, 0.0),
            robot_anchor_abc=(10.0, 20.0, 30.0),
            source_frame_id=1,
        )
        if left
        else None
    )
    right_anchor = (
        ArmCalibrationAnchor(
            pico_anchor_xyz=(0.0, 0.0, 0.0),
            robot_anchor_xyz=(0.0, 0.0, 0.0),
            robot_anchor_abc=(10.0, 20.0, 30.0),
            source_frame_id=1,
        )
        if right
        else None
    )
    return DualArmCalibrationState(left=left_anchor, right=right_anchor)


def test_disconnected_when_teleop_frame_missing() -> None:
    gate = TargetSafetyGate()

    decision = gate.evaluate(None, None, None, now_ns=1_000_000_000)

    assert decision.state == SafetyState.DISCONNECTED
    assert decision.allow_motion is False
    assert decision.safe_target is None
    assert decision.global_reason == "pico_timeout"


def test_disconnected_when_frame_is_stale() -> None:
    gate = TargetSafetyGate(SafetyConfig(pico_timeout_ms=100.0))
    frame = _teleop_frame(pc_receive_time_ns=1_000_000_000)

    decision = gate.evaluate(frame, None, None, now_ns=1_250_000_001)

    assert decision.state == SafetyState.DISCONNECTED
    assert decision.global_reason == "pico_timeout"


def test_wait_calibration_when_pose_valid_but_not_calibrated() -> None:
    gate = TargetSafetyGate()
    frame = _teleop_frame(pc_receive_time_ns=1_000_000_000)
    target = _target(left=_arm_target(0.01, 0.0, 0.0), right=_arm_target(0.01, 0.0, 0.0))

    decision = gate.evaluate(frame, target, None, now_ns=1_000_000_100)

    assert decision.state == SafetyState.WAIT_CALIBRATION
    assert decision.allow_motion is False
    assert decision.left_reason == "missing_calibration"
    assert decision.right_reason == "missing_calibration"


def test_wait_calibration_when_both_poses_invalid_and_not_calibrated() -> None:
    gate = TargetSafetyGate()
    frame = _teleop_frame(pc_receive_time_ns=1_000_000_000, left_valid=False, right_valid=False)

    decision = gate.evaluate(frame, None, _calibration(left=False, right=False), now_ns=1_000_000_100)

    assert decision.state == SafetyState.WAIT_CALIBRATION
    assert decision.global_reason == "missing_calibration"
    assert decision.allow_motion is False


def test_invalid_pose_with_calibration_enters_paused() -> None:
    gate = TargetSafetyGate()
    frame = _teleop_frame(pc_receive_time_ns=1_000_000_000, left_valid=False, right_valid=False)
    target = _target(left=_arm_target(0.01, 0.0, 0.0), right=_arm_target(0.01, 0.0, 0.0))

    decision = gate.evaluate(frame, target, _calibration(), now_ns=1_000_000_100)

    assert decision.state == SafetyState.PAUSED
    assert decision.allow_motion is False
    assert decision.global_reason == "invalid_pose"


def test_teleop_ready_when_calibrated_and_target_valid_but_enable_released() -> None:
    gate = TargetSafetyGate()
    frame = _teleop_frame(pc_receive_time_ns=1_000_000_000, left_grip=0.2, right_grip=0.2)
    target = _target(left=_arm_target(0.01, 0.0, 0.0), right=_arm_target(0.01, 0.0, 0.0))

    decision = gate.evaluate(frame, target, _calibration(), now_ns=1_000_000_100)

    assert decision.state == SafetyState.TELEOP_READY
    assert decision.allow_motion is False
    assert decision.global_reason == "enable_released"


def test_missing_robot_target_enters_paused() -> None:
    gate = TargetSafetyGate()
    frame = _teleop_frame(pc_receive_time_ns=1_000_000_000)

    decision = gate.evaluate(frame, None, _calibration(), now_ns=1_000_000_100)

    assert decision.state == SafetyState.PAUSED
    assert decision.allow_motion is False
    assert decision.global_reason == "target_invalid"


def test_teleop_active_allows_motion_when_targets_safe() -> None:
    gate = TargetSafetyGate()
    frame = _teleop_frame(pc_receive_time_ns=1_000_000_000)
    target = _target(left=_arm_target(0.01, 0.0, 0.0), right=_arm_target(0.02, 0.0, 0.0))

    decision = gate.evaluate(frame, target, _calibration(), now_ns=1_000_000_100)

    assert decision.state == SafetyState.TELEOP_ACTIVE
    assert decision.allow_motion is True
    assert decision.safe_target is not None
    assert decision.safe_target.left is not None
    assert decision.safe_target.right is not None


def test_single_arm_motion_allowed_by_default() -> None:
    gate = TargetSafetyGate()
    frame = _teleop_frame(pc_receive_time_ns=1_000_000_000)
    target = _target(left=None, right=_arm_target(0.01, 0.0, 0.0))

    decision = gate.evaluate(frame, target, _calibration(left=False, right=True), now_ns=1_000_000_100)

    assert decision.state == SafetyState.TELEOP_ACTIVE
    assert decision.allow_motion is True
    assert decision.left_allowed is False
    assert decision.right_allowed is True
    assert decision.safe_target is not None
    assert decision.safe_target.left is None
    assert decision.safe_target.right is not None


def test_invalid_left_side_is_filtered_while_right_side_passes() -> None:
    gate = TargetSafetyGate()
    frame = _teleop_frame(pc_receive_time_ns=1_000_000_000, left_valid=False, right_valid=True)
    target = _target(left=_arm_target(0.01, 0.0, 0.0), right=_arm_target(0.02, 0.0, 0.0))

    decision = gate.evaluate(frame, target, _calibration(), now_ns=1_000_000_100)

    assert decision.state == SafetyState.TELEOP_ACTIVE
    assert decision.allow_motion is True
    assert decision.left_allowed is False
    assert decision.right_allowed is True
    assert decision.left_reason == "invalid_pose"
    assert decision.safe_target is not None
    assert decision.safe_target.left is None
    assert decision.safe_target.right is not None


def test_single_arm_motion_blocked_when_disabled_in_config() -> None:
    gate = TargetSafetyGate(SafetyConfig(allow_single_arm_motion=False))
    frame = _teleop_frame(pc_receive_time_ns=1_000_000_000)
    target = _target(left=_arm_target(0.01, 0.0, 0.0), right=None)

    decision = gate.evaluate(frame, target, _calibration(left=True, right=False), now_ns=1_000_000_100)

    assert decision.state in {SafetyState.CALIBRATED, SafetyState.PAUSED, SafetyState.TELEOP_READY}
    assert decision.allow_motion is False
    assert decision.safe_target is None
    assert decision.left_reason == "single_arm_motion_not_allowed"


def test_require_both_arms_calibrated_blocks_motion() -> None:
    gate = TargetSafetyGate(SafetyConfig(require_both_arms_calibrated=True))
    frame = _teleop_frame(pc_receive_time_ns=1_000_000_000)
    target = _target(left=_arm_target(0.01, 0.0, 0.0), right=_arm_target(0.01, 0.0, 0.0))

    decision = gate.evaluate(frame, target, _calibration(left=True, right=False), now_ns=1_000_000_100)

    assert decision.state == SafetyState.WAIT_CALIBRATION
    assert decision.allow_motion is False
    assert decision.global_reason == "missing_calibration"


def test_small_target_step_within_limit_stays_active() -> None:
    gate = TargetSafetyGate(SafetyConfig(max_single_step_mm=50.0, max_velocity_mm_s=10_000.0))
    frame = _teleop_frame(pc_receive_time_ns=1_000_000_000)
    calibration = _calibration()

    decision_ok = gate.evaluate(
        frame,
        _target(left=_arm_target(0.0, 0.0, 0.0), right=_arm_target(0.0, 0.0, 0.0)),
        calibration,
        now_ns=1_000_000_100,
    )
    assert decision_ok.allow_motion is True

    # 10 mm step must pass when max_single_step_mm is 50 mm.
    decision_small_step = gate.evaluate(
        frame,
        _target(left=_arm_target(10.0, 0.0, 0.0), right=_arm_target(10.0, 0.0, 0.0)),
        calibration,
        now_ns=1_100_000_100,
    )

    assert decision_small_step.state == SafetyState.TELEOP_ACTIVE
    assert decision_small_step.allow_motion is True


def test_target_jump_enters_paused_and_blocks_motion() -> None:
    gate = TargetSafetyGate(SafetyConfig(max_single_step_mm=50.0, max_velocity_mm_s=10_000.0))
    frame = _teleop_frame(pc_receive_time_ns=1_000_000_000)
    calibration = _calibration()

    decision_ok = gate.evaluate(
        frame,
        _target(left=_arm_target(0.0, 0.0, 0.0), right=_arm_target(0.0, 0.0, 0.0)),
        calibration,
        now_ns=1_000_000_100,
    )
    assert decision_ok.allow_motion is True

    decision_jump = gate.evaluate(
        frame,
        # 100 mm jump must exceed the 50 mm single-step limit.
        _target(left=_arm_target(100.0, 0.0, 0.0), right=_arm_target(100.0, 0.0, 0.0)),
        calibration,
        now_ns=1_100_000_100,
    )

    assert decision_jump.state == SafetyState.PAUSED
    assert decision_jump.allow_motion is False
    assert decision_jump.left_reason == "target_jump"
    assert decision_jump.right_reason == "target_jump"


def test_velocity_limit_enters_paused_and_blocks_motion() -> None:
    gate = TargetSafetyGate(SafetyConfig(max_single_step_mm=1_000.0, max_velocity_mm_s=200.0))
    frame = _teleop_frame(pc_receive_time_ns=1_000_000_000)
    calibration = _calibration()

    decision_ok = gate.evaluate(
        frame,
        _target(left=_arm_target(0.0, 0.0, 0.0), right=_arm_target(0.0, 0.0, 0.0)),
        calibration,
        now_ns=1_000_000_100,
    )
    assert decision_ok.allow_motion is True

    decision_fast = gate.evaluate(
        frame,
        # 60 mm in 0.05 s = 1200 mm/s, above 200 mm/s limit.
        _target(left=_arm_target(60.0, 0.0, 0.0), right=_arm_target(60.0, 0.0, 0.0)),
        calibration,
        now_ns=1_050_000_100,
    )

    assert decision_fast.state == SafetyState.PAUSED
    assert decision_fast.allow_motion is False
    assert decision_fast.left_reason == "velocity_limit"
    assert decision_fast.right_reason == "velocity_limit"


def test_emergency_stop_blocks_until_cleared() -> None:
    gate = TargetSafetyGate()
    frame = _teleop_frame(pc_receive_time_ns=1_000_000_000)
    target = _target(left=_arm_target(0.01, 0.0, 0.0), right=_arm_target(0.01, 0.0, 0.0))

    gate.trigger_emergency_stop("operator_estop")
    blocked = gate.evaluate(frame, target, _calibration(), now_ns=1_000_000_100)

    assert blocked.state == SafetyState.EMERGENCY_STOP
    assert blocked.allow_motion is False
    assert blocked.global_reason == "operator_estop"

    gate.clear_emergency_stop()
    recovered = gate.evaluate(frame, target, _calibration(), now_ns=1_100_000_100)

    assert recovered.state == SafetyState.TELEOP_ACTIVE
    assert recovered.allow_motion is True