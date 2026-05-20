import pytest

from teleop.core.pose import Pose7
from teleop.core.robot_frame import DualArmRobotFeedback, RobotArmFeedback
from teleop.core.teleop_frame import TeleopArmInput, TeleopFrame
from teleop.transform.calibration import DualArmCalibrationState, detect_axis_click_calibration_request
from teleop.transform.coordinate_transform import PositionOnlyCoordinateTransformer


def _pose_xyz(x: float, y: float, z: float) -> Pose7:
    return Pose7.from_tuple((x, y, z, 0.0, 0.0, 0.0, 1.0))


def _arm_input(
    *,
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    valid: bool = True,
    axis_click: bool = False,
) -> TeleopArmInput:
    pose = _pose_xyz(*xyz) if valid else None
    return TeleopArmInput(
        pose_pico=pose,
        valid=valid,
        enable=True,
        gripper_position=0.5,
        gripper_changed=False,
        trigger=0.5,
        grip=0.9,
        axis_x=0.0,
        axis_y=0.0,
        axis_click=axis_click,
    )


def _teleop_frame(
    *,
    frame_id: int = 1,
    left_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    right_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    left_valid: bool = True,
    right_valid: bool = True,
    left_axis_click: bool = False,
    right_axis_click: bool = False,
) -> TeleopFrame:
    return TeleopFrame(
        frame_id=frame_id,
        source_device_id="pico_test",
        source_timestamp_ns=100,
        pc_receive_time_ns=200,
        left=_arm_input(xyz=left_xyz, valid=left_valid, axis_click=left_axis_click),
        right=_arm_input(xyz=right_xyz, valid=right_valid, axis_click=right_axis_click),
        start_pause_requested=False,
        cancel_requested=False,
        calibration_requested=False,
    )


def _robot_feedback(
    *,
    left_xyz: tuple[float, float, float] | None = None,
    right_xyz: tuple[float, float, float] | None = None,
    left_abc: tuple[float, float, float] = (10.0, 20.0, 30.0),
    right_abc: tuple[float, float, float] = (10.0, 20.0, 30.0),
    left_valid: bool = True,
    right_valid: bool = True,
) -> DualArmRobotFeedback:
    left = None
    right = None

    if left_xyz is not None:
        left = RobotArmFeedback(position_xyz=left_xyz, orientation_abc=left_abc, valid=left_valid)
    if right_xyz is not None:
        right = RobotArmFeedback(position_xyz=right_xyz, orientation_abc=right_abc, valid=right_valid)

    return DualArmRobotFeedback(left=left, right=right)


def test_calibration_stores_anchors_for_both_arms() -> None:
    transformer = PositionOnlyCoordinateTransformer()
    teleop = _teleop_frame(
        frame_id=42,
        left_xyz=(0.1, 0.2, 0.3),
        right_xyz=(-0.1, -0.2, -0.3),
    )
    feedback = _robot_feedback(
        left_xyz=(1000.0, 2000.0, 3000.0),
        right_xyz=(4000.0, 5000.0, 6000.0),
        left_abc=(7.0, 8.0, 9.0),
        right_abc=(10.0, 11.0, 12.0),
    )

    calibration = transformer.create_calibration(teleop, feedback, side=None)

    assert calibration.left is not None
    assert calibration.right is not None

    assert calibration.left.pico_anchor_xyz == pytest.approx((0.1, 0.2, 0.3))
    assert calibration.left.robot_anchor_xyz == pytest.approx((1000.0, 2000.0, 3000.0))
    assert calibration.left.robot_anchor_abc == pytest.approx((7.0, 8.0, 9.0))
    assert calibration.left.source_frame_id == 42

    assert calibration.right.pico_anchor_xyz == pytest.approx((-0.1, -0.2, -0.3))
    assert calibration.right.robot_anchor_xyz == pytest.approx((4000.0, 5000.0, 6000.0))
    assert calibration.right.robot_anchor_abc == pytest.approx((10.0, 11.0, 12.0))
    assert calibration.right.source_frame_id == 42


def test_right_arm_forward_motion_decreases_robot_x() -> None:
    transformer = PositionOnlyCoordinateTransformer()
    calibration = transformer.create_calibration(
        _teleop_frame(right_xyz=(0.0, 0.0, 0.0)),
        _robot_feedback(right_xyz=(1000.0, 2000.0, 3000.0), right_abc=(10.0, 20.0, 30.0)),
        side="right",
    )

    moved = _teleop_frame(right_xyz=(0.0, 0.0, 0.1))
    target = transformer.make_target(moved, None, calibration)

    assert target.right is not None
    assert target.right.valid is True
    assert target.right.position_xyz == pytest.approx((900.0, 2000.0, 3000.0))
    assert target.right.orientation_abc == pytest.approx((10.0, 20.0, 30.0))


def test_right_arm_user_up_increases_robot_y() -> None:
    transformer = PositionOnlyCoordinateTransformer()
    calibration = transformer.create_calibration(
        _teleop_frame(right_xyz=(0.0, 0.0, 0.0)),
        _robot_feedback(right_xyz=(1000.0, 2000.0, 3000.0)),
        side="right",
    )

    moved = _teleop_frame(right_xyz=(0.0, 0.1, 0.0))
    target = transformer.make_target(moved, None, calibration)

    assert target.right is not None
    assert target.right.position_xyz == pytest.approx((1000.0, 2100.0, 3000.0))


def test_right_arm_user_right_increases_robot_z() -> None:
    transformer = PositionOnlyCoordinateTransformer()
    calibration = transformer.create_calibration(
        _teleop_frame(right_xyz=(0.0, 0.0, 0.0)),
        _robot_feedback(right_xyz=(1000.0, 2000.0, 3000.0)),
        side="right",
    )

    moved = _teleop_frame(right_xyz=(0.1, 0.0, 0.0))
    target = transformer.make_target(moved, None, calibration)

    assert target.right is not None
    assert target.right.position_xyz == pytest.approx((1000.0, 2000.0, 3100.0))


def test_left_arm_axis_mapping_and_frozen_orientation() -> None:
    transformer = PositionOnlyCoordinateTransformer()
    calibration = transformer.create_calibration(
        _teleop_frame(left_xyz=(0.0, 0.0, 0.0)),
        _robot_feedback(left_xyz=(1000.0, 2000.0, 3000.0), left_abc=(10.0, 20.0, 30.0)),
        side="left",
    )

    forward_target = transformer.make_target(_teleop_frame(left_xyz=(0.0, 0.0, 0.1)), None, calibration)
    up_target = transformer.make_target(_teleop_frame(left_xyz=(0.0, 0.1, 0.0)), None, calibration)
    right_target = transformer.make_target(_teleop_frame(left_xyz=(0.1, 0.0, 0.0)), None, calibration)

    assert forward_target.left is not None
    assert up_target.left is not None
    assert right_target.left is not None

    assert forward_target.left.position_xyz == pytest.approx((900.0, 2000.0, 3000.0))
    assert up_target.left.position_xyz == pytest.approx((1000.0, 1900.0, 3000.0))
    assert right_target.left.position_xyz == pytest.approx((1000.0, 2000.0, 2900.0))

    assert forward_target.left.orientation_abc == pytest.approx((10.0, 20.0, 30.0))
    assert up_target.left.orientation_abc == pytest.approx((10.0, 20.0, 30.0))
    assert right_target.left.orientation_abc == pytest.approx((10.0, 20.0, 30.0))


def test_right_arm_scale_applied() -> None:
    transformer = PositionOnlyCoordinateTransformer(right_scale=2.0)
    calibration = transformer.create_calibration(
        _teleop_frame(right_xyz=(0.0, 0.0, 0.0)),
        _robot_feedback(right_xyz=(1000.0, 2000.0, 3000.0)),
        side="right",
    )

    moved = _teleop_frame(right_xyz=(0.0, 0.0, 0.1))
    target = transformer.make_target(moved, None, calibration)

    assert target.right is not None
    assert target.right.position_xyz == pytest.approx((800.0, 2000.0, 3000.0))


def test_custom_r_user_from_pico_is_used_before_arm_mapping() -> None:
    r_user_from_pico = (
        (0.0, 0.0, 1.0),
        (0.0, 1.0, 0.0),
        (1.0, 0.0, 0.0),
    )
    transformer = PositionOnlyCoordinateTransformer(r_user_from_pico=r_user_from_pico)

    calibration = transformer.create_calibration(
        _teleop_frame(right_xyz=(0.0, 0.0, 0.0)),
        _robot_feedback(right_xyz=(1000.0, 2000.0, 3000.0)),
        side="right",
    )

    moved = _teleop_frame(right_xyz=(0.1, 0.0, 0.0))
    target = transformer.make_target(moved, None, calibration)

    assert target.right is not None
    assert target.right.position_xyz == pytest.approx((900.0, 2000.0, 3000.0))


def test_make_target_without_calibration_returns_none_targets() -> None:
    transformer = PositionOnlyCoordinateTransformer()
    calibration = DualArmCalibrationState()

    target = transformer.make_target(_teleop_frame(), None, calibration)

    assert target.left is None
    assert target.right is None


def test_invalid_pico_pose_returns_invalid_target() -> None:
    transformer = PositionOnlyCoordinateTransformer()
    calibration = transformer.create_calibration(
        _teleop_frame(right_xyz=(0.0, 0.0, 0.0)),
        _robot_feedback(right_xyz=(1000.0, 2000.0, 3000.0), right_abc=(10.0, 20.0, 30.0)),
        side="right",
    )

    invalid_frame = _teleop_frame(right_valid=False)
    target = transformer.make_target(invalid_frame, None, calibration)

    assert target.right is not None
    assert target.right.valid is False
    assert target.right.reason == "invalid_pico_pose"
    assert target.right.orientation_abc == pytest.approx((10.0, 20.0, 30.0))


def test_axis_click_rising_edge_detection_left_right_and_any() -> None:
    prev = _teleop_frame(left_axis_click=False, right_axis_click=False)
    left_rise = _teleop_frame(left_axis_click=True, right_axis_click=False)
    left_held = _teleop_frame(left_axis_click=True, right_axis_click=False)
    left_idle = _teleop_frame(left_axis_click=False, right_axis_click=False)
    right_rise = _teleop_frame(left_axis_click=False, right_axis_click=True)

    assert detect_axis_click_calibration_request(prev, left_rise, side="left") is True
    assert detect_axis_click_calibration_request(prev, left_rise, side="right") is False
    assert detect_axis_click_calibration_request(prev, left_rise, side=None) is True

    assert detect_axis_click_calibration_request(left_rise, left_held, side="left") is False
    assert detect_axis_click_calibration_request(prev, left_idle, side="left") is False

    assert detect_axis_click_calibration_request(prev, right_rise, side="right") is True
    assert detect_axis_click_calibration_request(prev, right_rise, side=None) is True