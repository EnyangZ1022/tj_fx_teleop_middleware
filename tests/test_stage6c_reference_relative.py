import pytest

from teleop.core.pose import Pose7
from teleop.core.robot_frame import DualArmRobotFeedback, RobotArmFeedback
from teleop.core.teleop_frame import TeleopArmInput, TeleopFrame
from teleop.diagnostics.integration_checks import check_reference_relative_target
from teleop.transform.coordinate_transform import (
    DEFAULT_RIGHT_AXIS_MATRIX_FROM_USER,
    PositionOnlyCoordinateTransformer,
)


def _arm_input(xyz_m: tuple[float, float, float]) -> TeleopArmInput:
    pose = Pose7.from_tuple((xyz_m[0], xyz_m[1], xyz_m[2], 0.0, 0.0, 0.0, 1.0))
    return TeleopArmInput(
        pose_pico=pose,
        valid=True,
        enable=True,
        gripper_position=0.5,
        gripper_changed=False,
        trigger=0.1,
        grip=0.9,
        axis_x=0.0,
        axis_y=0.0,
        axis_click=False,
    )


def _frame(frame_id: int, xyz_m: tuple[float, float, float]) -> TeleopFrame:
    return TeleopFrame(
        frame_id=frame_id,
        source_device_id="test",
        source_timestamp_ns=1_000_000_000,
        pc_receive_time_ns=1_000_000_000,
        left=_arm_input((0.0, 0.0, 0.0)),
        right=_arm_input(xyz_m),
        start_pause_requested=False,
        cancel_requested=False,
        calibration_requested=False,
    )


def test_reference_relative_target_is_not_cumulative() -> None:
    pico_ref = (0.0, 0.0, 0.0)
    pico_now = (0.0, 0.0, 0.1)
    robot_ref_mm = (1000.0, 2000.0, 3000.0)

    target_1 = check_reference_relative_target(
        pico_ref_m=pico_ref,
        pico_now_m=pico_now,
        robot_ref_mm=robot_ref_mm,
        axis_matrix=DEFAULT_RIGHT_AXIS_MATRIX_FROM_USER,
        scale=1.0,
    )
    target_2 = check_reference_relative_target(
        pico_ref_m=pico_ref,
        pico_now_m=pico_now,
        robot_ref_mm=robot_ref_mm,
        axis_matrix=DEFAULT_RIGHT_AXIS_MATRIX_FROM_USER,
        scale=1.0,
    )

    assert target_1 == pytest.approx((1100.0, 2000.0, 3000.0))
    assert target_2 == pytest.approx((1100.0, 2000.0, 3000.0))


def test_orientation_is_frozen_in_position_only_mode() -> None:
    transformer = PositionOnlyCoordinateTransformer()

    calibration_frame = _frame(1, (0.0, 0.0, 0.0))
    feedback = DualArmRobotFeedback(
        left=RobotArmFeedback(position_xyz=(500.0, 1000.0, 1500.0), orientation_abc=(1.0, 2.0, 3.0), valid=True),
        right=RobotArmFeedback(position_xyz=(1000.0, 2000.0, 3000.0), orientation_abc=(10.0, 20.0, 30.0), valid=True),
    )

    calibration = transformer.create_calibration(calibration_frame, feedback, side="right")

    moved_frame = _frame(2, (0.0, 0.0, 0.1))
    target = transformer.make_target(moved_frame, feedback, calibration)

    assert target.right is not None
    assert target.right.position_xyz == pytest.approx((1100.0, 2000.0, 3000.0))
    assert target.right.orientation_abc == pytest.approx((10.0, 20.0, 30.0))
