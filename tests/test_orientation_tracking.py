from __future__ import annotations

import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from teleop.core.pose import Pose7
from teleop.core.robot_frame import DualArmRobotFeedback, RobotArmFeedback
from teleop.core.teleop_frame import TeleopArmInput, TeleopFrame
from teleop.transform.calibration import ArmCalibrationAnchor
from teleop.transform.coordinate_transform import PositionOnlyCoordinateTransformer, PositionOrientationCoordinateTransformer
from teleop.transform.orientation_transform import (
    OrientationTrackingConfig,
    RelativeOrientationTracker,
    T_L1_L,
    T_L1_R,
    T_PICO_TO_USERWORLD,
    T_W_TO_PICO,
    compute_absolute_arm_orientation_from_pico,
)


class _FakeOrientationConverter:
    """Test-only converter with a reversible rotvec-deg encoding for abc fields."""

    def abc_to_rotation_matrix(self, abc_deg: tuple[float, float, float]):
        rotvec = np.radians(np.asarray(abc_deg, dtype=float))
        return Rotation.from_rotvec(rotvec).as_matrix()

    def rotation_matrix_to_abc(self, rotmat):
        rotvec = Rotation.from_matrix(np.asarray(rotmat, dtype=float)).as_rotvec()
        values = np.degrees(rotvec)
        return (float(values[0]), float(values[1]), float(values[2]))


class _SpyOrientationConverter(_FakeOrientationConverter):
    def __init__(self) -> None:
        self.last_rotmat: np.ndarray | None = None

    def rotation_matrix_to_abc(self, rotmat):
        self.last_rotmat = np.asarray(rotmat, dtype=float)
        return (11.0, 22.0, 33.0)


def _pose_xyz_quat(x: float, y: float, z: float, quat_xyzw: tuple[float, float, float, float]) -> Pose7:
    return Pose7.from_tuple((x, y, z, quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3]))


def _quat_from_rotvec_deg(x_deg: float, y_deg: float, z_deg: float) -> tuple[float, float, float, float]:
    quat = Rotation.from_rotvec(np.radians(np.array([x_deg, y_deg, z_deg], dtype=float))).as_quat()
    return (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))


def _arm_input(pose: Pose7) -> TeleopArmInput:
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


def _frame(frame_id: int, left_pose: Pose7, right_pose: Pose7) -> TeleopFrame:
    return TeleopFrame(
        frame_id=frame_id,
        source_device_id="test",
        source_timestamp_ns=1_000_000_000,
        pc_receive_time_ns=1_000_000_000,
        left=_arm_input(left_pose),
        right=_arm_input(right_pose),
        start_pause_requested=False,
        cancel_requested=False,
        calibration_requested=False,
    )


def _feedback(
    left_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    right_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    left_abc: tuple[float, float, float] = (0.0, 0.0, 0.0),
    right_abc: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> DualArmRobotFeedback:
    return DualArmRobotFeedback(
        left=RobotArmFeedback(position_xyz=left_xyz, orientation_abc=left_abc, valid=True),
        right=RobotArmFeedback(position_xyz=right_xyz, orientation_abc=right_abc, valid=True),
    )


def _matrix_to_tuple(mat: np.ndarray) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    arr = np.asarray(mat, dtype=float)
    return (
        (float(arr[0, 0]), float(arr[0, 1]), float(arr[0, 2])),
        (float(arr[1, 0]), float(arr[1, 1]), float(arr[1, 2])),
        (float(arr[2, 0]), float(arr[2, 1]), float(arr[2, 2])),
    )


def _anchor(
    *,
    source_frame_id: int = 1,
    controller_ref_quat: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
    robot_anchor_abc: tuple[float, float, float] = (0.0, 0.0, 0.0),
    robot_anchor_rotmat: np.ndarray | None = None,
    controller_abs_rotmat: np.ndarray | None = None,
    orientation_offset_rotmat: np.ndarray | None = None,
) -> ArmCalibrationAnchor:
    if robot_anchor_rotmat is None:
        robot_anchor_rotmat = np.eye(3, dtype=float)

    return ArmCalibrationAnchor(
        pico_anchor_xyz=(0.0, 0.0, 0.0),
        robot_anchor_xyz=(0.0, 0.0, 0.0),
        robot_anchor_abc=robot_anchor_abc,
        source_frame_id=source_frame_id,
        controller_anchor_quat_xyzw=controller_ref_quat,
        robot_anchor_rotmat=_matrix_to_tuple(robot_anchor_rotmat),
        controller_abs_orientation_rotmat=(
            None if controller_abs_rotmat is None else _matrix_to_tuple(controller_abs_rotmat)
        ),
        orientation_offset_rotmat=(
            None if orientation_offset_rotmat is None else _matrix_to_tuple(orientation_offset_rotmat)
        ),
    )


def _assert_so3(matrix: np.ndarray) -> None:
    arr = np.asarray(matrix, dtype=float)
    assert arr.shape == (3, 3)
    assert np.allclose(arr.T @ arr, np.eye(3), atol=1e-6)
    assert np.linalg.det(arr) == pytest.approx(1.0, abs=1e-6)


def test_position_only_preserves_frozen_orientation_with_quaternion_change() -> None:
    transformer = PositionOnlyCoordinateTransformer()

    ref_pose = _pose_xyz_quat(0.0, 0.0, 0.0, _quat_from_rotvec_deg(0.0, 0.0, 0.0))
    moved_pose = _pose_xyz_quat(0.0, 0.0, 0.1, _quat_from_rotvec_deg(0.0, 0.0, 20.0))

    calibration = transformer.create_calibration(
        _frame(1, ref_pose, ref_pose),
        _feedback(right_xyz=(1000.0, 2000.0, 3000.0), right_abc=(10.0, 20.0, 30.0)),
        side="right",
    )

    target = transformer.make_target(
        teleop_frame=_frame(2, moved_pose, moved_pose),
        robot_feedback=None,
        calibration=calibration,
    )

    assert target.right is not None
    assert target.right.valid is True
    assert target.right.orientation_abc == pytest.approx((10.0, 20.0, 30.0))


def test_position_orientation_mode_changes_orientation() -> None:
    cfg = OrientationTrackingConfig(
        enabled=True,
        orientation_algorithm="relative_rotvec",
        rotation_scale=1.0,
        max_total_angle_deg=45.0,
        max_step_angle_deg=45.0,
    )
    converter = _FakeOrientationConverter()
    transformer = PositionOrientationCoordinateTransformer(
        orientation_config=cfg,
        orientation_converters_by_side={"left": converter, "right": converter},
    )

    ref_pose = _pose_xyz_quat(0.0, 0.0, 0.0, _quat_from_rotvec_deg(0.0, 0.0, 0.0))
    moved_pose = _pose_xyz_quat(0.0, 0.0, 0.1, _quat_from_rotvec_deg(0.0, 0.0, 10.0))

    calibration = transformer.create_calibration(
        _frame(1, ref_pose, ref_pose),
        _feedback(right_xyz=(1000.0, 2000.0, 3000.0), right_abc=(0.0, 0.0, 0.0)),
        side="right",
    )

    target = transformer.make_target(
        teleop_frame=_frame(2, moved_pose, moved_pose),
        robot_feedback=None,
        calibration=calibration,
    )

    assert target.right is not None
    assert target.right.valid is True
    assert target.right.orientation_abc != pytest.approx((0.0, 0.0, 0.0))


def test_rotvec_mapping_b_matrix() -> None:
    cfg = OrientationTrackingConfig(enabled=True)
    mapping = np.asarray(cfg.arm_config_for_robot_side("right").rotvec_mapping, dtype=float)

    assert tuple(mapping @ np.array([1.0, 0.0, 0.0])) == pytest.approx((0.0, 0.0, 1.0))
    assert tuple(mapping @ np.array([0.0, 1.0, 0.0])) == pytest.approx((0.0, -1.0, 0.0))
    assert tuple(mapping @ np.array([0.0, 0.0, 1.0])) == pytest.approx((1.0, 0.0, 0.0))


def test_rotvec_mapping_a_matrix() -> None:
    cfg = OrientationTrackingConfig(enabled=True)
    mapping = np.asarray(cfg.arm_config_for_robot_side("left").rotvec_mapping, dtype=float)

    assert tuple(mapping @ np.array([1.0, 0.0, 0.0])) == pytest.approx((0.0, 0.0, 1.0))
    assert tuple(mapping @ np.array([0.0, 1.0, 0.0])) == pytest.approx((0.0, -1.0, 0.0))
    assert tuple(mapping @ np.array([0.0, 0.0, 1.0])) == pytest.approx((-1.0, 0.0, 0.0))


def test_rotation_scale_halves_relative_angle() -> None:
    tracker = RelativeOrientationTracker(
        config=OrientationTrackingConfig(
            enabled=True,
            orientation_algorithm="relative_rotvec",
            rotation_scale=0.5,
            max_total_angle_deg=180.0,
            max_step_angle_deg=180.0,
        ),
        converters_by_side={"right": _FakeOrientationConverter()},
    )

    result = tracker.compute_for_side(
        robot_side="right",
        teleop_left=None,
        teleop_right=_pose_xyz_quat(0.0, 0.0, 0.0, _quat_from_rotvec_deg(0.0, 0.0, 20.0)),
        anchor=_anchor(),
    )

    assert result.success is True
    assert result.relative_angle_deg == pytest.approx(10.0, abs=0.5)


def test_max_total_angle_is_clamped() -> None:
    tracker = RelativeOrientationTracker(
        config=OrientationTrackingConfig(
            enabled=True,
            orientation_algorithm="relative_rotvec",
            rotation_scale=1.0,
            max_total_angle_deg=25.0,
            max_step_angle_deg=180.0,
        ),
        converters_by_side={"right": _FakeOrientationConverter()},
    )

    result = tracker.compute_for_side(
        robot_side="right",
        teleop_left=None,
        teleop_right=_pose_xyz_quat(0.0, 0.0, 0.0, _quat_from_rotvec_deg(0.0, 0.0, 90.0)),
        anchor=_anchor(),
    )

    assert result.success is True
    assert result.relative_angle_deg == pytest.approx(25.0, abs=0.5)


def test_max_step_angle_limits_consecutive_updates() -> None:
    tracker = RelativeOrientationTracker(
        config=OrientationTrackingConfig(
            enabled=True,
            orientation_algorithm="relative_rotvec",
            rotation_scale=1.0,
            max_total_angle_deg=180.0,
            max_step_angle_deg=2.0,
        ),
        converters_by_side={"right": _FakeOrientationConverter()},
    )

    _ = tracker.compute_for_side(
        robot_side="right",
        teleop_left=None,
        teleop_right=_pose_xyz_quat(0.0, 0.0, 0.0, _quat_from_rotvec_deg(0.0, 0.0, 0.0)),
        anchor=_anchor(),
    )
    second = tracker.compute_for_side(
        robot_side="right",
        teleop_left=None,
        teleop_right=_pose_xyz_quat(0.0, 0.0, 0.0, _quat_from_rotvec_deg(0.0, 0.0, 10.0)),
        anchor=_anchor(),
    )
    third = tracker.compute_for_side(
        robot_side="right",
        teleop_left=None,
        teleop_right=_pose_xyz_quat(0.0, 0.0, 0.0, _quat_from_rotvec_deg(0.0, 0.0, 10.0)),
        anchor=_anchor(),
    )

    assert second.success is True
    assert third.success is True
    assert second.relative_angle_deg is not None
    assert third.relative_angle_deg is not None
    assert second.relative_angle_deg <= 2.1
    assert third.relative_angle_deg <= 4.1


def test_relative_quaternion_path_avoids_large_wraparound_error() -> None:
    tracker = RelativeOrientationTracker(
        config=OrientationTrackingConfig(
            enabled=True,
            orientation_algorithm="relative_rotvec",
            relative_mode="world",
            rotation_scale=1.0,
            max_total_angle_deg=180.0,
            max_step_angle_deg=180.0,
        ),
        converters_by_side={"right": _FakeOrientationConverter()},
    )

    ref_quat = _quat_from_rotvec_deg(0.0, 0.0, 179.0)
    now_quat = _quat_from_rotvec_deg(0.0, 0.0, -179.0)

    result = tracker.compute_for_side(
        robot_side="right",
        teleop_left=None,
        teleop_right=_pose_xyz_quat(0.0, 0.0, 0.0, now_quat),
        anchor=_anchor(controller_ref_quat=ref_quat),
    )

    assert result.success is True
    assert result.relative_angle_deg is not None
    assert result.relative_angle_deg < 5.0
    assert math.isfinite(result.relative_angle_deg)


def test_orientation_algorithm_validation_rejects_unknown_value() -> None:
    with pytest.raises(ValueError):
        OrientationTrackingConfig(enabled=True, orientation_algorithm="unknown")


def test_absolute_fixed_matrices_are_valid_so3() -> None:
    _assert_so3(T_L1_L)
    _assert_so3(T_L1_R)
    _assert_so3(T_W_TO_PICO)
    _assert_so3(T_PICO_TO_USERWORLD)


def test_compute_absolute_orientation_is_so3_for_identity_quaternion() -> None:
    left = compute_absolute_arm_orientation_from_pico("left", (0.0, 0.0, 0.0, 1.0))
    right = compute_absolute_arm_orientation_from_pico("right", (0.0, 0.0, 0.0, 1.0))

    _assert_so3(left)
    _assert_so3(right)


def test_calibration_offset_identity_when_anchor_matches_absolute() -> None:
    q_anchor = _quat_from_rotvec_deg(10.0, -5.0, 15.0)
    abs_anchor = compute_absolute_arm_orientation_from_pico("right", q_anchor)

    offset = abs_anchor @ abs_anchor.T
    _assert_so3(offset)
    assert np.allclose(offset, np.eye(3), atol=1e-6)


def test_calibration_offset_composes_to_robot_anchor() -> None:
    q_anchor = _quat_from_rotvec_deg(5.0, 20.0, -10.0)
    abs_anchor = compute_absolute_arm_orientation_from_pico("left", q_anchor)
    robot_anchor = Rotation.from_rotvec(np.radians(np.array([20.0, -10.0, 5.0], dtype=float))).as_matrix()

    offset = robot_anchor @ abs_anchor.T
    reconstructed_robot_anchor = offset @ abs_anchor

    _assert_so3(offset)
    assert np.allclose(reconstructed_robot_anchor, robot_anchor, atol=1e-6)


def test_absolute_matrix_mode_is_continuous_immediately_after_calibration() -> None:
    converter = _FakeOrientationConverter()
    tracker = RelativeOrientationTracker(
        config=OrientationTrackingConfig(
            enabled=True,
            orientation_algorithm="absolute_matrix",
            use_calibration_offset=True,
            max_total_angle_deg=180.0,
            max_step_angle_deg=180.0,
        ),
        converters_by_side={"right": converter},
    )

    q_anchor = _quat_from_rotvec_deg(0.0, 0.0, 12.0)
    abs_anchor = compute_absolute_arm_orientation_from_pico("right", q_anchor)
    robot_anchor = Rotation.from_rotvec(np.radians(np.array([15.0, -8.0, 6.0], dtype=float))).as_matrix()
    offset = robot_anchor @ abs_anchor.T

    anchor = _anchor(
        source_frame_id=1,
        controller_ref_quat=q_anchor,
        robot_anchor_abc=converter.rotation_matrix_to_abc(robot_anchor),
        robot_anchor_rotmat=robot_anchor,
        controller_abs_rotmat=abs_anchor,
        orientation_offset_rotmat=offset,
    )

    result = tracker.compute_for_side(
        robot_side="right",
        teleop_left=None,
        teleop_right=_pose_xyz_quat(0.0, 0.0, 0.0, q_anchor),
        anchor=anchor,
    )

    assert result.success is True
    assert result.reason == "ok_absolute_matrix"
    assert result.orientation_abc_deg == pytest.approx(converter.rotation_matrix_to_abc(robot_anchor), abs=1e-6)
    assert result.relative_angle_deg == pytest.approx(0.0, abs=1e-6)


def test_absolute_matrix_mode_recalibration_stays_continuous() -> None:
    converter = _FakeOrientationConverter()
    tracker = RelativeOrientationTracker(
        config=OrientationTrackingConfig(
            enabled=True,
            orientation_algorithm="absolute_matrix",
            use_calibration_offset=True,
            max_total_angle_deg=180.0,
            max_step_angle_deg=180.0,
        ),
        converters_by_side={"right": converter},
    )

    q_anchor_1 = _quat_from_rotvec_deg(0.0, 0.0, 10.0)
    abs_anchor_1 = compute_absolute_arm_orientation_from_pico("right", q_anchor_1)
    robot_anchor_1 = Rotation.from_rotvec(np.radians(np.array([8.0, -4.0, 2.0], dtype=float))).as_matrix()
    offset_1 = robot_anchor_1 @ abs_anchor_1.T
    anchor_1 = _anchor(
        source_frame_id=1,
        controller_ref_quat=q_anchor_1,
        robot_anchor_abc=converter.rotation_matrix_to_abc(robot_anchor_1),
        robot_anchor_rotmat=robot_anchor_1,
        controller_abs_rotmat=abs_anchor_1,
        orientation_offset_rotmat=offset_1,
    )

    first = tracker.compute_for_side(
        robot_side="right",
        teleop_left=None,
        teleop_right=_pose_xyz_quat(0.0, 0.0, 0.0, q_anchor_1),
        anchor=anchor_1,
    )

    q_anchor_2 = _quat_from_rotvec_deg(-5.0, 15.0, 3.0)
    abs_anchor_2 = compute_absolute_arm_orientation_from_pico("right", q_anchor_2)
    robot_anchor_2 = Rotation.from_rotvec(np.radians(np.array([2.0, 10.0, -6.0], dtype=float))).as_matrix()
    offset_2 = robot_anchor_2 @ abs_anchor_2.T
    anchor_2 = _anchor(
        source_frame_id=2,
        controller_ref_quat=q_anchor_2,
        robot_anchor_abc=converter.rotation_matrix_to_abc(robot_anchor_2),
        robot_anchor_rotmat=robot_anchor_2,
        controller_abs_rotmat=abs_anchor_2,
        orientation_offset_rotmat=offset_2,
    )

    second = tracker.compute_for_side(
        robot_side="right",
        teleop_left=None,
        teleop_right=_pose_xyz_quat(0.0, 0.0, 0.0, q_anchor_2),
        anchor=anchor_2,
    )

    assert first.success is True
    assert second.success is True
    assert first.orientation_abc_deg == pytest.approx(converter.rotation_matrix_to_abc(robot_anchor_1), abs=1e-6)
    assert second.orientation_abc_deg == pytest.approx(converter.rotation_matrix_to_abc(robot_anchor_2), abs=1e-6)


def test_sdk_matrix_to_abc_receives_absolute_target_matrix() -> None:
    converter = _SpyOrientationConverter()
    tracker = RelativeOrientationTracker(
        config=OrientationTrackingConfig(
            enabled=True,
            orientation_algorithm="absolute_matrix",
            use_calibration_offset=False,
            max_total_angle_deg=180.0,
            max_step_angle_deg=180.0,
        ),
        converters_by_side={"right": converter},
    )

    q_now = _quat_from_rotvec_deg(2.0, -3.0, 7.0)
    anchor = _anchor(
        source_frame_id=1,
        controller_ref_quat=(0.0, 0.0, 0.0, 1.0),
        robot_anchor_abc=(0.0, 0.0, 0.0),
        robot_anchor_rotmat=np.eye(3, dtype=float),
    )

    result = tracker.compute_for_side(
        robot_side="right",
        teleop_left=None,
        teleop_right=_pose_xyz_quat(0.0, 0.0, 0.0, q_now),
        anchor=anchor,
    )

    assert result.success is True
    assert result.orientation_abc_deg == pytest.approx((11.0, 22.0, 33.0))
    assert converter.last_rotmat is not None

    expected_target = compute_absolute_arm_orientation_from_pico("right", q_now)
    assert np.allclose(converter.last_rotmat, expected_target, atol=1e-6)
