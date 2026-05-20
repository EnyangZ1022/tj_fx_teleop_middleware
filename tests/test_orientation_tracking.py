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
from teleop.transform.orientation_transform import OrientationTrackingConfig, RelativeOrientationTracker


class _FakeOrientationConverter:
    """Test-only converter with a reversible rotvec-deg encoding for abc fields."""

    def abc_to_rotation_matrix(self, abc_deg: tuple[float, float, float]):
        rotvec = np.radians(np.asarray(abc_deg, dtype=float))
        return Rotation.from_rotvec(rotvec).as_matrix()

    def rotation_matrix_to_abc(self, rotmat):
        rotvec = Rotation.from_matrix(np.asarray(rotmat, dtype=float)).as_rotvec()
        values = np.degrees(rotvec)
        return (float(values[0]), float(values[1]), float(values[2]))


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


def _anchor(
    source_frame_id: int = 1,
    controller_ref_quat: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> ArmCalibrationAnchor:
    return ArmCalibrationAnchor(
        pico_anchor_xyz=(0.0, 0.0, 0.0),
        robot_anchor_xyz=(0.0, 0.0, 0.0),
        robot_anchor_abc=(0.0, 0.0, 0.0),
        source_frame_id=source_frame_id,
        controller_anchor_quat_xyzw=controller_ref_quat,
        robot_anchor_rotmat=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
    )


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
