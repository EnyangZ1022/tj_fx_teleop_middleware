from __future__ import annotations

import math
from typing import Any, Sequence

import numpy as np

from teleop.core.robot_frame import DualArmRobotFeedback, DualArmRobotTarget, RobotArmTarget
from teleop.core.teleop_frame import TeleopArmInput, TeleopFrame
from teleop.core.units import position_m_to_mm
from teleop.transform.calibration import ArmCalibrationAnchor, DualArmCalibrationState, IDENTITY_MATRIX_3X3
from teleop.transform.orientation_transform import OrientationTrackingConfig, RelativeOrientationTracker


MatrixLike = Sequence[Sequence[float]]
VectorLike = Sequence[float]

DEFAULT_RIGHT_AXIS_MATRIX_FROM_USER: tuple[tuple[float, float, float], ...] = (
    (0.0, 0.0, -1.0),
    (0.0, 1.0, 0.0),
    (1.0, 0.0, 0.0),
)

DEFAULT_LEFT_AXIS_MATRIX_FROM_USER: tuple[tuple[float, float, float], ...] = (
    (0.0, 0.0, -1.0),
    (0.0, -1.0, 0.0),
    (-1.0, 0.0, 0.0),
)


class PositionOnlyCoordinateTransformer:
    """
    Position-only dual-arm transform.

    Policy for missing calibration:
    - If an arm is not calibrated, that arm target is returned as None.

    Units:
    - Pico pose and delta_pico are in meters.
    - Robot anchors and robot targets are in millimeters.
    - Robot orientation is in degrees and remains frozen at calibration.
    """

    def __init__(
        self,
        r_user_from_pico: MatrixLike = IDENTITY_MATRIX_3X3,
        left_axis_matrix_from_user: MatrixLike = DEFAULT_LEFT_AXIS_MATRIX_FROM_USER,
        right_axis_matrix_from_user: MatrixLike = DEFAULT_RIGHT_AXIS_MATRIX_FROM_USER,
        left_scale: float = 1.0,
        right_scale: float = 1.0,
        orientation_converters_by_side: dict[str, Any] | None = None,
    ):
        self._r_user_from_pico = _as_matrix3x3(r_user_from_pico, "r_user_from_pico")
        self._left_axis_matrix_from_user = _as_matrix3x3(
            left_axis_matrix_from_user,
            "left_axis_matrix_from_user",
        )
        self._right_axis_matrix_from_user = _as_matrix3x3(
            right_axis_matrix_from_user,
            "right_axis_matrix_from_user",
        )
        self._left_scale = _validate_scale(left_scale, "left_scale")
        self._right_scale = _validate_scale(right_scale, "right_scale")
        self._orientation_converters_by_side: dict[str, Any] = {}
        self.set_orientation_converters(orientation_converters_by_side)

    def set_orientation_converters(self, converters_by_side: dict[str, Any] | None) -> None:
        self._orientation_converters_by_side = {}
        if converters_by_side is None:
            return

        for side, converter in converters_by_side.items():
            side_norm = str(side).strip().lower()
            if side_norm in {"left", "right"} and converter is not None:
                self._orientation_converters_by_side[side_norm] = converter

    def create_calibration(
        self,
        teleop_frame: TeleopFrame,
        robot_feedback: DualArmRobotFeedback,
        side: str | None = None,
    ) -> DualArmCalibrationState:
        requested_sides = _requested_sides(side)

        left_anchor = self._maybe_create_anchor(teleop_frame, robot_feedback, "left") if "left" in requested_sides else None
        right_anchor = (
            self._maybe_create_anchor(teleop_frame, robot_feedback, "right") if "right" in requested_sides else None
        )

        return DualArmCalibrationState(
            left=left_anchor,
            right=right_anchor,
            r_user_from_pico=_matrix_to_tuple(self._r_user_from_pico),
            left_scale=self._left_scale,
            right_scale=self._right_scale,
        )

    def update_calibration(
        self,
        existing: DualArmCalibrationState,
        teleop_frame: TeleopFrame,
        robot_feedback: DualArmRobotFeedback,
        side: str | None = None,
    ) -> DualArmCalibrationState:
        requested_sides = _requested_sides(side)

        left_anchor = existing.left
        right_anchor = existing.right

        if "left" in requested_sides:
            maybe_left = self._maybe_create_anchor(teleop_frame, robot_feedback, "left")
            if maybe_left is not None:
                left_anchor = maybe_left

        if "right" in requested_sides:
            maybe_right = self._maybe_create_anchor(teleop_frame, robot_feedback, "right")
            if maybe_right is not None:
                right_anchor = maybe_right

        r_user_from_pico = _matrix_to_tuple(_as_matrix3x3(existing.r_user_from_pico, "existing.r_user_from_pico"))
        left_scale = _validate_scale(existing.left_scale, "existing.left_scale")
        right_scale = _validate_scale(existing.right_scale, "existing.right_scale")

        return DualArmCalibrationState(
            left=left_anchor,
            right=right_anchor,
            r_user_from_pico=r_user_from_pico,
            left_scale=left_scale,
            right_scale=right_scale,
        )

    def make_target(
        self,
        teleop_frame: TeleopFrame,
        robot_feedback: DualArmRobotFeedback | None,
        calibration: DualArmCalibrationState,
    ) -> DualArmRobotTarget:
        # Stage 3 target generation is anchor-based and does not depend on live robot feedback.
        _ = robot_feedback

        r_user_from_pico = _as_matrix3x3(calibration.r_user_from_pico, "calibration.r_user_from_pico")
        left_scale = _validate_scale(calibration.left_scale, "calibration.left_scale")
        right_scale = _validate_scale(calibration.right_scale, "calibration.right_scale")

        left_target = self._make_target_for_side(
            side="left",
            arm_input=teleop_frame.left,
            anchor=calibration.left,
            r_user_from_pico=r_user_from_pico,
            axis_matrix_from_user=self._left_axis_matrix_from_user,
            scale=left_scale,
        )
        right_target = self._make_target_for_side(
            side="right",
            arm_input=teleop_frame.right,
            anchor=calibration.right,
            r_user_from_pico=r_user_from_pico,
            axis_matrix_from_user=self._right_axis_matrix_from_user,
            scale=right_scale,
        )

        return DualArmRobotTarget(left=left_target, right=right_target)

    def _maybe_create_anchor(
        self,
        teleop_frame: TeleopFrame,
        robot_feedback: DualArmRobotFeedback,
        side: str,
    ) -> ArmCalibrationAnchor | None:
        arm_input = teleop_frame.left if side == "left" else teleop_frame.right
        feedback = robot_feedback.left if side == "left" else robot_feedback.right

        if not arm_input.valid or arm_input.pose_pico is None:
            return None
        if feedback is None or not feedback.valid:
            return None

        pose = arm_input.pose_pico
        robot_xyz = _vector_to_tuple(_as_vector3(feedback.position_xyz, f"robot_feedback.{side}.position_xyz"))
        robot_abc = _vector_to_tuple(_as_vector3(feedback.orientation_abc, f"robot_feedback.{side}.orientation_abc"))
        controller_quat = (float(pose.qx), float(pose.qy), float(pose.qz), float(pose.qw))

        robot_rotmat = None
        converter = self._orientation_converters_by_side.get(side)
        if converter is not None and hasattr(converter, "abc_to_rotation_matrix"):
            try:
                mat = converter.abc_to_rotation_matrix(robot_abc)
                if mat is not None:
                    robot_rotmat = _matrix_to_tuple(_as_matrix3x3(mat, f"robot_feedback.{side}.rotation_matrix"))
            except Exception:
                robot_rotmat = None

        return ArmCalibrationAnchor(
            pico_anchor_xyz=(float(pose.x), float(pose.y), float(pose.z)),
            robot_anchor_xyz=robot_xyz,
            robot_anchor_abc=robot_abc,
            source_frame_id=int(teleop_frame.frame_id),
            controller_anchor_quat_xyzw=controller_quat,
            robot_anchor_rotmat=robot_rotmat,
        )

    def _make_target_for_side(
        self,
        side: str,
        arm_input: TeleopArmInput,
        anchor: ArmCalibrationAnchor | None,
        r_user_from_pico: np.ndarray,
        axis_matrix_from_user: np.ndarray,
        scale: float,
    ) -> RobotArmTarget | None:
        if anchor is None:
            return None

        anchor_xyz = _as_vector3(anchor.robot_anchor_xyz, f"anchor.{side}.robot_anchor_xyz")
        anchor_abc = _as_vector3(anchor.robot_anchor_abc, f"anchor.{side}.robot_anchor_abc")

        if not arm_input.valid or arm_input.pose_pico is None:
            return RobotArmTarget(
                position_xyz=_vector_to_tuple(anchor_xyz),
                orientation_abc=_vector_to_tuple(anchor_abc),
                valid=False,
                reason="invalid_pico_pose",
            )

        pose = arm_input.pose_pico
        curr_pico_xyz = np.array([pose.x, pose.y, pose.z], dtype=float)
        pico_anchor_xyz = _as_vector3(anchor.pico_anchor_xyz, f"anchor.{side}.pico_anchor_xyz")

        delta_pico = curr_pico_xyz - pico_anchor_xyz
        delta_user = r_user_from_pico @ delta_pico
        delta_robot_m = float(scale) * (axis_matrix_from_user @ delta_user)
        delta_robot_mm = np.array(position_m_to_mm(_vector_to_tuple(delta_robot_m)), dtype=float)

        target_xyz = anchor_xyz + delta_robot_mm

        return RobotArmTarget(
            position_xyz=_vector_to_tuple(target_xyz),
            orientation_abc=_vector_to_tuple(anchor_abc),
            valid=True,
        )


class PositionOrientationCoordinateTransformer(PositionOnlyCoordinateTransformer):
    """Optional experimental position+orientation transformer.

    Position behavior stays identical to PositionOnlyCoordinateTransformer.
    Orientation behavior applies relative quaternion tracking and keeps strict failure handling.
    """

    def __init__(
        self,
        r_user_from_pico: MatrixLike = IDENTITY_MATRIX_3X3,
        left_axis_matrix_from_user: MatrixLike = DEFAULT_LEFT_AXIS_MATRIX_FROM_USER,
        right_axis_matrix_from_user: MatrixLike = DEFAULT_RIGHT_AXIS_MATRIX_FROM_USER,
        left_scale: float = 1.0,
        right_scale: float = 1.0,
        orientation_config: OrientationTrackingConfig | None = None,
        orientation_converters_by_side: dict[str, Any] | None = None,
    ):
        super().__init__(
            r_user_from_pico=r_user_from_pico,
            left_axis_matrix_from_user=left_axis_matrix_from_user,
            right_axis_matrix_from_user=right_axis_matrix_from_user,
            left_scale=left_scale,
            right_scale=right_scale,
            orientation_converters_by_side=orientation_converters_by_side,
        )
        self._orientation_config = (
            orientation_config if orientation_config is not None else OrientationTrackingConfig(enabled=False)
        )
        self._orientation_tracker = RelativeOrientationTracker(
            config=self._orientation_config,
            converters_by_side=self._orientation_converters_by_side,
        )
        self._latest_orientation_debug: dict[str, dict[str, float | str | bool | None]] = {
            "left": {
                "enabled": bool(self._orientation_config.enabled),
                "reason": "idle",
                "relative_angle_deg": None,
            },
            "right": {
                "enabled": bool(self._orientation_config.enabled),
                "reason": "idle",
                "relative_angle_deg": None,
            },
        }

    def set_orientation_converters(self, converters_by_side: dict[str, Any] | None) -> None:
        super().set_orientation_converters(converters_by_side)
        if hasattr(self, "_orientation_tracker"):
            self._orientation_tracker.set_converters(self._orientation_converters_by_side)

    def latest_orientation_debug(self) -> dict[str, dict[str, float | str | bool | None]]:
        return {
            "left": dict(self._latest_orientation_debug["left"]),
            "right": dict(self._latest_orientation_debug["right"]),
        }

    def make_target(
        self,
        teleop_frame: TeleopFrame,
        robot_feedback: DualArmRobotFeedback | None,
        calibration: DualArmCalibrationState,
    ) -> DualArmRobotTarget:
        base = super().make_target(teleop_frame=teleop_frame, robot_feedback=robot_feedback, calibration=calibration)

        if not self._orientation_config.enabled:
            self._latest_orientation_debug["left"] = {
                "enabled": False,
                "reason": "disabled",
                "relative_angle_deg": 0.0,
            }
            self._latest_orientation_debug["right"] = {
                "enabled": False,
                "reason": "disabled",
                "relative_angle_deg": 0.0,
            }
            return base

        left = self._apply_orientation_for_side(
            robot_side="left",
            target=base.left,
            anchor=calibration.left,
            teleop_frame=teleop_frame,
        )
        right = self._apply_orientation_for_side(
            robot_side="right",
            target=base.right,
            anchor=calibration.right,
            teleop_frame=teleop_frame,
        )

        return DualArmRobotTarget(left=left, right=right)

    def _apply_orientation_for_side(
        self,
        *,
        robot_side: str,
        target: RobotArmTarget | None,
        anchor: ArmCalibrationAnchor | None,
        teleop_frame: TeleopFrame,
    ) -> RobotArmTarget | None:
        if target is None:
            self._latest_orientation_debug[robot_side] = {
                "enabled": True,
                "reason": "no_target",
                "relative_angle_deg": None,
            }
            return None

        if not target.valid:
            self._latest_orientation_debug[robot_side] = {
                "enabled": True,
                "reason": "target_invalid",
                "relative_angle_deg": None,
            }
            return target

        if anchor is None:
            self._latest_orientation_debug[robot_side] = {
                "enabled": True,
                "reason": "missing_calibration",
                "relative_angle_deg": None,
            }
            return RobotArmTarget(
                position_xyz=target.position_xyz,
                orientation_abc=target.orientation_abc,
                valid=False,
                reason="missing_calibration",
            )

        result = self._orientation_tracker.compute_for_side(
            robot_side=robot_side,
            teleop_left=teleop_frame.left.pose_pico,
            teleop_right=teleop_frame.right.pose_pico,
            anchor=anchor,
        )
        self._latest_orientation_debug[robot_side] = {
            "enabled": True,
            "reason": result.reason,
            "relative_angle_deg": result.relative_angle_deg,
        }

        if not result.success or result.orientation_abc_deg is None:
            return RobotArmTarget(
                position_xyz=target.position_xyz,
                orientation_abc=target.orientation_abc,
                valid=False,
                reason=f"orientation_transform_failed:{result.reason}",
            )

        return RobotArmTarget(
            position_xyz=target.position_xyz,
            orientation_abc=result.orientation_abc_deg,
            valid=True,
            reason=target.reason,
        )


def _requested_sides(side: str | None) -> tuple[str, ...]:
    if side is None:
        return ("left", "right")

    normalized = side.strip().lower()
    if normalized not in {"left", "right"}:
        raise ValueError("side must be 'left', 'right', or None")
    return (normalized,)


def _validate_scale(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _as_matrix3x3(matrix: MatrixLike, name: str) -> np.ndarray:
    arr = np.asarray(matrix, dtype=float)
    if arr.shape != (3, 3):
        raise ValueError(f"{name} must be a 3x3 matrix")
    if not np.isfinite(arr).all():
        raise ValueError(f"{name} must contain finite values")
    return arr


def _as_vector3(values: VectorLike, name: str) -> np.ndarray:
    arr = np.asarray(values, dtype=float)
    if arr.shape != (3,):
        raise ValueError(f"{name} must be a 3-element vector")
    if not np.isfinite(arr).all():
        raise ValueError(f"{name} must contain finite values")
    return arr


def _matrix_to_tuple(matrix: np.ndarray) -> tuple[tuple[float, float, float], ...]:
    return tuple(tuple(float(v) for v in row) for row in matrix.tolist())


def _vector_to_tuple(values: np.ndarray) -> tuple[float, float, float]:
    return tuple(float(v) for v in values.tolist())  # type: ignore[return-value]


__all__ = [
    "DEFAULT_RIGHT_AXIS_MATRIX_FROM_USER",
    "DEFAULT_LEFT_AXIS_MATRIX_FROM_USER",
    "PositionOnlyCoordinateTransformer",
    "PositionOrientationCoordinateTransformer",
]