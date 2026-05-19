from __future__ import annotations

import math
from typing import Sequence

import numpy as np

from teleop.core.robot_frame import DualArmRobotFeedback, DualArmRobotTarget, RobotArmTarget
from teleop.core.teleop_frame import TeleopArmInput, TeleopFrame
from teleop.transform.calibration import ArmCalibrationAnchor, DualArmCalibrationState, IDENTITY_MATRIX_3X3


MatrixLike = Sequence[Sequence[float]]
VectorLike = Sequence[float]

DEFAULT_RIGHT_AXIS_MATRIX_FROM_USER: tuple[tuple[float, float, float], ...] = (
    (0.0, 0.0, 1.0),
    (0.0, 1.0, 0.0),
    (1.0, 0.0, 0.0),
)

DEFAULT_LEFT_AXIS_MATRIX_FROM_USER: tuple[tuple[float, float, float], ...] = (
    (0.0, 0.0, 1.0),
    (0.0, -1.0, 0.0),
    (-1.0, 0.0, 0.0),
)


class PositionOnlyCoordinateTransformer:
    """
    Position-only dual-arm transform.

    Policy for missing calibration:
    - If an arm is not calibrated, that arm target is returned as None.
    """

    def __init__(
        self,
        r_user_from_pico: MatrixLike = IDENTITY_MATRIX_3X3,
        left_axis_matrix_from_user: MatrixLike = DEFAULT_LEFT_AXIS_MATRIX_FROM_USER,
        right_axis_matrix_from_user: MatrixLike = DEFAULT_RIGHT_AXIS_MATRIX_FROM_USER,
        left_scale: float = 1.0,
        right_scale: float = 1.0,
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

        return ArmCalibrationAnchor(
            pico_anchor_xyz=(float(pose.x), float(pose.y), float(pose.z)),
            robot_anchor_xyz=robot_xyz,
            robot_anchor_abc=robot_abc,
            source_frame_id=int(teleop_frame.frame_id),
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
        delta_robot = float(scale) * (axis_matrix_from_user @ delta_user)

        target_xyz = anchor_xyz + delta_robot

        return RobotArmTarget(
            position_xyz=_vector_to_tuple(target_xyz),
            orientation_abc=_vector_to_tuple(anchor_abc),
            valid=True,
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
]