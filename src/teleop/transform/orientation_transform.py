from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from teleop.core.pose import Pose7
from teleop.transform.calibration import ArmCalibrationAnchor


Mat3Tuple = tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]


DEFAULT_ARM_A_ROTVEC_MAPPING: Mat3Tuple = (
    (0.0, 0.0, -1.0),
    (0.0, -1.0, 0.0),
    (1.0, 0.0, 0.0),
)

DEFAULT_ARM_B_ROTVEC_MAPPING: Mat3Tuple = (
    (0.0, 0.0, 1.0),
    (0.0, -1.0, 0.0),
    (1.0, 0.0, 0.0),
)


@dataclass(frozen=True)
class ArmOrientationConfig:
    controller_side: str
    rotvec_mapping: Mat3Tuple

    def __post_init__(self) -> None:
        side = str(self.controller_side).strip().lower()
        if side not in {"left", "right"}:
            raise ValueError("controller_side must be 'left' or 'right'")
        object.__setattr__(self, "controller_side", side)

        arr = _as_matrix3x3(self.rotvec_mapping, "rotvec_mapping")
        object.__setattr__(self, "rotvec_mapping", _matrix_to_tuple(arr))


@dataclass(frozen=True)
class OrientationTrackingConfig:
    enabled: bool = False
    relative_mode: str = "world"
    rotation_scale: float = 0.4
    max_total_angle_deg: float = 25.0
    max_step_angle_deg: float = 2.0
    arm_a: ArmOrientationConfig = field(
        default_factory=lambda: ArmOrientationConfig(
            controller_side="left",
            rotvec_mapping=DEFAULT_ARM_A_ROTVEC_MAPPING,
        )
    )
    arm_b: ArmOrientationConfig = field(
        default_factory=lambda: ArmOrientationConfig(
            controller_side="right",
            rotvec_mapping=DEFAULT_ARM_B_ROTVEC_MAPPING,
        )
    )

    def __post_init__(self) -> None:
        mode = str(self.relative_mode).strip().lower()
        if mode not in {"world", "local"}:
            raise ValueError("relative_mode must be 'world' or 'local'")
        object.__setattr__(self, "relative_mode", mode)

        if not math.isfinite(float(self.rotation_scale)):
            raise ValueError("rotation_scale must be finite")
        if float(self.rotation_scale) < 0.0:
            raise ValueError("rotation_scale must be >= 0")

        if not math.isfinite(float(self.max_total_angle_deg)) or float(self.max_total_angle_deg) <= 0.0:
            raise ValueError("max_total_angle_deg must be positive finite")
        if not math.isfinite(float(self.max_step_angle_deg)) or float(self.max_step_angle_deg) <= 0.0:
            raise ValueError("max_step_angle_deg must be positive finite")

        if not isinstance(self.arm_a, ArmOrientationConfig):
            raise ValueError("arm_a must be ArmOrientationConfig")
        if not isinstance(self.arm_b, ArmOrientationConfig):
            raise ValueError("arm_b must be ArmOrientationConfig")

    def arm_config_for_robot_side(self, side: str) -> ArmOrientationConfig:
        side_norm = str(side).strip().lower()
        if side_norm == "left":
            return self.arm_a
        if side_norm == "right":
            return self.arm_b
        raise ValueError("side must be 'left' or 'right'")


@dataclass(frozen=True)
class OrientationTrackingResult:
    success: bool
    orientation_abc_deg: tuple[float, float, float] | None = None
    relative_angle_deg: float | None = None
    reason: str = ""


class SDKOrientationConverter:
    """Use SDK kinematics helper conversions to avoid guessing Euler order."""

    def __init__(self, kine: Any):
        self._kine = kine

    def abc_to_rotation_matrix(self, abc_deg: tuple[float, float, float]) -> np.ndarray | None:
        if self._kine is None:
            return None

        pose_mat = self._kine.xyzabc_to_mat4x4([0.0, 0.0, 0.0, abc_deg[0], abc_deg[1], abc_deg[2]])
        if not pose_mat:
            return None

        arr = np.asarray(pose_mat, dtype=float)
        if arr.shape != (4, 4):
            return None
        return arr[:3, :3]

    def rotation_matrix_to_abc(self, rotmat: np.ndarray) -> tuple[float, float, float] | None:
        if self._kine is None:
            return None

        arr = np.asarray(rotmat, dtype=float)
        if arr.shape != (3, 3):
            return None

        pose = np.eye(4, dtype=float)
        pose[:3, :3] = arr

        xyzabc = self._kine.mat4x4_to_xyzabc(pose.tolist())
        if not xyzabc or len(xyzabc) < 6:
            return None

        return (float(xyzabc[3]), float(xyzabc[4]), float(xyzabc[5]))


class RelativeOrientationTracker:
    """Compute per-arm tracked orientation from relative controller quaternion."""

    def __init__(
        self,
        config: OrientationTrackingConfig,
        converters_by_side: dict[str, Any] | None = None,
    ):
        self._config = config
        self._converters_by_side: dict[str, Any] = {}
        self.set_converters(converters_by_side)

        self._last_rotvec_by_side: dict[str, np.ndarray] = {}
        self._last_anchor_frame_by_side: dict[str, int] = {}
        self._last_relative_angle_deg_by_side: dict[str, float | None] = {
            "left": None,
            "right": None,
        }

    def set_converters(self, converters_by_side: dict[str, Any] | None) -> None:
        self._converters_by_side = {}
        if converters_by_side is None:
            return

        for side, converter in converters_by_side.items():
            side_norm = str(side).strip().lower()
            if side_norm in {"left", "right"} and converter is not None:
                self._converters_by_side[side_norm] = converter

    def reset(self) -> None:
        self._last_rotvec_by_side.clear()
        self._last_anchor_frame_by_side.clear()
        self._last_relative_angle_deg_by_side = {
            "left": None,
            "right": None,
        }

    def latest_relative_angle_deg(self, side: str) -> float | None:
        side_norm = str(side).strip().lower()
        return self._last_relative_angle_deg_by_side.get(side_norm)

    def compute_for_side(
        self,
        *,
        robot_side: str,
        teleop_left: Pose7 | None,
        teleop_right: Pose7 | None,
        anchor: ArmCalibrationAnchor,
    ) -> OrientationTrackingResult:
        side = str(robot_side).strip().lower()
        if side not in {"left", "right"}:
            return OrientationTrackingResult(False, reason="invalid_side")

        if not self._config.enabled:
            return OrientationTrackingResult(
                success=True,
                orientation_abc_deg=anchor.robot_anchor_abc,
                relative_angle_deg=0.0,
                reason="disabled",
            )

        arm_cfg = self._config.arm_config_for_robot_side(side)
        controller_pose = teleop_left if arm_cfg.controller_side == "left" else teleop_right
        if controller_pose is None:
            return OrientationTrackingResult(False, reason="controller_pose_missing")

        if anchor.controller_anchor_quat_xyzw is None:
            return OrientationTrackingResult(False, reason="controller_anchor_quaternion_missing")

        if self._last_anchor_frame_by_side.get(side) != int(anchor.source_frame_id):
            self._last_anchor_frame_by_side[side] = int(anchor.source_frame_id)
            self._last_rotvec_by_side.pop(side, None)

        try:
            q_ref = _normalize_quaternion_xyzw(anchor.controller_anchor_quat_xyzw)
            q_now = _normalize_quaternion_xyzw((controller_pose.qx, controller_pose.qy, controller_pose.qz, controller_pose.qw))

            r_ref = Rotation.from_quat(q_ref)
            r_now = Rotation.from_quat(q_now)
            if self._config.relative_mode == "world":
                r_delta = r_now * r_ref.inv()
            else:
                r_delta = r_ref.inv() * r_now

            ctrl_rotvec = np.asarray(r_delta.as_rotvec(), dtype=float)
            mapping = _as_matrix3x3(arm_cfg.rotvec_mapping, f"{side}.rotvec_mapping")

            robot_rotvec = float(self._config.rotation_scale) * (mapping @ ctrl_rotvec)
            robot_rotvec = _clamp_total_angle(
                robot_rotvec,
                max_total_deg=float(self._config.max_total_angle_deg),
            )

            prev_rotvec = self._last_rotvec_by_side.get(side)
            robot_rotvec = _limit_step_angle(
                previous=prev_rotvec,
                desired=robot_rotvec,
                max_step_deg=float(self._config.max_step_angle_deg),
            )

            relative_angle_deg = float(np.linalg.norm(robot_rotvec) * 180.0 / math.pi)

            converter = self._converters_by_side.get(side)
            if anchor.robot_anchor_rotmat is not None:
                ref_mat = _as_matrix3x3(anchor.robot_anchor_rotmat, f"anchor.{side}.robot_anchor_rotmat")
            elif converter is not None:
                ref_mat = converter.abc_to_rotation_matrix(anchor.robot_anchor_abc)
                if ref_mat is None:
                    return OrientationTrackingResult(False, reason="robot_reference_matrix_failed")
            else:
                return OrientationTrackingResult(False, reason="orientation_converter_missing")

            r_target = Rotation.from_matrix(ref_mat) * Rotation.from_rotvec(robot_rotvec)

            if converter is None:
                return OrientationTrackingResult(False, reason="orientation_converter_missing")

            abc = converter.rotation_matrix_to_abc(r_target.as_matrix())
            if abc is None:
                return OrientationTrackingResult(False, reason="target_abc_conversion_failed")

            self._last_rotvec_by_side[side] = robot_rotvec
            self._last_relative_angle_deg_by_side[side] = relative_angle_deg

            return OrientationTrackingResult(
                success=True,
                orientation_abc_deg=abc,
                relative_angle_deg=relative_angle_deg,
                reason="ok",
            )
        except Exception as exc:
            return OrientationTrackingResult(False, reason=f"orientation_exception:{exc.__class__.__name__}")


def _normalize_quaternion_xyzw(values: tuple[float, float, float, float]) -> np.ndarray:
    q = np.asarray(values, dtype=float)
    if q.shape != (4,):
        raise ValueError("quaternion must have shape (4,)")
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or norm <= 1e-9:
        raise ValueError("quaternion norm must be > 0")
    return q / norm


def _as_matrix3x3(matrix: Any, name: str) -> np.ndarray:
    arr = np.asarray(matrix, dtype=float)
    if arr.shape != (3, 3):
        raise ValueError(f"{name} must be a 3x3 matrix")
    if not np.isfinite(arr).all():
        raise ValueError(f"{name} must contain finite values")
    return arr


def _matrix_to_tuple(matrix: np.ndarray) -> Mat3Tuple:
    return (
        (float(matrix[0, 0]), float(matrix[0, 1]), float(matrix[0, 2])),
        (float(matrix[1, 0]), float(matrix[1, 1]), float(matrix[1, 2])),
        (float(matrix[2, 0]), float(matrix[2, 1]), float(matrix[2, 2])),
    )


def _clamp_total_angle(rotvec: np.ndarray, max_total_deg: float) -> np.ndarray:
    max_total_rad = float(max_total_deg) * math.pi / 180.0
    angle = float(np.linalg.norm(rotvec))
    if angle <= max_total_rad or angle <= 1e-12:
        return rotvec
    return rotvec * (max_total_rad / angle)


def _limit_step_angle(previous: np.ndarray | None, desired: np.ndarray, max_step_deg: float) -> np.ndarray:
    if previous is None:
        return desired

    max_step_rad = float(max_step_deg) * math.pi / 180.0
    if max_step_rad <= 0.0:
        return desired

    r_prev = Rotation.from_rotvec(previous)
    r_des = Rotation.from_rotvec(desired)

    r_delta = r_prev.inv() * r_des
    delta_rotvec = np.asarray(r_delta.as_rotvec(), dtype=float)
    delta_angle = float(np.linalg.norm(delta_rotvec))

    if delta_angle <= max_step_rad or delta_angle <= 1e-12:
        return desired

    limited_delta = delta_rotvec * (max_step_rad / delta_angle)
    r_limited = r_prev * Rotation.from_rotvec(limited_delta)
    return np.asarray(r_limited.as_rotvec(), dtype=float)


__all__ = [
    "Mat3Tuple",
    "DEFAULT_ARM_A_ROTVEC_MAPPING",
    "DEFAULT_ARM_B_ROTVEC_MAPPING",
    "ArmOrientationConfig",
    "OrientationTrackingConfig",
    "OrientationTrackingResult",
    "SDKOrientationConverter",
    "RelativeOrientationTracker",
]
