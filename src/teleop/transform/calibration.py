from __future__ import annotations

from dataclasses import dataclass
import math

from teleop.core.teleop_frame import TeleopFrame


Mat3Tuple = tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]

IDENTITY_MATRIX_3X3: Mat3Tuple = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)


@dataclass(frozen=True)
class ArmCalibrationAnchor:
    """Calibration anchor for one arm."""

    pico_anchor_xyz: tuple[float, float, float]
    robot_anchor_xyz: tuple[float, float, float]
    robot_anchor_abc: tuple[float, float, float]
    source_frame_id: int


@dataclass(frozen=True)
class DualArmCalibrationState:
    """Calibration state for position-only dual-arm teleoperation."""

    left: ArmCalibrationAnchor | None = None
    right: ArmCalibrationAnchor | None = None
    r_user_from_pico: tuple[tuple[float, float, float], ...] = IDENTITY_MATRIX_3X3
    left_scale: float = 1.0
    right_scale: float = 1.0

    def __post_init__(self) -> None:
        if not _is_valid_matrix_3x3(self.r_user_from_pico):
            raise ValueError("r_user_from_pico must be a finite 3x3 matrix")
        if not math.isfinite(float(self.left_scale)):
            raise ValueError("left_scale must be finite")
        if not math.isfinite(float(self.right_scale)):
            raise ValueError("right_scale must be finite")

    @property
    def left_calibrated(self) -> bool:
        return self.left is not None

    @property
    def right_calibrated(self) -> bool:
        return self.right is not None

    def is_calibrated(self, side: str) -> bool:
        side_key = _normalize_side(side)
        if side_key == "left":
            return self.left_calibrated
        return self.right_calibrated


def detect_axis_click_calibration_request(
    previous: TeleopFrame | None,
    current: TeleopFrame,
    side: str | None = None,
) -> bool:
    """Detect axisClick rising edge for calibration request intent."""
    if side is None:
        return _axis_click_rising_edge(previous, current, "left") or _axis_click_rising_edge(previous, current, "right")

    side_key = _normalize_side(side)
    return _axis_click_rising_edge(previous, current, side_key)


def _axis_click_rising_edge(previous: TeleopFrame | None, current: TeleopFrame, side: str) -> bool:
    prev_pressed = _axis_click_for_side(previous, side) if previous is not None else False
    curr_pressed = _axis_click_for_side(current, side)
    return (not prev_pressed) and curr_pressed


def _axis_click_for_side(frame: TeleopFrame, side: str) -> bool:
    return bool(getattr(frame, side).axis_click)


def _normalize_side(side: str) -> str:
    side_key = side.strip().lower()
    if side_key not in {"left", "right"}:
        raise ValueError("side must be 'left' or 'right'")
    return side_key


def _is_valid_matrix_3x3(matrix: tuple[tuple[float, float, float], ...]) -> bool:
    if len(matrix) != 3:
        return False
    for row in matrix:
        if len(row) != 3:
            return False
        for value in row:
            if not math.isfinite(float(value)):
                return False
    return True


__all__ = [
    "Mat3Tuple",
    "IDENTITY_MATRIX_3X3",
    "ArmCalibrationAnchor",
    "DualArmCalibrationState",
    "detect_axis_click_calibration_request",
]