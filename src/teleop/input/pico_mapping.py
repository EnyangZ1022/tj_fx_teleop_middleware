from __future__ import annotations

from teleop.core.pico_frame import PicoControllerState, PicoRawFrame
from teleop.core.teleop_frame import TeleopArmInput, TeleopFrame


class PicoInputMapper:
    """Map low-level PicoRawFrame input into TeleopFrame semantics."""

    _SUPPORTED_ANALOG_FIELDS = {"trigger", "grip", "axis_x", "axis_y"}

    def __init__(
        self,
        enable_field: str = "grip",
        enable_threshold: float = 0.8,
        gripper_field: str = "trigger",
        gripper_deadband: float = 0.01,
    ):
        if enable_field not in self._SUPPORTED_ANALOG_FIELDS:
            raise ValueError(f"Unsupported enable_field: {enable_field}")
        if gripper_field not in self._SUPPORTED_ANALOG_FIELDS:
            raise ValueError(f"Unsupported gripper_field: {gripper_field}")
        if gripper_deadband < 0.0:
            raise ValueError("gripper_deadband must be >= 0.0")

        self._enable_field = enable_field
        self._enable_threshold = float(enable_threshold)
        self._gripper_field = gripper_field
        self._gripper_deadband = float(gripper_deadband)
        self._last_gripper_position: dict[str, float | None] = {
            "left": None,
            "right": None,
        }

    def map_frame(self, frame: PicoRawFrame) -> TeleopFrame:
        left = self._map_arm("left", frame.left_ctrl, frame.left_valid)
        right = self._map_arm("right", frame.right_ctrl, frame.right_valid)

        # Stage 2 policy: each reserved button is exposed as a simple ORed request.
        start_pause_requested = bool(frame.left_ctrl.primary_button or frame.right_ctrl.primary_button)
        cancel_requested = bool(frame.left_ctrl.secondary_button or frame.right_ctrl.secondary_button)
        # Stage 9 policy: calibration request is handled by axisClick rising-edge detection
        # in orchestration using detect_axis_click_calibration_request(previous, current, side).
        calibration_requested = False

        return TeleopFrame(
            frame_id=frame.frame_id,
            source_device_id=frame.device_id,
            source_timestamp_ns=frame.pico_timestamp_ns,
            pc_receive_time_ns=frame.pc_receive_time_ns,
            left=left,
            right=right,
            start_pause_requested=start_pause_requested,
            cancel_requested=cancel_requested,
            calibration_requested=calibration_requested,
            receiver_seq=frame.receiver_seq,
        )

    def _map_arm(self, side: str, ctrl: PicoControllerState, valid: bool) -> TeleopArmInput:
        enable_value = self._read_analog_field(ctrl, self._enable_field)
        gripper_value = self._read_analog_field(ctrl, self._gripper_field)

        enable = enable_value > self._enable_threshold
        gripper_position = self._clamp(1.0 - gripper_value, 0.0, 1.0)

        prev = self._last_gripper_position[side]
        gripper_changed = prev is None or abs(gripper_position - prev) >= self._gripper_deadband
        if gripper_changed:
            self._last_gripper_position[side] = gripper_position

        return TeleopArmInput(
            pose_pico=ctrl.pose if valid else None,
            valid=valid,
            enable=enable,
            gripper_position=gripper_position,
            gripper_changed=gripper_changed,
            trigger=float(ctrl.trigger),
            grip=float(ctrl.grip),
            axis_x=float(ctrl.axis_x),
            axis_y=float(ctrl.axis_y),
            axis_click=bool(ctrl.axis_click),
        )

    @staticmethod
    def _read_analog_field(ctrl: PicoControllerState, field: str) -> float:
        return float(getattr(ctrl, field))

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        if value < low:
            return low
        if value > high:
            return high
        return value
