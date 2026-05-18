from __future__ import annotations

from dataclasses import dataclass

from teleop.core.pose import Pose7


@dataclass(frozen=True)
class TeleopArmInput:
    pose_pico: Pose7 | None
    valid: bool
    enable: bool
    gripper_position: float
    gripper_changed: bool
    trigger: float
    grip: float
    axis_x: float
    axis_y: float
    axis_click: bool


@dataclass(frozen=True)
class TeleopFrame:
    frame_id: int
    source_device_id: str
    source_timestamp_ns: int
    pc_receive_time_ns: int
    left: TeleopArmInput
    right: TeleopArmInput
    # Stage 2 keeps these as level-triggered button requests for simplicity.
    start_pause_requested: bool
    cancel_requested: bool
    calibration_requested: bool
