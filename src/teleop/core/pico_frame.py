from __future__ import annotations

from dataclasses import dataclass

from teleop.core.pose import Pose7


@dataclass(frozen=True)
class PicoControllerState:
    pose: Pose7
    trigger: float
    grip: float
    axis_x: float
    axis_y: float
    axis_click: bool
    primary_button: bool
    secondary_button: bool
    menu_button: bool


@dataclass(frozen=True)
class PicoRawFrame:
    frame_id: int
    device_id: str
    pico_timestamp_ns: int
    pc_receive_time_ns: int
    head_pose: Pose7
    left_ctrl: PicoControllerState
    right_ctrl: PicoControllerState

    @property
    def left_valid(self) -> bool:
        return self.left_ctrl.pose.is_valid()

    @property
    def right_valid(self) -> bool:
        return self.right_ctrl.pose.is_valid()
