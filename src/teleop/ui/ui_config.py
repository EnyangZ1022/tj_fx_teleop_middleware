from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class UIConfig:
    enabled: bool = False
    update_hz: float = 20.0
    window_title: str = "TJ-FX Teleop Diagnostic UI"

    show_3d_view: bool = True
    show_status_panel: bool = True
    show_error_lines: bool = True
    axis_length_mm: float = 100.0
    ball_size: float = 10.0
    history_length: int = 200
    coordinate_unit: str = "mm"

    def __post_init__(self) -> None:
        if float(self.update_hz) <= 0.0:
            raise ValueError("update_hz must be positive")
        if float(self.axis_length_mm) <= 0.0:
            raise ValueError("axis_length_mm must be positive")
        if float(self.ball_size) <= 0.0:
            raise ValueError("ball_size must be positive")
        if int(self.history_length) <= 0:
            raise ValueError("history_length must be positive")
        if not str(self.coordinate_unit).strip():
            raise ValueError("coordinate_unit must not be empty")

    def timer_interval_ms(self) -> int:
        return max(1, int(round(1000.0 / float(self.update_hz))))


__all__ = ["UIConfig"]
