from __future__ import annotations

from enum import Enum


class TeleopMode(str, Enum):
    POSITION_ONLY = "position_only"
    POSITION_ORIENTATION = "position_orientation"


_ALLOWED_TELEOP_MODES = {mode.value for mode in TeleopMode}


def normalize_teleop_mode(value: str | TeleopMode) -> str:
    mode = str(value).strip().lower()
    if mode not in _ALLOWED_TELEOP_MODES:
        raise ValueError(
            f"teleop_mode must be one of {sorted(_ALLOWED_TELEOP_MODES)}, got {value!r}"
        )
    return mode


__all__ = ["TeleopMode", "normalize_teleop_mode"]
