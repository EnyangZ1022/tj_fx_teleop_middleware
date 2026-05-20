from __future__ import annotations

from dataclasses import dataclass
import math


_ALLOWED_IK_MODES = {"fixed_reference_only", "zsp_negative_z"}


def _normalize_zsp_para(name: str, values: tuple[float, ...]) -> tuple[float, ...]:
    output = tuple(float(v) for v in values)
    if len(output) != 6:
        raise ValueError(f"{name} must contain exactly 6 values")
    if not all(math.isfinite(v) for v in output):
        raise ValueError(f"{name} must contain only finite values")
    return output


@dataclass(frozen=True)
class IKSolverConfig:
    mode: str = "zsp_negative_z"
    enable_zsp: bool = True
    zsp_type: int = 1
    zsp_para_left: tuple[float, ...] = (1.0, -1.0, -1.0, 0.0, 0.0, 0.0)
    zsp_para_right: tuple[float, ...] = (1.0, 1.0, -1.0, 0.0, 0.0, 0.0)
    keep_fixed_reference: bool = True

    def __post_init__(self) -> None:
        mode = str(self.mode).strip().lower()
        if mode not in _ALLOWED_IK_MODES:
            raise ValueError(f"mode must be one of {_ALLOWED_IK_MODES}, got {self.mode!r}")
        object.__setattr__(self, "mode", mode)

        zsp_values_left = _normalize_zsp_para("zsp_para_left", self.zsp_para_left)
        zsp_values_right = _normalize_zsp_para("zsp_para_right", self.zsp_para_right)
        object.__setattr__(self, "zsp_para_left", zsp_values_left)
        object.__setattr__(self, "zsp_para_right", zsp_values_right)

        # fixed_reference_only mode always disables ZSP path.
        if mode == "fixed_reference_only":
            object.__setattr__(self, "enable_zsp", False)

    def use_zsp(self) -> bool:
        return bool(self.enable_zsp and self.mode == "zsp_negative_z")

    def zsp_para_for_side(self, side: str) -> tuple[float, ...]:
        side_norm = str(side).strip().lower()
        if side_norm == "left":
            return self.zsp_para_left
        if side_norm == "right":
            return self.zsp_para_right
        raise ValueError("side must be 'left' or 'right'")


__all__ = ["IKSolverConfig"]
