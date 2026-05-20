from __future__ import annotations

from dataclasses import dataclass
import math


_ALLOWED_IK_MODES = {"fixed_reference_only", "zsp_negative_z"}


@dataclass(frozen=True)
class IKSolverConfig:
    mode: str = "zsp_negative_z"
    enable_zsp: bool = True
    zsp_type: int = 1
    zsp_para: tuple[float, ...] = (0.0, 0.0, -1.0, 0.0, 0.0, 0.0)
    keep_fixed_reference: bool = True

    def __post_init__(self) -> None:
        mode = str(self.mode).strip().lower()
        if mode not in _ALLOWED_IK_MODES:
            raise ValueError(f"mode must be one of {_ALLOWED_IK_MODES}, got {self.mode!r}")
        object.__setattr__(self, "mode", mode)

        zsp_values = tuple(float(v) for v in self.zsp_para)
        if len(zsp_values) != 6:
            raise ValueError("zsp_para must contain exactly 6 values")
        if not all(math.isfinite(v) for v in zsp_values):
            raise ValueError("zsp_para must contain only finite values")
        object.__setattr__(self, "zsp_para", zsp_values)

        # fixed_reference_only mode always disables ZSP path.
        if mode == "fixed_reference_only":
            object.__setattr__(self, "enable_zsp", False)

    def use_zsp(self) -> bool:
        return bool(self.enable_zsp and self.mode == "zsp_negative_z")


__all__ = ["IKSolverConfig"]
