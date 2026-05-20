from __future__ import annotations

import importlib
import logging
from pathlib import Path
import sys
from typing import Sequence


def sdk_arm_to_index(arm: str) -> int:
    arm_norm = arm.strip().upper()
    if arm_norm == "A":
        return 0
    if arm_norm == "B":
        return 1
    raise ValueError("arm must be 'A' or 'B'")


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _ensure_python_sdk_on_path() -> None:
    sdk_dir = _repo_root() / "python_sdk"
    sdk_dir_str = str(sdk_dir)
    if sdk_dir_str not in sys.path:
        sys.path.insert(0, sdk_dir_str)


def _import_marvin_kine_class():
    _ensure_python_sdk_on_path()
    module = importlib.import_module("fx_kine")
    if not hasattr(module, "Marvin_Kine"):
        raise RuntimeError("fx_kine.Marvin_Kine not found")
    return module.Marvin_Kine


def _quiet_fx_kine_python_logger() -> None:
    # fx_kine uses logger name "debug_printer" and INFO logs are too noisy for runtime loops.
    logging.getLogger("debug_printer").setLevel(logging.ERROR)


class ArmKinematicsAdapter:
    """SDK-backed FK adapter for one arm."""

    def __init__(self, arm: str, kine_cfg_path: str, disable_kine_logs: bool = True):
        self._arm = arm.strip().upper()
        self._arm_index = sdk_arm_to_index(self._arm)
        self._kine_cfg_path = str(kine_cfg_path)
        self._disable_kine_logs = bool(disable_kine_logs)
        self._kine = None
        self._initialized = False

    @property
    def is_initialized(self) -> bool:
        return self._initialized

    def initialize(self) -> None:
        MarvinKine = _import_marvin_kine_class()

        if self._disable_kine_logs:
            _quiet_fx_kine_python_logger()

        kine = MarvinKine()

        if self._disable_kine_logs and hasattr(kine, "log_switch"):
            try:
                kine.log_switch(0)
            except Exception:
                # Best effort only; some wrappers/platforms may not expose this API reliably.
                pass

        cfg_path = Path(self._kine_cfg_path)
        if not cfg_path.is_absolute():
            cfg_path = _repo_root() / cfg_path

        cfg_data = kine.load_config(self._arm_index, str(cfg_path))
        if not cfg_data:
            raise RuntimeError(f"Failed to load kinematics config for arm {self._arm}: {cfg_path}")

        try:
            robot_type = int(cfg_data["TYPE"][self._arm_index])
            dh = cfg_data["DH"][self._arm_index]
            pnva = cfg_data["PNVA"][self._arm_index]
            j67 = cfg_data["BD"][self._arm_index]
        except Exception as exc:
            raise RuntimeError(f"Invalid kinematics config data for arm {self._arm}") from exc

        ok = kine.initial_kine(robot_type=robot_type, dh=dh, pnva=pnva, j67=j67)
        if not ok:
            raise RuntimeError(f"Failed to initialize kinematics for arm {self._arm}")

        self._kine = kine
        self._initialized = True

    def fk_xyzabc_mm_deg(self, q_deg: Sequence[float]) -> tuple[float, float, float, float, float, float]:
        if not self._initialized or self._kine is None:
            raise RuntimeError("Kinematics adapter is not initialized")

        q = tuple(float(v) for v in q_deg)
        if len(q) != 7:
            raise ValueError("q_deg must have length 7")

        fk_mat = self._kine.fk(list(q))
        if not fk_mat:
            raise RuntimeError("FK failed")

        xyzabc = self._kine.mat4x4_to_xyzabc(fk_mat)
        if not xyzabc:
            raise RuntimeError("mat4x4_to_xyzabc failed")
        if len(xyzabc) < 6:
            raise RuntimeError("FK output xyzabc must have at least 6 elements")

        return (
            float(xyzabc[0]),
            float(xyzabc[1]),
            float(xyzabc[2]),
            float(xyzabc[3]),
            float(xyzabc[4]),
            float(xyzabc[5]),
        )


__all__ = [
    "sdk_arm_to_index",
    "ArmKinematicsAdapter",
]
