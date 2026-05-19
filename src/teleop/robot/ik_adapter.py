from __future__ import annotations

import importlib
from pathlib import Path
import sys
from typing import Sequence

from teleop.robot.kinematics_adapter import ArmKinematicsAdapter


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _ensure_python_sdk_on_path() -> None:
    sdk_dir = _repo_root() / "python_sdk"
    sdk_dir_str = str(sdk_dir)
    if sdk_dir_str not in sys.path:
        sys.path.insert(0, sdk_dir_str)


def _import_fx_inv_kine_solve_para_class():
    _ensure_python_sdk_on_path()
    module = importlib.import_module("fx_kine")
    if not hasattr(module, "FX_InvKineSolvePara"):
        raise RuntimeError("fx_kine.FX_InvKineSolvePara not found")
    return module.FX_InvKineSolvePara


def _to_vec3(name: str, values: tuple[float, float, float] | Sequence[float]) -> tuple[float, float, float]:
    output = tuple(float(v) for v in values)
    if len(output) != 3:
        raise ValueError(f"{name} must have length 3")
    return (output[0], output[1], output[2])


def _to_vec7(name: str, values: Sequence[float]) -> tuple[float, float, float, float, float, float, float]:
    output = tuple(float(v) for v in values)
    if len(output) != 7:
        raise ValueError(f"{name} must have length 7")
    return (
        output[0],
        output[1],
        output[2],
        output[3],
        output[4],
        output[5],
        output[6],
    )


class ArmIKAdapter:
    """IK adapter for one arm using the SDK kinematics object."""

    def __init__(self, kinematics_adapter: ArmKinematicsAdapter):
        self._kinematics_adapter = kinematics_adapter

    def solve_xyzabc_mm_deg(
        self,
        position_xyz_mm: tuple[float, float, float],
        orientation_abc_deg: tuple[float, float, float],
        ik_reference_q_deg: Sequence[float],
    ) -> tuple[float, float, float, float, float, float, float] | None:
        xyz = _to_vec3("position_xyz_mm", position_xyz_mm)
        abc = _to_vec3("orientation_abc_deg", orientation_abc_deg)
        ref_q = _to_vec7("ik_reference_q_deg", ik_reference_q_deg)

        if not self._kinematics_adapter.is_initialized:
            return None

        kine = getattr(self._kinematics_adapter, "_kine", None)
        if kine is None:
            return None

        try:
            FXInvKineSolvePara = _import_fx_inv_kine_solve_para_class()

            pose_mm_deg = [xyz[0], xyz[1], xyz[2], abc[0], abc[1], abc[2]]
            pose_mat = kine.xyzabc_to_mat4x4(pose_mm_deg)
            if not pose_mat:
                return None

            pose_1x16 = kine.mat4x4_to_mat1x16(pose_mat)
            if not pose_1x16 or len(pose_1x16) != 16:
                return None

            sp = FXInvKineSolvePara()
            sp.set_input_ik_target_tcp(pose_1x16)
            sp.set_input_ik_ref_joint(ref_q)

            ik_result = kine.ik(structure_data=sp)
            if not ik_result:
                return None

            if not hasattr(ik_result, "m_Output_RetJoint"):
                return None
            q = ik_result.m_Output_RetJoint.to_list()
            if len(q) != 7:
                return None

            return _to_vec7("ik_result_q_deg", q)
        except Exception:
            return None


__all__ = ["ArmIKAdapter"]
