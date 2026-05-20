from __future__ import annotations

import sys
import types

import pytest

from teleop.robot.ik_adapter import ArmIKAdapter
from teleop.robot.ik_config import IKSolverConfig


class _FakeRetJoint:
    def __init__(self, values):
        self._values = list(values)

    def to_list(self):
        return list(self._values)


class _FakeIKResult:
    def __init__(self, values):
        self.m_Output_RetJoint = _FakeRetJoint(values)


class _FakeKineSuccess:
    def __init__(self):
        self.last_structure_data = None

    def xyzabc_to_mat4x4(self, xyzabc):
        _ = xyzabc
        return [
            [1.0, 0.0, 0.0, 100.0],
            [0.0, 1.0, 0.0, 200.0],
            [0.0, 0.0, 1.0, 300.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def mat4x4_to_mat1x16(self, pose_mat):
        output = []
        for row in pose_mat:
            output.extend(row)
        return output

    def ik(self, structure_data):
        self.last_structure_data = structure_data
        return _FakeIKResult([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0])


class _FakeKineFailMatrix(_FakeKineSuccess):
    def xyzabc_to_mat4x4(self, xyzabc):
        _ = xyzabc
        return False


class _FakeKineFailIK(_FakeKineSuccess):
    def ik(self, structure_data):
        _ = structure_data
        return None


class _FakeKinematicsAdapter:
    def __init__(self, kine):
        self.is_initialized = True
        self._kine = kine


class _FakeFXInvKineSolvePara:
    def __init__(self):
        self.target = None
        self.ref = None
        self.zsp_type = None
        self.zsp_para = None

    def set_input_ik_target_tcp(self, matrix):
        self.target = list(matrix)

    def set_input_ik_ref_joint(self, values):
        self.ref = list(values)

    def set_input_ik_zsp_type(self, value):
        self.zsp_type = int(value)

    def set_input_ik_zsp_para(self, values):
        self.zsp_para = [float(v) for v in values]


class _FakeFXInvKineSolveParaFailZSP(_FakeFXInvKineSolvePara):
    def set_input_ik_zsp_type(self, value):
        _ = value
        raise RuntimeError("zsp_type_not_supported")


def _install_fake_fx_kine(monkeypatch, solve_para_cls=_FakeFXInvKineSolvePara) -> None:
    fake_module = types.ModuleType("fx_kine")
    fake_module.FX_InvKineSolvePara = solve_para_cls
    monkeypatch.setitem(sys.modules, "fx_kine", fake_module)


def test_ik_adapter_success_returns_q7(monkeypatch) -> None:
    _install_fake_fx_kine(monkeypatch)

    fake_kine = _FakeKineSuccess()
    adapter = ArmIKAdapter(_FakeKinematicsAdapter(fake_kine))
    q = adapter.solve_xyzabc_mm_deg(
        position_xyz_mm=(100.0, 200.0, 300.0),
        orientation_abc_deg=(10.0, 20.0, 30.0),
        ik_reference_q_deg=(90.0, -90.0, -90.0, -90.0, 0.0, 0.0, 0.0),
    )

    assert q == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)
    assert fake_kine.last_structure_data is not None
    assert fake_kine.last_structure_data.ref == [90.0, -90.0, -90.0, -90.0, 0.0, 0.0, 0.0]


def test_ik_adapter_failure_returns_none(monkeypatch) -> None:
    _install_fake_fx_kine(monkeypatch)

    adapter_matrix_fail = ArmIKAdapter(_FakeKinematicsAdapter(_FakeKineFailMatrix()))
    q1 = adapter_matrix_fail.solve_xyzabc_mm_deg(
        position_xyz_mm=(100.0, 200.0, 300.0),
        orientation_abc_deg=(10.0, 20.0, 30.0),
        ik_reference_q_deg=(90.0, -90.0, -90.0, -90.0, 0.0, 0.0, 0.0),
    )
    assert q1 is None

    adapter_ik_fail = ArmIKAdapter(_FakeKinematicsAdapter(_FakeKineFailIK()))
    q2 = adapter_ik_fail.solve_xyzabc_mm_deg(
        position_xyz_mm=(100.0, 200.0, 300.0),
        orientation_abc_deg=(10.0, 20.0, 30.0),
        ik_reference_q_deg=(90.0, -90.0, -90.0, -90.0, 0.0, 0.0, 0.0),
    )
    assert q2 is None


def test_ik_adapter_applies_zsp_when_enabled(monkeypatch) -> None:
    _install_fake_fx_kine(monkeypatch)

    fake_kine = _FakeKineSuccess()
    adapter = ArmIKAdapter(
        _FakeKinematicsAdapter(fake_kine),
        config=IKSolverConfig(
            mode="zsp_negative_z",
            enable_zsp=True,
            zsp_para_left=(1.0, -1.0, -1.0, 0.0, 0.0, 0.0),
            zsp_para_right=(9.0, 9.0, 9.0, 0.0, 0.0, 0.0),
        ),
        robot_side="left",
    )
    q = adapter.solve_xyzabc_mm_deg(
        position_xyz_mm=(100.0, 200.0, 300.0),
        orientation_abc_deg=(10.0, 20.0, 30.0),
        ik_reference_q_deg=(90.0, -90.0, -90.0, -90.0, 0.0, 0.0, 0.0),
    )

    assert q is not None
    assert adapter.last_solver_note == "zsp_applied"
    assert fake_kine.last_structure_data is not None
    assert fake_kine.last_structure_data.zsp_type == 1
    assert fake_kine.last_structure_data.zsp_para == [1.0, -1.0, -1.0, 0.0, 0.0, 0.0]
    # Fixed reference remains active in ZSP mode.
    assert fake_kine.last_structure_data.ref == [90.0, -90.0, -90.0, -90.0, 0.0, 0.0, 0.0]


def test_ik_adapter_applies_right_side_zsp_para(monkeypatch) -> None:
    _install_fake_fx_kine(monkeypatch)

    fake_kine = _FakeKineSuccess()
    adapter = ArmIKAdapter(
        _FakeKinematicsAdapter(fake_kine),
        config=IKSolverConfig(
            mode="zsp_negative_z",
            enable_zsp=True,
            zsp_para_left=(1.0, -1.0, -1.0, 0.0, 0.0, 0.0),
            zsp_para_right=(2.0, -2.0, -2.0, 0.0, 0.0, 0.0),
        ),
        robot_side="right",
    )
    q = adapter.solve_xyzabc_mm_deg(
        position_xyz_mm=(100.0, 200.0, 300.0),
        orientation_abc_deg=(10.0, 20.0, 30.0),
        ik_reference_q_deg=(90.0, -90.0, -90.0, -90.0, 0.0, 0.0, 0.0),
    )

    assert q is not None
    assert adapter.last_solver_note == "zsp_applied"
    assert fake_kine.last_structure_data is not None
    assert fake_kine.last_structure_data.zsp_para == [2.0, -2.0, -2.0, 0.0, 0.0, 0.0]


def test_ik_adapter_does_not_apply_zsp_when_disabled(monkeypatch) -> None:
    _install_fake_fx_kine(monkeypatch)

    fake_kine = _FakeKineSuccess()
    adapter = ArmIKAdapter(
        _FakeKinematicsAdapter(fake_kine),
        config=IKSolverConfig(mode="fixed_reference_only"),
    )
    q = adapter.solve_xyzabc_mm_deg(
        position_xyz_mm=(100.0, 200.0, 300.0),
        orientation_abc_deg=(10.0, 20.0, 30.0),
        ik_reference_q_deg=(90.0, -90.0, -90.0, -90.0, 0.0, 0.0, 0.0),
    )

    assert q is not None
    assert adapter.last_solver_note == "fixed_reference_only"
    assert fake_kine.last_structure_data is not None
    assert fake_kine.last_structure_data.zsp_type is None
    assert fake_kine.last_structure_data.zsp_para is None


def test_ik_adapter_zsp_assignment_failure_falls_back(monkeypatch) -> None:
    _install_fake_fx_kine(monkeypatch, solve_para_cls=_FakeFXInvKineSolveParaFailZSP)

    fake_kine = _FakeKineSuccess()
    adapter = ArmIKAdapter(
        _FakeKinematicsAdapter(fake_kine),
        config=IKSolverConfig(mode="zsp_negative_z", enable_zsp=True),
    )
    q = adapter.solve_xyzabc_mm_deg(
        position_xyz_mm=(100.0, 200.0, 300.0),
        orientation_abc_deg=(10.0, 20.0, 30.0),
        ik_reference_q_deg=(90.0, -90.0, -90.0, -90.0, 0.0, 0.0, 0.0),
    )

    assert q is not None
    assert adapter.last_solver_note.startswith("zsp_fallback_fixed_reference:")
