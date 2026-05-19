from __future__ import annotations

import sys
import types

from teleop.robot.ik_adapter import ArmIKAdapter


class _FakeRetJoint:
    def __init__(self, values):
        self._values = list(values)

    def to_list(self):
        return list(self._values)


class _FakeIKResult:
    def __init__(self, values):
        self.m_Output_RetJoint = _FakeRetJoint(values)


class _FakeKineSuccess:
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
        _ = structure_data
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

    def set_input_ik_target_tcp(self, matrix):
        self.target = list(matrix)

    def set_input_ik_ref_joint(self, values):
        self.ref = list(values)


def _install_fake_fx_kine(monkeypatch) -> None:
    fake_module = types.ModuleType("fx_kine")
    fake_module.FX_InvKineSolvePara = _FakeFXInvKineSolvePara
    monkeypatch.setitem(sys.modules, "fx_kine", fake_module)


def test_ik_adapter_success_returns_q7(monkeypatch) -> None:
    _install_fake_fx_kine(monkeypatch)

    adapter = ArmIKAdapter(_FakeKinematicsAdapter(_FakeKineSuccess()))
    q = adapter.solve_xyzabc_mm_deg(
        position_xyz_mm=(100.0, 200.0, 300.0),
        orientation_abc_deg=(10.0, 20.0, 30.0),
        ik_reference_q_deg=(90.0, -90.0, -90.0, -90.0, 0.0, 0.0, 0.0),
    )

    assert q == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)


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
