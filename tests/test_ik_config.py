from __future__ import annotations

import pytest

from teleop.robot.ik_config import IKSolverConfig


def test_ik_solver_config_defaults_enable_per_arm_zsp_params() -> None:
    cfg = IKSolverConfig()

    assert cfg.mode == "zsp_negative_z"
    assert cfg.enable_zsp is True
    assert cfg.zsp_type == 1
    assert cfg.zsp_para_left == (1.0, -1.0, -1.0, 0.0, 0.0, 0.0)
    assert cfg.zsp_para_right == (1.0, 1.0, -1.0, 0.0, 0.0, 0.0)
    assert cfg.use_zsp() is True


def test_ik_solver_config_zsp_para_length_must_be_6() -> None:
    cfg = IKSolverConfig(
        zsp_para_left=(0.0, 0.0, -1.0, 0.0, 0.0, 0.0),
        zsp_para_right=(1.0, -1.0, -1.0, 0.0, 0.0, 0.0),
    )
    assert len(cfg.zsp_para_left) == 6
    assert len(cfg.zsp_para_right) == 6


def test_ik_solver_config_invalid_left_zsp_para_length_raises() -> None:
    with pytest.raises(ValueError):
        IKSolverConfig(zsp_para_left=(0.0, 0.0, -1.0))


def test_ik_solver_config_invalid_right_zsp_para_length_raises() -> None:
    with pytest.raises(ValueError):
        IKSolverConfig(zsp_para_right=(0.0, 0.0, -1.0))


def test_ik_solver_config_fixed_reference_mode_disables_zsp() -> None:
    cfg = IKSolverConfig(mode="fixed_reference_only", enable_zsp=True)

    assert cfg.mode == "fixed_reference_only"
    assert cfg.enable_zsp is False
    assert cfg.use_zsp() is False
