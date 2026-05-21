from __future__ import annotations

import importlib.util
from pathlib import Path


def _load_run_full_teleop_module():
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "run_full_teleop.py"
    spec = importlib.util.spec_from_file_location("run_full_teleop", script_path)
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_cli_default_is_position_only() -> None:
    module = _load_run_full_teleop_module()

    args = module.parse_args([])
    cfg = module._build_app_config(args)

    assert cfg.teleop_mode == "position_only"
    assert cfg.control_mode == "joint_position"
    assert cfg.orientation_tracking.enabled is False
    assert cfg.orientation_tracking.orientation_algorithm == "absolute_matrix"
    assert cfg.orientation_filter.enabled is False


def test_cli_teleop_mode_position_orientation() -> None:
    module = _load_run_full_teleop_module()

    args = module.parse_args(["--teleop-mode", "position_orientation"])
    cfg = module._build_app_config(args)

    assert cfg.teleop_mode == "position_orientation"
    assert cfg.orientation_tracking.enabled is True
    assert cfg.orientation_tracking.orientation_algorithm == "absolute_matrix"
    assert cfg.orientation_filter.enabled is True
    assert cfg.orientation_filter.tau_s == 0.02
    assert cfg.orientation_filter.fallback_dt_s == 0.01


def test_cli_enable_orientation_shorthand() -> None:
    module = _load_run_full_teleop_module()

    args = module.parse_args(["--enable-orientation"])
    cfg = module._build_app_config(args)

    assert cfg.teleop_mode == "position_orientation"
    assert cfg.orientation_tracking.enabled is True
    assert cfg.orientation_tracking.orientation_algorithm == "absolute_matrix"
    assert cfg.orientation_filter.enabled is True


def test_cli_orientation_algorithm_relative_rotvec() -> None:
    module = _load_run_full_teleop_module()

    args = module.parse_args(["--teleop-mode", "position_orientation", "--orientation-algorithm", "relative_rotvec"])
    cfg = module._build_app_config(args)

    assert cfg.orientation_tracking.enabled is True
    assert cfg.orientation_tracking.orientation_algorithm == "relative_rotvec"


def test_cli_disable_orientation_filter() -> None:
    module = _load_run_full_teleop_module()

    args = module.parse_args(["--teleop-mode", "position_orientation", "--disable-orientation-filter"])
    cfg = module._build_app_config(args)

    assert cfg.orientation_filter.enabled is False


def test_cli_orientation_filter_tau_override() -> None:
    module = _load_run_full_teleop_module()

    args = module.parse_args([
        "--teleop-mode",
        "position_orientation",
        "--orientation-filter-tau",
        "0.03",
        "--orientation-filter-fallback-dt",
        "0.02",
    ])
    cfg = module._build_app_config(args)

    assert cfg.orientation_filter.enabled is True
    assert cfg.orientation_filter.tau_s == 0.03
    assert cfg.orientation_filter.fallback_dt_s == 0.02


def test_cli_control_mode_joint_impedance() -> None:
    module = _load_run_full_teleop_module()

    args = module.parse_args(["--control-mode", "joint_impedance"])
    cfg = module._build_app_config(args)

    assert cfg.control_mode == "joint_impedance"


def test_validate_runtime_args_rejects_impedance_send_without_move_to_ready() -> None:
    module = _load_run_full_teleop_module()

    args = module.parse_args([
        "--enable-send",
        "--confirm",
        "--control-mode",
        "joint_impedance",
    ])
    message = module._validate_runtime_args(args)

    assert message == "joint_impedance mode requires --move-to-ready for real robot sending in this MVP."


def test_validate_runtime_args_allows_impedance_send_with_move_to_ready() -> None:
    module = _load_run_full_teleop_module()

    args = module.parse_args([
        "--enable-send",
        "--confirm",
        "--control-mode",
        "joint_impedance",
        "--move-to-ready",
    ])
    message = module._validate_runtime_args(args)

    assert message is None


def test_validate_runtime_args_rejects_orientation_filter_enable_disable_conflict() -> None:
    module = _load_run_full_teleop_module()

    args = module.parse_args([
        "--enable-orientation-filter",
        "--disable-orientation-filter",
    ])
    message = module._validate_runtime_args(args)

    assert message == "--enable-orientation-filter and --disable-orientation-filter cannot be used together"


def test_cli_joint_step_limit_mode_and_max_step_override() -> None:
    module = _load_run_full_teleop_module()

    args = module.parse_args([
        "--joint-step-limit-mode",
        "ramp",
        "--max-joint-step-deg",
        "1.8",
    ])
    robot_cfg = module._build_robot_command_config(args)

    assert robot_cfg.joint_step_limit_mode == "ramp"
    assert robot_cfg.max_joint_step_deg == 1.8


def test_cli_joint_step_limit_defaults_preserve_robot_command_default() -> None:
    module = _load_run_full_teleop_module()

    args = module.parse_args([])
    robot_cfg = module._build_robot_command_config(args)

    assert robot_cfg.joint_step_limit_mode == "reject"
    assert robot_cfg.max_joint_step_deg == module.RobotCommandConfig().max_joint_step_deg


def test_validate_runtime_args_rejects_non_positive_max_joint_step_deg() -> None:
    module = _load_run_full_teleop_module()

    args = module.parse_args(["--max-joint-step-deg", "0"])
    message = module._validate_runtime_args(args)

    assert message == "--max-joint-step-deg must be > 0"
