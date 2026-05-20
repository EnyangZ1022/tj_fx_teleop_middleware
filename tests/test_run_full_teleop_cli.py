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
    assert cfg.orientation_tracking.enabled is False


def test_cli_teleop_mode_position_orientation() -> None:
    module = _load_run_full_teleop_module()

    args = module.parse_args(["--teleop-mode", "position_orientation"])
    cfg = module._build_app_config(args)

    assert cfg.teleop_mode == "position_orientation"
    assert cfg.orientation_tracking.enabled is True


def test_cli_enable_orientation_shorthand() -> None:
    module = _load_run_full_teleop_module()

    args = module.parse_args(["--enable-orientation"])
    cfg = module._build_app_config(args)

    assert cfg.teleop_mode == "position_orientation"
    assert cfg.orientation_tracking.enabled is True
