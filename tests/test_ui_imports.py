from __future__ import annotations

import importlib
import importlib.util
from pathlib import Path

import pytest


def test_ui_core_modules_import_without_sdk() -> None:
    modules = [
        "teleop.ui",
        "teleop.ui.ui_config",
        "teleop.ui.snapshot",
        "teleop.ui.snapshot_builder",
    ]
    for module_name in modules:
        module = importlib.import_module(module_name)
        assert module is not None


def test_ui_gui_modules_import_when_gui_dependencies_available() -> None:
    pytest.importorskip("PySide6")
    pytest.importorskip("pyqtgraph")

    modules = [
        "teleop.ui.scene3d",
        "teleop.ui.status_panel",
        "teleop.ui.main_window",
        "teleop.ui.app",
    ]
    for module_name in modules:
        module = importlib.import_module(module_name)
        assert module is not None


def test_stage8_scripts_import_without_running_app() -> None:
    script_paths = [
        Path("scripts/run_teleop_ui_mock.py"),
        Path("scripts/run_teleop_ui_from_log.py"),
    ]

    for script_path in script_paths:
        module = _import_script_module(script_path)
        assert hasattr(module, "main")


def _import_script_module(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem, str(path))
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
