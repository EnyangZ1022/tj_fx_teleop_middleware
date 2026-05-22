from __future__ import annotations

import importlib.util
from pathlib import Path

from teleop.diagnostics.integration_checks import (
    validate_command_safety_defaults,
    validate_ready_pose_config,
)
from teleop.robot.command_adapter import RobotCommandAdapter
from teleop.robot.command_config import RobotCommandConfig


class _FakeRobot:
    def __init__(self):
        self.calls: list[tuple] = []

    def clear_set(self):
        self.calls.append(("clear_set",))
        return 1

    def set_joint_cmd_pose(self, arm: str, joints: list[float]):
        self.calls.append(("set_joint_cmd_pose", arm, tuple(float(v) for v in joints)))
        return 1

    def send_cmd(self):
        self.calls.append(("send_cmd",))
        return 1


class _FakeKineAdapter:
    is_initialized = True


class _FakeSDKConfig:
    left_arm = "A"
    right_arm = "B"


class _FakeSDKAdapter:
    def __init__(self):
        self.connected = True
        self.robot = _FakeRobot()
        self.dcss = object()
        self._config = _FakeSDKConfig()
        self.left_kine = _FakeKineAdapter()
        self.right_kine = _FakeKineAdapter()

    def disconnect(self):
        self.connected = False


class _FakeIK:
    def solve_xyzabc_mm_deg(self, position_xyz_mm, orientation_abc_deg, ik_reference_q_deg):
        _ = (position_xyz_mm, orientation_abc_deg, ik_reference_q_deg)
        return (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)


def test_ready_pose_validation_and_separation() -> None:
    left_ready = (90.0, -60.0, -90.0, -90.0, 0.0, 0.0, 0.0)
    right_ready = (90.0, 60.0, -90.0, -90.0, 0.0, 0.0, 0.0)
    left_ik_ref = (90.0, -90.0, -90.0, -90.0, 0.0, 0.0, 0.0)
    right_ik_ref = (90.0, 90.0, -90.0, -90.0, 0.0, 0.0, 0.0)

    errors = validate_ready_pose_config(left_ready, right_ready, left_ik_ref, right_ik_ref)
    assert errors == []

    zero_errors = validate_ready_pose_config((0, 0, 0, 0, 0, 0, 0), right_ready, left_ik_ref, right_ik_ref)
    assert any("must not be all zeros" in item for item in zero_errors)

    length_errors = validate_ready_pose_config((1, 2, 3), right_ready, left_ik_ref, right_ik_ref)
    assert any("length 7" in item for item in length_errors)

    separation_errors = validate_ready_pose_config(left_ik_ref, right_ready, left_ik_ref, right_ik_ref)
    assert any("separate" in item for item in separation_errors)


def test_command_safety_defaults_and_adapter_blocking() -> None:
    cfg = RobotCommandConfig()
    errors = validate_command_safety_defaults(cfg)
    assert errors == []
    assert cfg.dry_run is True
    assert cfg.command_enabled is False
    assert cfg.joint_limit_mode == "reject"

    sdk_adapter = _FakeSDKAdapter()
    adapter = RobotCommandAdapter(
        sdk_adapter=sdk_adapter,
        config=RobotCommandConfig(dry_run=False, command_enabled=False),
    )
    adapter.prepare()
    adapter.left_ik_adapter = _FakeIK()

    from teleop.core.command_frame import ArmCommandTarget, DualArmCommandTarget

    command = DualArmCommandTarget(
        left=ArmCommandTarget(
            position_xyz_mm=(1000.0, 2000.0, 3000.0),
            orientation_abc_deg=(10.0, 20.0, 30.0),
            ik_reference_q_deg=(90.0, -90.0, -90.0, -90.0, 0.0, 0.0, 0.0),
            valid=True,
        ),
        right=None,
    )
    result = adapter.send_command(command, now_ns=1_000_000_000)

    assert result["left_reason"] == "command_disabled"
    assert not any(call[0] == "set_joint_cmd_pose" for call in sdk_adapter.robot.calls)


def test_stage6c_scripts_import_without_hardware_side_effects() -> None:
    root = Path(__file__).resolve().parents[1]

    mapping_script = root / "scripts" / "check_coordinate_mapping.py"
    pipeline_script = root / "scripts" / "check_stage6_pipeline_dry.py"

    spec1 = importlib.util.spec_from_file_location("check_coordinate_mapping", mapping_script)
    assert spec1 is not None
    assert spec1.loader is not None
    module1 = importlib.util.module_from_spec(spec1)
    spec1.loader.exec_module(module1)
    assert hasattr(module1, "main")

    spec2 = importlib.util.spec_from_file_location("check_stage6_pipeline_dry", pipeline_script)
    assert spec2 is not None
    assert spec2.loader is not None
    module2 = importlib.util.module_from_spec(spec2)
    spec2.loader.exec_module(module2)
    assert hasattr(module2, "main")
