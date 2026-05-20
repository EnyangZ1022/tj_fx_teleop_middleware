from __future__ import annotations

from dataclasses import dataclass

from teleop.core.command_frame import ArmCommandTarget, DualArmCommandTarget
import teleop.robot.command_adapter as command_adapter_module
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

    def set_vel_acc(self, arm: str, velRatio: int, AccRatio: int):
        self.calls.append(("set_vel_acc", arm, velRatio, AccRatio))
        return 1

    def set_state(self, arm: str, state: int):
        self.calls.append(("set_state", arm, state))
        return 1

    def set_impedance_type(self, arm: str, type: int):
        self.calls.append(("set_impedance_type", arm, type))
        return 1

    def set_joint_kd_params(self, arm: str, K: list[float], D: list[float]):
        self.calls.append(("set_joint_kd_params", arm, tuple(K), tuple(D)))
        return 1

    def set_vel_est_step(self, arm: str, time: int):
        self.calls.append(("set_vel_est_step", arm, time))
        return 1


@dataclass
class _FakeKineAdapter:
    is_initialized: bool = True


@dataclass
class _FakeSDKConfig:
    left_arm: str = "A"
    right_arm: str = "B"


class _FakeSDKAdapter:
    def __init__(self):
        self.connected = True
        self.robot = _FakeRobot()
        self.dcss = object()
        self._config = _FakeSDKConfig()
        self.left_kine = _FakeKineAdapter()
        self.right_kine = _FakeKineAdapter()
        self.disconnect_calls = 0

    def disconnect(self):
        self.disconnect_calls += 1
        self.connected = False


class _QueueIK:
    def __init__(self, queue: list[tuple[float, ...] | None]):
        self._queue = list(queue)
        self.calls: list[tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, ...]]] = []

    def solve_xyzabc_mm_deg(self, position_xyz_mm, orientation_abc_deg, ik_reference_q_deg):
        self.calls.append((tuple(position_xyz_mm), tuple(orientation_abc_deg), tuple(ik_reference_q_deg)))
        if not self._queue:
            return None
        value = self._queue.pop(0)
        return value


def _arm_target(x_mm: float = 100.0, *, valid: bool = True, ik_ref: tuple[float, ...] = (90, -90, -90, -90, 0, 0, 0)):
    return ArmCommandTarget(
        position_xyz_mm=(x_mm, 200.0, 300.0),
        orientation_abc_deg=(10.0, 20.0, 30.0),
        ik_reference_q_deg=ik_ref,
        valid=valid,
    )


def _dual(left: ArmCommandTarget | None, right: ArmCommandTarget | None) -> DualArmCommandTarget:
    return DualArmCommandTarget(left=left, right=right)


def _prepare_adapter(config: RobotCommandConfig | None = None) -> tuple[RobotCommandAdapter, _FakeSDKAdapter]:
    sdk = _FakeSDKAdapter()
    adapter = RobotCommandAdapter(sdk_adapter=sdk, config=config)
    adapter.prepare()
    return adapter, sdk


def test_command_adapter_dry_run_no_send() -> None:
    adapter, sdk = _prepare_adapter(RobotCommandConfig(dry_run=True, command_enabled=False))
    adapter.left_ik_adapter = _QueueIK([(1, 2, 3, 4, 5, 6, 7)])
    adapter.right_ik_adapter = _QueueIK([(11, 12, 13, 14, 15, 16, 17)])

    result = adapter.send_command(_dual(_arm_target(), _arm_target(110.0, ik_ref=(90, 90, -90, -90, 0, 0, 0))))

    assert result["dry_run"] is True
    assert result["left_q_deg"] == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)
    assert result["right_q_deg"] == (11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0)
    assert not any(c[0] == "set_joint_cmd_pose" for c in sdk.robot.calls)
    assert not any(c[0] == "send_cmd" for c in sdk.robot.calls)


def test_command_disabled_rejects_send() -> None:
    adapter, sdk = _prepare_adapter(RobotCommandConfig(dry_run=False, command_enabled=False))
    adapter.left_ik_adapter = _QueueIK([(1, 2, 3, 4, 5, 6, 7)])

    result = adapter.send_command(_dual(_arm_target(), None))

    assert result["ok"] is False
    assert result["left_reason"] == "command_disabled"
    assert not any(c[0] == "set_joint_cmd_pose" for c in sdk.robot.calls)


def test_command_enabled_sends_joint_command() -> None:
    adapter, sdk = _prepare_adapter(RobotCommandConfig(dry_run=False, command_enabled=False))
    adapter.left_ik_adapter = _QueueIK([(1, 2, 3, 4, 5, 6, 7)])
    adapter.enable_commands()

    result = adapter.send_command(_dual(_arm_target(), None), now_ns=1_000_000_000)

    assert result["left_sent"] is True
    assert result["left_reason"] == "sent"
    assert any(c[0] == "set_joint_cmd_pose" for c in sdk.robot.calls)
    assert any(c[0] == "send_cmd" for c in sdk.robot.calls)


def test_missing_side_only_right_processed() -> None:
    adapter, sdk = _prepare_adapter(RobotCommandConfig(dry_run=False, command_enabled=True))
    adapter.left_ik_adapter = _QueueIK([(1, 2, 3, 4, 5, 6, 7)])
    adapter.right_ik_adapter = _QueueIK([(11, 12, 13, 14, 15, 16, 17)])

    result = adapter.send_command(_dual(None, _arm_target(110.0, ik_ref=(90, 90, -90, -90, 0, 0, 0))), now_ns=1_000_000_000)

    assert result["left_sent"] is False
    assert result["left_reason"] == "no_target"
    assert result["right_sent"] is True
    right_calls = [c for c in sdk.robot.calls if c[0] == "set_joint_cmd_pose"]
    assert len(right_calls) == 1
    assert right_calls[0][1] == "B"


def test_ik_failure_rejects_side() -> None:
    adapter, sdk = _prepare_adapter(RobotCommandConfig(dry_run=False, command_enabled=True))
    adapter.left_ik_adapter = _QueueIK([None])

    result = adapter.send_command(_dual(_arm_target(), None), now_ns=1_000_000_000)

    assert result["left_sent"] is False
    assert result["left_reason"] == "ik_failed"
    assert not any(c[0] == "set_joint_cmd_pose" for c in sdk.robot.calls)


def test_joint_step_limit_rejects_large_delta() -> None:
    adapter, sdk = _prepare_adapter(
        RobotCommandConfig(dry_run=False, command_enabled=True, max_joint_step_deg=5.0, max_joint_velocity_deg_s=1000.0)
    )
    adapter.left_ik_adapter = _QueueIK([
        (1, 1, 1, 1, 1, 1, 1),
        (20, 1, 1, 1, 1, 1, 1),
    ])

    first = adapter.send_command(_dual(_arm_target(), None), now_ns=1_000_000_000)
    second = adapter.send_command(_dual(_arm_target(101.0), None), now_ns=2_000_000_000)

    assert first["left_sent"] is True
    assert second["left_sent"] is False
    assert second["left_reason"] == "joint_step_limit"

    set_joint_calls = [c for c in sdk.robot.calls if c[0] == "set_joint_cmd_pose"]
    assert len(set_joint_calls) == 1


def test_joint_velocity_limit_rejects_excess_speed() -> None:
    adapter, _ = _prepare_adapter(
        RobotCommandConfig(dry_run=False, command_enabled=True, max_joint_step_deg=50.0, max_joint_velocity_deg_s=20.0)
    )
    adapter.left_ik_adapter = _QueueIK([
        (1, 1, 1, 1, 1, 1, 1),
        (10, 1, 1, 1, 1, 1, 1),
    ])

    first = adapter.send_command(_dual(_arm_target(), None), now_ns=1_000_000_000)
    second = adapter.send_command(_dual(_arm_target(101.0), None), now_ns=1_050_000_000)

    assert first["left_sent"] is True
    assert second["left_sent"] is False
    assert second["left_reason"] == "joint_velocity_limit"


def test_pause_disables_future_send() -> None:
    adapter, sdk = _prepare_adapter(RobotCommandConfig(dry_run=False, command_enabled=True))
    adapter.left_ik_adapter = _QueueIK([(1, 2, 3, 4, 5, 6, 7)])

    adapter.pause()
    result = adapter.send_command(_dual(_arm_target(), None), now_ns=1_000_000_000)

    assert result["left_reason"] == "command_disabled"
    assert not any(c[0] == "set_joint_cmd_pose" for c in sdk.robot.calls)


def test_stop_disables_and_disconnects() -> None:
    adapter, sdk = _prepare_adapter(RobotCommandConfig(dry_run=False, command_enabled=True))

    adapter.stop()

    assert sdk.disconnect_calls == 1
    assert adapter.active is False
    assert sdk.connected is False


def test_fixed_ik_reference_is_used_not_previous_solution() -> None:
    adapter, _ = _prepare_adapter(RobotCommandConfig(dry_run=True, command_enabled=False))
    left_queue = _QueueIK([
        (1, 2, 3, 4, 5, 6, 7),
        (9, 8, 7, 6, 5, 4, 3),
    ])
    adapter.left_ik_adapter = left_queue

    ref1 = (90.0, -90.0, -90.0, -90.0, 0.0, 0.0, 0.0)
    ref2 = (30.0, -20.0, -10.0, -5.0, 1.0, 2.0, 3.0)

    adapter.send_command(_dual(_arm_target(100.0, ik_ref=ref1), None), now_ns=1_000_000_000)
    adapter.send_command(_dual(_arm_target(101.0, ik_ref=ref2), None), now_ns=2_000_000_000)

    assert left_queue.calls[0][2] == ref1
    assert left_queue.calls[1][2] == ref2


def test_enter_command_mode_joint_impedance_calls_sdk_methods() -> None:
    adapter, sdk = _prepare_adapter(RobotCommandConfig(control_mode="joint_impedance", command_enabled=False))

    adapter.enter_command_mode()

    call_names = [c[0] for c in sdk.robot.calls]
    assert "set_state" in call_names
    assert "set_impedance_type" in call_names
    assert "set_joint_kd_params" in call_names
    assert "set_vel_est_step" in call_names


def test_dry_run_does_not_require_enable_flag() -> None:
    adapter, _ = _prepare_adapter(RobotCommandConfig(dry_run=True, command_enabled=False))
    adapter.left_ik_adapter = _QueueIK([(1, 2, 3, 4, 5, 6, 7)])

    result = adapter.send_command(_dual(_arm_target(), None), now_ns=1_000_000_000)

    assert result["left_reason"] == "dry_run"


def test_send_command_rejects_when_not_connected() -> None:
    adapter, sdk = _prepare_adapter(RobotCommandConfig(dry_run=False, command_enabled=True))
    adapter.left_ik_adapter = _QueueIK([(1, 2, 3, 4, 5, 6, 7)])
    sdk.connected = False

    result = adapter.send_command(_dual(_arm_target(), None), now_ns=1_000_000_000)

    assert result["left_reason"] == "not_connected"


def test_prepare_uses_arm_ik_adapter_class(monkeypatch) -> None:
    created = {"count": 0, "modes": []}

    class _FakeArmIKAdapterCtor:
        def __init__(self, kinematics_adapter, config=None):
            _ = kinematics_adapter
            created["count"] += 1
            created["modes"].append(getattr(config, "mode", None))

        def solve_xyzabc_mm_deg(self, position_xyz_mm, orientation_abc_deg, ik_reference_q_deg):
            _ = (position_xyz_mm, orientation_abc_deg, ik_reference_q_deg)
            return (1, 2, 3, 4, 5, 6, 7)

    monkeypatch.setattr(command_adapter_module, "ArmIKAdapter", _FakeArmIKAdapterCtor)

    sdk = _FakeSDKAdapter()
    adapter = RobotCommandAdapter(sdk_adapter=sdk, config=RobotCommandConfig())
    adapter.prepare()

    assert created["count"] == 2
    assert created["modes"] == ["zsp_negative_z", "zsp_negative_z"]
