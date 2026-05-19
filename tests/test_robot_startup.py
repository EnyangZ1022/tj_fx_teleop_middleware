from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

import teleop.robot.startup as startup
from teleop.robot.startup import (
    RobotStartupConfig,
    enter_position_mode,
    move_arm_to_ready_pose,
    send_joint_command,
    wait_until_joint_target_reached,
)


class _FakeDCSS:
    pass


class _FakeRobot:
    def __init__(self):
        self.calls: list[tuple] = []
        self._frame_serial = 0
        self._feedback_sequences: dict[str, list[tuple[float, ...]]] = {
            "A": [tuple([0.0] * 7)],
            "B": [tuple([0.0] * 7)],
        }
        self._feedback_indices = {"A": 0, "B": 0}

    def set_feedback_sequence(self, arm: str, sequence: list[tuple[float, ...]]) -> None:
        self._feedback_sequences[arm] = [tuple(float(v) for v in q) for q in sequence]
        self._feedback_indices[arm] = 0

    def _next_feedback(self, arm: str) -> tuple[float, ...]:
        sequence = self._feedback_sequences[arm]
        idx = self._feedback_indices[arm]
        value = sequence[min(idx, len(sequence) - 1)]
        if idx < len(sequence) - 1:
            self._feedback_indices[arm] = idx + 1
        return value

    def clear_set(self):
        self.calls.append(("clear_set",))
        return 1

    def set_vel_acc(self, arm: str, velRatio: int, AccRatio: int):
        self.calls.append(("set_vel_acc", arm, velRatio, AccRatio))
        return 1

    def set_state(self, arm: str, state: int):
        self.calls.append(("set_state", arm, state))
        return 1

    def set_joint_cmd_pose(self, arm: str, joints: list[float]):
        self.calls.append(("set_joint_cmd_pose", arm, tuple(float(v) for v in joints)))
        self.set_feedback_sequence(arm, [tuple(float(v) for v in joints)])
        return 1

    def send_cmd(self):
        self.calls.append(("send_cmd",))
        return 1

    def subscribe(self, dcss):
        _ = dcss
        self._frame_serial += 1
        qa = self._next_feedback("A")
        qb = self._next_feedback("B")
        return {
            "outputs": [
                {"frame_serial": self._frame_serial, "fb_joint_pos": list(qa)},
                {"frame_serial": self._frame_serial, "fb_joint_pos": list(qb)},
            ]
        }


def test_startup_config_ready_pose_length_validation() -> None:
    cfg = RobotStartupConfig()
    assert len(cfg.left_ready_q_deg) == 7
    assert len(cfg.right_ready_q_deg) == 7

    with pytest.raises(ValueError):
        RobotStartupConfig(left_ready_q_deg=(1, 2, 3, 4, 5, 6))


def test_enter_position_mode_call_order(monkeypatch) -> None:
    monkeypatch.setattr(startup.time, "sleep", lambda _s: None)

    robot = _FakeRobot()
    enter_position_mode(robot=robot, arm="A", vel_ratio=20, acc_ratio=20)

    call_names = [c[0] for c in robot.calls]
    assert call_names == [
        "clear_set",
        "set_vel_acc",
        "send_cmd",
        "clear_set",
        "set_state",
        "send_cmd",
    ]
    assert robot.calls[1] == ("set_vel_acc", "A", 20, 20)
    assert robot.calls[4] == ("set_state", "A", 1)


def test_send_joint_command_validation_and_call_order() -> None:
    robot = _FakeRobot()

    with pytest.raises(ValueError):
        send_joint_command(robot=robot, arm="A", joints_deg=[0, 0, 0, 0, 0, 0])

    send_joint_command(robot=robot, arm="A", joints_deg=[1, 2, 3, 4, 5, 6, 7])
    call_names = [c[0] for c in robot.calls]
    assert call_names == ["clear_set", "set_joint_cmd_pose", "send_cmd"]
    assert robot.calls[1] == ("set_joint_cmd_pose", "A", (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0))


def test_wait_until_joint_target_reached_success(monkeypatch) -> None:
    monkeypatch.setattr(startup.time, "sleep", lambda _s: None)

    robot = _FakeRobot()
    target = (10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    robot.set_feedback_sequence("A", [
        (20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        (10.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    ])

    reached, last_error = wait_until_joint_target_reached(
        robot=robot,
        dcss=_FakeDCSS(),
        arm="A",
        target_joints_deg=target,
        tol_deg=0.5,
        stable_samples=2,
        timeout_s=1.0,
        check_period_s=0.01,
    )

    assert reached is True
    assert last_error <= 0.5


def test_wait_until_joint_target_reached_timeout(monkeypatch) -> None:
    monkeypatch.setattr(startup.time, "sleep", lambda _s: None)

    tick = {"t": 0.0}

    def _fake_monotonic() -> float:
        tick["t"] += 0.1
        return tick["t"]

    monkeypatch.setattr(startup.time, "monotonic", _fake_monotonic)

    robot = _FakeRobot()
    robot.set_feedback_sequence("A", [(100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)])

    reached, last_error = wait_until_joint_target_reached(
        robot=robot,
        dcss=_FakeDCSS(),
        arm="A",
        target_joints_deg=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        tol_deg=1.0,
        stable_samples=3,
        timeout_s=0.35,
        check_period_s=0.01,
    )

    assert reached is False
    assert last_error > 1.0


def test_move_arm_to_ready_pose(monkeypatch) -> None:
    monkeypatch.setattr(startup.time, "sleep", lambda _s: None)

    robot = _FakeRobot()
    cfg = RobotStartupConfig(
        vel_ratio=20,
        acc_ratio=20,
        home_timeout_s=1.0,
        home_tol_deg=1.0,
        home_stable_samples=2,
        check_period_s=0.01,
        pre_wait_s=0.0,
    )

    move_arm_to_ready_pose(
        robot=robot,
        dcss=_FakeDCSS(),
        arm="A",
        ready_joints_deg=cfg.left_ready_q_deg,
        startup_config=cfg,
    )

    call_names = [c[0] for c in robot.calls]
    assert call_names[:6] == [
        "clear_set",
        "set_vel_acc",
        "send_cmd",
        "clear_set",
        "set_state",
        "send_cmd",
    ]
    assert ("set_joint_cmd_pose", "A", cfg.left_ready_q_deg) in robot.calls


def test_move_script_importable_without_execution() -> None:
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "move_robot_to_ready_pose.py"

    spec = importlib.util.spec_from_file_location("move_robot_to_ready_pose", script_path)
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    assert hasattr(module, "parse_args")
    assert hasattr(module, "main")
