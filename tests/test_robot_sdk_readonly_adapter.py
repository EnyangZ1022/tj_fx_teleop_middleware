from __future__ import annotations

import sys
import types

import pytest

from teleop.core.robot_frame import DualArmRobotFeedback, RobotArmFeedback
from teleop.robot.kinematics_adapter import ArmKinematicsAdapter, sdk_arm_to_index
from teleop.robot.sdk_adapter import RobotSDKReadOnlyAdapter
from teleop.robot.sdk_config import RobotSDKConfig


class _FakeDCSS:
    pass


class _FakeRobotBase:
    def __init__(self):
        self.connect_calls = 0
        self.clear_set_calls = 0
        self.clear_error_calls: list[str] = []
        self.send_cmd_calls = 0
        self.log_switch_calls: list[str] = []
        self.local_log_switch_calls: list[str] = []
        self.release_calls = 0

        self.set_state_calls = 0
        self.set_joint_cmd_pose_calls = 0
        self.move_calls = 0

        self._sub_counter = 0

    def connect(self, robot_ip: str):
        self.connect_calls += 1
        return 1

    def clear_set(self):
        self.clear_set_calls += 1
        return 1

    def clear_error(self, arm: str):
        self.clear_error_calls.append(arm)
        return 1

    def send_cmd(self):
        self.send_cmd_calls += 1
        return 1

    def log_switch(self, flag: str):
        self.log_switch_calls.append(flag)
        return 1

    def local_log_switch(self, flag: str):
        self.local_log_switch_calls.append(flag)
        return 1

    def release_robot(self):
        self.release_calls += 1
        return 1

    # Motion-like methods that should not be used in Stage 6A.
    def set_state(self, *args, **kwargs):
        self.set_state_calls += 1
        return 1

    def set_joint_cmd_pose(self, *args, **kwargs):
        self.set_joint_cmd_pose_calls += 1
        return 1

    def move(self, *args, **kwargs):
        self.move_calls += 1
        return 1

    def subscribe(self, dcss):
        self._sub_counter += 1
        serial = self._sub_counter
        return {
            "outputs": [
                {
                    "frame_serial": serial,
                    "fb_joint_pos": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0],
                },
                {
                    "frame_serial": serial,
                    "fb_joint_pos": [11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0],
                },
            ]
        }


class _FakeRobotConnectFail(_FakeRobotBase):
    def connect(self, robot_ip: str):
        self.connect_calls += 1
        return 0


class _FakeRobotNoFrames(_FakeRobotBase):
    def subscribe(self, dcss):
        return {
            "outputs": [
                {"frame_serial": 0, "fb_joint_pos": [0.0] * 7},
                {"frame_serial": 0, "fb_joint_pos": [0.0] * 7},
            ]
        }


class _FakeMarvinKine:
    def __init__(self):
        self.load_calls: list[tuple[int, str]] = []
        self.initial_calls: list[tuple[int, list, list, list]] = []
        self.fk_calls: list[list[float]] = []

    def load_config(self, arm_type: int, config_path: str):
        self.load_calls.append((arm_type, config_path))
        return {
            "TYPE": [1007, 1007],
            "DH": [
                [[0.0, 0.0, 0.0, 0.0] for _ in range(8)],
                [[0.0, 0.0, 0.0, 0.0] for _ in range(8)],
            ],
            "PNVA": [
                [[0.0, 0.0, 0.0, 0.0] for _ in range(7)],
                [[0.0, 0.0, 0.0, 0.0] for _ in range(7)],
            ],
            "BD": [
                [[0.0, 0.0, 0.0] for _ in range(4)],
                [[0.0, 0.0, 0.0] for _ in range(4)],
            ],
        }

    def initial_kine(self, robot_type: int, dh: list, pnva: list, j67: list):
        self.initial_calls.append((robot_type, dh, pnva, j67))
        return True

    def fk(self, joints: list[float]):
        self.fk_calls.append(joints)
        return [
            [1.0, 0.0, 0.0, 100.0],
            [0.0, 1.0, 0.0, 200.0],
            [0.0, 0.0, 1.0, 300.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def mat4x4_to_xyzabc(self, pose_mat: list):
        _ = pose_mat
        return [100.0, 200.0, 300.0, 10.0, 20.0, 30.0]


def _install_fake_fx_kine(monkeypatch, kine_class=_FakeMarvinKine):
    fake_module = types.ModuleType("fx_kine")
    fake_module.Marvin_Kine = kine_class
    monkeypatch.setitem(sys.modules, "fx_kine", fake_module)


def _install_fake_fx_robot(monkeypatch, robot_class):
    fake_module = types.ModuleType("fx_robot")
    fake_module.Marvin_Robot = robot_class
    fake_module.DCSS = _FakeDCSS
    monkeypatch.setitem(sys.modules, "fx_robot", fake_module)


def test_arm_mapping_a_b_and_invalid() -> None:
    assert sdk_arm_to_index("A") == 0
    assert sdk_arm_to_index("a") == 0
    assert sdk_arm_to_index("B") == 1
    assert sdk_arm_to_index("b") == 1
    with pytest.raises(ValueError):
        sdk_arm_to_index("C")


def test_kinematics_fk_xyzabc_mm_deg(monkeypatch) -> None:
    _install_fake_fx_kine(monkeypatch)
    adapter = ArmKinematicsAdapter(arm="A", kine_cfg_path="assets/kinematics/ccs_m6_40.MvKDCfg")

    adapter.initialize()
    result = adapter.fk_xyzabc_mm_deg([0, 1, 2, 3, 4, 5, 6])

    assert adapter.is_initialized is True
    assert result == pytest.approx((100.0, 200.0, 300.0, 10.0, 20.0, 30.0))


def test_connect_success_and_clear_errors_called(monkeypatch) -> None:
    _install_fake_fx_robot(monkeypatch, _FakeRobotBase)
    _install_fake_fx_kine(monkeypatch)

    cfg = RobotSDKConfig(connect_check_samples=3, connect_check_interval_s=0.0, connect_settle_s=0.0)
    adapter = RobotSDKReadOnlyAdapter(cfg)
    adapter.connect()

    assert adapter.connected is True
    assert adapter.robot is not None
    assert adapter.robot.clear_set_calls == 1
    assert adapter.robot.send_cmd_calls == 1
    assert adapter.robot.clear_error_calls == ["A", "B"]


def test_connect_failure_raises_runtime_error(monkeypatch) -> None:
    _install_fake_fx_robot(monkeypatch, _FakeRobotConnectFail)
    _install_fake_fx_kine(monkeypatch)

    adapter = RobotSDKReadOnlyAdapter(RobotSDKConfig(connect_settle_s=0.0))
    with pytest.raises(RuntimeError):
        adapter.connect()


def test_no_feedback_frames_raises_runtime_error(monkeypatch) -> None:
    _install_fake_fx_robot(monkeypatch, _FakeRobotNoFrames)
    _install_fake_fx_kine(monkeypatch)

    cfg = RobotSDKConfig(connect_check_samples=3, connect_check_interval_s=0.0, connect_settle_s=0.0)
    adapter = RobotSDKReadOnlyAdapter(cfg)

    with pytest.raises(RuntimeError):
        adapter.connect()


def test_get_joint_feedback_left_returns_7_floats(monkeypatch) -> None:
    _install_fake_fx_robot(monkeypatch, _FakeRobotBase)
    _install_fake_fx_kine(monkeypatch)

    adapter = RobotSDKReadOnlyAdapter(RobotSDKConfig(connect_check_samples=3, connect_check_interval_s=0.0, connect_settle_s=0.0))
    adapter.connect()

    q = adapter.get_joint_feedback("left")

    assert len(q) == 7
    assert q == pytest.approx((1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0))


def test_get_arm_feedback_returns_mm_deg(monkeypatch) -> None:
    _install_fake_fx_robot(monkeypatch, _FakeRobotBase)
    _install_fake_fx_kine(monkeypatch)

    adapter = RobotSDKReadOnlyAdapter(RobotSDKConfig(connect_check_samples=3, connect_check_interval_s=0.0, connect_settle_s=0.0))
    adapter.connect()

    feedback = adapter.get_arm_feedback("left")

    assert isinstance(feedback, RobotArmFeedback)
    assert feedback.position_xyz == pytest.approx((100.0, 200.0, 300.0))
    assert feedback.orientation_abc == pytest.approx((10.0, 20.0, 30.0))
    assert feedback.valid is True


def test_get_dual_arm_feedback_returns_both_sides(monkeypatch) -> None:
    _install_fake_fx_robot(monkeypatch, _FakeRobotBase)
    _install_fake_fx_kine(monkeypatch)

    adapter = RobotSDKReadOnlyAdapter(RobotSDKConfig(connect_check_samples=3, connect_check_interval_s=0.0, connect_settle_s=0.0))
    adapter.connect()

    dual_feedback = adapter.get_dual_arm_feedback()

    assert isinstance(dual_feedback, DualArmRobotFeedback)
    assert dual_feedback.left is not None
    assert dual_feedback.right is not None


def test_no_motion_methods_are_called_in_stage6a(monkeypatch) -> None:
    _install_fake_fx_robot(monkeypatch, _FakeRobotBase)
    _install_fake_fx_kine(monkeypatch)

    adapter = RobotSDKReadOnlyAdapter(RobotSDKConfig(connect_check_samples=3, connect_check_interval_s=0.0, connect_settle_s=0.0))
    adapter.connect()
    _ = adapter.get_dual_arm_feedback()
    adapter.disconnect()

    assert adapter.robot is not None
    assert adapter.robot.set_state_calls == 0
    assert adapter.robot.set_joint_cmd_pose_calls == 0
    assert adapter.robot.move_calls == 0
