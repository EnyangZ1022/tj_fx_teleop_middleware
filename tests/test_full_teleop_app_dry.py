from __future__ import annotations

import importlib.util
from pathlib import Path

from teleop.app import FullTeleopApp, FullTeleopAppConfig
from teleop.core.pose import Pose7
from teleop.core.robot_frame import DualArmRobotFeedback, RobotArmFeedback
from teleop.core.teleop_frame import TeleopArmInput, TeleopFrame


def _pose(x: float, y: float, z: float) -> Pose7:
    return Pose7.from_tuple((x, y, z, 0.0, 0.0, 0.0, 1.0))


def _arm_input(*, xyz: tuple[float, float, float], grip: float, axis_click: bool) -> TeleopArmInput:
    return TeleopArmInput(
        pose_pico=_pose(*xyz),
        valid=True,
        enable=grip >= 0.8,
        gripper_position=0.5,
        gripper_changed=False,
        trigger=0.1,
        grip=grip,
        axis_x=0.0,
        axis_y=0.0,
        axis_click=axis_click,
    )


def _frame(
    *,
    frame_id: int,
    now_ns: int,
    left_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    right_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    grip: float = 0.9,
    left_axis_click: bool = False,
    right_axis_click: bool = False,
) -> TeleopFrame:
    return TeleopFrame(
        frame_id=frame_id,
        source_device_id="pico_test",
        source_timestamp_ns=now_ns,
        pc_receive_time_ns=now_ns,
        left=_arm_input(xyz=left_xyz, grip=grip, axis_click=left_axis_click),
        right=_arm_input(xyz=right_xyz, grip=grip, axis_click=right_axis_click),
        start_pause_requested=False,
        cancel_requested=False,
        calibration_requested=False,
    )


class _FakeTeleopProvider:
    def __init__(self, frames: list[TeleopFrame | None]):
        self._frames = list(frames)
        self._index = 0
        self.started = False
        self.stopped = False

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stopped = True

    def get_latest(self) -> TeleopFrame | None:
        if not self._frames:
            return None
        if self._index >= len(self._frames):
            return self._frames[-1]
        value = self._frames[self._index]
        self._index += 1
        return value


class _FakeSDKAdapter:
    def __init__(self):
        self.connected = False
        self.connect_calls = 0
        self.disconnect_calls = 0

    def connect(self) -> None:
        self.connect_calls += 1
        self.connected = True

    def disconnect(self) -> None:
        self.disconnect_calls += 1
        self.connected = False

    def get_dual_arm_feedback(self) -> DualArmRobotFeedback:
        return DualArmRobotFeedback(
            left=RobotArmFeedback(
                position_xyz=(1000.0, 2000.0, 3000.0),
                orientation_abc=(10.0, 20.0, 30.0),
                valid=True,
            ),
            right=RobotArmFeedback(
                position_xyz=(1200.0, 2200.0, 3200.0),
                orientation_abc=(40.0, 50.0, 60.0),
                valid=True,
            ),
        )


class _FakeCommandAdapter:
    def __init__(self, dry_run: bool):
        self.dry_run = bool(dry_run)
        self.prepared = False
        self.command_enabled = False
        self.active = True
        self.send_calls = 0
        self.sdk_send_calls = 0

    def prepare(self) -> None:
        self.prepared = True

    def enter_command_mode(self) -> None:
        return

    def enable_commands(self) -> None:
        self.command_enabled = True

    def disable_commands(self) -> None:
        self.command_enabled = False

    def stop(self) -> None:
        self.active = False

    def send_command(self, command, now_ns=None):
        _ = (command, now_ns)
        self.send_calls += 1

        if not self.dry_run and self.command_enabled:
            self.sdk_send_calls += 1
            reason = "sent"
        elif self.dry_run:
            reason = "dry_run"
        else:
            reason = "command_disabled"

        return {
            "ok": reason in {"sent", "dry_run"},
            "dry_run": self.dry_run,
            "left_sent": not self.dry_run and self.command_enabled,
            "right_sent": not self.dry_run and self.command_enabled,
            "left_reason": reason,
            "right_reason": reason,
            "left_q_deg": (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0),
            "right_q_deg": (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0),
        }


def test_app_no_command_send_without_calibration() -> None:
    cfg = FullTeleopAppConfig(connect_pico=True, connect_robot=True, dry_run=True, enable_send=False)
    provider = _FakeTeleopProvider([
        _frame(frame_id=1, now_ns=1_000_000_000, grip=0.9, left_axis_click=False, right_axis_click=False),
    ])
    sdk = _FakeSDKAdapter()
    cmd = _FakeCommandAdapter(dry_run=True)

    app = FullTeleopApp(config=cfg, teleop_provider=provider, sdk_adapter=sdk, command_adapter=cmd)
    app.initialize()
    app.step_once(1_010_000_000)
    app.shutdown()

    assert cmd.send_calls == 0


def test_axis_click_rising_edge_only_triggers_calibration_once() -> None:
    cfg = FullTeleopAppConfig(connect_pico=True, connect_robot=True, dry_run=True, enable_send=False)
    provider = _FakeTeleopProvider(
        [
            _frame(frame_id=1, now_ns=1_000_000_000, left_axis_click=False, right_axis_click=False),
            _frame(frame_id=2, now_ns=1_020_000_000, left_axis_click=True, right_axis_click=False),
            _frame(frame_id=3, now_ns=1_040_000_000, left_axis_click=True, right_axis_click=False),
        ]
    )
    sdk = _FakeSDKAdapter()
    cmd = _FakeCommandAdapter(dry_run=True)

    app = FullTeleopApp(config=cfg, teleop_provider=provider, sdk_adapter=sdk, command_adapter=cmd)
    app.initialize()
    app.step_once(1_010_000_000)
    app.step_once(1_030_000_000)
    app.step_once(1_050_000_000)
    app.shutdown()

    assert app.calibration_state is not None
    assert app.calibration_state.left is not None
    assert app.calibration_state.left.source_frame_id == 2


def test_app_no_command_send_when_enable_released() -> None:
    cfg = FullTeleopAppConfig(connect_pico=True, connect_robot=True, dry_run=True, enable_send=False)
    provider = _FakeTeleopProvider(
        [
            _frame(frame_id=1, now_ns=1_000_000_000, grip=0.9, left_axis_click=False, right_axis_click=False),
            _frame(frame_id=2, now_ns=1_020_000_000, grip=0.9, left_axis_click=True, right_axis_click=True),
            _frame(frame_id=3, now_ns=1_040_000_000, grip=0.1, left_axis_click=False, right_axis_click=False),
        ]
    )
    sdk = _FakeSDKAdapter()
    cmd = _FakeCommandAdapter(dry_run=True)

    app = FullTeleopApp(config=cfg, teleop_provider=provider, sdk_adapter=sdk, command_adapter=cmd)
    app.initialize()
    app.step_once(1_010_000_000)
    app.step_once(1_030_000_000)
    send_calls_after_calibration = cmd.send_calls
    app.step_once(1_050_000_000)
    app.shutdown()

    assert cmd.send_calls == send_calls_after_calibration


def test_app_reaches_command_adapter_in_dry_run_without_sdk_send() -> None:
    cfg = FullTeleopAppConfig(connect_pico=True, connect_robot=True, dry_run=True, enable_send=False)
    provider = _FakeTeleopProvider(
        [
            _frame(frame_id=1, now_ns=1_000_000_000, left_axis_click=False, right_axis_click=False),
            _frame(frame_id=2, now_ns=1_020_000_000, left_axis_click=True, right_axis_click=True),
            _frame(
                frame_id=3,
                now_ns=1_040_000_000,
                left_xyz=(0.0, 0.0, 0.05),
                right_xyz=(0.0, 0.0, 0.05),
                left_axis_click=False,
                right_axis_click=False,
            ),
        ]
    )
    sdk = _FakeSDKAdapter()
    cmd = _FakeCommandAdapter(dry_run=True)

    app = FullTeleopApp(config=cfg, teleop_provider=provider, sdk_adapter=sdk, command_adapter=cmd)
    app.initialize()
    app.step_once(1_010_000_000)
    app.step_once(1_030_000_000)
    app.step_once(1_050_000_000)
    app.shutdown()

    assert cmd.send_calls >= 1
    assert cmd.sdk_send_calls == 0


def test_initialize_without_robot_connection() -> None:
    cfg = FullTeleopAppConfig(connect_pico=False, connect_robot=False, dry_run=True)
    app = FullTeleopApp(config=cfg)

    app.initialize()
    app.shutdown()

    assert app.sdk_adapter is None


def test_initialize_without_pico_connection() -> None:
    cfg = FullTeleopAppConfig(connect_pico=False, connect_robot=False, dry_run=True)
    app = FullTeleopApp(config=cfg)

    app.initialize()
    app.shutdown()

    assert app.teleop_provider is None


def test_app_command_config_uses_control_mode_and_rate() -> None:
    cfg = FullTeleopAppConfig(
        connect_pico=False,
        connect_robot=False,
        dry_run=True,
        control_mode="joint_impedance",
        command_rate_hz=50.0,
    )

    app = FullTeleopApp(config=cfg)

    assert app.robot_command_config.control_mode == "joint_impedance"
    assert app.robot_command_config.ctrl_hz == 50


def test_run_full_teleop_script_importable_without_side_effects() -> None:
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "run_full_teleop.py"

    spec = importlib.util.spec_from_file_location("run_full_teleop", script_path)
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    assert hasattr(module, "parse_args")
    assert hasattr(module, "main")
