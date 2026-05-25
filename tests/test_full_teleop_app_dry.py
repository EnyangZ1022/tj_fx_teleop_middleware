from __future__ import annotations

import importlib.util
from pathlib import Path
import time

import pytest

from teleop.app import FullTeleopApp, FullTeleopAppConfig
from teleop.core.pose import Pose7
from teleop.core.robot_frame import DualArmRobotFeedback, RobotArmFeedback
from teleop.core.teleop_frame import TeleopArmInput, TeleopFrame
from teleop.filtering import OrientationFilterConfig
from teleop.logging import LoggingConfig
from teleop.safety import SafetyConfig, TargetSafetyGate
from teleop.transform.coordinate_transform import PositionOrientationCoordinateTransformer
from teleop.transform.orientation_transform import OrientationTrackingConfig
from teleop.app.full_teleop_app import sleep_until


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
    receiver_seq: int | None = None,
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
        receiver_seq=receiver_seq,
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


class _SpyLogger:
    def __init__(self):
        self.started = False
        self.stopped = False
        self.events: list[tuple[str, dict | None]] = []
        self.frames: list[tuple[str, dict | None]] = []
        self.performance: list[tuple[str, dict | None]] = []
        self.timing: list[tuple[str, dict | None]] = []
        self.errors: list[tuple[str, dict | None]] = []

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stopped = True

    def log_event(self, event: str, payload: dict | None = None, level: str = "INFO") -> None:
        _ = level
        self.events.append((event, payload))

    def log_frame(self, event: str, payload: dict | None = None, level: str = "DEBUG") -> None:
        _ = level
        self.frames.append((event, payload))

    def log_performance(self, event: str, payload: dict | None = None, level: str = "DEBUG") -> None:
        _ = level
        self.performance.append((event, payload))

    def log_timing(self, event: str, payload: dict | None = None, level: str = "DEBUG") -> None:
        _ = level
        self.timing.append((event, payload))

    def log_error(self, event: str, payload: dict | None = None) -> None:
        self.errors.append((event, payload))

    def get_stats(self):
        class _Stats:
            enabled = True

        return _Stats()


class _CountingSafetyGate(TargetSafetyGate):
    def __init__(self, config: SafetyConfig | None = None):
        super().__init__(config=config)
        self.reacquire_reset_calls = 0

    def reset_reacquire_offsets(self) -> None:
        self.reacquire_reset_calls += 1
        super().reset_reacquire_offsets()


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


def test_timing_logging_mode_emits_lightweight_timing_record() -> None:
    cfg = FullTeleopAppConfig(connect_pico=False, connect_robot=False, dry_run=True, logging_enabled=True)
    spy_logger = _SpyLogger()
    logging_cfg = LoggingConfig(
        enabled=True,
        logging_mode="timing",
        record_events=False,
        record_frames=False,
        record_performance=False,
        record_timing=True,
    )

    app = FullTeleopApp(
        config=cfg,
        logging_config=logging_cfg,
        logger=spy_logger,
    )

    app.initialize()
    app.step_once(1_010_000_000, deadline_late_ms=0.2)
    app.shutdown()

    assert len(spy_logger.timing) >= 1
    assert len(spy_logger.frames) == 0

    timing_event, timing_payload = spy_logger.timing[-1]
    assert timing_event == "teleop_timing"
    assert timing_payload is not None

    required_keys = {
        "loop_seq",
        "loop_wall_ns",
        "loop_perf_ns",
        "loop_dt_ms",
        "loop_total_ms",
        "deadline_late_ms",
        "overrun",
        "pico_receiver_seq",
        "pico_receiver_seq_delta",
        "pico_skipped_receiver_frames",
        "pico_resample_mode",
        "pico_prediction_used",
        "pico_prediction_h_ms",
        "pico_prediction_clamped",
        "pico_prediction_frame_age_ms",
        "pico_prediction_reason",
        "latest_left_input_speed_mm_s",
        "latest_right_input_speed_mm_s",
        "predicted_left_pos_step_mm",
        "predicted_right_pos_step_mm",
        "read_pico_ms",
        "read_feedback_ms",
        "calibration_update_ms",
        "transform_ms",
        "safety_ms",
        "scheduler_ms",
        "send_command_ms",
        "publish_snapshot_ms",
        "loop_tail_ms",
        "command_ready",
        "left_sent",
        "right_sent",
        "left_reason",
        "right_reason",
        "ik_reference_mode",
        "left_ik_reference_source",
        "right_ik_reference_source",
        "joint_ramp_profile",
        "max_joint_step_deg",
        "max_joint_velocity_deg_s",
        "nominal_allowed_step_deg",
        "safety_target_limit_mode",
        "safety_reacquire_mode",
        "safety_left_clamped",
        "safety_right_clamped",
        "safety_left_clamp_reason",
        "safety_right_clamp_reason",
        "safety_left_raw_distance_mm",
        "safety_right_raw_distance_mm",
        "safety_left_allowed_distance_mm",
        "safety_right_allowed_distance_mm",
        "safety_left_clamp_distance_mm",
        "safety_right_clamp_distance_mm",
        "safety_left_raw_to_safe_error_mm",
        "safety_right_raw_to_safe_error_mm",
        "safety_left_reanchored",
        "safety_right_reanchored",
        "safety_left_reanchor_reason",
        "safety_right_reanchor_reason",
        "safety_left_reanchor_gap_ms",
        "safety_right_reanchor_gap_ms",
        "safety_left_reanchor_offset_norm_mm",
        "safety_right_reanchor_offset_norm_mm",
        "safety_left_clamp_streak_ms",
        "safety_right_clamp_streak_ms",
    }
    assert required_keys.issubset(set(timing_payload.keys()))
    assert timing_payload.get("ik_reference_mode") == "fixed"
    assert timing_payload.get("joint_ramp_profile") == "manual"
    assert timing_payload.get("safety_target_limit_mode") == "reject"
    assert timing_payload.get("safety_reacquire_mode") == "none"

    forbidden_keys = {
        "pico_left_xyz_m",
        "pico_left_quat_xyzw",
        "pico_right_xyz_m",
        "pico_right_quat_xyzw",
        "feedback_left_xyz_mm",
        "feedback_left_abc_deg",
        "feedback_right_xyz_mm",
        "feedback_right_abc_deg",
        "command_left_q_deg",
        "command_right_q_deg",
        "command_left_candidate_q_deg",
        "command_right_candidate_q_deg",
        "command_left_sent_q_deg",
        "command_right_sent_q_deg",
    }
    assert forbidden_keys.isdisjoint(set(timing_payload.keys()))


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


def test_axis_click_calibration_resets_safety_reacquire_offsets() -> None:
    cfg = FullTeleopAppConfig(connect_pico=True, connect_robot=True, dry_run=True, enable_send=False)
    provider = _FakeTeleopProvider(
        [
            _frame(frame_id=1, now_ns=1_000_000_000, left_axis_click=False, right_axis_click=False),
            _frame(frame_id=2, now_ns=1_020_000_000, left_axis_click=True, right_axis_click=False),
            _frame(frame_id=3, now_ns=1_040_000_000, left_axis_click=False, right_axis_click=False),
        ]
    )
    sdk = _FakeSDKAdapter()
    cmd = _FakeCommandAdapter(dry_run=True)
    safety_gate = _CountingSafetyGate(
        SafetyConfig(target_limit_mode="clamp", reacquire_mode="position_offset")
    )

    app = FullTeleopApp(
        config=cfg,
        teleop_provider=provider,
        sdk_adapter=sdk,
        command_adapter=cmd,
        safety_gate=safety_gate,
    )
    app.initialize()
    app.step_once(1_010_000_000)
    app.step_once(1_030_000_000)
    app.step_once(1_050_000_000)
    app.shutdown()

    assert safety_gate.reacquire_reset_calls >= 1


def test_timing_logging_reports_receiver_seq_delta_and_skipped_frames() -> None:
    cfg = FullTeleopAppConfig(connect_pico=True, connect_robot=False, dry_run=True, logging_enabled=True)
    provider = _FakeTeleopProvider(
        [
            _frame(frame_id=1, now_ns=1_000_000_000, receiver_seq=100),
            _frame(frame_id=2, now_ns=1_010_000_000, receiver_seq=102),
        ]
    )
    spy_logger = _SpyLogger()
    logging_cfg = LoggingConfig(
        enabled=True,
        logging_mode="timing",
        record_events=False,
        record_frames=False,
        record_performance=False,
        record_timing=True,
    )

    app = FullTeleopApp(
        config=cfg,
        teleop_provider=provider,
        logging_config=logging_cfg,
        logger=spy_logger,
    )

    app.initialize()
    app.step_once(1_005_000_000, deadline_late_ms=0.0)
    app.step_once(1_015_000_000, deadline_late_ms=0.0)
    app.shutdown()

    assert len(spy_logger.timing) >= 2
    _, payload = spy_logger.timing[-1]
    assert payload is not None
    assert payload.get("pico_receiver_seq") == 102
    assert payload.get("pico_receiver_seq_delta") == 2
    assert payload.get("pico_skipped_receiver_frames") == 1


def test_predictive_resample_emits_prediction_metadata_and_clamp() -> None:
    cfg = FullTeleopAppConfig(
        connect_pico=True,
        connect_robot=False,
        dry_run=True,
        logging_enabled=True,
        pico_resample_mode="predictive",
        pico_extrapolation_horizon_ms=15.0,
        pico_prediction_max_frame_age_ms=50.0,
        pico_velocity_filter_beta=1.0,
        pico_max_predicted_step_mm=5.0,
    )
    provider = _FakeTeleopProvider(
        [
            _frame(frame_id=1, now_ns=1_000_000_000, left_xyz=(0.0, 0.0, 0.0), right_xyz=(0.0, 0.0, 0.0), receiver_seq=100),
            _frame(frame_id=2, now_ns=1_010_000_000, left_xyz=(0.01, 0.0, 0.0), right_xyz=(0.01, 0.0, 0.0), receiver_seq=101),
        ]
    )
    spy_logger = _SpyLogger()
    logging_cfg = LoggingConfig(
        enabled=True,
        logging_mode="timing",
        record_events=False,
        record_frames=False,
        record_performance=False,
        record_timing=True,
    )

    app = FullTeleopApp(
        config=cfg,
        teleop_provider=provider,
        logging_config=logging_cfg,
        logger=spy_logger,
    )

    app.initialize()
    app.step_once(1_005_000_000, deadline_late_ms=0.0)
    app.step_once(1_015_000_000, deadline_late_ms=0.0)
    app.step_once(1_025_000_000, deadline_late_ms=0.0)
    app.shutdown()

    assert len(spy_logger.timing) >= 3
    _, payload = spy_logger.timing[-1]
    assert payload is not None
    assert payload.get("pico_resample_mode") == "predictive"
    assert payload.get("pico_prediction_used") is True
    assert payload.get("pico_prediction_clamped") is True
    assert payload.get("pico_prediction_reason") == "step_clamped"
    assert payload.get("predicted_left_pos_step_mm") == pytest.approx(5.0, abs=1e-6)
    assert payload.get("predicted_right_pos_step_mm") == pytest.approx(5.0, abs=1e-6)


def test_predictive_resample_resets_after_calibration() -> None:
    cfg = FullTeleopAppConfig(
        connect_pico=True,
        connect_robot=True,
        dry_run=True,
        enable_send=False,
        logging_enabled=True,
        pico_resample_mode="predictive",
        pico_velocity_filter_beta=1.0,
    )
    provider = _FakeTeleopProvider(
        [
            _frame(frame_id=1, now_ns=1_000_000_000, left_axis_click=False, right_axis_click=False, receiver_seq=200),
            _frame(frame_id=2, now_ns=1_010_000_000, left_axis_click=True, right_axis_click=True, receiver_seq=201),
        ]
    )
    sdk = _FakeSDKAdapter()
    cmd = _FakeCommandAdapter(dry_run=True)
    spy_logger = _SpyLogger()
    logging_cfg = LoggingConfig(
        enabled=True,
        logging_mode="timing",
        record_events=False,
        record_frames=False,
        record_performance=False,
        record_timing=True,
    )

    app = FullTeleopApp(
        config=cfg,
        teleop_provider=provider,
        sdk_adapter=sdk,
        command_adapter=cmd,
        logging_config=logging_cfg,
        logger=spy_logger,
    )

    app.initialize()
    app.step_once(1_005_000_000, deadline_late_ms=0.0)
    app.step_once(1_015_000_000, deadline_late_ms=0.0)
    app.step_once(1_025_000_000, deadline_late_ms=0.0)
    app.shutdown()

    assert len(spy_logger.timing) >= 3
    _, payload = spy_logger.timing[-1]
    assert payload is not None
    assert payload.get("pico_prediction_used") is False
    assert payload.get("pico_prediction_reason") == "no_previous_frame"


def test_run_full_teleop_script_importable_without_side_effects() -> None:
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "run_full_teleop.py"

    spec = importlib.util.spec_from_file_location("run_full_teleop", script_path)
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    assert hasattr(module, "parse_args")
    assert hasattr(module, "main")


def test_sleep_until_returns_immediately_when_deadline_passed() -> None:
    start = time.perf_counter()
    sleep_until(start - 0.001, spin_threshold_s=0.0005)
    assert (time.perf_counter() - start) < 0.01


def test_sleep_until_waits_until_future_deadline() -> None:
    start = time.perf_counter()
    sleep_until(start + 0.002, spin_threshold_s=0.0002)
    elapsed = time.perf_counter() - start

    assert elapsed >= 0.001
    assert elapsed < 0.05


def test_calibration_triggers_orientation_filter_reset_hook() -> None:
    class _SpyPositionOrientationTransformer(PositionOrientationCoordinateTransformer):
        def __init__(self):
            super().__init__(
                orientation_config=OrientationTrackingConfig(enabled=True, orientation_algorithm="absolute_matrix"),
                orientation_filter_config=OrientationFilterConfig(enabled=True),
            )
            self.reset_calls: list[tuple[str | None, int]] = []

        def reset_orientation_filter_from_frame(self, teleop_frame: TeleopFrame, side: str | None = None) -> None:
            self.reset_calls.append((side, int(teleop_frame.frame_id)))
            super().reset_orientation_filter_from_frame(teleop_frame=teleop_frame, side=side)

    cfg = FullTeleopAppConfig(
        connect_pico=True,
        connect_robot=True,
        dry_run=True,
        enable_send=False,
        teleop_mode="position_orientation",
    )
    provider = _FakeTeleopProvider(
        [
            _frame(frame_id=1, now_ns=1_000_000_000, left_axis_click=False, right_axis_click=False),
            _frame(frame_id=2, now_ns=1_020_000_000, left_axis_click=True, right_axis_click=False),
        ]
    )
    sdk = _FakeSDKAdapter()
    cmd = _FakeCommandAdapter(dry_run=True)
    transformer = _SpyPositionOrientationTransformer()

    app = FullTeleopApp(
        config=cfg,
        teleop_provider=provider,
        sdk_adapter=sdk,
        command_adapter=cmd,
        coordinate_transformer=transformer,
    )
    app.initialize()
    app.step_once(1_010_000_000)
    app.step_once(1_030_000_000)
    app.shutdown()

    assert transformer.reset_calls == [(None, 2)]
