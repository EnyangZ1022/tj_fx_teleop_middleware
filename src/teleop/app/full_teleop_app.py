from __future__ import annotations

from dataclasses import replace
import time
from typing import Any

from teleop.app.app_config import FullTeleopAppConfig
from teleop.control.command_scheduler import CommandSchedulerConfig, FixedRateCommandScheduler
from teleop.control.target_buffer import TargetBuffer
from teleop.core.command_frame import CommandLoopDiagnostics, DualArmCommandTarget
from teleop.core.teleop_mode import TeleopMode
from teleop.core.robot_frame import DualArmRobotFeedback, DualArmRobotTarget
from teleop.core.teleop_frame import TeleopFrame
from teleop.input.pico_mapping import PicoInputMapper
from teleop.input.pico_provider import PicoProvider
from teleop.input.teleop_provider import TeleopProvider
from teleop.logging import AsyncSessionLogger, LoggingConfig, NullSessionLogger
from teleop.robot import (
    RobotCommandAdapter,
    RobotCommandConfig,
    RobotSDKConfig,
    RobotSDKReadOnlyAdapter,
    RobotStartupAdapter,
    RobotStartupConfig,
)
from teleop.safety import SafetyConfig, TargetSafetyGate
from teleop.safety.state_machine import SafetyDecision, SafetyState
from teleop.transform.calibration import DualArmCalibrationState, detect_axis_click_calibration_request
from teleop.transform.coordinate_transform import PositionOnlyCoordinateTransformer, PositionOrientationCoordinateTransformer
from teleop.transform.orientation_transform import SDKOrientationConverter
from teleop.ui.snapshot import LatestSnapshotStore
from teleop.ui.snapshot_builder import build_visualization_snapshot
from teleop.ui.ui_config import UIConfig


def sleep_until(deadline_s: float, spin_threshold_s: float = 0.0005) -> None:
    """Sleep until a deadline with a short spin window to reduce wake-up jitter."""
    spin_threshold = max(0.0, float(spin_threshold_s))

    while True:
        remaining_s = float(deadline_s) - time.perf_counter()
        if remaining_s <= 0.0:
            return

        if remaining_s > spin_threshold:
            time.sleep(max(0.0, remaining_s - spin_threshold))
            continue

        # Busy spin only inside a small window near the deadline.
        continue


class FullTeleopApp:
    """Full teleoperation orchestration app for Stage 9 integration.

    The app owns runtime orchestration only:
    Pico -> TeleopFrame -> calibration -> transform -> safety -> buffer -> scheduler -> command adapter.

    UI and logging are optional observers. UI never sends commands and logging stays asynchronous.
    """

    def __init__(
        self,
        config: FullTeleopAppConfig,
        *,
        pico_mapper: PicoInputMapper | None = None,
        robot_sdk_config: RobotSDKConfig | None = None,
        robot_startup_config: RobotStartupConfig | None = None,
        safety_config: SafetyConfig | None = None,
        command_scheduler_config: CommandSchedulerConfig | None = None,
        robot_command_config: RobotCommandConfig | None = None,
        logging_config: LoggingConfig | None = None,
        ui_config: UIConfig | None = None,
        teleop_provider: TeleopProvider | None = None,
        sdk_adapter: RobotSDKReadOnlyAdapter | None = None,
        command_adapter: RobotCommandAdapter | None = None,
        logger: AsyncSessionLogger | NullSessionLogger | None = None,
        snapshot_store: LatestSnapshotStore | None = None,
        coordinate_transformer: PositionOnlyCoordinateTransformer | PositionOrientationCoordinateTransformer | None = None,
        safety_gate: TargetSafetyGate | None = None,
        target_buffer: TargetBuffer | None = None,
        scheduler: FixedRateCommandScheduler | None = None,
    ):
        self.config = config

        self.pico_mapper = pico_mapper if pico_mapper is not None else PicoInputMapper()

        self.robot_sdk_config = robot_sdk_config if robot_sdk_config is not None else RobotSDKConfig(robot_ip=config.robot_ip)
        self.robot_startup_config = (
            robot_startup_config
            if robot_startup_config is not None
            else RobotStartupConfig(pre_wait_s=float(config.startup_wait_s))
        )
        self.safety_config = safety_config if safety_config is not None else SafetyConfig()

        base_scheduler_config = command_scheduler_config if command_scheduler_config is not None else CommandSchedulerConfig()
        if float(base_scheduler_config.rate_hz) != float(config.command_rate_hz):
            base_scheduler_config = replace(base_scheduler_config, rate_hz=float(config.command_rate_hz))
        self.command_scheduler_config = base_scheduler_config

        base_robot_command_config = robot_command_config if robot_command_config is not None else RobotCommandConfig()
        send_left = bool(base_robot_command_config.send_left)
        send_right = bool(base_robot_command_config.send_right)
        if config.single_arm_mode == "left":
            send_right = False
        elif config.single_arm_mode == "right":
            send_left = False
        ctrl_hz = max(1, int(round(float(config.command_rate_hz))))
        self.robot_command_config = replace(
            base_robot_command_config,
            dry_run=bool(config.dry_run),
            command_enabled=bool(config.enable_send),
            control_mode=str(config.control_mode),
            ctrl_hz=ctrl_hz,
            send_left=send_left,
            send_right=send_right,
        )

        if logging_config is None:
            self.logging_config = LoggingConfig(enabled=bool(config.logging_enabled))
        else:
            self.logging_config = replace(
                logging_config,
                enabled=bool(config.logging_enabled and logging_config.enabled),
            )

        if ui_config is None:
            self.ui_config = UIConfig(enabled=bool(config.ui_enabled))
        else:
            self.ui_config = replace(ui_config, enabled=bool(config.ui_enabled and ui_config.enabled))

        self.teleop_provider = teleop_provider
        self.sdk_adapter = sdk_adapter
        self.command_adapter = command_adapter

        if coordinate_transformer is not None:
            self.coordinate_transformer = coordinate_transformer
        elif self.config.teleop_mode == TeleopMode.POSITION_ORIENTATION.value:
            self.coordinate_transformer = PositionOrientationCoordinateTransformer(
                orientation_config=self.config.orientation_tracking,
                orientation_filter_config=self.config.orientation_filter,
            )
        else:
            self.coordinate_transformer = PositionOnlyCoordinateTransformer()
        self.safety_gate = safety_gate if safety_gate is not None else TargetSafetyGate(self.safety_config)
        self.target_buffer = target_buffer if target_buffer is not None else TargetBuffer()
        self.scheduler = (
            scheduler
            if scheduler is not None
            else FixedRateCommandScheduler(self.target_buffer, config=self.command_scheduler_config)
        )

        if logger is not None:
            self.logger = logger
        elif self.logging_config.enabled:
            self.logger = AsyncSessionLogger(self.logging_config)
        else:
            self.logger = NullSessionLogger()

        if snapshot_store is not None:
            self.snapshot_store = snapshot_store
        elif self.ui_config.enabled:
            self.snapshot_store = LatestSnapshotStore()
        else:
            self.snapshot_store = None

        self._initialized = False
        self._running = False

        self._previous_teleop_frame: TeleopFrame | None = None
        self._calibration_state: DualArmCalibrationState | None = None

        self._latest_feedback: DualArmRobotFeedback | None = None
        self._latest_target: DualArmRobotTarget | None = None
        self._latest_decision: SafetyDecision | None = None
        self._latest_diagnostics: CommandLoopDiagnostics | None = None
        self._latest_command_result: dict[str, Any] | None = None
        self._latest_orientation_debug: dict[str, dict[str, float | str | bool | None]] = {
            "left": {"enabled": False, "reason": "idle", "relative_angle_deg": None},
            "right": {"enabled": False, "reason": "idle", "relative_angle_deg": None},
        }
        self._loop_start_time_s: float | None = None
        self._last_loop_perf_ns: int | None = None
        self._last_pico_pc_receive_ns: int | None = None
        self._last_pico_source_timestamp_ns: int | None = None
        self._last_seen_receiver_seq: int | None = None

    @property
    def calibration_state(self) -> DualArmCalibrationState | None:
        return self._calibration_state

    def initialize(self) -> None:
        if self._initialized:
            return

        self.logger.start()

        try:
            if self.config.connect_pico and self.teleop_provider is None:
                receiver_timing_callback = self._on_receiver_timing_payload if self._is_timing_logging_mode() else None
                self.teleop_provider = TeleopProvider(
                    pico_provider=PicoProvider(on_receiver_timing=receiver_timing_callback),
                    mapper=self.pico_mapper,
                )

            if self.config.connect_pico and self.teleop_provider is not None:
                self.teleop_provider.start()

            if self.config.connect_robot and self.sdk_adapter is None:
                self.sdk_adapter = RobotSDKReadOnlyAdapter(config=self.robot_sdk_config)

            if self.config.connect_robot and self.sdk_adapter is not None:
                self.sdk_adapter.connect()
                self._attach_orientation_converters_from_sdk()

                if self.config.move_to_ready:
                    startup = RobotStartupAdapter(
                        sdk_adapter=self.sdk_adapter,
                        startup_config=self.robot_startup_config,
                    )
                    startup.move_to_ready_pose(dry_run=bool(self.config.dry_run))

                if self.command_adapter is None:
                    self.command_adapter = RobotCommandAdapter(
                        sdk_adapter=self.sdk_adapter,
                        config=self.robot_command_config,
                    )

                self.command_adapter.prepare()

                if self.config.enable_send:
                    self.command_adapter.enter_command_mode()
                    self.command_adapter.enable_commands()

            self._initialized = True
            self._reset_orientation_filter_runtime()
            self.logger.log_event(
                "full_app_initialized",
                payload={
                    "connect_pico": bool(self.config.connect_pico),
                    "connect_robot": bool(self.config.connect_robot),
                    "dry_run": bool(self.config.dry_run),
                    "enable_send": bool(self.config.enable_send),
                    "spin_threshold_s": float(self.config.spin_threshold_s),
                    "ui_enabled": bool(self.ui_config.enabled),
                    "logging_enabled": bool(self.logging_config.enabled),
                    "logging_mode": str(self.logging_config.logging_mode),
                    "teleop_mode": str(self.config.teleop_mode),
                    "orientation_tracking_enabled": bool(self.config.orientation_tracking.enabled),
                    "orientation_algorithm": str(self.config.orientation_tracking.orientation_algorithm),
                    "orientation_use_calibration_offset": bool(self.config.orientation_tracking.use_calibration_offset),
                    "orientation_relative_mode": str(self.config.orientation_tracking.relative_mode),
                    "orientation_rotation_scale": float(self.config.orientation_tracking.rotation_scale),
                    "orientation_max_total_angle_deg": float(self.config.orientation_tracking.max_total_angle_deg),
                    "orientation_max_step_angle_deg": float(self.config.orientation_tracking.max_step_angle_deg),
                    "orientation_filter_enabled": bool(self.config.orientation_filter.enabled),
                    "orientation_filter_tau_s": float(self.config.orientation_filter.tau_s),
                    "orientation_filter_fallback_dt_s": float(self.config.orientation_filter.fallback_dt_s),
                    "orientation_filter_reset_on_calibration": bool(
                        self.config.orientation_filter.reset_on_calibration
                    ),
                },
            )
        except Exception:
            self.shutdown()
            raise

    def run(self) -> None:
        if not self._initialized:
            self.initialize()

        self._running = True
        self._loop_start_time_s = time.perf_counter()
        period_s = self.scheduler.period_s()
        next_tick = time.perf_counter()

        try:
            while self._running:
                loop_now_s = time.perf_counter()
                deadline_late_ms = max(0.0, float(loop_now_s - next_tick) * 1000.0)
                now_ns = time.time_ns()
                self.step_once(now_ns, deadline_late_ms=deadline_late_ms)

                if self.config.max_runtime_s is not None and self._loop_start_time_s is not None:
                    elapsed_s = time.perf_counter() - self._loop_start_time_s
                    if elapsed_s >= float(self.config.max_runtime_s):
                        break

                next_tick += period_s
                now_s = time.perf_counter()
                if next_tick > now_s:
                    sleep_until(next_tick, spin_threshold_s=float(self.config.spin_threshold_s))
                else:
                    # Overrun: resync to current time to avoid drift accumulation.
                    next_tick = now_s
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        self._running = False

        if self.command_adapter is not None:
            try:
                self.command_adapter.disable_commands()
            except Exception:
                pass

            try:
                self.command_adapter.stop()
            except Exception:
                pass

        if self.command_adapter is None and self.sdk_adapter is not None:
            try:
                self.sdk_adapter.disconnect()
            except Exception:
                pass

        if self.teleop_provider is not None:
            try:
                self.teleop_provider.stop()
            except Exception:
                pass

        try:
            self.logger.log_event("full_app_shutdown")
        except Exception:
            pass

        self.logger.stop()
        self._reset_orientation_filter_runtime()
        self._last_loop_perf_ns = None
        self._last_pico_pc_receive_ns = None
        self._last_pico_source_timestamp_ns = None
        self._last_seen_receiver_seq = None
        self._initialized = False

    def step_once(self, now_ns: int, *, deadline_late_ms: float | None = None) -> None:
        curr_ns = int(now_ns)
        loop_perf_ns = time.perf_counter_ns()

        if self._last_loop_perf_ns is None:
            loop_dt_ms = None
        else:
            loop_dt_ms = max(0.0, float(loop_perf_ns - self._last_loop_perf_ns) / 1_000_000.0)
        self._last_loop_perf_ns = loop_perf_ns

        teleop_frame = self._read_teleop_frame()
        perf_after_read_pico_ns = time.perf_counter_ns()

        feedback = self._read_robot_feedback()
        perf_after_feedback_ns = time.perf_counter_ns()

        if teleop_frame is not None and self._previous_teleop_frame is None:
            # Reset filter memory when teleop stream starts/restarts to avoid stale state.
            self._reset_orientation_filter_runtime()

        calibration_side = self.config.single_arm_mode
        calibration_requested = False
        if teleop_frame is not None:
            calibration_requested = detect_axis_click_calibration_request(
                self._previous_teleop_frame,
                teleop_frame,
                side=calibration_side,
            )

        if calibration_requested and teleop_frame is not None and feedback is not None:
            if self._calibration_state is None:
                self._calibration_state = self.coordinate_transformer.create_calibration(
                    teleop_frame,
                    feedback,
                    side=calibration_side,
                )
            else:
                self._calibration_state = self.coordinate_transformer.update_calibration(
                    self._calibration_state,
                    teleop_frame,
                    feedback,
                    side=calibration_side,
                )

            self._reset_orientation_filter_on_calibration(
                teleop_frame=teleop_frame,
                side=calibration_side,
            )

            self.logger.log_event(
                "calibration_requested",
                payload={
                    "side": calibration_side or "both",
                    "frame_id": int(teleop_frame.frame_id),
                },
            )
        perf_after_calibration_ns = time.perf_counter_ns()

        robot_target = self._build_robot_target(teleop_frame, feedback)
        self._refresh_orientation_debug_state()

        if robot_target is not None:
            for side in ("left", "right"):
                side_target = robot_target.left if side == "left" else robot_target.right
                if side_target is not None and not side_target.valid and str(side_target.reason).startswith(
                    "orientation_transform_failed"
                ):
                    self.logger.log_error(
                        "orientation_transform_failed",
                        payload={
                            "side": side,
                            "reason": str(side_target.reason),
                            "frame_id": int(teleop_frame.frame_id) if teleop_frame is not None else None,
                        },
                    )
        perf_after_transform_ns = time.perf_counter_ns()

        decision = self.safety_gate.evaluate(
            teleop_frame=teleop_frame,
            robot_target=robot_target,
            calibration=self._calibration_state,
            now_ns=curr_ns,
        )

        if decision.safe_target is not None and decision.allow_motion:
            self.target_buffer.update(decision.safe_target, timestamp_ns=curr_ns)
        else:
            # Never keep sending old targets once current safety evaluation blocks motion.
            self.target_buffer.clear()
        perf_after_safety_ns = time.perf_counter_ns()

        command_target, diagnostics = self.scheduler.step(curr_ns)
        command_target = self._apply_single_arm_mode_to_command(command_target)
        perf_after_scheduler_ns = time.perf_counter_ns()

        command_result = None
        if command_target is not None and self.command_adapter is not None:
            command_result = self.command_adapter.send_command(command_target, now_ns=curr_ns)
        perf_after_send_ns = time.perf_counter_ns()

        self._publish_snapshot(
            now_ns=curr_ns,
            teleop_frame=teleop_frame,
            feedback=feedback,
            target=robot_target,
            decision=decision,
            diagnostics=diagnostics,
            command_result=command_result,
        )
        perf_after_snapshot_ns = time.perf_counter_ns()

        frame_id = int(teleop_frame.frame_id) if teleop_frame is not None else None
        prev_frame_id = int(self._previous_teleop_frame.frame_id) if self._previous_teleop_frame is not None else None
        receiver_seq = int(teleop_frame.receiver_seq) if teleop_frame is not None and teleop_frame.receiver_seq is not None else None

        pico_frame_new: bool | None = None
        if frame_id is not None:
            pico_frame_new = bool(prev_frame_id is None or frame_id != prev_frame_id)

        pico_receiver_seq_delta: int | None = None
        pico_skipped_receiver_frames: int | None = None
        if receiver_seq is not None and self._last_seen_receiver_seq is not None:
            pico_receiver_seq_delta = int(receiver_seq - self._last_seen_receiver_seq)
            pico_skipped_receiver_frames = int(max(0, pico_receiver_seq_delta - 1))
        if receiver_seq is not None:
            self._last_seen_receiver_seq = receiver_seq

        frame_age_ms: float | None = None
        pico_pc_rx_dt_ms: float | None = None
        pico_internal_dt_ms: float | None = None

        if teleop_frame is not None:
            pc_receive_ns = int(teleop_frame.pc_receive_time_ns)
            if pc_receive_ns > 0:
                frame_age_ms = max(0.0, float(curr_ns - pc_receive_ns) / 1_000_000.0)
                if self._last_pico_pc_receive_ns is not None:
                    pico_pc_rx_dt_ms = max(0.0, float(pc_receive_ns - self._last_pico_pc_receive_ns) / 1_000_000.0)
                self._last_pico_pc_receive_ns = pc_receive_ns

            source_timestamp_ns = int(teleop_frame.source_timestamp_ns)
            if source_timestamp_ns > 0:
                if self._last_pico_source_timestamp_ns is not None:
                    pico_internal_dt_ms = max(
                        0.0,
                        float(source_timestamp_ns - self._last_pico_source_timestamp_ns) / 1_000_000.0,
                    )
                self._last_pico_source_timestamp_ns = source_timestamp_ns

        self._latest_feedback = feedback
        self._latest_target = robot_target
        self._latest_decision = decision
        self._latest_diagnostics = diagnostics
        self._latest_command_result = command_result
        self._previous_teleop_frame = teleop_frame

        left_pose = teleop_frame.left.pose_pico if teleop_frame is not None else None
        right_pose = teleop_frame.right.pose_pico if teleop_frame is not None else None

        perf_before_log_ns = time.perf_counter_ns()
        if self._is_full_logging_mode():
            self._log_full_step(
                teleop_frame=teleop_frame,
                feedback=feedback,
                decision=decision,
                command_target=command_target,
                diagnostics=diagnostics,
                command_result=command_result,
                left_pose=left_pose,
                right_pose=right_pose,
            )
        elif self._is_timing_logging_mode():
            period_ms = float(self.scheduler.period_s()) * 1000.0
            self._log_timing_step(
                curr_ns=curr_ns,
                loop_perf_ns=loop_perf_ns,
                loop_dt_ms=loop_dt_ms,
                deadline_late_ms=deadline_late_ms,
                period_ms=period_ms,
                teleop_frame=teleop_frame,
                feedback=feedback,
                robot_target=robot_target,
                decision=decision,
                diagnostics=diagnostics,
                command_target=command_target,
                command_result=command_result,
                pico_frame_new=pico_frame_new,
                receiver_seq=receiver_seq,
                pico_receiver_seq_delta=pico_receiver_seq_delta,
                pico_skipped_receiver_frames=pico_skipped_receiver_frames,
                frame_age_ms=frame_age_ms,
                pico_pc_rx_dt_ms=pico_pc_rx_dt_ms,
                pico_internal_dt_ms=pico_internal_dt_ms,
                perf_after_read_pico_ns=perf_after_read_pico_ns,
                perf_after_feedback_ns=perf_after_feedback_ns,
                perf_after_calibration_ns=perf_after_calibration_ns,
                perf_after_transform_ns=perf_after_transform_ns,
                perf_after_safety_ns=perf_after_safety_ns,
                perf_after_scheduler_ns=perf_after_scheduler_ns,
                perf_after_send_ns=perf_after_send_ns,
                perf_after_snapshot_ns=perf_after_snapshot_ns,
                perf_before_log_ns=perf_before_log_ns,
            )

    def _is_full_logging_mode(self) -> bool:
        return bool(self.logging_config.enabled) and str(self.logging_config.logging_mode) == "full"

    def _is_timing_logging_mode(self) -> bool:
        return bool(self.logging_config.enabled) and str(self.logging_config.logging_mode) == "timing"

    def _on_receiver_timing_payload(self, payload: dict[str, object]) -> None:
        if not self._is_timing_logging_mode():
            return

        log_receiver_timing = getattr(self.logger, "log_receiver_timing", None)
        if callable(log_receiver_timing):
            log_receiver_timing("pico_receiver_timing", payload=payload)
            return

        # Fallback keeps diagnostics best-effort if custom loggers do not implement receiver timing.
        self.logger.log_performance("pico_receiver_timing", payload=payload)

    def _log_full_step(
        self,
        *,
        teleop_frame: TeleopFrame | None,
        feedback: DualArmRobotFeedback | None,
        decision: SafetyDecision,
        command_target: DualArmCommandTarget | None,
        diagnostics: CommandLoopDiagnostics,
        command_result: dict[str, Any] | None,
        left_pose: Any,
        right_pose: Any,
    ) -> None:
        self.logger.log_frame(
            "teleop_step",
            payload={
                "frame_id": int(teleop_frame.frame_id) if teleop_frame is not None else None,
                "safety_state": str(decision.state.value),
                "allow_motion": bool(decision.allow_motion),
                "command_ready": bool(command_target is not None),
                "teleop_mode": str(self.config.teleop_mode),
                "orientation_tracking_enabled": bool(self.config.orientation_tracking.enabled),
                "orientation_algorithm": str(self.config.orientation_tracking.orientation_algorithm),
                "orientation_use_calibration_offset": bool(self.config.orientation_tracking.use_calibration_offset),
                "orientation_filter_enabled": bool(self.config.orientation_filter.enabled),
                "orientation_filter_tau_s": float(self.config.orientation_filter.tau_s),
                "orientation_filter_fallback_dt_s": float(self.config.orientation_filter.fallback_dt_s),
                "left_relative_angle_deg": self._latest_orientation_debug["left"].get("relative_angle_deg"),
                "right_relative_angle_deg": self._latest_orientation_debug["right"].get("relative_angle_deg"),
                "pico_left_xyz_m": [left_pose.x, left_pose.y, left_pose.z] if left_pose is not None else None,
                "pico_left_quat_xyzw": [left_pose.qx, left_pose.qy, left_pose.qz, left_pose.qw] if left_pose is not None else None,
                "pico_right_xyz_m": [right_pose.x, right_pose.y, right_pose.z] if right_pose is not None else None,
                "pico_right_quat_xyzw": [right_pose.qx, right_pose.qy, right_pose.qz, right_pose.qw] if right_pose is not None else None,
                "feedback_left_xyz_mm": (
                    list(feedback.left.position_xyz)
                    if feedback is not None and feedback.left is not None
                    else None
                ),
                "feedback_left_abc_deg": (
                    list(feedback.left.orientation_abc)
                    if feedback is not None and feedback.left is not None
                    else None
                ),
                "feedback_right_xyz_mm": (
                    list(feedback.right.position_xyz)
                    if feedback is not None and feedback.right is not None
                    else None
                ),
                "feedback_right_abc_deg": (
                    list(feedback.right.orientation_abc)
                    if feedback is not None and feedback.right is not None
                    else None
                ),
                "command_left_q_deg": command_result.get("left_q_deg") if command_result is not None else None,
                "command_right_q_deg": command_result.get("right_q_deg") if command_result is not None else None,
                "command_left_candidate_q_deg": (
                    command_result.get("left_candidate_q_deg") if command_result is not None else None
                ),
                "command_right_candidate_q_deg": (
                    command_result.get("right_candidate_q_deg") if command_result is not None else None
                ),
                "command_left_sent_q_deg": command_result.get("left_sent_q_deg") if command_result is not None else None,
                "command_right_sent_q_deg": command_result.get("right_sent_q_deg") if command_result is not None else None,
                "command_left_step_delta_deg": (
                    command_result.get("left_step_delta_deg") if command_result is not None else None
                ),
                "command_right_step_delta_deg": (
                    command_result.get("right_step_delta_deg") if command_result is not None else None
                ),
                "command_left_velocity_delta_deg_s": (
                    command_result.get("left_velocity_delta_deg_s") if command_result is not None else None
                ),
                "command_right_velocity_delta_deg_s": (
                    command_result.get("right_velocity_delta_deg_s") if command_result is not None else None
                ),
                "command_left_allowed_step_deg": (
                    command_result.get("left_allowed_step_deg") if command_result is not None else None
                ),
                "command_right_allowed_step_deg": (
                    command_result.get("right_allowed_step_deg") if command_result is not None else None
                ),
                "command_left_joint_ramped": (
                    command_result.get("left_joint_ramped") if command_result is not None else None
                ),
                "command_right_joint_ramped": (
                    command_result.get("right_joint_ramped") if command_result is not None else None
                ),
                "command_left_step_ramped": command_result.get("left_step_ramped") if command_result is not None else None,
                "command_right_step_ramped": (
                    command_result.get("right_step_ramped") if command_result is not None else None
                ),
                "command_left_sent": command_result.get("left_sent") if command_result is not None else None,
                "command_right_sent": command_result.get("right_sent") if command_result is not None else None,
                "command_left_reason": command_result.get("left_reason") if command_result is not None else None,
                "command_right_reason": command_result.get("right_reason") if command_result is not None else None,
            },
        )
        self.logger.log_performance(
            "command_loop",
            payload={
                "dt_ms": float(diagnostics.dt_ms),
                "target_age_ms": diagnostics.target_age_ms,
                "zoh": bool(diagnostics.used_zero_order_hold),
                "limited": bool(diagnostics.limited),
                "limit_reason": str(diagnostics.limit_reason),
            },
        )

    def _log_timing_step(
        self,
        *,
        curr_ns: int,
        loop_perf_ns: int,
        loop_dt_ms: float | None,
        deadline_late_ms: float | None,
        period_ms: float,
        teleop_frame: TeleopFrame | None,
        feedback: DualArmRobotFeedback | None,
        robot_target: DualArmRobotTarget | None,
        decision: SafetyDecision,
        diagnostics: CommandLoopDiagnostics,
        command_target: DualArmCommandTarget | None,
        command_result: dict[str, Any] | None,
        pico_frame_new: bool | None,
        receiver_seq: int | None,
        pico_receiver_seq_delta: int | None,
        pico_skipped_receiver_frames: int | None,
        frame_age_ms: float | None,
        pico_pc_rx_dt_ms: float | None,
        pico_internal_dt_ms: float | None,
        perf_after_read_pico_ns: int,
        perf_after_feedback_ns: int,
        perf_after_calibration_ns: int,
        perf_after_transform_ns: int,
        perf_after_safety_ns: int,
        perf_after_scheduler_ns: int,
        perf_after_send_ns: int,
        perf_after_snapshot_ns: int,
        perf_before_log_ns: int,
    ) -> None:
        frame_id = int(teleop_frame.frame_id) if teleop_frame is not None else None
        loop_dt_ms_value = float(loop_dt_ms) if loop_dt_ms is not None else float(diagnostics.dt_ms)
        deadline_late_ms_value = max(0.0, float(deadline_late_ms)) if deadline_late_ms is not None else 0.0
        loop_total_ms = max(0.0, float(perf_before_log_ns - loop_perf_ns) / 1_000_000.0)
        overrun = bool(deadline_late_ms_value > 0.0 or loop_total_ms > float(period_ms))

        command_ready = bool(command_target is not None)
        left_sent = bool(command_result.get("left_sent")) if command_result is not None else False
        right_sent = bool(command_result.get("right_sent")) if command_result is not None else False
        left_reason = str(command_result.get("left_reason")) if command_result is not None else "not_sent"
        right_reason = str(command_result.get("right_reason")) if command_result is not None else "not_sent"
        send_ok = bool(command_result.get("ok")) if command_result is not None else False
        send_failed = bool(command_ready and (command_result is None or not send_ok))

        payload: dict[str, Any] = {
            "loop_seq": int(diagnostics.sequence_id),
            "loop_wall_ns": int(curr_ns),
            "loop_perf_ns": int(loop_perf_ns),
            "loop_dt_ms": loop_dt_ms_value,
            "loop_total_ms": loop_total_ms,
            "deadline_late_ms": deadline_late_ms_value,
            "overrun": overrun,
            "pico_frame_id": frame_id,
            "frame_seq": frame_id,
            "pico_frame_new": pico_frame_new,
            "pico_receiver_seq": receiver_seq,
            "pico_receiver_seq_delta": pico_receiver_seq_delta,
            "pico_skipped_receiver_frames": pico_skipped_receiver_frames,
            "frame_age_ms": frame_age_ms,
            "pico_pc_rx_dt_ms": pico_pc_rx_dt_ms,
            "pico_internal_dt_ms": pico_internal_dt_ms,
            "read_pico_ms": max(0.0, float(perf_after_read_pico_ns - loop_perf_ns) / 1_000_000.0),
            "read_feedback_ms": max(0.0, float(perf_after_feedback_ns - perf_after_read_pico_ns) / 1_000_000.0),
            "calibration_update_ms": max(
                0.0,
                float(perf_after_calibration_ns - perf_after_feedback_ns) / 1_000_000.0,
            ),
            "transform_ms": max(0.0, float(perf_after_transform_ns - perf_after_calibration_ns) / 1_000_000.0),
            "safety_ms": max(0.0, float(perf_after_safety_ns - perf_after_transform_ns) / 1_000_000.0),
            "scheduler_ms": max(0.0, float(perf_after_scheduler_ns - perf_after_safety_ns) / 1_000_000.0),
            "send_command_ms": max(0.0, float(perf_after_send_ns - perf_after_scheduler_ns) / 1_000_000.0),
            "publish_snapshot_ms": max(0.0, float(perf_after_snapshot_ns - perf_after_send_ns) / 1_000_000.0),
            "loop_tail_ms": max(0.0, float(perf_before_log_ns - perf_after_snapshot_ns) / 1_000_000.0),
            "command_ready": command_ready,
            "target_available": bool(robot_target is not None),
            "no_target": bool(robot_target is None),
            "left_sent": left_sent,
            "right_sent": right_sent,
            "left_reason": left_reason,
            "right_reason": right_reason,
            "send_ok": send_ok,
            "send_failed": send_failed,
            "safety_state": str(decision.state.value),
            "safety_reason": str(decision.global_reason),
            "safety_left_reason": str(decision.left_reason),
            "safety_right_reason": str(decision.right_reason),
            "feedback_available": bool(feedback is not None),
            "scheduler_dt_ms": float(diagnostics.dt_ms),
            "scheduler_target_age_ms": diagnostics.target_age_ms,
            "scheduler_zoh": bool(diagnostics.used_zero_order_hold),
            "scheduler_limited": bool(diagnostics.limited),
            "scheduler_limit_reason": str(diagnostics.limit_reason),
        }

        log_timing = getattr(self.logger, "log_timing", None)
        if callable(log_timing):
            log_timing("teleop_timing", payload=payload)
        else:
            self.logger.log_performance("teleop_timing", payload=payload)

    def request_stop(self) -> None:
        self._running = False

    def _read_teleop_frame(self) -> TeleopFrame | None:
        if not self.config.connect_pico or self.teleop_provider is None:
            return None

        try:
            return self.teleop_provider.get_latest()
        except Exception as exc:
            self.logger.log_error("pico_read_failed", payload={"error": str(exc)})
            return None

    def _read_robot_feedback(self) -> DualArmRobotFeedback | None:
        if not self.config.connect_robot or self.sdk_adapter is None:
            return None

        if not bool(getattr(self.sdk_adapter, "connected", False)):
            return None

        try:
            return self.sdk_adapter.get_dual_arm_feedback()
        except Exception as exc:
            self.logger.log_error("robot_feedback_failed", payload={"error": str(exc)})
            return None

    def _build_robot_target(
        self,
        teleop_frame: TeleopFrame | None,
        feedback: DualArmRobotFeedback | None,
    ) -> DualArmRobotTarget | None:
        if teleop_frame is None or self._calibration_state is None:
            return None

        target = self.coordinate_transformer.make_target(
            teleop_frame=teleop_frame,
            robot_feedback=feedback,
            calibration=self._calibration_state,
        )
        return self._apply_single_arm_mode_to_target(target)

    def _refresh_orientation_debug_state(self) -> None:
        if hasattr(self.coordinate_transformer, "latest_orientation_debug"):
            try:
                debug = self.coordinate_transformer.latest_orientation_debug()
                if isinstance(debug, dict):
                    self._latest_orientation_debug = {
                        "left": dict(debug.get("left", {})),
                        "right": dict(debug.get("right", {})),
                    }
            except Exception:
                pass

    def _attach_orientation_converters_from_sdk(self) -> None:
        if not hasattr(self.coordinate_transformer, "set_orientation_converters"):
            return
        if self.sdk_adapter is None:
            return

        converters: dict[str, SDKOrientationConverter] = {}

        left_kine_obj = getattr(getattr(self.sdk_adapter, "left_kine", None), "_kine", None)
        if left_kine_obj is not None:
            converters["left"] = SDKOrientationConverter(left_kine_obj)

        right_kine_obj = getattr(getattr(self.sdk_adapter, "right_kine", None), "_kine", None)
        if right_kine_obj is not None:
            converters["right"] = SDKOrientationConverter(right_kine_obj)

        try:
            self.coordinate_transformer.set_orientation_converters(converters)
        except Exception:
            # Orientation converter wiring is best effort and must not break startup.
            pass

    def _reset_orientation_filter_runtime(self) -> None:
        if not hasattr(self.coordinate_transformer, "reset_orientation_filter_all"):
            return
        try:
            self.coordinate_transformer.reset_orientation_filter_all()
        except Exception:
            # Orientation filter reset is best effort and must not block runtime.
            pass

    def _reset_orientation_filter_on_calibration(
        self,
        *,
        teleop_frame: TeleopFrame,
        side: str | None,
    ) -> None:
        if not hasattr(self.coordinate_transformer, "reset_orientation_filter_from_frame"):
            return
        try:
            self.coordinate_transformer.reset_orientation_filter_from_frame(teleop_frame=teleop_frame, side=side)
        except Exception:
            # Calibration should still succeed even if filter reset hook fails.
            pass

    def _apply_single_arm_mode_to_target(self, target: DualArmRobotTarget | None) -> DualArmRobotTarget | None:
        if target is None:
            return None

        if self.config.single_arm_mode == "left":
            return DualArmRobotTarget(left=target.left, right=None)
        if self.config.single_arm_mode == "right":
            return DualArmRobotTarget(left=None, right=target.right)
        return target

    def _apply_single_arm_mode_to_command(
        self,
        command: DualArmCommandTarget | None,
    ) -> DualArmCommandTarget | None:
        if command is None:
            return None

        if self.config.single_arm_mode == "left":
            return DualArmCommandTarget(left=command.left, right=None)
        if self.config.single_arm_mode == "right":
            return DualArmCommandTarget(left=None, right=command.right)
        return command

    def _publish_snapshot(
        self,
        *,
        now_ns: int,
        teleop_frame: TeleopFrame | None,
        feedback: DualArmRobotFeedback | None,
        target: DualArmRobotTarget | None,
        decision: SafetyDecision,
        diagnostics: CommandLoopDiagnostics,
        command_result: dict[str, Any] | None,
    ) -> None:
        if self.snapshot_store is None:
            return

        if teleop_frame is not None and int(teleop_frame.pc_receive_time_ns) > 0:
            pico_frame_age_ms = max(0.0, float(now_ns - int(teleop_frame.pc_receive_time_ns)) / 1_000_000.0)
        else:
            pico_frame_age_ms = None

        left_calibrated = bool(self._calibration_state is not None and self._calibration_state.left is not None)
        right_calibrated = bool(self._calibration_state is not None and self._calibration_state.right is not None)

        ik_status = "idle"
        if command_result is not None:
            left_reason = str(command_result.get("left_reason", ""))
            right_reason = str(command_result.get("right_reason", ""))
            if "ik_failed" in {left_reason, right_reason}:
                ik_status = "ik_failed"
            elif command_result.get("left_q_deg") is not None or command_result.get("right_q_deg") is not None:
                ik_status = "ok"

        if not self.config.connect_robot:
            sdk_status = "disabled"
        elif self.sdk_adapter is None:
            sdk_status = "uninitialized"
        elif bool(getattr(self.sdk_adapter, "connected", False)):
            sdk_status = "connected"
        else:
            sdk_status = "disconnected"

        snapshot = build_visualization_snapshot(
            timestamp_ns=now_ns,
            target=target,
            feedback=feedback,
            safety_decision=decision,
            command_diagnostics=diagnostics,
            logging_stats=self.logger.get_stats(),
            pico_connected=teleop_frame is not None,
            robot_connected=bool(self.sdk_adapter is not None and getattr(self.sdk_adapter, "connected", False)),
            pico_frame_age_ms=pico_frame_age_ms,
            enable_left=bool(teleop_frame.left.enable) if teleop_frame is not None else False,
            enable_right=bool(teleop_frame.right.enable) if teleop_frame is not None else False,
            left_calibrated=left_calibrated,
            right_calibrated=right_calibrated,
            ik_status=ik_status,
            sdk_status=sdk_status,
            teleop_mode=str(self.config.teleop_mode),
            orientation_tracking_enabled=bool(self.config.orientation_tracking.enabled),
            orientation_relative_mode=(
                str(self.config.orientation_tracking.relative_mode)
                if bool(self.config.orientation_tracking.enabled)
                else ""
            ),
            left_relative_angle_deg=self._latest_orientation_debug["left"].get("relative_angle_deg"),
            right_relative_angle_deg=self._latest_orientation_debug["right"].get("relative_angle_deg"),
        )
        self.snapshot_store.set(snapshot)


__all__ = ["FullTeleopApp", "SafetyState", "sleep_until"]
