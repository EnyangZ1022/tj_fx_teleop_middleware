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

    @property
    def calibration_state(self) -> DualArmCalibrationState | None:
        return self._calibration_state

    def initialize(self) -> None:
        if self._initialized:
            return

        self.logger.start()

        try:
            if self.config.connect_pico and self.teleop_provider is None:
                self.teleop_provider = TeleopProvider(
                    pico_provider=PicoProvider(),
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
            self.logger.log_event(
                "full_app_initialized",
                payload={
                    "connect_pico": bool(self.config.connect_pico),
                    "connect_robot": bool(self.config.connect_robot),
                    "dry_run": bool(self.config.dry_run),
                    "enable_send": bool(self.config.enable_send),
                    "ui_enabled": bool(self.ui_config.enabled),
                    "logging_enabled": bool(self.logging_config.enabled),
                    "teleop_mode": str(self.config.teleop_mode),
                    "orientation_tracking_enabled": bool(self.config.orientation_tracking.enabled),
                    "orientation_relative_mode": str(self.config.orientation_tracking.relative_mode),
                    "orientation_rotation_scale": float(self.config.orientation_tracking.rotation_scale),
                    "orientation_max_total_angle_deg": float(self.config.orientation_tracking.max_total_angle_deg),
                    "orientation_max_step_angle_deg": float(self.config.orientation_tracking.max_step_angle_deg),
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
                now_ns = time.time_ns()
                self.step_once(now_ns)

                if self.config.max_runtime_s is not None and self._loop_start_time_s is not None:
                    elapsed_s = time.perf_counter() - self._loop_start_time_s
                    if elapsed_s >= float(self.config.max_runtime_s):
                        break

                next_tick += period_s
                sleep_s = next_tick - time.perf_counter()
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
                else:
                    next_tick = time.perf_counter()
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
        self._initialized = False

    def step_once(self, now_ns: int) -> None:
        curr_ns = int(now_ns)

        teleop_frame = self._read_teleop_frame()
        feedback = self._read_robot_feedback()

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

            self.logger.log_event(
                "calibration_requested",
                payload={
                    "side": calibration_side or "both",
                    "frame_id": int(teleop_frame.frame_id),
                },
            )

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

        command_target, diagnostics = self.scheduler.step(curr_ns)
        command_target = self._apply_single_arm_mode_to_command(command_target)

        command_result = None
        if command_target is not None and self.command_adapter is not None:
            command_result = self.command_adapter.send_command(command_target, now_ns=curr_ns)

        self._publish_snapshot(
            now_ns=curr_ns,
            teleop_frame=teleop_frame,
            feedback=feedback,
            target=robot_target,
            decision=decision,
            diagnostics=diagnostics,
            command_result=command_result,
        )

        self._latest_feedback = feedback
        self._latest_target = robot_target
        self._latest_decision = decision
        self._latest_diagnostics = diagnostics
        self._latest_command_result = command_result
        self._previous_teleop_frame = teleop_frame

        self.logger.log_frame(
            "teleop_step",
            payload={
                "frame_id": int(teleop_frame.frame_id) if teleop_frame is not None else None,
                "safety_state": str(decision.state.value),
                "allow_motion": bool(decision.allow_motion),
                "command_ready": bool(command_target is not None),
                "teleop_mode": str(self.config.teleop_mode),
                "orientation_tracking_enabled": bool(self.config.orientation_tracking.enabled),
                "left_relative_angle_deg": self._latest_orientation_debug["left"].get("relative_angle_deg"),
                "right_relative_angle_deg": self._latest_orientation_debug["right"].get("relative_angle_deg"),
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


__all__ = ["FullTeleopApp", "SafetyState"]
