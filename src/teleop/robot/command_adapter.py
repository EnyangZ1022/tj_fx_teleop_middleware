from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any, Sequence

from teleop.core.command_frame import ArmCommandTarget, DualArmCommandTarget
from teleop.robot.command_config import RobotCommandConfig
from teleop.robot.ik_adapter import ArmIKAdapter
from teleop.robot.sdk_adapter import RobotSDKReadOnlyAdapter
from teleop.robot.startup import enter_position_mode


def _normalize_q7(values: Sequence[float]) -> tuple[float, float, float, float, float, float, float]:
    q = tuple(float(v) for v in values)
    if len(q) != 7:
        raise ValueError("q must have length 7")
    return (q[0], q[1], q[2], q[3], q[4], q[5], q[6])


def _joint_delta_deg(q_new: Sequence[float], q_ref: Sequence[float]) -> tuple[float, float, float, float, float, float, float]:
    q_new_7 = _normalize_q7(q_new)
    q_ref_7 = _normalize_q7(q_ref)
    return (
        q_new_7[0] - q_ref_7[0],
        q_new_7[1] - q_ref_7[1],
        q_new_7[2] - q_ref_7[2],
        q_new_7[3] - q_ref_7[3],
        q_new_7[4] - q_ref_7[4],
        q_new_7[5] - q_ref_7[5],
        q_new_7[6] - q_ref_7[6],
    )


def _max_abs_joint_delta(q_new: Sequence[float], q_ref: Sequence[float]) -> float:
    return max(abs(v) for v in _joint_delta_deg(q_new, q_ref))


def _clip_joint_step(
    q_candidate: Sequence[float],
    q_ref: Sequence[float],
    max_step_deg: float,
) -> tuple[float, float, float, float, float, float, float]:
    q_candidate_7 = _normalize_q7(q_candidate)
    q_ref_7 = _normalize_q7(q_ref)
    max_step = abs(float(max_step_deg))

    def _clip_delta(delta: float) -> float:
        return max(-max_step, min(max_step, float(delta)))

    delta = _joint_delta_deg(q_candidate_7, q_ref_7)
    clipped = tuple(_clip_delta(v) for v in delta)
    return (
        q_ref_7[0] + clipped[0],
        q_ref_7[1] + clipped[1],
        q_ref_7[2] + clipped[2],
        q_ref_7[3] + clipped[3],
        q_ref_7[4] + clipped[4],
        q_ref_7[5] + clipped[5],
        q_ref_7[6] + clipped[6],
    )


def _validate_arm_target(target: ArmCommandTarget) -> tuple[bool, str]:
    if not bool(target.valid):
        reason = str(target.reason).strip()
        return False, reason if reason else "target_invalid"

    try:
        if len(target.position_xyz_mm) != 3:
            return False, "invalid_position"
        if len(target.orientation_abc_deg) != 3:
            return False, "invalid_orientation"
        if len(target.ik_reference_q_deg) != 7:
            return False, "invalid_ik_reference"
    except Exception:
        return False, "invalid_target"

    return True, "ok"


@dataclass(frozen=True)
class _SideProcessResult:
    sent: bool
    reason: str
    q_deg: tuple[float, float, float, float, float, float, float] | None
    solver_note: str
    step_delta_deg: float | None
    velocity_delta_deg_s: float | None
    allowed_step_deg: float | None
    joint_ramped: bool
    candidate_q_deg: tuple[float, float, float, float, float, float, float] | None
    sent_q_deg: tuple[float, float, float, float, float, float, float] | None


@dataclass(frozen=True)
class _PreparedSideCommand:
    send_planned: bool
    pre_reason: str
    q_to_send: tuple[float, float, float, float, float, float, float] | None
    q_display_deg: tuple[float, float, float, float, float, float, float] | None
    solver_note: str
    step_delta_deg: float | None
    velocity_delta_deg_s: float | None
    allowed_step_deg: float | None
    joint_ramped: bool
    candidate_q_deg: tuple[float, float, float, float, float, float, float] | None


def _new_side_result(
    *,
    sent: bool,
    reason: str,
    q_deg: tuple[float, float, float, float, float, float, float] | None,
    solver_note: str,
    step_delta_deg: float | None,
    velocity_delta_deg_s: float | None,
    allowed_step_deg: float | None,
    joint_ramped: bool,
    candidate_q_deg: tuple[float, float, float, float, float, float, float] | None,
    sent_q_deg: tuple[float, float, float, float, float, float, float] | None,
) -> _SideProcessResult:
    return _SideProcessResult(
        sent=bool(sent),
        reason=str(reason),
        q_deg=q_deg,
        solver_note=str(solver_note),
        step_delta_deg=(float(step_delta_deg) if step_delta_deg is not None else None),
        velocity_delta_deg_s=(float(velocity_delta_deg_s) if velocity_delta_deg_s is not None else None),
        allowed_step_deg=(float(allowed_step_deg) if allowed_step_deg is not None else None),
        joint_ramped=bool(joint_ramped),
        candidate_q_deg=candidate_q_deg,
        sent_q_deg=sent_q_deg,
    )


def _new_result(dry_run: bool) -> dict[str, Any]:
    return {
        "ok": False,
        "dry_run": bool(dry_run),
        "left_sent": False,
        "right_sent": False,
        "left_reason": "not_processed",
        "right_reason": "not_processed",
        "left_q_deg": None,
        "right_q_deg": None,
        "left_candidate_q_deg": None,
        "right_candidate_q_deg": None,
        "left_sent_q_deg": None,
        "right_sent_q_deg": None,
        "left_step_delta_deg": None,
        "right_step_delta_deg": None,
        "left_velocity_delta_deg_s": None,
        "right_velocity_delta_deg_s": None,
        "left_allowed_step_deg": None,
        "right_allowed_step_deg": None,
        "left_joint_ramped": False,
        "right_joint_ramped": False,
        "left_step_ramped": False,
        "right_step_ramped": False,
        "left_solver_note": "",
        "right_solver_note": "",
    }


class RobotCommandAdapter:
    """Convert scheduled command targets into IK and optional SDK joint commands."""

    def __init__(self, sdk_adapter: RobotSDKReadOnlyAdapter, config: RobotCommandConfig | None = None):
        self._sdk_adapter = sdk_adapter
        self._config = config if config is not None else RobotCommandConfig()

        self.left_ik_adapter: ArmIKAdapter | None = None
        self.right_ik_adapter: ArmIKAdapter | None = None

        self.last_sent_left_q_deg: tuple[float, float, float, float, float, float, float] | None = None
        self.last_sent_right_q_deg: tuple[float, float, float, float, float, float, float] | None = None
        self.last_send_time_left_ns: int | None = None
        self.last_send_time_right_ns: int | None = None

        self._prepared: bool = False
        self._commands_enabled: bool = False
        self.active: bool = False

    def prepare(self) -> None:
        if not self._sdk_adapter.connected:
            raise RuntimeError("SDK adapter must be connected before prepare")
        if self._sdk_adapter.left_kine is None or self._sdk_adapter.right_kine is None:
            raise RuntimeError("Kinematics adapters are not available")
        if not self._sdk_adapter.left_kine.is_initialized or not self._sdk_adapter.right_kine.is_initialized:
            raise RuntimeError("Kinematics adapters must be initialized")

        self.left_ik_adapter = ArmIKAdapter(
            self._sdk_adapter.left_kine,
            config=self._config.ik_solver,
            robot_side="left",
        )
        self.right_ik_adapter = ArmIKAdapter(
            self._sdk_adapter.right_kine,
            config=self._config.ik_solver,
            robot_side="right",
        )

        self.reset_last_sent()
        self._commands_enabled = bool(self._config.command_enabled)
        self._prepared = True
        self.active = True

    def enable_commands(self) -> None:
        if not self._prepared:
            raise RuntimeError("prepare() must be called before enable_commands()")
        self._commands_enabled = True

    def disable_commands(self) -> None:
        self._commands_enabled = False

    def enter_command_mode(self) -> None:
        if not self._prepared:
            raise RuntimeError("prepare() must be called before enter_command_mode()")

        robot = self._sdk_adapter.robot
        if robot is None or not self._sdk_adapter.connected:
            raise RuntimeError("Robot is not connected")

        left_arm = self._sdk_adapter._config.left_arm
        right_arm = self._sdk_adapter._config.right_arm

        if self._config.control_mode == "joint_position":
            enter_position_mode(robot, left_arm, self._config.vel_ratio, self._config.acc_ratio)
            enter_position_mode(robot, right_arm, self._config.vel_ratio, self._config.acc_ratio)
            return

        # Optional joint impedance mode; not enabled by default.
        self._enter_joint_impedance_mode(robot, left_arm)
        self._enter_joint_impedance_mode(robot, right_arm)

    def _enter_joint_impedance_mode(self, robot: Any, arm: str) -> None:
        robot.clear_set()
        robot.set_vel_acc(arm=arm, velRatio=int(self._config.vel_ratio), AccRatio=int(self._config.acc_ratio))
        robot.send_cmd()
        time.sleep(0.02)

        if hasattr(robot, "set_state"):
            robot.clear_set()
            robot.set_state(arm=arm, state=3)
            robot.send_cmd()
            time.sleep(0.02)

        if hasattr(robot, "set_impedance_type"):
            robot.clear_set()
            robot.set_impedance_type(arm=arm, type=1)
            robot.send_cmd()
            time.sleep(0.02)

        if hasattr(robot, "set_joint_kd_params"):
            robot.clear_set()
            robot.set_joint_kd_params(arm=arm, K=list(self._config.joint_k), D=list(self._config.joint_d))
            robot.send_cmd()
            time.sleep(0.02)

        if hasattr(robot, "set_vel_est_step"):
            step_ms = max(1, int(round(1000.0 / float(self._config.ctrl_hz))))
            robot.set_vel_est_step(arm=arm, time=step_ms)

    def send_command(self, command: DualArmCommandTarget, now_ns: int | None = None) -> dict[str, Any]:
        dry_run = bool(self._config.dry_run)
        result = _new_result(dry_run)

        if not self._prepared or not self.active:
            result["left_reason"] = "not_prepared"
            result["right_reason"] = "not_prepared"
            return result

        robot = self._sdk_adapter.robot
        if not self._sdk_adapter.connected or robot is None:
            result["left_reason"] = "not_connected"
            result["right_reason"] = "not_connected"
            return result

        if not dry_run and not self._commands_enabled:
            result["left_reason"] = "command_disabled"
            result["right_reason"] = "command_disabled"
            return result

        now = int(time.time_ns() if now_ns is None else now_ns)
        left_old_time_ns = self.last_send_time_left_ns
        right_old_time_ns = self.last_send_time_right_ns

        left_prepared = self._prepare_side(
            side="left",
            target=command.left,
            ik_adapter=self.left_ik_adapter,
            send_allowed=bool(self._config.send_left),
            last_q=self.last_sent_left_q_deg,
            now_ns=now,
            old_time_ns=left_old_time_ns,
        )

        right_prepared = self._prepare_side(
            side="right",
            target=command.right,
            ik_adapter=self.right_ik_adapter,
            send_allowed=bool(self._config.send_right),
            last_q=self.last_sent_right_q_deg,
            now_ns=now,
            old_time_ns=right_old_time_ns,
        )

        packet_sent = False
        if not dry_run:
            commands: list[tuple[str, tuple[float, float, float, float, float, float, float]]] = []
            if left_prepared.send_planned and left_prepared.q_to_send is not None:
                commands.append((self._sdk_adapter._config.left_arm, left_prepared.q_to_send))
            if right_prepared.send_planned and right_prepared.q_to_send is not None:
                commands.append((self._sdk_adapter._config.right_arm, right_prepared.q_to_send))

            if commands:
                packet_sent = self._send_prepared_commands(robot=robot, commands=commands)

        left_result = self._finalize_side_result(
            prepared=left_prepared,
            dry_run=dry_run,
            packet_sent=packet_sent,
        )
        right_result = self._finalize_side_result(
            prepared=right_prepared,
            dry_run=dry_run,
            packet_sent=packet_sent,
        )

        result["left_sent"] = left_result.sent
        result["right_sent"] = right_result.sent
        result["left_reason"] = left_result.reason
        result["right_reason"] = right_result.reason
        result["left_q_deg"] = left_result.q_deg
        result["right_q_deg"] = right_result.q_deg
        result["left_candidate_q_deg"] = left_result.candidate_q_deg
        result["right_candidate_q_deg"] = right_result.candidate_q_deg
        result["left_sent_q_deg"] = left_result.sent_q_deg
        result["right_sent_q_deg"] = right_result.sent_q_deg
        result["left_step_delta_deg"] = left_result.step_delta_deg
        result["right_step_delta_deg"] = right_result.step_delta_deg
        result["left_velocity_delta_deg_s"] = left_result.velocity_delta_deg_s
        result["right_velocity_delta_deg_s"] = right_result.velocity_delta_deg_s
        result["left_allowed_step_deg"] = left_result.allowed_step_deg
        result["right_allowed_step_deg"] = right_result.allowed_step_deg
        result["left_joint_ramped"] = left_result.joint_ramped
        result["right_joint_ramped"] = right_result.joint_ramped
        # Backward-compatible aliases for existing logs and analysis scripts.
        result["left_step_ramped"] = left_result.joint_ramped
        result["right_step_ramped"] = right_result.joint_ramped
        result["left_solver_note"] = left_result.solver_note
        result["right_solver_note"] = right_result.solver_note

        if left_result.sent and left_result.sent_q_deg is not None:
            self.last_sent_left_q_deg = left_result.sent_q_deg
            self.last_send_time_left_ns = now
        if right_result.sent and right_result.sent_q_deg is not None:
            self.last_sent_right_q_deg = right_result.sent_q_deg
            self.last_send_time_right_ns = now

        if dry_run:
            result["ok"] = left_result.reason == "dry_run" or right_result.reason == "dry_run"
        else:
            result["ok"] = bool(left_result.sent or right_result.sent)

        return result

    def _prepare_side(
        self,
        side: str,
        target: ArmCommandTarget | None,
        ik_adapter: ArmIKAdapter | None,
        send_allowed: bool,
        last_q: tuple[float, float, float, float, float, float, float] | None,
        now_ns: int,
        old_time_ns: int | None,
    ) -> _PreparedSideCommand:
        if target is None:
            return _PreparedSideCommand(
                send_planned=False,
                pre_reason="no_target",
                q_to_send=None,
                q_display_deg=None,
                solver_note="",
                step_delta_deg=None,
                velocity_delta_deg_s=None,
                allowed_step_deg=None,
                joint_ramped=False,
                candidate_q_deg=None,
            )

        if not send_allowed:
            return _PreparedSideCommand(
                send_planned=False,
                pre_reason="send_disabled",
                q_to_send=None,
                q_display_deg=None,
                solver_note="",
                step_delta_deg=None,
                velocity_delta_deg_s=None,
                allowed_step_deg=None,
                joint_ramped=False,
                candidate_q_deg=None,
            )

        if ik_adapter is None:
            return _PreparedSideCommand(
                send_planned=False,
                pre_reason="ik_adapter_missing",
                q_to_send=None,
                q_display_deg=None,
                solver_note="ik_adapter_missing",
                step_delta_deg=None,
                velocity_delta_deg_s=None,
                allowed_step_deg=None,
                joint_ramped=False,
                candidate_q_deg=None,
            )

        valid, reason = _validate_arm_target(target)
        if not valid:
            return _PreparedSideCommand(
                send_planned=False,
                pre_reason=reason,
                q_to_send=None,
                q_display_deg=None,
                solver_note="",
                step_delta_deg=None,
                velocity_delta_deg_s=None,
                allowed_step_deg=None,
                joint_ramped=False,
                candidate_q_deg=None,
            )

        q = ik_adapter.solve_xyzabc_mm_deg(
            position_xyz_mm=target.position_xyz_mm,
            orientation_abc_deg=target.orientation_abc_deg,
            ik_reference_q_deg=target.ik_reference_q_deg,
        )
        solver_note = str(getattr(ik_adapter, "last_solver_note", ""))
        if q is None:
            return _PreparedSideCommand(
                send_planned=False,
                pre_reason="ik_failed",
                q_to_send=None,
                q_display_deg=None,
                solver_note=solver_note,
                step_delta_deg=None,
                velocity_delta_deg_s=None,
                allowed_step_deg=None,
                joint_ramped=False,
                candidate_q_deg=None,
            )

        if len(q) != 7:
            return _PreparedSideCommand(
                send_planned=False,
                pre_reason="invalid_ik_result",
                q_to_send=None,
                q_display_deg=None,
                solver_note=solver_note,
                step_delta_deg=None,
                velocity_delta_deg_s=None,
                allowed_step_deg=None,
                joint_ramped=False,
                candidate_q_deg=None,
            )

        q_candidate = _normalize_q7(q)
        q_to_send = q_candidate
        step_delta_deg: float | None = None
        velocity_delta_deg_s: float | None = None
        allowed_step_deg: float | None = None
        joint_ramped = False

        if last_q is not None:
            step_delta_deg = _max_abs_joint_delta(q_candidate, last_q)
            max_step_deg = float(self._config.max_joint_step_deg)
            dt_s: float | None = None
            if old_time_ns is not None:
                dt_candidate_s = float(now_ns - old_time_ns) / 1_000_000_000.0
                if dt_candidate_s > 0.0:
                    dt_s = dt_candidate_s

            # Fallback to nominal command period when previous timestamp is unavailable/invalid.
            if dt_s is None:
                ctrl_hz = float(self._config.ctrl_hz)
                if ctrl_hz > 0.0:
                    dt_s = 1.0 / ctrl_hz

            if dt_s is None or dt_s <= 0.0:
                return _PreparedSideCommand(
                    send_planned=False,
                    pre_reason="invalid_dt_no_fallback",
                    q_to_send=None,
                    q_display_deg=q_candidate,
                    solver_note=solver_note,
                    step_delta_deg=step_delta_deg,
                    velocity_delta_deg_s=None,
                    allowed_step_deg=None,
                    joint_ramped=False,
                    candidate_q_deg=q_candidate,
                )

            velocity_step_deg = float(self._config.max_joint_velocity_deg_s) * dt_s
            allowed_step_deg = min(max_step_deg, velocity_step_deg)
            velocity_delta_deg_s = step_delta_deg / dt_s

            if step_delta_deg > allowed_step_deg:
                if self._config.joint_limit_mode == "reject":
                    if step_delta_deg > max_step_deg:
                        reject_reason = "joint_step_limit"
                    elif step_delta_deg > velocity_step_deg:
                        reject_reason = "joint_velocity_limit"
                    else:
                        reject_reason = "joint_limit"
                    return _PreparedSideCommand(
                        send_planned=False,
                        pre_reason=reject_reason,
                        q_to_send=None,
                        q_display_deg=q_candidate,
                        solver_note=solver_note,
                        step_delta_deg=step_delta_deg,
                        velocity_delta_deg_s=velocity_delta_deg_s,
                        allowed_step_deg=allowed_step_deg,
                        joint_ramped=False,
                        candidate_q_deg=q_candidate,
                    )

                if self._config.joint_limit_mode == "ramp":
                    q_to_send = _clip_joint_step(
                        q_candidate=q_candidate,
                        q_ref=last_q,
                        max_step_deg=allowed_step_deg,
                    )
                    joint_ramped = True
                else:
                    raise ValueError(f"Unsupported joint_limit_mode={self._config.joint_limit_mode!r}")

        return _PreparedSideCommand(
            send_planned=True,
            pre_reason="ready_to_send",
            q_to_send=q_to_send,
            q_display_deg=q_to_send,
            solver_note=solver_note,
            step_delta_deg=step_delta_deg,
            velocity_delta_deg_s=velocity_delta_deg_s,
            allowed_step_deg=allowed_step_deg,
            joint_ramped=joint_ramped,
            candidate_q_deg=q_candidate,
        )

    def _finalize_side_result(
        self,
        *,
        prepared: _PreparedSideCommand,
        dry_run: bool,
        packet_sent: bool,
    ) -> _SideProcessResult:
        if not prepared.send_planned:
            return _new_side_result(
                sent=False,
                reason=prepared.pre_reason,
                q_deg=prepared.q_display_deg,
                solver_note=prepared.solver_note,
                step_delta_deg=prepared.step_delta_deg,
                velocity_delta_deg_s=prepared.velocity_delta_deg_s,
                allowed_step_deg=prepared.allowed_step_deg,
                joint_ramped=prepared.joint_ramped,
                candidate_q_deg=prepared.candidate_q_deg,
                sent_q_deg=None,
            )

        if prepared.q_to_send is None:
            return _new_side_result(
                sent=False,
                reason="send_failed",
                q_deg=None,
                solver_note=prepared.solver_note,
                step_delta_deg=prepared.step_delta_deg,
                velocity_delta_deg_s=prepared.velocity_delta_deg_s,
                allowed_step_deg=prepared.allowed_step_deg,
                joint_ramped=prepared.joint_ramped,
                candidate_q_deg=prepared.candidate_q_deg,
                sent_q_deg=None,
            )

        if dry_run:
            return _new_side_result(
                sent=False,
                reason="dry_run",
                q_deg=prepared.q_to_send,
                solver_note=prepared.solver_note,
                step_delta_deg=prepared.step_delta_deg,
                velocity_delta_deg_s=prepared.velocity_delta_deg_s,
                allowed_step_deg=prepared.allowed_step_deg,
                joint_ramped=prepared.joint_ramped,
                candidate_q_deg=prepared.candidate_q_deg,
                sent_q_deg=prepared.q_to_send,
            )

        if packet_sent:
            return _new_side_result(
                sent=True,
                reason="sent",
                q_deg=prepared.q_to_send,
                solver_note=prepared.solver_note,
                step_delta_deg=prepared.step_delta_deg,
                velocity_delta_deg_s=prepared.velocity_delta_deg_s,
                allowed_step_deg=prepared.allowed_step_deg,
                joint_ramped=prepared.joint_ramped,
                candidate_q_deg=prepared.candidate_q_deg,
                sent_q_deg=prepared.q_to_send,
            )

        return _new_side_result(
            sent=False,
            reason="send_failed",
            q_deg=prepared.q_to_send,
            solver_note=prepared.solver_note,
            step_delta_deg=prepared.step_delta_deg,
            velocity_delta_deg_s=prepared.velocity_delta_deg_s,
            allowed_step_deg=prepared.allowed_step_deg,
            joint_ramped=prepared.joint_ramped,
            candidate_q_deg=prepared.candidate_q_deg,
            sent_q_deg=None,
        )

    def _send_prepared_commands(
        self,
        *,
        robot: Any,
        commands: list[tuple[str, tuple[float, float, float, float, float, float, float]]],
    ) -> bool:
        if not commands:
            return False

        try:
            if not bool(robot.clear_set()):
                return False

            for arm, q_deg in commands:
                if not bool(robot.set_joint_cmd_pose(arm=arm, joints=list(q_deg))):
                    return False

            return bool(robot.send_cmd())
        except Exception:
            return False

    def pause(self) -> None:
        self.disable_commands()

    def stop(self) -> None:
        self.disable_commands()
        self.active = False
        self._prepared = False
        self.reset_last_sent()
        try:
            self._sdk_adapter.disconnect()
        except Exception:
            # Best effort shutdown for MVP safety.
            pass

    def reset_last_sent(self) -> None:
        self.last_sent_left_q_deg = None
        self.last_sent_right_q_deg = None
        self.last_send_time_left_ns = None
        self.last_send_time_right_ns = None


__all__ = ["RobotCommandAdapter"]
