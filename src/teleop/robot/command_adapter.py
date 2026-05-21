from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any, Sequence

from teleop.core.command_frame import ArmCommandTarget, DualArmCommandTarget
from teleop.robot.command_config import RobotCommandConfig
from teleop.robot.ik_adapter import ArmIKAdapter
from teleop.robot.sdk_adapter import RobotSDKReadOnlyAdapter
from teleop.robot.startup import enter_position_mode, send_joint_command


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


def _append_solver_note(base_note: str, extra_note: str) -> str:
    if not base_note:
        return str(extra_note)
    return f"{base_note}|{extra_note}"


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
    step_ramped: bool
    candidate_q_deg: tuple[float, float, float, float, float, float, float] | None
    sent_q_deg: tuple[float, float, float, float, float, float, float] | None


def _new_side_result(
    *,
    sent: bool,
    reason: str,
    q_deg: tuple[float, float, float, float, float, float, float] | None,
    solver_note: str,
    step_delta_deg: float | None,
    step_ramped: bool,
    candidate_q_deg: tuple[float, float, float, float, float, float, float] | None,
    sent_q_deg: tuple[float, float, float, float, float, float, float] | None,
) -> _SideProcessResult:
    return _SideProcessResult(
        sent=bool(sent),
        reason=str(reason),
        q_deg=q_deg,
        solver_note=str(solver_note),
        step_delta_deg=(float(step_delta_deg) if step_delta_deg is not None else None),
        step_ramped=bool(step_ramped),
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
        self.last_send_time_ns: int | None = None

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
        old_time_ns = self.last_send_time_ns

        left_result = self._process_side(
            side="left",
            arm=self._sdk_adapter._config.left_arm,
            target=command.left,
            ik_adapter=self.left_ik_adapter,
            send_allowed=bool(self._config.send_left),
            last_q=self.last_sent_left_q_deg,
            now_ns=now,
            old_time_ns=old_time_ns,
            robot=robot,
            dry_run=dry_run,
        )

        right_result = self._process_side(
            side="right",
            arm=self._sdk_adapter._config.right_arm,
            target=command.right,
            ik_adapter=self.right_ik_adapter,
            send_allowed=bool(self._config.send_right),
            last_q=self.last_sent_right_q_deg,
            now_ns=now,
            old_time_ns=old_time_ns,
            robot=robot,
            dry_run=dry_run,
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
        result["left_step_ramped"] = left_result.step_ramped
        result["right_step_ramped"] = right_result.step_ramped
        result["left_solver_note"] = left_result.solver_note
        result["right_solver_note"] = right_result.solver_note

        if left_result.sent and left_result.sent_q_deg is not None:
            self.last_sent_left_q_deg = left_result.sent_q_deg
        if right_result.sent and right_result.sent_q_deg is not None:
            self.last_sent_right_q_deg = right_result.sent_q_deg
        if left_result.sent or right_result.sent:
            self.last_send_time_ns = now

        if dry_run:
            result["ok"] = left_result.reason == "dry_run" or right_result.reason == "dry_run"
        else:
            result["ok"] = bool(left_result.sent or right_result.sent)

        return result

    def _process_side(
        self,
        side: str,
        arm: str,
        target: ArmCommandTarget | None,
        ik_adapter: ArmIKAdapter | None,
        send_allowed: bool,
        last_q: tuple[float, float, float, float, float, float, float] | None,
        now_ns: int,
        old_time_ns: int | None,
        robot: Any,
        dry_run: bool,
    ) -> _SideProcessResult:
        if target is None:
            return _new_side_result(
                sent=False,
                reason="no_target",
                q_deg=None,
                solver_note="",
                step_delta_deg=None,
                step_ramped=False,
                candidate_q_deg=None,
                sent_q_deg=None,
            )

        if not send_allowed:
            return _new_side_result(
                sent=False,
                reason="send_disabled",
                q_deg=None,
                solver_note="",
                step_delta_deg=None,
                step_ramped=False,
                candidate_q_deg=None,
                sent_q_deg=None,
            )

        if ik_adapter is None:
            return _new_side_result(
                sent=False,
                reason="ik_adapter_missing",
                q_deg=None,
                solver_note="ik_adapter_missing",
                step_delta_deg=None,
                step_ramped=False,
                candidate_q_deg=None,
                sent_q_deg=None,
            )

        valid, reason = _validate_arm_target(target)
        if not valid:
            return _new_side_result(
                sent=False,
                reason=reason,
                q_deg=None,
                solver_note="",
                step_delta_deg=None,
                step_ramped=False,
                candidate_q_deg=None,
                sent_q_deg=None,
            )

        q = ik_adapter.solve_xyzabc_mm_deg(
            position_xyz_mm=target.position_xyz_mm,
            orientation_abc_deg=target.orientation_abc_deg,
            ik_reference_q_deg=target.ik_reference_q_deg,
        )
        solver_note = str(getattr(ik_adapter, "last_solver_note", ""))
        if q is None:
            return _new_side_result(
                sent=False,
                reason="ik_failed",
                q_deg=None,
                solver_note=solver_note,
                step_delta_deg=None,
                step_ramped=False,
                candidate_q_deg=None,
                sent_q_deg=None,
            )

        if len(q) != 7:
            return _new_side_result(
                sent=False,
                reason="invalid_ik_result",
                q_deg=None,
                solver_note=solver_note,
                step_delta_deg=None,
                step_ramped=False,
                candidate_q_deg=None,
                sent_q_deg=None,
            )

        q_candidate = _normalize_q7(q)
        q_to_send = q_candidate
        step_delta_deg: float | None = None
        step_ramped = False

        if last_q is not None:
            step_delta_deg = _max_abs_joint_delta(q_candidate, last_q)
            if step_delta_deg > float(self._config.max_joint_step_deg):
                if self._config.joint_step_limit_mode == "reject":
                    return _new_side_result(
                        sent=False,
                        reason="joint_step_limit",
                        q_deg=q_candidate,
                        solver_note=solver_note,
                        step_delta_deg=step_delta_deg,
                        step_ramped=False,
                        candidate_q_deg=q_candidate,
                        sent_q_deg=None,
                    )

                if self._config.joint_step_limit_mode == "ramp":
                    q_to_send = _clip_joint_step(
                        q_candidate=q_candidate,
                        q_ref=last_q,
                        max_step_deg=float(self._config.max_joint_step_deg),
                    )
                    step_ramped = True
                    solver_note = _append_solver_note(solver_note, "joint_step_ramped")
                else:
                    raise ValueError(f"Unsupported joint_step_limit_mode={self._config.joint_step_limit_mode!r}")

            if old_time_ns is not None:
                dt_s = float(now_ns - old_time_ns) / 1_000_000_000.0
                if dt_s <= 0.0:
                    return _new_side_result(
                        sent=False,
                        reason="invalid_dt",
                        q_deg=q_to_send,
                        solver_note=solver_note,
                        step_delta_deg=step_delta_deg,
                        step_ramped=step_ramped,
                        candidate_q_deg=q_candidate,
                        sent_q_deg=None,
                    )
                max_velocity = _max_abs_joint_delta(q_to_send, last_q) / dt_s
                if max_velocity > float(self._config.max_joint_velocity_deg_s):
                    return _new_side_result(
                        sent=False,
                        reason="joint_velocity_limit",
                        q_deg=q_to_send,
                        solver_note=solver_note,
                        step_delta_deg=step_delta_deg,
                        step_ramped=step_ramped,
                        candidate_q_deg=q_candidate,
                        sent_q_deg=None,
                    )

        if dry_run:
            return _new_side_result(
                sent=False,
                reason="dry_run",
                q_deg=q_to_send,
                solver_note=solver_note,
                step_delta_deg=step_delta_deg,
                step_ramped=step_ramped,
                candidate_q_deg=q_candidate,
                sent_q_deg=q_to_send,
            )

        try:
            send_joint_command(robot=robot, arm=arm, joints_deg=q_to_send)
            return _new_side_result(
                sent=True,
                reason="sent",
                q_deg=q_to_send,
                solver_note=solver_note,
                step_delta_deg=step_delta_deg,
                step_ramped=step_ramped,
                candidate_q_deg=q_candidate,
                sent_q_deg=q_to_send,
            )
        except Exception:
            return _new_side_result(
                sent=False,
                reason="send_failed",
                q_deg=q_to_send,
                solver_note=solver_note,
                step_delta_deg=step_delta_deg,
                step_ramped=step_ramped,
                candidate_q_deg=q_candidate,
                sent_q_deg=None,
            )

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
        self.last_send_time_ns = None


__all__ = ["RobotCommandAdapter"]
