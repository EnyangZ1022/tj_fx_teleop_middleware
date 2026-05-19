from __future__ import annotations

import time
from typing import Any, Sequence

from teleop.robot.kinematics_adapter import sdk_arm_to_index
from teleop.robot.sdk_adapter import RobotSDKReadOnlyAdapter
from teleop.robot.startup_config import RobotStartupConfig


def _normalize_joint_tuple(values: Sequence[float]) -> tuple[float, float, float, float, float, float, float]:
    joints = tuple(float(v) for v in values)
    if len(joints) != 7:
        raise ValueError("joints_deg must have length 7")
    return (
        joints[0],
        joints[1],
        joints[2],
        joints[3],
        joints[4],
        joints[5],
        joints[6],
    )


def max_joint_abs_error_deg(current_joints_deg: Sequence[float], target_joints_deg: Sequence[float]) -> float:
    current = _normalize_joint_tuple(current_joints_deg)
    target = _normalize_joint_tuple(target_joints_deg)
    return max(abs(c - t) for c, t in zip(current, target))


def send_joint_command(robot: Any, arm: str, joints_deg: Sequence[float]) -> None:
    joints = _normalize_joint_tuple(joints_deg)

    robot.clear_set()
    robot.set_joint_cmd_pose(arm=arm, joints=list(joints))
    robot.send_cmd()


def enter_position_mode(robot: Any, arm: str, vel_ratio: int, acc_ratio: int) -> None:
    if int(vel_ratio) <= 0:
        raise ValueError("vel_ratio must be positive")
    if int(acc_ratio) <= 0:
        raise ValueError("acc_ratio must be positive")

    robot.clear_set()
    robot.set_vel_acc(arm=arm, velRatio=int(vel_ratio), AccRatio=int(acc_ratio))
    robot.send_cmd()
    time.sleep(0.05)

    robot.clear_set()
    robot.set_state(arm=arm, state=1)
    robot.send_cmd()
    time.sleep(0.05)


def get_current_joints(robot: Any, dcss: Any, arm: str) -> tuple[float, float, float, float, float, float, float]:
    idx = sdk_arm_to_index(arm)

    sub_data = robot.subscribe(dcss)
    if not isinstance(sub_data, dict):
        raise RuntimeError("Failed to subscribe feedback from robot")

    outputs = sub_data.get("outputs")
    if not isinstance(outputs, list) or len(outputs) <= idx:
        raise RuntimeError(f"Missing outputs for arm={arm}")

    joints = outputs[idx].get("fb_joint_pos")
    if not isinstance(joints, list) or len(joints) != 7:
        raise RuntimeError(f"Invalid fb_joint_pos for arm={arm}")

    return _normalize_joint_tuple(joints)


def wait_until_joint_target_reached(
    robot: Any,
    dcss: Any,
    arm: str,
    target_joints_deg: Sequence[float],
    tol_deg: float,
    stable_samples: int,
    timeout_s: float,
    check_period_s: float,
) -> tuple[bool, float]:
    if float(tol_deg) <= 0.0:
        raise ValueError("tol_deg must be positive")
    if int(stable_samples) <= 0:
        raise ValueError("stable_samples must be positive")
    if float(timeout_s) <= 0.0:
        raise ValueError("timeout_s must be positive")
    if float(check_period_s) <= 0.0:
        raise ValueError("check_period_s must be positive")

    target = _normalize_joint_tuple(target_joints_deg)

    stable_count = 0
    last_error = float("inf")
    start_t = time.monotonic()

    while True:
        current = get_current_joints(robot, dcss, arm)
        last_error = max_joint_abs_error_deg(current, target)

        if last_error <= float(tol_deg):
            stable_count += 1
            if stable_count >= int(stable_samples):
                return True, last_error
        else:
            stable_count = 0

        if (time.monotonic() - start_t) >= float(timeout_s):
            return False, last_error

        time.sleep(float(check_period_s))


def move_arm_to_ready_pose(
    robot: Any,
    dcss: Any,
    arm: str,
    ready_joints_deg: Sequence[float],
    startup_config: RobotStartupConfig,
    dry_run: bool = False,
) -> None:
    ready = _normalize_joint_tuple(ready_joints_deg)

    enter_position_mode(
        robot=robot,
        arm=arm,
        vel_ratio=startup_config.vel_ratio,
        acc_ratio=startup_config.acc_ratio,
    )

    if startup_config.pre_wait_s > 0.0:
        time.sleep(float(startup_config.pre_wait_s))

    if dry_run:
        return

    send_joint_command(robot=robot, arm=arm, joints_deg=ready)

    reached, last_error = wait_until_joint_target_reached(
        robot=robot,
        dcss=dcss,
        arm=arm,
        target_joints_deg=ready,
        tol_deg=startup_config.home_tol_deg,
        stable_samples=startup_config.home_stable_samples,
        timeout_s=startup_config.home_timeout_s,
        check_period_s=startup_config.check_period_s,
    )
    if not reached:
        raise RuntimeError(
            f"Arm {arm} failed to reach ready pose within timeout, last max joint error={last_error:.3f} deg"
        )


def move_dual_arm_to_ready_pose(
    startup_config: RobotStartupConfig,
    robot_sdk_adapter: RobotSDKReadOnlyAdapter | None = None,
    *,
    robot: Any | None = None,
    dcss: Any | None = None,
    left_arm: str | None = None,
    right_arm: str | None = None,
    dry_run: bool = False,
) -> None:
    if robot_sdk_adapter is not None:
        robot = robot_sdk_adapter.robot
        dcss = robot_sdk_adapter.dcss
        # Reuse side-to-SDK arm mapping from the existing Stage 6A adapter config.
        left_arm = robot_sdk_adapter._config.left_arm
        right_arm = robot_sdk_adapter._config.right_arm

    if robot is None or dcss is None:
        raise RuntimeError("robot and dcss must be available before startup motion")
    if left_arm is None or right_arm is None:
        raise RuntimeError("left_arm and right_arm must be provided")

    # Safety-first MVP policy: move one arm at a time.
    move_arm_to_ready_pose(
        robot=robot,
        dcss=dcss,
        arm=left_arm,
        ready_joints_deg=startup_config.left_ready_q_deg,
        startup_config=startup_config,
        dry_run=dry_run,
    )
    move_arm_to_ready_pose(
        robot=robot,
        dcss=dcss,
        arm=right_arm,
        ready_joints_deg=startup_config.right_ready_q_deg,
        startup_config=startup_config,
        dry_run=dry_run,
    )


class RobotStartupAdapter:
    """Safe startup adapter to move robot arms to configured ready pose only."""

    def __init__(
        self,
        sdk_adapter: RobotSDKReadOnlyAdapter,
        startup_config: RobotStartupConfig | None = None,
    ) -> None:
        self._sdk_adapter = sdk_adapter
        self._startup_config = startup_config if startup_config is not None else RobotStartupConfig()

    def move_to_ready_pose(self, dry_run: bool = False) -> None:
        if not self._sdk_adapter.connected:
            self._sdk_adapter.connect()

        move_dual_arm_to_ready_pose(
            startup_config=self._startup_config,
            robot_sdk_adapter=self._sdk_adapter,
            dry_run=dry_run,
        )


__all__ = [
    "max_joint_abs_error_deg",
    "send_joint_command",
    "enter_position_mode",
    "get_current_joints",
    "wait_until_joint_target_reached",
    "move_arm_to_ready_pose",
    "move_dual_arm_to_ready_pose",
    "RobotStartupAdapter",
]
