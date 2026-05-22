from __future__ import annotations

import importlib
from pathlib import Path
import sys
import time
from typing import Any

from teleop.core.robot_frame import DualArmRobotFeedback, RobotArmFeedback
from teleop.robot.kinematics_adapter import ArmKinematicsAdapter, sdk_arm_to_index
from teleop.robot.sdk_config import RobotSDKConfig


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _ensure_python_sdk_on_path() -> None:
    sdk_dir = _repo_root() / "python_sdk"
    sdk_dir_str = str(sdk_dir)
    if sdk_dir_str not in sys.path:
        sys.path.insert(0, sdk_dir_str)


def _import_sdk_symbols() -> tuple[type, type]:
    _ensure_python_sdk_on_path()
    module = importlib.import_module("fx_robot")
    if not hasattr(module, "Marvin_Robot") or not hasattr(module, "DCSS"):
        raise RuntimeError("fx_robot Marvin_Robot/DCSS symbols not found")
    return module.Marvin_Robot, module.DCSS


class RobotSDKReadOnlyAdapter:
    """Read-only SDK adapter: connect, clear errors, subscribe feedback, and run FK."""

    def __init__(self, config: RobotSDKConfig | None = None):
        self._config = config if config is not None else RobotSDKConfig()
        self.robot = None
        self.dcss = None
        self.left_kine: ArmKinematicsAdapter | None = None
        self.right_kine: ArmKinematicsAdapter | None = None
        self.connected: bool = False

    def connect(self) -> None:
        MarvinRobot, DCSS = _import_sdk_symbols()

        self.robot = MarvinRobot()
        self.dcss = DCSS()

        connected = self.robot.connect(self._config.robot_ip)
        if not connected:
            raise RuntimeError(f"Failed to connect robot at {self._config.robot_ip}")

        time.sleep(max(0.0, float(self._config.connect_settle_s)))

        self.clear_errors()

        if self._config.disable_sdk_logs:
            if hasattr(self.robot, "log_switch"):
                self.robot.log_switch("0")
            if hasattr(self.robot, "local_log_switch"):
                self.robot.local_log_switch("0")

        self.left_kine = ArmKinematicsAdapter(
            arm=self._config.left_arm,
            kine_cfg_path=self._config.kine_cfg,
            disable_kine_logs=self._config.disable_kine_logs,
        )
        self.right_kine = ArmKinematicsAdapter(
            arm=self._config.right_arm,
            kine_cfg_path=self._config.kine_cfg,
            disable_kine_logs=self._config.disable_kine_logs,
        )
        self.left_kine.initialize()
        self.right_kine.initialize()

        self.check_feedback_stream()
        self.connected = True

    def disconnect(self) -> None:
        try:
            if self.robot is not None and hasattr(self.robot, "release_robot"):
                self.robot.release_robot()
        finally:
            self.connected = False

    def clear_errors(self) -> None:
        if self.robot is None:
            raise RuntimeError("Robot is not initialized")

        self.robot.clear_set()
        self.robot.clear_error(self._config.left_arm)
        self.robot.clear_error(self._config.right_arm)
        self.robot.send_cmd()

    def check_feedback_stream(self) -> None:
        if self.robot is None or self.dcss is None:
            raise RuntimeError("Robot/DCSS not initialized")

        left_idx = sdk_arm_to_index(self._config.left_arm)
        right_idx = sdk_arm_to_index(self._config.right_arm)

        prev_left = None
        prev_right = None
        left_seen_nonzero = False
        right_seen_nonzero = False
        left_updated = False
        right_updated = False

        for _ in range(max(1, int(self._config.connect_check_samples))):
            sub_data = self.robot.subscribe(self.dcss)
            if not isinstance(sub_data, dict):
                time.sleep(max(0.0, float(self._config.connect_check_interval_s)))
                continue

            outputs = sub_data.get("outputs")
            if not isinstance(outputs, list) or len(outputs) <= max(left_idx, right_idx):
                time.sleep(max(0.0, float(self._config.connect_check_interval_s)))
                continue

            left_serial = int(outputs[left_idx].get("frame_serial", 0))
            right_serial = int(outputs[right_idx].get("frame_serial", 0))

            if left_serial > 0:
                left_seen_nonzero = True
            if right_serial > 0:
                right_seen_nonzero = True

            if prev_left is not None and left_serial != prev_left:
                left_updated = True
            if prev_right is not None and right_serial != prev_right:
                right_updated = True

            prev_left = left_serial
            prev_right = right_serial

            time.sleep(max(0.0, float(self._config.connect_check_interval_s)))

        if not (left_seen_nonzero and right_seen_nonzero and left_updated and right_updated):
            raise RuntimeError(
                "Robot feedback stream check failed: frame_serial did not become nonzero and update for both arms"
            )

    def get_joint_feedback(self, side: str) -> tuple[float, float, float, float, float, float, float]:
        side_norm = side.strip().lower()
        if side_norm not in {"left", "right"}:
            raise ValueError("side must be 'left' or 'right'")

        outputs = self._subscribe_outputs()
        return self._extract_joint_feedback_from_outputs(outputs, side_norm)

    def _subscribe_outputs(self) -> list[Any]:
        if not self.connected or self.robot is None or self.dcss is None:
            raise RuntimeError("Robot adapter is not connected")

        sub_data = self.robot.subscribe(self.dcss)
        if not isinstance(sub_data, dict):
            raise RuntimeError("Failed to subscribe robot feedback")

        outputs = sub_data.get("outputs")
        if not isinstance(outputs, list):
            raise RuntimeError("Missing outputs in subscribed robot feedback")
        return outputs

    def _extract_joint_feedback_from_outputs(
        self,
        outputs: list[Any],
        side_norm: str,
    ) -> tuple[float, float, float, float, float, float, float]:
        arm = self._config.left_arm if side_norm == "left" else self._config.right_arm
        idx = sdk_arm_to_index(arm)

        if len(outputs) <= idx:
            raise RuntimeError(f"Missing outputs for side={side_norm}")

        side_output = outputs[idx]
        if not isinstance(side_output, dict):
            raise RuntimeError(f"Invalid output payload for side={side_norm}")

        q = side_output.get("fb_joint_pos")
        if not isinstance(q, list) or len(q) != 7:
            raise RuntimeError(f"Invalid joint feedback for side={side_norm}")

        return tuple(float(v) for v in q)

    def _arm_feedback_from_joint_q(
        self,
        side_norm: str,
        q_deg: tuple[float, float, float, float, float, float, float],
    ) -> RobotArmFeedback:
        if side_norm == "left":
            if self.left_kine is None or not self.left_kine.is_initialized:
                raise RuntimeError("Left arm kinematics is not initialized")
            xyzabc = self.left_kine.fk_xyzabc_mm_deg(q_deg)
        else:
            if self.right_kine is None or not self.right_kine.is_initialized:
                raise RuntimeError("Right arm kinematics is not initialized")
            xyzabc = self.right_kine.fk_xyzabc_mm_deg(q_deg)

        return RobotArmFeedback(
            position_xyz=(float(xyzabc[0]), float(xyzabc[1]), float(xyzabc[2])),
            orientation_abc=(float(xyzabc[3]), float(xyzabc[4]), float(xyzabc[5])),
            valid=True,
        )

    def get_arm_feedback(self, side: str) -> RobotArmFeedback:
        side_norm = side.strip().lower()
        if side_norm not in {"left", "right"}:
            raise ValueError("side must be 'left' or 'right'")

        q_deg = self.get_joint_feedback(side_norm)
        return self._arm_feedback_from_joint_q(side_norm, q_deg)

    def get_dual_arm_feedback(self) -> DualArmRobotFeedback:
        # Stage 6A policy: if one side fails, raise RuntimeError rather than returning partial data.
        outputs = self._subscribe_outputs()
        left_q = self._extract_joint_feedback_from_outputs(outputs, "left")
        right_q = self._extract_joint_feedback_from_outputs(outputs, "right")

        left = self._arm_feedback_from_joint_q("left", left_q)
        right = self._arm_feedback_from_joint_q("right", right_q)
        return DualArmRobotFeedback(left=left, right=right)


__all__ = ["RobotSDKReadOnlyAdapter"]
