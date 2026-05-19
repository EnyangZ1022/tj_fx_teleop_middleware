from __future__ import annotations

from teleop.core.robot_frame import DualArmRobotTarget


class TargetBuffer:
    """Store latest safe robot target for fixed-rate scheduling."""

    def __init__(self) -> None:
        self._latest_target: DualArmRobotTarget | None = None
        self._latest_timestamp_ns: int | None = None

    def update(self, target: DualArmRobotTarget, timestamp_ns: int) -> None:
        self._latest_target = target
        self._latest_timestamp_ns = int(timestamp_ns)

    def get_latest(self, now_ns: int) -> tuple[DualArmRobotTarget | None, float | None]:
        if self._latest_target is None or self._latest_timestamp_ns is None:
            return None, None

        age_ns = int(now_ns) - self._latest_timestamp_ns
        age_ms = max(0.0, float(age_ns) / 1_000_000.0)
        return self._latest_target, age_ms

    def latest_timestamp_ns(self) -> int | None:
        return self._latest_timestamp_ns

    def clear(self) -> None:
        self._latest_target = None
        self._latest_timestamp_ns = None


__all__ = ["TargetBuffer"]
