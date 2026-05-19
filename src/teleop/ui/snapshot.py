from __future__ import annotations

from dataclasses import dataclass
import math
import threading


Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class ArmVisualizationSnapshot:
    target_xyz_mm: Vec3 | None = None
    feedback_xyz_mm: Vec3 | None = None
    target_abc_deg: Vec3 | None = None
    feedback_abc_deg: Vec3 | None = None
    target_valid: bool = False
    feedback_valid: bool = False
    calibrated: bool = False
    active: bool = False
    error_norm_mm: float | None = None
    status: str = ""


@dataclass(frozen=True)
class TeleopVisualizationSnapshot:
    timestamp_ns: int
    left: ArmVisualizationSnapshot
    right: ArmVisualizationSnapshot
    pico_connected: bool = False
    robot_connected: bool = False
    safety_state: str = "UNKNOWN"
    global_status: str = ""
    enable_left: bool = False
    enable_right: bool = False
    pico_frame_age_ms: float | None = None
    command_loop_dt_ms: float | None = None
    target_age_ms: float | None = None
    ik_status: str = ""
    sdk_status: str = ""
    logging_enabled: bool = False
    dropped_log_count: int = 0


def compute_error_norm_mm(
    target_xyz: tuple[float, float, float] | None,
    feedback_xyz: tuple[float, float, float] | None,
) -> float | None:
    if target_xyz is None or feedback_xyz is None:
        return None

    try:
        dx = float(target_xyz[0]) - float(feedback_xyz[0])
        dy = float(target_xyz[1]) - float(feedback_xyz[1])
        dz = float(target_xyz[2]) - float(feedback_xyz[2])
    except (TypeError, ValueError, IndexError):
        return None

    if not (math.isfinite(dx) and math.isfinite(dy) and math.isfinite(dz)):
        return None

    return float(math.sqrt(dx * dx + dy * dy + dz * dz))


class LatestSnapshotStore:
    """Thread-safe latest-value store for UI snapshots."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: TeleopVisualizationSnapshot | None = None

    def set(self, snapshot: TeleopVisualizationSnapshot) -> None:
        with self._lock:
            self._latest = snapshot

    def get(self) -> TeleopVisualizationSnapshot | None:
        with self._lock:
            return self._latest

    def clear(self) -> None:
        with self._lock:
            self._latest = None


__all__ = [
    "Vec3",
    "ArmVisualizationSnapshot",
    "TeleopVisualizationSnapshot",
    "compute_error_norm_mm",
    "LatestSnapshotStore",
]
