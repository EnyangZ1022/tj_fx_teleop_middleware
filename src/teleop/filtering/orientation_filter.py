from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


_HEMISPHERE_EPS = 1e-12
_SLERP_LINEAR_THRESHOLD = 0.9995


@dataclass(frozen=True)
class OrientationFilterConfig:
    enabled: bool = True
    tau_s: float = 0.02
    fallback_dt_s: float = 0.01
    reset_on_calibration: bool = True

    def __post_init__(self) -> None:
        tau_s = float(self.tau_s)
        fallback_dt_s = float(self.fallback_dt_s)
        if not math.isfinite(tau_s) or tau_s <= 0.0:
            raise ValueError("tau_s must be a positive finite value")
        if not math.isfinite(fallback_dt_s) or fallback_dt_s <= 0.0:
            raise ValueError("fallback_dt_s must be a positive finite value")


def normalize_quat_xyzw(q: Sequence[float]) -> tuple[float, float, float, float]:
    arr = np.asarray(q, dtype=float)
    if arr.shape != (4,):
        raise ValueError("quaternion must have shape (4,) in xyzw order")
    if not np.isfinite(arr).all():
        raise ValueError("quaternion values must be finite")
    norm = float(np.linalg.norm(arr))
    if not math.isfinite(norm) or norm <= _HEMISPHERE_EPS:
        raise ValueError("quaternion norm must be > 0")
    unit = arr / norm
    return (float(unit[0]), float(unit[1]), float(unit[2]), float(unit[3]))


def quat_dot_xyzw(q1: Sequence[float], q2: Sequence[float]) -> float:
    a = np.asarray(normalize_quat_xyzw(q1), dtype=float)
    b = np.asarray(normalize_quat_xyzw(q2), dtype=float)
    return float(np.dot(a, b))


def ensure_same_quat_hemisphere(
    q: Sequence[float],
    q_ref: Sequence[float],
) -> tuple[float, float, float, float]:
    q_unit = np.asarray(normalize_quat_xyzw(q), dtype=float)
    q_ref_unit = np.asarray(normalize_quat_xyzw(q_ref), dtype=float)
    if float(np.dot(q_unit, q_ref_unit)) < 0.0:
        q_unit = -q_unit
    return (float(q_unit[0]), float(q_unit[1]), float(q_unit[2]), float(q_unit[3]))


def quat_slerp_xyzw(
    q0: Sequence[float],
    q1: Sequence[float],
    alpha: float,
) -> tuple[float, float, float, float]:
    a = min(1.0, max(0.0, float(alpha)))

    q0_unit = np.asarray(normalize_quat_xyzw(q0), dtype=float)
    q1_unit = np.asarray(normalize_quat_xyzw(q1), dtype=float)

    dot = float(np.dot(q0_unit, q1_unit))
    if dot < 0.0:
        q1_unit = -q1_unit
        dot = -dot

    dot = min(1.0, max(-1.0, dot))

    if dot >= _SLERP_LINEAR_THRESHOLD:
        blended = (1.0 - a) * q0_unit + a * q1_unit
        return normalize_quat_xyzw(blended)

    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    if abs(sin_theta) <= _HEMISPHERE_EPS:
        blended = (1.0 - a) * q0_unit + a * q1_unit
        return normalize_quat_xyzw(blended)

    w0 = math.sin((1.0 - a) * theta) / sin_theta
    w1 = math.sin(a * theta) / sin_theta
    blended = w0 * q0_unit + w1 * q1_unit
    return normalize_quat_xyzw(blended)


class QuaternionSlerpLowPassFilter:
    def __init__(self, config: OrientationFilterConfig):
        self._config = config
        self._prev_filtered: tuple[float, float, float, float] | None = None
        self._prev_timestamp_ns: int | None = None

    def reset(
        self,
        quat_xyzw: Sequence[float] | None = None,
        timestamp_ns: int | None = None,
    ) -> None:
        self._prev_filtered = None
        self._prev_timestamp_ns = None
        if quat_xyzw is not None:
            self._prev_filtered = normalize_quat_xyzw(quat_xyzw)
        if timestamp_ns is not None:
            self._prev_timestamp_ns = int(timestamp_ns)

    def update(
        self,
        quat_xyzw: Sequence[float],
        timestamp_ns: int | None = None,
        dt_s: float | None = None,
    ) -> tuple[float, float, float, float]:
        raw_quat = normalize_quat_xyzw(quat_xyzw)

        if not bool(self._config.enabled):
            self._prev_filtered = raw_quat
            if timestamp_ns is not None:
                self._prev_timestamp_ns = int(timestamp_ns)
            return raw_quat

        if self._prev_filtered is None:
            self._prev_filtered = raw_quat
            if timestamp_ns is not None:
                self._prev_timestamp_ns = int(timestamp_ns)
            return raw_quat

        dt_used = self._resolve_dt_s(timestamp_ns=timestamp_ns, dt_s=dt_s)
        prev_filtered = self._prev_filtered
        aligned_raw = ensure_same_quat_hemisphere(raw_quat, prev_filtered)

        alpha = 1.0 - math.exp(-float(dt_used) / float(self._config.tau_s))
        filtered = quat_slerp_xyzw(prev_filtered, aligned_raw, alpha)

        self._prev_filtered = filtered
        if timestamp_ns is not None:
            self._prev_timestamp_ns = int(timestamp_ns)
        return filtered

    def _resolve_dt_s(self, timestamp_ns: int | None, dt_s: float | None) -> float:
        if timestamp_ns is not None and self._prev_timestamp_ns is not None:
            dt_from_ts = (int(timestamp_ns) - int(self._prev_timestamp_ns)) * 1e-9
            if math.isfinite(dt_from_ts) and dt_from_ts > 0.0:
                return float(dt_from_ts)

        if dt_s is not None:
            dt_value = float(dt_s)
            if math.isfinite(dt_value) and dt_value > 0.0:
                return dt_value

        return float(self._config.fallback_dt_s)


class DualArmOrientationFilter:
    def __init__(self, config: OrientationFilterConfig):
        self.left = QuaternionSlerpLowPassFilter(config)
        self.right = QuaternionSlerpLowPassFilter(config)

    def reset_side(
        self,
        side: str,
        quat_xyzw: Sequence[float] | None = None,
        timestamp_ns: int | None = None,
    ) -> None:
        side_key = _normalize_side(side)
        if side_key == "left":
            self.left.reset(quat_xyzw=quat_xyzw, timestamp_ns=timestamp_ns)
            return
        self.right.reset(quat_xyzw=quat_xyzw, timestamp_ns=timestamp_ns)

    def reset_all(self) -> None:
        self.left.reset()
        self.right.reset()

    def update_side(
        self,
        side: str,
        quat_xyzw: Sequence[float],
        timestamp_ns: int | None = None,
        dt_s: float | None = None,
    ) -> tuple[float, float, float, float]:
        side_key = _normalize_side(side)
        if side_key == "left":
            return self.left.update(quat_xyzw=quat_xyzw, timestamp_ns=timestamp_ns, dt_s=dt_s)
        return self.right.update(quat_xyzw=quat_xyzw, timestamp_ns=timestamp_ns, dt_s=dt_s)


def _normalize_side(side: str) -> str:
    side_norm = str(side).strip().lower()
    if side_norm in {"left", "a"}:
        return "left"
    if side_norm in {"right", "b"}:
        return "right"
    raise ValueError("side must be one of: 'left', 'right', 'A', 'B'")


__all__ = [
    "OrientationFilterConfig",
    "QuaternionSlerpLowPassFilter",
    "DualArmOrientationFilter",
    "normalize_quat_xyzw",
    "quat_dot_xyzw",
    "quat_slerp_xyzw",
    "ensure_same_quat_hemisphere",
]
