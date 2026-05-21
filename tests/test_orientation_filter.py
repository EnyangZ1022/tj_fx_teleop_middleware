from __future__ import annotations

import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from teleop.filtering.orientation_filter import (
    DualArmOrientationFilter,
    OrientationFilterConfig,
    QuaternionSlerpLowPassFilter,
    normalize_quat_xyzw,
    quat_dot_xyzw,
    quat_slerp_xyzw,
)


def _quat_from_rotvec_deg(x_deg: float, y_deg: float, z_deg: float) -> tuple[float, float, float, float]:
    quat = Rotation.from_rotvec(np.radians(np.array([x_deg, y_deg, z_deg], dtype=float))).as_quat()
    return (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))


def _quat_angle_deg(q_ref: tuple[float, float, float, float], q_now: tuple[float, float, float, float]) -> float:
    r_ref = Rotation.from_quat(q_ref)
    r_now = Rotation.from_quat(q_now)
    delta = r_ref.inv() * r_now
    return float(np.linalg.norm(delta.as_rotvec()) * 180.0 / math.pi)


def _assert_same_orientation(q0: tuple[float, float, float, float], q1: tuple[float, float, float, float]) -> None:
    dot = quat_dot_xyzw(q0, q1)
    assert abs(dot) == pytest.approx(1.0, abs=1e-6)


def test_normalize_quat_xyzw_returns_unit_quaternion() -> None:
    q = normalize_quat_xyzw((0.0, 0.0, 0.0, 2.0))
    assert np.linalg.norm(np.asarray(q, dtype=float)) == pytest.approx(1.0, abs=1e-9)


def test_quat_slerp_xyzw_alpha_zero_returns_q0() -> None:
    q0 = _quat_from_rotvec_deg(0.0, 0.0, 10.0)
    q1 = _quat_from_rotvec_deg(0.0, 0.0, 35.0)

    out = quat_slerp_xyzw(q0, q1, 0.0)
    _assert_same_orientation(out, q0)


def test_quat_slerp_xyzw_alpha_one_returns_q1() -> None:
    q0 = _quat_from_rotvec_deg(0.0, 0.0, 10.0)
    q1 = _quat_from_rotvec_deg(0.0, 0.0, 35.0)

    out = quat_slerp_xyzw(q0, q1, 1.0)
    _assert_same_orientation(out, q1)


def test_quat_slerp_xyzw_handles_negative_equivalent_quaternion_without_jump() -> None:
    q0 = (0.0, 0.0, 0.0, 1.0)
    q1 = (0.0, 0.0, 0.0, -1.0)

    out = quat_slerp_xyzw(q0, q1, 0.5)
    _assert_same_orientation(out, q0)


def test_low_pass_first_update_initializes_state_with_input() -> None:
    filt = QuaternionSlerpLowPassFilter(OrientationFilterConfig(enabled=True, tau_s=0.02, fallback_dt_s=0.01))

    q_raw = _quat_from_rotvec_deg(5.0, -3.0, 9.0)
    out = filt.update(q_raw, timestamp_ns=1)

    _assert_same_orientation(out, normalize_quat_xyzw(q_raw))


def test_low_pass_uses_partial_slerp_for_nominal_tau_and_dt() -> None:
    cfg = OrientationFilterConfig(enabled=True, tau_s=0.02, fallback_dt_s=0.01)
    filt = QuaternionSlerpLowPassFilter(cfg)

    q_ref = (0.0, 0.0, 0.0, 1.0)
    q_raw = _quat_from_rotvec_deg(0.0, 0.0, 30.0)

    _ = filt.update(q_ref, timestamp_ns=0)
    out = filt.update(q_raw, timestamp_ns=10_000_000)

    alpha = 1.0 - math.exp(-0.01 / 0.02)
    out_angle = _quat_angle_deg(q_ref, out)

    assert 0.0 < alpha < 1.0
    assert 0.0 < out_angle < 30.0
    assert out_angle == pytest.approx(30.0 * alpha, abs=0.5)


def test_larger_tau_moves_more_slowly_for_same_dt() -> None:
    q_ref = (0.0, 0.0, 0.0, 1.0)
    q_raw = _quat_from_rotvec_deg(0.0, 0.0, 30.0)

    fast = QuaternionSlerpLowPassFilter(OrientationFilterConfig(enabled=True, tau_s=0.01, fallback_dt_s=0.01))
    slow = QuaternionSlerpLowPassFilter(OrientationFilterConfig(enabled=True, tau_s=0.04, fallback_dt_s=0.01))

    _ = fast.update(q_ref, timestamp_ns=0)
    _ = slow.update(q_ref, timestamp_ns=0)

    out_fast = fast.update(q_raw, timestamp_ns=10_000_000)
    out_slow = slow.update(q_raw, timestamp_ns=10_000_000)

    angle_fast = _quat_angle_deg(q_ref, out_fast)
    angle_slow = _quat_angle_deg(q_ref, out_slow)

    assert angle_slow < angle_fast


def test_reset_clears_memory_and_next_update_matches_reset_quaternion() -> None:
    filt = QuaternionSlerpLowPassFilter(OrientationFilterConfig(enabled=True, tau_s=0.02, fallback_dt_s=0.01))

    _ = filt.update((0.0, 0.0, 0.0, 1.0), timestamp_ns=0)
    _ = filt.update(_quat_from_rotvec_deg(0.0, 0.0, 20.0), timestamp_ns=10_000_000)

    q_new = _quat_from_rotvec_deg(10.0, 0.0, 0.0)
    filt.reset(q_new, timestamp_ns=20_000_000)

    out = filt.update(q_new, timestamp_ns=30_000_000)
    _assert_same_orientation(out, q_new)


def test_dual_arm_orientation_filter_keeps_sides_independent() -> None:
    filt = DualArmOrientationFilter(OrientationFilterConfig(enabled=True, tau_s=0.02, fallback_dt_s=0.01))

    q_id = (0.0, 0.0, 0.0, 1.0)
    q_left = _quat_from_rotvec_deg(20.0, 0.0, 0.0)

    _ = filt.update_side("left", q_id, timestamp_ns=0)
    _ = filt.update_side("right", q_id, timestamp_ns=0)

    left_out = filt.update_side("A", q_left, timestamp_ns=10_000_000)
    right_out = filt.update_side("B", q_id, timestamp_ns=10_000_000)

    assert _quat_angle_deg(q_id, left_out) > 0.0
    _assert_same_orientation(right_out, q_id)


def test_disabled_filter_returns_normalized_raw_quaternion() -> None:
    filt = QuaternionSlerpLowPassFilter(OrientationFilterConfig(enabled=False, tau_s=0.02, fallback_dt_s=0.01))

    out = filt.update((0.0, 0.0, 0.0, 2.0), timestamp_ns=1)
    assert out == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_invalid_filter_config_values_raise_value_error() -> None:
    with pytest.raises(ValueError):
        OrientationFilterConfig(tau_s=0.0)

    with pytest.raises(ValueError):
        OrientationFilterConfig(tau_s=-0.01)

    with pytest.raises(ValueError):
        OrientationFilterConfig(fallback_dt_s=0.0)

    with pytest.raises(ValueError):
        OrientationFilterConfig(fallback_dt_s=-0.01)
