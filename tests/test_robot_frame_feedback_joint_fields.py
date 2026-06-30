from __future__ import annotations

import pytest

from teleop.core.robot_frame import RobotArmFeedback


def test_robot_arm_feedback_stores_joint_feedback_fields() -> None:
    feedback = RobotArmFeedback(
        position_xyz=(100.0, 200.0, 300.0),
        orientation_abc=(10.0, 20.0, 30.0),
        valid=True,
        q_deg=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0),
        qd_deg_s=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7),
        tau=(0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07),
    )

    assert feedback.q_deg == pytest.approx((1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0))
    assert feedback.qd_deg_s == pytest.approx((0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7))
    assert feedback.tau == pytest.approx((0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07))
