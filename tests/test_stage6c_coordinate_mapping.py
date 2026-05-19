import pytest

from teleop.diagnostics.integration_checks import check_coordinate_axis_mapping


def test_right_arm_coordinate_mapping() -> None:
    mapping = check_coordinate_axis_mapping("right")

    assert mapping["user_+Z"] == pytest.approx((1.0, 0.0, 0.0))
    assert mapping["user_+Y"] == pytest.approx((0.0, 1.0, 0.0))
    assert mapping["user_+X"] == pytest.approx((0.0, 0.0, 1.0))


def test_left_arm_coordinate_mapping() -> None:
    mapping = check_coordinate_axis_mapping("left")

    assert mapping["user_+Z"] == pytest.approx((1.0, 0.0, 0.0))
    assert mapping["user_+Y"] == pytest.approx((0.0, -1.0, 0.0))
    assert mapping["user_+X"] == pytest.approx((0.0, 0.0, -1.0))
