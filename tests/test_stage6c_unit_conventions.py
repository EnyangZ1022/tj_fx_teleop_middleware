from teleop.diagnostics.integration_checks import check_position_unit_conversion


def test_meter_to_millimeter_conversion_examples() -> None:
    assert check_position_unit_conversion((0.1, 0.0, 0.0), (100.0, 0.0, 0.0)) is True
    assert check_position_unit_conversion((0.001, 0.0, 0.0), (1.0, 0.0, 0.0)) is True
