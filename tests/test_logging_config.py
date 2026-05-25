from teleop.logging.log_config import LoggingConfig


def test_logging_config_defaults_are_safe() -> None:
    cfg = LoggingConfig()

    assert cfg.enabled is False
    assert cfg.logging_mode == "full"
    assert cfg.record_frames is False
    assert cfg.record_performance is False
    assert cfg.record_timing is False
