from __future__ import annotations

from dataclasses import dataclass


_ALLOWED_LOGGING_MODES = {"off", "timing", "full"}


@dataclass(frozen=True)
class LoggingConfig:
    enabled: bool = False
    logging_mode: str = "full"
    log_dir: str = "logs"
    session_name: str | None = None

    record_events: bool = True
    record_frames: bool = False
    record_performance: bool = False
    record_timing: bool = False

    frame_sample_hz: float = 10.0
    performance_sample_hz: float = 10.0

    max_queue_size: int = 10000
    batch_size: int = 100
    flush_interval_s: float = 1.0
    drop_when_full: bool = True

    json_indent: int | None = None

    def __post_init__(self) -> None:
        mode = str(self.logging_mode).strip().lower()
        if mode not in _ALLOWED_LOGGING_MODES:
            raise ValueError(f"logging_mode must be one of {sorted(_ALLOWED_LOGGING_MODES)}")
        object.__setattr__(self, "logging_mode", mode)

        if int(self.max_queue_size) <= 0:
            raise ValueError("max_queue_size must be positive")
        if int(self.batch_size) <= 0:
            raise ValueError("batch_size must be positive")
        if float(self.flush_interval_s) <= 0.0:
            raise ValueError("flush_interval_s must be positive")

        if float(self.frame_sample_hz) <= 0.0:
            raise ValueError("frame_sample_hz must be positive")
        if float(self.performance_sample_hz) <= 0.0:
            raise ValueError("performance_sample_hz must be positive")

        if self.json_indent is not None and int(self.json_indent) < 0:
            raise ValueError("json_indent must be >= 0 when provided")


__all__ = ["LoggingConfig"]
