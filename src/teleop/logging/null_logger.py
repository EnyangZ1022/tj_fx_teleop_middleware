from __future__ import annotations

from teleop.logging.async_logger import LoggingStats


class NullSessionLogger:
    def start(self) -> None:
        return

    def stop(self) -> None:
        return

    def is_enabled(self) -> bool:
        return False

    def log_event(self, event: str, payload: dict | None = None, level: str = "INFO") -> None:
        _ = (event, payload, level)
        return

    def log_frame(self, event: str, payload: dict | None = None, level: str = "DEBUG") -> None:
        _ = (event, payload, level)
        return

    def log_performance(self, event: str, payload: dict | None = None, level: str = "DEBUG") -> None:
        _ = (event, payload, level)
        return

    def log_timing(self, event: str, payload: dict | None = None, level: str = "DEBUG") -> None:
        _ = (event, payload, level)
        return

    def log_receiver_timing(self, event: str, payload: dict | None = None, level: str = "DEBUG") -> None:
        _ = (event, payload, level)
        return

    def log_error(self, event: str, payload: dict | None = None) -> None:
        _ = (event, payload)
        return

    def get_stats(self) -> LoggingStats:
        return LoggingStats(
            enabled=False,
            started=False,
            records_enqueued=0,
            records_written=0,
            records_dropped=0,
            event_records=0,
            frame_records=0,
            performance_records=0,
            timing_records=0,
            receiver_timing_records=0,
            error_records=0,
            queue_size=0,
        )

    def reset_stats(self) -> None:
        return


__all__ = ["NullSessionLogger"]
