from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
from datetime import datetime
import json
from pathlib import Path
import threading
import time
from typing import Callable, TextIO

from teleop.logging.log_config import LoggingConfig
from teleop.logging.log_schema import LogRecord, now_ns, to_jsonable


_LOW_PRIORITY_RECORD_TYPES = {"frame", "performance", "timing", "receiver_timing"}


@dataclass(frozen=True)
class LoggingStats:
    enabled: bool
    started: bool
    records_enqueued: int
    records_written: int
    records_dropped: int
    event_records: int
    frame_records: int
    performance_records: int
    timing_records: int
    receiver_timing_records: int
    error_records: int
    queue_size: int


class AsyncSessionLogger:
    """Asynchronous JSONL logger with non-blocking producer semantics."""

    def __init__(self, config: LoggingConfig | None = None):
        self._config = config if config is not None else LoggingConfig()
        self._enabled = bool(self._config.enabled)

        self._started = False
        self._sequence_id = 0

        self._queue: deque[LogRecord] = deque()
        self._queue_lock = threading.Lock()
        self._queue_event = threading.Event()

        self._stats_lock = threading.Lock()
        self._records_enqueued = 0
        self._records_written = 0
        self._records_dropped = 0
        self._event_records = 0
        self._frame_records = 0
        self._performance_records = 0
        self._timing_records = 0
        self._receiver_timing_records = 0
        self._error_records = 0

        self._session_dir: Path | None = None
        self._session_file: Path | None = None
        self._receiver_timing_file: Path | None = None
        self._writer_handle: TextIO | None = None
        self._receiver_timing_handle: TextIO | None = None

        self._stop_event = threading.Event()
        self._writer_thread: threading.Thread | None = None

        self._last_frame_log_ns: int | None = None
        self._last_performance_log_ns: int | None = None

    @property
    def session_dir(self) -> Path | None:
        return self._session_dir

    @property
    def session_file(self) -> Path | None:
        return self._session_file

    @property
    def receiver_timing_file(self) -> Path | None:
        return self._receiver_timing_file

    def is_enabled(self) -> bool:
        return self._enabled

    def start(self) -> None:
        if not self._enabled or self._started:
            return

        try:
            self._session_dir, self._session_file = self._create_session_paths()
            self._session_dir.mkdir(parents=True, exist_ok=True)

            self._writer_handle = self._session_file.open("a", encoding="utf-8")
            if bool(self._config.record_receiver_timing):
                self._receiver_timing_file = self._session_dir / "teleop_receiver_timing.jsonl"
                self._receiver_timing_handle = self._receiver_timing_file.open("a", encoding="utf-8")
            else:
                self._receiver_timing_file = None
                self._receiver_timing_handle = None
            self._write_metadata_file()

            self._stop_event.clear()
            self._queue_event.clear()
            self._writer_thread = threading.Thread(
                target=self._writer_loop,
                name="teleop-async-logger",
                daemon=True,
            )
            self._writer_thread.start()
            self._started = True
        except Exception:
            # Logger startup failure must not break caller.
            self._cleanup_writer_resources()
            self._started = False

    def stop(self) -> None:
        if not self._enabled:
            return

        if not self._started:
            self._cleanup_writer_resources()
            return

        self._stop_event.set()
        self._queue_event.set()

        thread = self._writer_thread
        if thread is not None:
            thread.join(timeout=5.0)

        self._cleanup_writer_resources()
        self._started = False

    def log_event(self, event: str, payload: dict | None = None, level: str = "INFO") -> None:
        if not self._config.record_events:
            return
        self._log(record_type="event", event=event, payload=payload, level=level)

    def log_frame(self, event: str, payload: dict | None = None, level: str = "DEBUG") -> None:
        if not self._config.record_frames:
            return

        ts = now_ns()
        if not self._should_sample(ts, self._last_frame_log_ns, self._config.frame_sample_hz):
            return
        self._last_frame_log_ns = ts

        self._log(record_type="frame", event=event, payload=payload, level=level, timestamp_ns=ts)

    def log_performance(self, event: str, payload: dict | None = None, level: str = "DEBUG") -> None:
        if not self._config.record_performance:
            return

        ts = now_ns()
        if not self._should_sample(ts, self._last_performance_log_ns, self._config.performance_sample_hz):
            return
        self._last_performance_log_ns = ts

        self._log(record_type="performance", event=event, payload=payload, level=level, timestamp_ns=ts)

    def log_timing(self, event: str, payload: dict | None = None, level: str = "DEBUG") -> None:
        if not self._config.record_timing:
            return

        if not self._enabled or not self._started:
            return

        payload_dict = dict(payload) if payload is not None else {}
        enqueue_start_ns = time.perf_counter_ns()

        def _on_enqueued() -> None:
            payload_dict["log_enqueue_ms"] = float(time.perf_counter_ns() - enqueue_start_ns) / 1_000_000.0

        try:
            record = LogRecord(
                record_type="timing",
                timestamp_ns=int(now_ns()),
                level=str(level),
                event=str(event),
                payload=payload_dict,
                sequence_id=self._next_sequence_id(),
            )
            self._enqueue_record(record, on_enqueued=_on_enqueued)
        except Exception:
            # Logging must never throw into caller path.
            return

    def log_receiver_timing(self, event: str, payload: dict | None = None, level: str = "DEBUG") -> None:
        if not self._config.record_receiver_timing:
            return

        timestamp_ns = None
        if isinstance(payload, dict):
            ts_value = payload.get("pc_receive_wall_ns")
            if isinstance(ts_value, int):
                timestamp_ns = ts_value
            elif isinstance(ts_value, str):
                try:
                    timestamp_ns = int(ts_value)
                except ValueError:
                    timestamp_ns = None

        self._log(
            record_type="receiver_timing",
            event=event,
            payload=payload,
            level=level,
            timestamp_ns=timestamp_ns,
        )

    def log_error(self, event: str, payload: dict | None = None) -> None:
        self._log(record_type="error", event=event, payload=payload, level="ERROR")

    def get_stats(self) -> LoggingStats:
        queue_size = self._queue_size()
        with self._stats_lock:
            return LoggingStats(
                enabled=self._enabled,
                started=self._started,
                records_enqueued=self._records_enqueued,
                records_written=self._records_written,
                records_dropped=self._records_dropped,
                event_records=self._event_records,
                frame_records=self._frame_records,
                performance_records=self._performance_records,
                timing_records=self._timing_records,
                receiver_timing_records=self._receiver_timing_records,
                error_records=self._error_records,
                queue_size=queue_size,
            )

    def reset_stats(self) -> None:
        with self._stats_lock:
            self._records_enqueued = 0
            self._records_written = 0
            self._records_dropped = 0
            self._event_records = 0
            self._frame_records = 0
            self._performance_records = 0
            self._timing_records = 0
            self._receiver_timing_records = 0
            self._error_records = 0

    def _log(
        self,
        *,
        record_type: str,
        event: str,
        payload: dict | None,
        level: str,
        timestamp_ns: int | None = None,
    ) -> None:
        if not self._enabled or not self._started:
            return

        try:
            record = LogRecord(
                record_type=str(record_type),
                timestamp_ns=int(now_ns() if timestamp_ns is None else timestamp_ns),
                level=str(level),
                event=str(event),
                payload=dict(payload) if payload is not None else {},
                sequence_id=self._next_sequence_id(),
            )
            self._enqueue_record(record)
        except Exception:
            # Logging must never throw into caller path.
            return

    def _enqueue_record(self, record: LogRecord, on_enqueued: Callable[[], None] | None = None) -> None:
        high_priority = record.record_type not in _LOW_PRIORITY_RECORD_TYPES

        with self._queue_lock:
            max_size = int(self._config.max_queue_size)
            current_size = len(self._queue)

            if current_size < max_size:
                self._queue.append(record)
                if on_enqueued is not None:
                    on_enqueued()
                self._record_enqueued_stats(record)
                self._queue_event.set()
                return

            if not bool(self._config.drop_when_full):
                self._record_dropped()
                return

            if high_priority:
                dropped_low = self._drop_one_low_priority_locked()
                if dropped_low:
                    self._queue.append(record)
                    if on_enqueued is not None:
                        on_enqueued()
                    self._record_enqueued_stats(record)
                    self._queue_event.set()
                    return

            self._record_dropped()

    def _drop_one_low_priority_locked(self) -> bool:
        for index, existing in enumerate(self._queue):
            if existing.record_type in _LOW_PRIORITY_RECORD_TYPES:
                del self._queue[index]
                self._record_dropped()
                return True
        return False

    def _record_enqueued_stats(self, record: LogRecord) -> None:
        with self._stats_lock:
            self._records_enqueued += 1
            if record.record_type == "event":
                self._event_records += 1
            elif record.record_type == "frame":
                self._frame_records += 1
            elif record.record_type == "performance":
                self._performance_records += 1
            elif record.record_type == "timing":
                self._timing_records += 1
            elif record.record_type == "receiver_timing":
                self._receiver_timing_records += 1
            elif record.record_type == "error":
                self._error_records += 1

    def _record_written_count(self, count: int) -> None:
        with self._stats_lock:
            self._records_written += int(count)

    def _record_dropped(self) -> None:
        with self._stats_lock:
            self._records_dropped += 1

    def _queue_size(self) -> int:
        with self._queue_lock:
            return len(self._queue)

    def _next_sequence_id(self) -> int:
        with self._stats_lock:
            self._sequence_id += 1
            return self._sequence_id

    def _writer_loop(self) -> None:
        last_flush_t = time.monotonic()

        while not self._stop_event.is_set() or self._queue_size() > 0:
            self._queue_event.wait(timeout=float(self._config.flush_interval_s))
            self._queue_event.clear()

            batch = self._drain_batch(int(self._config.batch_size))
            if batch:
                self._write_batch(batch)

            now_t = time.monotonic()
            if now_t - last_flush_t >= float(self._config.flush_interval_s):
                self._safe_flush()
                last_flush_t = now_t

        while True:
            batch = self._drain_batch(int(self._config.batch_size))
            if not batch:
                break
            self._write_batch(batch)
        self._safe_flush()

    def _drain_batch(self, batch_size: int) -> list[LogRecord]:
        items: list[LogRecord] = []
        with self._queue_lock:
            while self._queue and len(items) < batch_size:
                items.append(self._queue.popleft())
        return items

    def _write_batch(self, records: list[LogRecord]) -> None:
        written = 0
        for record in records:
            try:
                handle = self._writer_handle
                if record.record_type == "receiver_timing":
                    handle = self._receiver_timing_handle
                if handle is None:
                    self._record_dropped()
                    continue

                line = json.dumps(to_jsonable(asdict(record)), ensure_ascii=False)
                handle.write(line)
                handle.write("\n")
                written += 1
            except Exception:
                self._record_dropped()

        if written > 0:
            self._record_written_count(written)

    def _safe_flush(self) -> None:
        for handle in (self._writer_handle, self._receiver_timing_handle):
            if handle is None:
                continue
            try:
                handle.flush()
            except Exception:
                continue

    def _cleanup_writer_resources(self) -> None:
        handle = self._writer_handle
        receiver_handle = self._receiver_timing_handle
        self._writer_handle = None
        self._receiver_timing_handle = None
        self._receiver_timing_file = None
        self._writer_thread = None

        if handle is not None:
            try:
                handle.flush()
            except Exception:
                pass
            try:
                handle.close()
            except Exception:
                pass

        if receiver_handle is not None:
            try:
                receiver_handle.flush()
            except Exception:
                pass
            try:
                receiver_handle.close()
            except Exception:
                pass

    def _create_session_paths(self) -> tuple[Path, Path]:
        base_dir = Path(self._config.log_dir)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_name = self._sanitize_session_name(self._config.session_name)

        session_dir = base_dir / f"{timestamp}_{session_name}"
        if self._config.logging_mode == "timing":
            session_file = session_dir / "teleop_timing.jsonl"
        else:
            session_file = session_dir / "teleop_session.jsonl"
        return session_dir, session_file

    def _write_metadata_file(self) -> None:
        if self._session_dir is None:
            return

        metadata = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "logging_config": to_jsonable(asdict(self._config)),
        }

        meta_path = self._session_dir / "session_meta.json"
        try:
            with meta_path.open("w", encoding="utf-8") as meta_file:
                json.dump(metadata, meta_file, ensure_ascii=False, indent=self._config.json_indent)
                meta_file.write("\n")
        except Exception:
            return

    @staticmethod
    def _sanitize_session_name(session_name: str | None) -> str:
        text = (session_name or "session").strip()
        if not text:
            text = "session"

        safe_chars = []
        for ch in text:
            if ch.isalnum() or ch in {"-", "_"}:
                safe_chars.append(ch)
            else:
                safe_chars.append("_")
        return "".join(safe_chars)

    @staticmethod
    def _should_sample(curr_ns: int, last_ns: int | None, sample_hz: float) -> bool:
        if last_ns is None:
            return True
        period_ns = int(round(1_000_000_000.0 / float(sample_hz)))
        return (curr_ns - last_ns) >= period_ns


__all__ = ["LoggingStats", "AsyncSessionLogger"]
