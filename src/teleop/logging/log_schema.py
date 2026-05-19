from __future__ import annotations

from dataclasses import asdict, dataclass, is_dataclass
from enum import Enum
from pathlib import Path
import time
from typing import Any


try:
    import numpy as _np
except Exception:  # pragma: no cover - numpy may be absent in some environments.
    _np = None


@dataclass(frozen=True)
class LogRecord:
    record_type: str
    timestamp_ns: int
    level: str
    event: str
    payload: dict[str, object]
    sequence_id: int | None = None


def now_ns() -> int:
    return time.time_ns()


def to_jsonable(obj: Any) -> object:
    """Best-effort conversion to JSON-serializable Python objects."""
    try:
        if obj is None or isinstance(obj, (str, int, float, bool)):
            return obj

        if isinstance(obj, Path):
            return str(obj)

        if isinstance(obj, Enum):
            return obj.name

        if is_dataclass(obj):
            return to_jsonable(asdict(obj))

        if isinstance(obj, dict):
            return {str(k): to_jsonable(v) for k, v in obj.items()}

        if isinstance(obj, (list, tuple, set)):
            return [to_jsonable(v) for v in obj]

        if _np is not None:
            if isinstance(obj, _np.generic):
                return obj.item()
            if isinstance(obj, _np.ndarray):
                return obj.tolist()

        return str(obj)
    except Exception:
        try:
            return str(obj)
        except Exception:
            return "<unserializable>"


__all__ = ["LogRecord", "now_ns", "to_jsonable"]
