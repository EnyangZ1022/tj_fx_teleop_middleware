from __future__ import annotations

from typing import Callable

from teleop.core.pico_frame import PicoRawFrame
from teleop.input.pico_receiver import PicoReceiver


class PicoProvider:
    """High-level wrapper that hides Pico transport details."""

    def __init__(self, on_receiver_timing: Callable[[dict[str, object]], None] | None = None):
        self._receiver = PicoReceiver(on_receiver_timing=on_receiver_timing)

    def start(self) -> None:
        self._receiver.start()

    def stop(self) -> None:
        self._receiver.stop()

    def get_latest(self) -> PicoRawFrame | None:
        return self._receiver.get_latest_frame()
