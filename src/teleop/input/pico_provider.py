from __future__ import annotations

from teleop.core.pico_frame import PicoRawFrame
from teleop.input.pico_receiver import PicoReceiver


class PicoProvider:
    """High-level wrapper that hides Pico transport details."""

    def __init__(self):
        self._receiver = PicoReceiver()

    def start(self) -> None:
        self._receiver.start()

    def stop(self) -> None:
        self._receiver.stop()

    def get_latest(self) -> PicoRawFrame | None:
        return self._receiver.get_latest_frame()
