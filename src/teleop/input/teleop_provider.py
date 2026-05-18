from __future__ import annotations

from teleop.core.teleop_frame import TeleopFrame
from teleop.input.pico_mapping import PicoInputMapper
from teleop.input.pico_provider import PicoProvider


class TeleopProvider:
    """High-level provider that exposes mapped teleoperation input frames."""

    def __init__(self, pico_provider: PicoProvider | None = None, mapper: PicoInputMapper | None = None):
        self._pico_provider = pico_provider if pico_provider is not None else PicoProvider()
        self._mapper = mapper if mapper is not None else PicoInputMapper()
        self._latest_mapped: TeleopFrame | None = None
        self._latest_raw_frame_id: int | None = None

    def start(self) -> None:
        self._pico_provider.start()

    def stop(self) -> None:
        self._pico_provider.stop()

    def get_latest(self) -> TeleopFrame | None:
        raw_frame = self._pico_provider.get_latest()
        if raw_frame is None:
            return None

        if self._latest_raw_frame_id == raw_frame.frame_id and self._latest_mapped is not None:
            return self._latest_mapped

        mapped = self._mapper.map_frame(raw_frame)
        self._latest_raw_frame_id = raw_frame.frame_id
        self._latest_mapped = mapped
        return mapped
