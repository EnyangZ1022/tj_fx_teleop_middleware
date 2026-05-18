from __future__ import annotations

from pathlib import Path
import statistics
import sys
import threading
import time

# Allow running this script directly without installing the package.
ROOT_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from teleop.core.pico_frame import PicoRawFrame
from teleop.input.pico_receiver import PicoReceiver


class FrequencyDiagnostics:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.total_frames = 0
        self.first_pc_receive_time_ns: int | None = None
        self.last_pc_receive_time_ns: int | None = None
        self.last_pico_timestamp_ns: int | None = None

        self.pico_non_monotonic_count = 0
        self.pc_dt_samples_ns: list[int] = []

        self.left_valid_count = 0
        self.right_valid_count = 0
        self.left_zero_pose_invalid_count = 0
        self.right_zero_pose_invalid_count = 0

    def on_frame(self, frame: PicoRawFrame) -> None:
        with self._lock:
            self.total_frames += 1

            pc_ts = frame.pc_receive_time_ns
            if self.first_pc_receive_time_ns is None:
                self.first_pc_receive_time_ns = pc_ts
            if self.last_pc_receive_time_ns is not None and pc_ts > self.last_pc_receive_time_ns:
                self.pc_dt_samples_ns.append(pc_ts - self.last_pc_receive_time_ns)
            self.last_pc_receive_time_ns = pc_ts

            pico_ts = frame.pico_timestamp_ns
            if self.last_pico_timestamp_ns is not None and pico_ts <= self.last_pico_timestamp_ns:
                self.pico_non_monotonic_count += 1
            self.last_pico_timestamp_ns = pico_ts

            if frame.left_valid:
                self.left_valid_count += 1
            elif frame.left_ctrl.pose.is_zero_pose():
                self.left_zero_pose_invalid_count += 1

            if frame.right_valid:
                self.right_valid_count += 1
            elif frame.right_ctrl.pose.is_zero_pose():
                self.right_zero_pose_invalid_count += 1

    def snapshot(self) -> dict[str, float | int | str]:
        with self._lock:
            elapsed_s = 0.0
            if (
                self.first_pc_receive_time_ns is not None
                and self.last_pc_receive_time_ns is not None
                and self.last_pc_receive_time_ns > self.first_pc_receive_time_ns
            ):
                elapsed_s = (self.last_pc_receive_time_ns - self.first_pc_receive_time_ns) / 1e9

            fps = (self.total_frames / elapsed_s) if elapsed_s > 0.0 else 0.0

            left_valid_ratio = (self.left_valid_count / self.total_frames) if self.total_frames > 0 else 0.0
            right_valid_ratio = (self.right_valid_count / self.total_frames) if self.total_frames > 0 else 0.0

            dt_summary = "n/a"
            if len(self.pc_dt_samples_ns) >= 5:
                dt_ms = [sample / 1e6 for sample in self.pc_dt_samples_ns]
                dt_ms_sorted = sorted(dt_ms)
                idx_95 = int(round((len(dt_ms_sorted) - 1) * 0.95))
                p95 = dt_ms_sorted[idx_95]
                dt_summary = (
                    f"mean={statistics.fmean(dt_ms):.3f} min={min(dt_ms):.3f} "
                    f"max={max(dt_ms):.3f} p95={p95:.3f}"
                )

            return {
                "total_frames": self.total_frames,
                "elapsed_s": elapsed_s,
                "fps": fps,
                "pico_non_monotonic_count": self.pico_non_monotonic_count,
                "dt_summary": dt_summary,
                "left_valid_ratio": left_valid_ratio,
                "right_valid_ratio": right_valid_ratio,
                "left_zero_pose_invalid_count": self.left_zero_pose_invalid_count,
                "right_zero_pose_invalid_count": self.right_zero_pose_invalid_count,
            }


def main() -> None:
    diag = FrequencyDiagnostics()
    receiver = PicoReceiver(on_frame=diag.on_frame)
    receiver.start()

    print("Pico frequency diagnostic started. Press Ctrl+C to stop.")

    try:
        while True:
            snap = diag.snapshot()
            print(
                "frames={frames} elapsed={elapsed:.1f}s fps={fps:.2f} "
                "pico_non_mono={non_mono} pc_dt_ms[{dt}] "
                "valid_ratio(L/R)=({left_ratio:.2%}/{right_ratio:.2%}) "
                "zero_pose_invalid(L/R)=({left_zero}/{right_zero})".format(
                    frames=snap["total_frames"],
                    elapsed=snap["elapsed_s"],
                    fps=snap["fps"],
                    non_mono=snap["pico_non_monotonic_count"],
                    dt=snap["dt_summary"],
                    left_ratio=snap["left_valid_ratio"],
                    right_ratio=snap["right_valid_ratio"],
                    left_zero=snap["left_zero_pose_invalid_count"],
                    right_zero=snap["right_zero_pose_invalid_count"],
                )
            )
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()
        print("Pico frequency diagnostic stopped.")


if __name__ == "__main__":
    main()
