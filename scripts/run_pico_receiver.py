from __future__ import annotations

from pathlib import Path
import sys
import time

# Allow running this script directly without installing the package.
ROOT_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from teleop.input.pico_receiver import PicoReceiver


def main() -> None:
    receiver = PicoReceiver()
    receiver.start()
    print("Pico receiver started. Press Ctrl+C to stop.")

    try:
        while True:
            frame = receiver.get_latest_frame()
            if frame is None:
                print("frame_id=-1 left_valid=False right_valid=False left_trigger=0.00 right_trigger=0.00 left_grip=0.00 right_grip=0.00")
            else:
                print(
                    "frame_id={frame_id} left_valid={left_valid} right_valid={right_valid} "
                    "left_trigger={left_trigger:.2f} right_trigger={right_trigger:.2f} "
                    "left_grip={left_grip:.2f} right_grip={right_grip:.2f}".format(
                        frame_id=frame.frame_id,
                        left_valid=frame.left_valid,
                        right_valid=frame.right_valid,
                        left_trigger=frame.left_ctrl.trigger,
                        right_trigger=frame.right_ctrl.trigger,
                        left_grip=frame.left_ctrl.grip,
                        right_grip=frame.right_ctrl.grip,
                    )
                )
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()
        print("Pico receiver stopped.")


if __name__ == "__main__":
    main()
