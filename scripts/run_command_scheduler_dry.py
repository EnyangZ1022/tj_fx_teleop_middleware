from __future__ import annotations

from pathlib import Path
import sys

# Allow running this script directly without installing the package.
ROOT_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT_DIR / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from teleop.control.command_scheduler import FixedRateCommandScheduler
from teleop.control.target_buffer import TargetBuffer
from teleop.control.target_limiter import TargetLimiter, TargetLimiterConfig
from teleop.core.robot_frame import DualArmRobotTarget, RobotArmTarget


def _make_right_target(x_mm: float, y_mm: float, z_mm: float) -> DualArmRobotTarget:
    return DualArmRobotTarget(
        left=None,
        right=RobotArmTarget(
            position_xyz=(x_mm, y_mm, z_mm),
            orientation_abc=(10.0, 20.0, 30.0),
            valid=True,
        ),
    )


def main() -> None:
    buffer = TargetBuffer()
    limiter = TargetLimiter(
        TargetLimiterConfig(
            max_single_step_mm=5.0,
            max_cartesian_velocity_mm_s=200.0,
            clip_instead_of_reject=True,
        )
    )
    scheduler = FixedRateCommandScheduler(target_buffer=buffer, limiter=limiter)

    now_ns = 1_000_000_000
    step_ns = scheduler.period_ns()

    # First safe target arrives.
    buffer.update(_make_right_target(1000.0, 2000.0, 3000.0), timestamp_ns=now_ns)

    for tick in range(12):
        # Inject a jump target to demonstrate limiter clipping.
        if tick == 4:
            buffer.update(_make_right_target(1100.0, 2000.0, 3000.0), timestamp_ns=now_ns)

        command, diag = scheduler.step(now_ns=now_ns)

        if command is None or command.right is None:
            print(
                f"tick={tick:02d} seq={diag.sequence_id} cmd=None "
                f"age_ms={diag.target_age_ms} zoh={diag.used_zero_order_hold} "
                f"limited={diag.limited} reason={diag.limit_reason}"
            )
        else:
            print(
                f"tick={tick:02d} seq={diag.sequence_id} "
                f"pos_mm={command.right.position_xyz_mm} "
                f"ik_ref={command.right.ik_reference_q_deg} "
                f"age_ms={diag.target_age_ms:.1f} "
                f"zoh={diag.used_zero_order_hold} limited={diag.limited} reason={diag.limit_reason}"
            )

        now_ns += step_ns


if __name__ == "__main__":
    main()
