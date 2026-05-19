# Command Loop Notes (Stage 5)

## Stage 5 Purpose

Stage 5 adds a fixed-rate command scheduling layer that prepares command-side targets after safety-gated robot targets are available.

Pipeline at this stage:

- `TeleopFrame`
- calibration + coordinate transform
- `DualArmRobotTarget`
- safety gate
- safe robot target
- target buffer
- fixed-rate command scheduler
- scheduled command target

## Why fixed rates use 100 Hz / 50 Hz

The robot controller inner loop is 1000 Hz. Command rates of 100 Hz and 50 Hz divide 1000 Hz cleanly, which simplifies deterministic timing alignment in future SDK integration.

Pico input frequency must not directly drive robot command frequency. Input is asynchronous and jittery relative to the robot inner loop.

## Unit convention (audited)

- Pico position: meter
- Teleop pose position: meter
- Robot feedback/target position: millimeter
- Robot orientation: degree
- Joint angle: degree
- Time: second

## Fixed IK reference angles

Stage 5 command targets carry fixed IK reference joint angles for future adapter use.

- left: `[90, -90, -90, -90, 0, 0, 0]`
- right: `[90, 90, -90, -90, 0, 0, 0]`

These references remain fixed in this stage.

## Stage boundaries

- Stage 5 does not call IK.
- Stage 5 does not call SDK.
- Stage 5 does not send commands to robot.

It only prepares scheduled command targets for future SDK adapter integration.

## Zero-order hold and target age timeout

The scheduler runs at fixed logical ticks via `step(now_ns)`.

- If the latest buffered target is still fresh, repeated ticks can reuse it (zero-order hold).
- If target age exceeds timeout, scheduler returns no command.

## Command-side limiting

Command-side limiting is a second layer after safety gate:

- single-step Cartesian limit (`max_single_step_mm`)
- Cartesian velocity limit (`max_cartesian_velocity_mm_s`)

The stricter bound is applied each tick. Limiter can clip or reject per side.

Orientation is preserved unchanged in Stage 5.

## Non-blocking behavior

Command scheduling and diagnostics should remain non-blocking. Future logging should not block a real-time command loop.

## Stage 6C Integration Notes

- Scheduler consumes safety-gated targets and outputs `DualArmCommandTarget` only.
- Scheduler does not read Pico directly.
- Scheduler does not call SDK directly.
- Units remain explicit: xyz in mm, abc in deg, q reference in deg, time in s.
- Fixed IK references are attached by configuration and should remain fixed.

Cross-links:

- `docs/xrobotoolkit_upstream_audit.md`
- `docs/stage6c_integration_checklist.md`
