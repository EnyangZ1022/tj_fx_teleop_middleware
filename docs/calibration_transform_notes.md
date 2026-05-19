# Calibration and Coordinate Transform Notes (Stage 3)

## Stage 3 Purpose

Stage 3 adds a pure-Python, position-only calibration and coordinate transform layer:

- Input: `TeleopFrame` + robot feedback snapshot
- Internal state: calibration anchors (`DualArmCalibrationState`)
- Output: position-only robot targets (`DualArmRobotTarget`)

This stage does not call the robot SDK and is fully testable with synthetic pytest data.

## Why absolute Pico world origin is not used

Pico world origin can shift after restart/recenter. Therefore, Stage 3 uses relative motion from a calibration anchor instead of absolute world coordinates.

For each arm:

`delta_pico = pico_position - pico_anchor`

## Position-only transform formula

Stage 3 target position is:

`delta_robot_mm = 1000.0 * scale * A_arm * R_user_from_pico * (pico_position_m - pico_anchor_m)`

`target_position_mm = robot_anchor_mm + delta_robot_mm`

Where:

- `robot_anchor_mm` is the robot end-effector position captured at calibration (millimeter).
- `A_arm` maps user semantic axes into the arm SDK frame.
- `R_user_from_pico` maps Pico world displacement into user-intended forward/right/up displacement.
- `scale` is per-arm motion gain.

## Coordinate conventions used in Stage 3

Pico controller frame (validated in current project):

- `+X`: user right
- `+Y`: user up
- `+Z`: user forward

Pico / Teleop position unit:

- meter

Right arm SDK frame:

- `+X`: forward
- `+Y`: up
- `+Z`: right

Left arm SDK frame:

- `+X`: forward
- `+Y`: down
- `+Z`: left

Robot SDK-side units used in Stage 3 output:

- position: millimeter
- orientation: degree

## Independent left/right calibration

Left and right arm SDK frame origins are different. Because of this:

- left Pico controller controls only left arm;
- right Pico controller controls only right arm;
- left and right anchors are stored independently;
- each generated target is directly expressed in that arm's own SDK frame.

## Orientation policy (frozen orientation)

Stage 3 is position-only teleoperation.

- During calibration, each arm stores `robot_anchor_abc` from robot feedback.
- During target generation, orientation remains fixed:
  - `target_orientation = robot_anchor_abc`

No orientation teleoperation is applied in this stage.

## Why `R_user_from_pico` is explicit

`R_user_from_pico` defaults to identity in MVP, but it is intentionally explicit so future calibration can account for:

- user standing/sitting posture differences,
- recentering effects,
- imperfect alignment between natural user motions and raw Pico world axes.

## Calibration trigger intent

Axis click (`axisClick`) is the intended MVP calibration trigger, using rising-edge detection.

- previous `False` -> current `True` means calibration requested.
- held `True` does not retrigger.

## Out of scope in Stage 3

- robot SDK calls and real robot connection
- filtering
- interpolation
- safety state machine
- workspace limits
- orientation teleoperation
- UI

## Stage 6C Integration Alignment

Stage 6C audit confirms this transform is reference-relative and should remain so.

- We do not use XR absolute world origin as robot absolute target origin.
- We use activation/deadman or calibration references to compute displacement-based targets.
- We do not accumulate targets as `target_t = target_{t-1} + delta_t`.
- Head pose is not a control dependency.

Input naming in this project remains:

- `grip`: deadman/enable
- `trigger`: gripper control
- `axisClick`: calibration request
- `menuButton`: unused in MVP calibration flow

See also:

- `docs/xrobotoolkit_upstream_audit.md`
- `docs/stage6c_integration_checklist.md`