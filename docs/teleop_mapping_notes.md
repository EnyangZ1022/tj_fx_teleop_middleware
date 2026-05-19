# Teleop Mapping Notes

## Stage 2 Purpose

Stage 2 introduces a semantic mapping layer that converts Pico transport-level input frames into teleoperation-facing input frames. It keeps the implementation lightweight and testable for MVP progress.

## PicoRawFrame vs TeleopFrame

- `PicoRawFrame`: protocol-level data parsed from Pico transport messages.
- `TeleopFrame`: semantic input for teleoperation modules, preserving timestamps and validity while exposing higher-level intent fields.

## Current Input Semantic Mapping

- Enable / deadman switch: `grip > 0.8`.
- Gripper control: `gripper_position = 1.0 - trigger`.
- Gripper deadband: `0.01`.
- Reserved button semantics:
  - `primaryButton`: start / pause request
  - `secondaryButton`: cancel / reset request
  - `axisClick`: preferred calibration request trigger (rising edge)
  - `menuButton`: unused in MVP (not safety-critical)

Calibration request is detected in orchestration via axisClick rising edge between previous/current frames.
Direct TeleopFrame mapping should not treat menuButton as calibration request.

`axisX` and `axisY` are carried through for future usage. Stage 2 does not implement joystick behavior.

## Validity Rule

If Pico tracking is lost, pose may become all zeros. Zero pose and invalid pose are not treated as valid teleoperation input.

## Timing Assumption

- Pico input rate is typically around 93-95 Hz.
- Robot controller internal loop may run at 1000 Hz.
- Future command loop should use rates that divide 1000 Hz cleanly, such as 100 Hz or 50 Hz.
- Stage 2 does not implement interpolation or fixed-rate command scheduling.

## Coordinate Assumption

Stage 2 preserves Pico pose as Pico-frame pose. No robot coordinate transform is performed yet. Future coordinate calibration may use SDK utilities if available.
