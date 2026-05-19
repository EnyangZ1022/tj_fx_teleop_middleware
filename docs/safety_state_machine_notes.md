# Safety State Machine Notes (Stage 4)

## Stage 4 Purpose

Stage 4 adds a pure-Python safety state machine and target safety gate.

Input:

- `TeleopFrame`
- `DualArmCalibrationState`
- `DualArmRobotTarget`

Output:

- `SafetyDecision`
- optional filtered `safe_target`

## Why a safety gate is needed

Stage 3 can generate position-only robot targets. Before any future SDK command sending, a safety gate must verify that each target is safe and context is valid.

## State definitions

- `DISCONNECTED`: no frame or stale frame.
- `PICO_CONNECTED`: frame exists but system is not ready for motion.
- `WAIT_CALIBRATION`: calibration is missing for required sides.
- `CALIBRATED`: calibration exists but motion is not currently allowed.
- `TELEOP_READY`: calibrated and valid, waiting for deadman enable.
- `TELEOP_ACTIVE`: at least one side passes all checks and motion is allowed.
- `PAUSED`: temporary safety block (for example invalid target/pose, jump, velocity).
- `ERROR`: error-latched stop condition.
- `EMERGENCY_STOP`: emergency stop latched, no motion allowed.

## Conditions that block motion

- no Pico frame
- stale frame
- invalid or zero pose
- missing calibration
- enable released
- missing or invalid `RobotTarget`
- target jump too large
- target velocity too high
- emergency stop

## Input semantics

- `grip` is the deadman switch input.
- `axisClick` is intended for calibration request trigger, not for enable.

## Stage boundary

Stage 4 does not send commands to the robot. It only evaluates safety and returns `SafetyDecision`.

Future Stage 5 command loop should call the safety gate before sending any target.

Future SDK adapter should consume only `safe_target` from `SafetyDecision`.
