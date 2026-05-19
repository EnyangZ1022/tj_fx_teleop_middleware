# Pico Interface Notes

This document records currently confirmed interface information for the Pico teleoperation input.

## 1. Communication protocol

- UDP broadcast port: 29888, used for Pico discovery.
- TCP server port: 63901, used for receiving Pico frame data.
- TCP message structure: head, cmd, length, payload, timestamp, tail.
- Important command values:
  - CMD_CONNECT
  - CMD_HEARTBEAT
  - CMD_STATE_JSON
  - CMD_TCPIP
- State JSON is double-encoded:
  - Outer JSON contains `value`.
  - `value` is another JSON string.

## 2. Parsed frame structure

Current parsed data:

- device_id
- timestamp_ns
- head_pose
- left_ctrl
- right_ctrl

Pose format: `(x, y, z, qx, qy, qz, qw)`

Position unit: meter

Quaternion: local-to-world, according to customer confirmation.

## 3. Customer-confirmed coordinate information

- World frame origin is related to the head ground projection point at first startup or recenter.
- Restarting or recentering Pico may change the world frame.
- Customer reported axes: +X right, +Y up, +Z forward.
- This should be double-checked against common OpenXR description (+X right, +Y up, -Z forward).
- Z-axis convention is pending final confirmation if needed.

## 4. Tracking behavior

- If controller tracking is lost or blocked, pose may become zero pose.
- Software should treat zero pose as invalid.
- Invalid pose must not be sent to robot control.

## 5. Frame rate

- Customer reported Pico sending rate: 93-95 Hz.
- Software should not assume perfectly fixed timing.
- PC receive timestamp and jitter should still be measured in dry-run tests.

## 6. Button mapping

| JSON field | Pico button | Type |
|---|---|---|
| `trigger` | Index trigger | float 0-1 |
| `grip` | Side grip button | float 0-1 |
| `primaryButton` | A button on right controller / X button on left controller | bool |
| `secondaryButton` | B button on right controller / Y button on left controller | bool |
| `menuButton` | Menu button | bool |
| `axisX` / `axisY` | Joystick | float -1 to 1 |
| `axisClick` | Joystick click | bool |

## 7. Proposed MVP control mapping

- `grip > 0.8` as teleoperation enable / deadman switch.
- `trigger` as gripper continuous control.
- `gripper_position = 1.0 - trigger`.
- Gripper command deadband = 0.01.
- `primaryButton` reserved for start / pause.
- `secondaryButton` reserved for cancel / reset.
- `axisClick` preferred for calibration request (rising edge detection).
- `menuButton` is unused in MVP calibration flow and not safety-critical.

## 8. Out-of-scope for Prompt 1

- No robot SDK control yet.
- No UI yet.
- No coordinate transformation yet.
- No filtering or safety state machine yet.
- Prompt 1 only refactors Pico input into a callable and testable module.
