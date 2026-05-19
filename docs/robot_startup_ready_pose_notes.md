# Robot Startup Ready Pose Notes (Stage 6B-pre)

## Stage 6B-pre Purpose

Stage 6B-pre is a safe startup-only step before teleoperation.

This stage is not teleoperation.

It only prepares the robot by moving to a known ready joint posture in low-speed position mode, then stops.

## Startup Parameters

- vel_ratio: 20
- acc_ratio: 20
- unit for joint angles: degree
- unit for time: second

## Ready Pose (Degree)

- left: [90, -60, -90, -90, 0, 0, 0]
- right: [90, 60, -90, -90, 0, 0, 0]

## Why Not Use [0, 0, 0, 0, 0, 0, 0] as Startup Pose

A full-zero joint vector is not guaranteed to be a safe or reachable posture for the real robot installation.

Using all-zero joints as startup can increase risk of unexpected links/cable posture, workspace collisions, or singular-like states depending on mounting and current configuration.

The configured ready pose is selected as a safer operational posture for this MVP startup flow.

## Ready Pose vs IK Reference

- Ready pose: actual startup posture used to physically place the robot before teleoperation.
- IK reference: fixed reference joint vector used only by future IK solving.

Do not overwrite IK reference when adjusting startup ready pose.

## Safety Checklist Before Running Startup Script

- Robot area is clear and operators are informed.
- Physical emergency stop is available and reachable.
- Arm mapping between project sides and SDK labels A/B is confirmed.
- Startup velocity and acceleration are low (20/20 by default).
- Pico teleoperation input is not connected to robot motion.
- No continuous command loop is running.

## Out of Scope in Stage 6B-pre

- Pico input integration
- TeleopFrame usage
- IK solving
- Cartesian command sending
- Fixed-rate command scheduler integration
- Joint impedance mode
- Continuous teleoperation mode
