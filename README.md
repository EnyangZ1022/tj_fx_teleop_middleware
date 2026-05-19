# tj_fx_teleop_middleware

A lightweight MVP middleware for Pico-based teleoperation of a fixed-base dual-arm robot.

## Overview

This repository focuses on turning Pico motion/controller input into clean internal data structures that can be consumed by later teleoperation stages.

## Architecture (MVP)

Pico input -> TeleopFrame -> coordinate transform -> safety gate -> target buffer -> fixed-rate scheduler -> robot SDK adapter

Stage 1 covers Pico input refactor and parser testability.

Stage 2 adds semantic mapping from PicoRawFrame to TeleopFrame for higher-level teleoperation modules.

Stage 3 adds position-only calibration and dual-arm coordinate transform logic (pure Python, no robot SDK calls yet).

Stage 4 adds a safety state machine and target safety gate that decides whether Stage 3 targets are safe to pass to a future command loop.

Stage 5 adds fixed-rate command scheduling, zero-order hold buffering, and unit-audited command targets for future SDK adapter integration.

Stage 6A adds a read-only robot SDK adapter that connects, checks feedback stream, and exposes dual-arm FK feedback for calibration.

Stage 6B-pre adds a safe startup-ready-pose module that moves both arms to configured low-speed joint ready positions and then stops (not teleoperation).

Stage 6B adds a minimal robot command adapter that consumes scheduled DualArmCommandTarget, solves IK, and sends joint commands with strict safety guards.

Stage 6C adds integration validation checks, upstream XRoboToolkit alignment audit, coordinate/unit/reference-relative consistency tests, and dry-run diagnostics scripts.

## Pico Interface Notes

See `docs/pico_interface_notes.md` for currently confirmed protocol and coordinate assumptions.

## Teleop Mapping Notes

See `docs/teleop_mapping_notes.md` for Stage 2 semantic mapping and timing assumptions.

## Calibration and Transform Notes

See `docs/calibration_transform_notes.md` for Stage 3 calibration anchors, transform formula, and arm-frame mapping assumptions.

## Safety State Machine Notes

See `docs/safety_state_machine_notes.md` for Stage 4 safety states, gating conditions, and output decision policy.

## Command Loop Notes

See `docs/command_loop_notes.md` for Stage 5 scheduler behavior, unit conventions, fixed IK references, and limiter policy.

## Robot SDK Read-Only Notes

See `docs/robot_sdk_readonly_notes.md` for Stage 6A robot connection flow, feedback conversion, and arm mapping policy.

## Robot Startup Ready Pose Notes

See `docs/robot_startup_ready_pose_notes.md` for Stage 6B-pre startup scope, ready-pose safety policy, and run checklist.

## Robot Command Adapter Notes

See `docs/robot_command_adapter_notes.md` for Stage 6B command adapter flow, fixed IK reference policy, and send safety limits.

## XRoboToolkit Upstream Audit

See `docs/xrobotoolkit_upstream_audit.md` for Stage 6C upstream alignment conclusions and measured-data coordinate policy.

## Stage 6C Integration Checklist

See `docs/stage6c_integration_checklist.md` for Stage 6C integration verification checklist.

## Validation

Run the following checks:

```bash
pytest -q
python -m compileall src scripts tests
```

Manual hardware startup command (only when safe):

```bash
python scripts/move_robot_to_ready_pose.py --robot-ip 192.168.1.190
```

Manual dry-run command adapter check (default safe mode):

```bash
python scripts/robot_command_dry_run.py --robot-ip 192.168.1.190 --dry-run
```

Manual real send (only after ready pose and operator confirmation):

```bash
python scripts/robot_command_dry_run.py --robot-ip 192.168.1.190 --enable-send --delta-mm 2
```

Manual Stage 6C dry diagnostics (no hardware required):

```bash
python scripts/check_coordinate_mapping.py
python scripts/check_stage6_pipeline_dry.py
```

PICO 4 Ultra hardware validation notes:

Device: PICO 4 Ultra
App: XRoboToolkit-PICO-1.1.1.apk

Observed controller coordinate direction:
- Moving controller upward mainly increases Y.
- Moving controller to the user's right mainly increases X.
- Moving controller forward mainly increases Z.

Preliminary conclusion:
The controller pose output is consistent with the customer-reported convention:
+X right, +Y up, +Z forward.

Caution:
The test was performed by free-hand motion, so each motion may contain coupled changes across multiple axes. A more constrained single-axis test is recommended before final robot calibration.

Observed issue:
Head pose is currently invalid / not available in the inspected frames.

Observed input frequency:
The measured receive rate is around 80-85 Hz in the current test environment, lower than the previously reported 93-95 Hz. The robot command loop should remain decoupled from the Pico input rate.
