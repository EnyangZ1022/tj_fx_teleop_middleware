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

Stage 7 adds optional asynchronous logging/recording/replay diagnostics infrastructure with non-blocking queue-based writing and disabled-by-default safety settings.

Stage 8 adds an optional PySide6 + PyQtGraph diagnostic UI that visualizes dual-arm target/feedback/error/status at a default 20 Hz refresh using a latest-snapshot model.

Stage 9 fixes integration semantics (safety units in mm/mm_s and axisClick rising-edge calibration trigger) and adds the full orchestration entrypoint that connects all stages with safe dry-run defaults.

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

## Logging and Replay Notes

See `docs/logging_replay_notes.md` for Stage 7 optional logging architecture, performance-safety policy, and replay scope.

## UI Visualization Notes

See `docs/ui_visualization_notes.md` for Stage 8 diagnostic UI scope, refresh policy, and run commands.

## Full Teleop App Notes

See `docs/full_teleop_app_notes.md` for Stage 9 orchestration flow, runtime safety defaults, and full app launch commands.

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

Manual Stage 7 dry diagnostics (no hardware required):

```bash
python scripts/logging_dry_run.py
python scripts/analyze_teleop_log.py --input <jsonl>
python scripts/replay_teleop_log.py --input <jsonl>
```

Manual Stage 8 UI diagnostics:

```bash
python scripts/run_teleop_ui_mock.py
python scripts/run_teleop_ui_from_log.py --input <jsonl>
```

Manual Stage 9 full app commands:

```bash
# Pico only, no robot
python scripts/run_full_teleop.py --no-robot --dry-run --ui

# Robot feedback plus dry-run command path
python scripts/run_full_teleop.py --robot-ip 192.168.1.190 --dry-run --ui

# Real send only after staged checks and on-site safety confirmation
python scripts/run_full_teleop.py --robot-ip 192.168.1.190 --move-to-ready --enable-send --confirm --ui
```

Stage 7 logging defaults are safe by design:

- logging is disabled by default
- frame/performance recording is disabled by default
- logging must remain optional and non-blocking

Stage 8 UI defaults are safe by design:

- UI is disabled by default in config
- UI is diagnostic-only and does not send robot commands
- UI refresh defaults to 20 Hz (50 ms timer)

Stage 9 full-app defaults are safe by design:

- dry_run is true by default
- enable_send is false by default
- logging is disabled by default
- calibration trigger uses axisClick rising edge (menuButton unused in MVP)
- real robot command send requires explicit `--enable-send --confirm` and typing `YES`

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
