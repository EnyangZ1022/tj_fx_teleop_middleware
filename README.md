# tj_fx_teleop_middleware

A lightweight MVP middleware for Pico-based teleoperation of a fixed-base dual-arm robot.

## Overview

This repository focuses on turning Pico motion/controller input into clean internal data structures that can be consumed by later teleoperation stages.

## Architecture (MVP)

Pico input -> TeleopFrame -> coordinate transform -> filtering/safety -> robot SDK adapter

Stage 1 covers Pico input refactor and parser testability.

Stage 2 adds semantic mapping from PicoRawFrame to TeleopFrame for higher-level teleoperation modules.

Stage 3 adds position-only calibration and dual-arm coordinate transform logic (pure Python, no robot SDK calls yet).

## Pico Interface Notes

See `docs/pico_interface_notes.md` for currently confirmed protocol and coordinate assumptions.

## Teleop Mapping Notes

See `docs/teleop_mapping_notes.md` for Stage 2 semantic mapping and timing assumptions.

## Calibration and Transform Notes

See `docs/calibration_transform_notes.md` for Stage 3 calibration anchors, transform formula, and arm-frame mapping assumptions.

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
