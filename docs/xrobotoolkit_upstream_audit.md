# XRoboToolkit Upstream Audit (Stage 6C)

## Purpose of This Audit

XRoboToolkit is used as an upstream reference only.

This project does not copy the full XRoboToolkit teleoperation runtime stack.

## Useful Upstream Ideas We Reuse

- controller input naming and field semantics
- reference-relative teleoperation paradigm
- position-only vs full-pose control separation
- target visualization concept for debugging

## What We Do Not Reuse Directly

- XR absolute world origin as robot absolute target source
- full upstream runtime architecture and dependencies
- head pose as a required control dependency

## Coordinate Convention Decision

Upstream documentation may contain coordinate text ambiguity.

Our implementation follows measured received data in this project:

- controller +X: user right
- controller +Y: user up
- controller +Z: user forward/inward
- handedness: left-handed based on received data behavior

If any external statement conflicts with measured data, measured data takes priority in this MVP.

## Reference-Relative Policy

Target generation is reference-relative:

- capture controller reference pose at activation/deadman rising edge or calibration
- capture robot FK reference pose at the same moment
- use converted displacement from reference to compute new target

This means:

- no dependence on XR absolute world origin
- no cumulative integration of old target history

## Input Semantics in This Project

- grip: enable/deadman
- trigger: gripper control
- axisClick: calibration request / optional manual recalibration
- primaryButton: start/confirm candidate
- secondaryButton: pause/cancel candidate

Terminology guidance:

Use activation/deadman rising edge wording for reference capture.

Do not use physical trigger press wording for activation, because trigger is reserved for gripper behavior here.
