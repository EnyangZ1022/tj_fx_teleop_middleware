# IK ZSP Mode Notes (Stage 10)

## Background

Previous IK behavior in this project used fixed reference joints only.

In real-hardware validation, IK may fail or become posture-sticky when the arm moves far from initial posture.

## Stage 10 Trial Strategy

Stage 10 introduces an optional ZSP / specified-plane IK mode.

Developer-provided suggested values:

- `m_Input_IK_ZSPType = 1`
- `m_Input_IK_ZSPPara = [0, 0, -1, 0, 0, 0]`

Interpretation:

- enable specified-plane preference
- prefer elbow opening toward negative Z direction

## Current Adapter Behavior

- Fixed reference input (`ik_reference_q_deg`) is still passed.
- ZSP settings are applied optionally by config.
- If SDK wrapper field assignment fails, adapter falls back without crashing.
- Solver notes are exposed for diagnostics (`zsp_applied` or fallback note).

## Config

See `configs/robot_command.yaml`:

- `ik_solver.mode`
- `ik_solver.enable_zsp`
- `ik_solver.zsp_type`
- `ik_solver.zsp_para`
- `ik_solver.keep_fixed_reference`

## Revert to Previous Behavior

Use either:

- `enable_zsp=false`, or
- `mode=fixed_reference_only`

## Scope

This is an experimental hardware-validation patch.
It does not remove fixed-reference IK and does not change command safety defaults.
