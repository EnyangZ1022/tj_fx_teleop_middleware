# Robot Command Adapter Notes (Stage 6B)

## Stage 6B Purpose

Stage 6B adds a minimal robot command adapter that consumes scheduled command targets and converts them to joint commands through IK.

This stage does not read Pico input directly.

## Data Source and Scope

Input to this adapter is `DualArmCommandTarget` from the upstream scheduler path:

Safety gate -> target buffer -> fixed-rate command scheduler -> robot command adapter.

The adapter should not bypass safety gate/scheduler and should not recompute teleop calibration.

## Unit Convention

- position xyz: millimeter (mm)
- orientation abc: degree (deg)
- joint q: degree (deg)
- time: second (s)

## IK Flow

Per arm, IK follows:

1. xyzabc -> mat4x4
2. mat4x4 -> mat1x16
3. create `FX_InvKineSolvePara`
4. set IK target pose and fixed IK reference joints
5. call kinematics IK
6. read output q[7] in degree

## Fixed IK Reference Policy

Use the fixed `ik_reference_q_deg` provided by each command target/config.

Do not use previous IK output as next reference.

This policy is validated again in Stage 6C integration checks.

## Stage 10 Optional ZSP IK Mode

Stage 10 adds an optional IK solver strategy:

- mode: `zsp_negative_z`
- zsp_type: `1`
- zsp_para: `[0, 0, -1, 0, 0, 0]`

This mode is experimental and intended for real-hardware validation.

Fixed reference is still passed and remains the fallback behavior.
To revert to previous behavior, set either:

- `enable_zsp=false`, or
- `mode=fixed_reference_only`

## Startup Dependency

Before Stage 6B command sending, robot startup should pass Stage 6B-pre and reach ready pose safely.

## Safety Policy

- dry-run is enabled by default
- command_enabled is false by default
- IK failure rejects command
- invalid target rejects command
- joint step limit rejects large jumps
- joint velocity limit rejects excessive speed
- pause stops sending new commands
- emergency stop is physical; software side only blocks sending and can disconnect

Input semantics for upstream control intent remain:

- `grip`: deadman/enable
- `trigger`: gripper control
- `axisClick`: calibration request

## Out of Scope in Stage 6B

- direct Pico-to-robot integration inside command adapter
- orientation teleoperation
- UI
- async logging
- full continuous teleoperation integration demo
- Stage 6C audit/planning

## Stage 6C Cross-References

- `docs/xrobotoolkit_upstream_audit.md`
- `docs/stage6c_integration_checklist.md`
