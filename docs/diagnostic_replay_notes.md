# Diagnostic Replay Notes

## Purpose

`analyze_teleop_jump.py` is an offline diagnostic tool for recorded `teleop_session.jsonl` logs.

It helps answer:

- whether command joint angles `q` jump relative to ready pose,
- whether/when robot feedback starts moving after command send,
- what early reject reasons appear around calibration/enable/first send.

## Scope and Safety

This tool is offline-only.

- It does not connect to PICO.
- It does not connect to robot.
- It does not send robot commands.
- It does not import robot SDK dynamic libraries.
- It does not recompute IK.

The tool only analyzes already-recorded log data.

## Inputs and Outputs

Input:

- `teleop_session.jsonl`

Outputs under output directory (default `teleop_jump_analysis`):

- `jump_timeseries.csv`
- `jump_report.md`
- `command_q_jump_deg.png` (if plotting available and not disabled)
- `feedback_displacement_mm.png` (if plotting available and not disabled)

## Example

```bash
python scripts/analyze_teleop_jump.py --input logs/<session>/teleop_session.jsonl
```

## IK Replay / Q-Series Analysis

`replay_teleop_ik.py` adds offline q-series diagnostics around reject events.

It helps answer:

- whether recorded command q jumps before reject events,
- which joints contribute most to step and velocity spikes,
- whether reject reasons like `joint_step_limit` or `joint_velocity_limit` are explained by q discontinuity.

This tool is still offline:

- It does not connect to PICO.
- It does not connect to robot.
- It does not send robot commands.

Current status:

- robust recorded command q analysis is fully supported,
- full recompute mode requires target xyzabc in logs and SDK availability.

Plots are intentionally split by arm/metric to stay readable:

- `left_q_joints.png`
- `right_q_joints.png`
- `q_step_deg.png`
- `q_velocity_deg_s.png`

When runtime command logging includes step-limit diagnostics, use these fields to explain apparent "stuck" or lag behavior:

- `command_left_step_delta_deg` / `command_right_step_delta_deg`
- `command_left_velocity_delta_deg_s` / `command_right_velocity_delta_deg_s`
- `command_left_allowed_step_deg` / `command_right_allowed_step_deg`
- `command_left_joint_ramped` / `command_right_joint_ramped`
- `command_left_candidate_q_deg` / `command_right_candidate_q_deg`
- `command_left_sent_q_deg` / `command_right_sent_q_deg`

In `joint_limit_mode=ramp`, `candidate_q_deg` can remain far from `sent_q_deg` while the adapter emits bounded intermediate joint commands.

Example:

```bash
python scripts/replay_teleop_ik.py --input logs/<session>/teleop_session.jsonl --mode recorded
```
