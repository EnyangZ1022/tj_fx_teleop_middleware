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
