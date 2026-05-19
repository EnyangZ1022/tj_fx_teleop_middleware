# UI Visualization Notes (Stage 8)

## Stage 8 Purpose

Stage 8 adds a lightweight PySide6 + PyQtGraph diagnostic visualization UI for teleoperation state.

The UI is diagnostic-only and does not introduce new robot motion behavior.

## Scope and Safety Rules

- UI does not run the command loop.
- UI does not send robot commands.
- UI does not own safety logic.
- UI does not own command scheduler logic.
- UI widgets do not connect directly to Pico or robot hardware.
- UI refresh path does not perform blocking file I/O.
- UI remains optional and can be disabled by config.

## Refresh Rate Policy

- Default update rate is 20 Hz.
- The Qt timer interval is 50 ms.
- Command loop rates (100 Hz / 50 Hz) remain separate from UI refresh.

## Latest-Snapshot Model

The UI reads only the latest snapshot from an in-memory, thread-safe latest-value store.

No blocking waits are used for UI updates.

## Visual Meaning

- Target ball: teleoperation target position in robot Cartesian space (mm).
- Feedback ball: robot FK feedback end-effector position (mm).
- Error line: line segment between target and feedback per arm.
- Calibration status: per-arm readiness state.
- Safety state: current high-level gate/state-machine status.

## Dependencies

- PySide6
- pyqtgraph
- PyOpenGL

## Run Commands

Mock UI (no hardware required):

```bash
python scripts/run_teleop_ui_mock.py
```

Log replay UI (analysis-only):

```bash
python scripts/run_teleop_ui_from_log.py --input <jsonl>
```

## Config

Stage 8 UI defaults are defined in `configs/ui.yaml` and disabled by default (`ui.enabled=false`).
