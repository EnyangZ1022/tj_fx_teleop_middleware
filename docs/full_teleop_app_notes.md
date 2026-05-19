# Full Teleop App Notes (Stage 9)

## Stage 9 Purpose

Stage 9 adds a single orchestration entrypoint that connects all implemented stages while keeping safety-first defaults.

Default runtime policy is dry-run and no robot command send.

## End-to-end Data Flow

PicoReceiver -> TeleopFrame -> Calibration -> RobotTarget -> SafetyGate -> TargetBuffer -> Scheduler -> CommandAdapter

Optional observers:
- diagnostic UI snapshot updates
- asynchronous logging

## Safety-first Defaults

- dry_run defaults to true
- enable_send defaults to false
- UI defaults to disabled
- logging defaults to disabled
- move_to_ready defaults to false

Real send is only allowed with explicit CLI opt-in and interactive YES confirmation.

## Calibration Process

- Press axisClick rising edge to request calibration.
- Calibration captures controller reference and robot FK reference.
- Hold grip (deadman) to activate teleoperation motion path.
- Trigger remains gripper semantic input (gripper command integration can remain future work).

menuButton is unused for MVP calibration logic.

## Unit Conventions

- Pico/controller position: meters
- RobotTarget and CommandTarget position: millimeters
- Robot orientation abc: degrees
- Joint q: degrees
- Safety step limit: mm
- Safety velocity limit: mm/s

## Running Commands

Pico-only dry run:

python scripts/run_full_teleop.py --no-robot --dry-run --ui

Robot read-only plus dry run:

python scripts/run_full_teleop.py --robot-ip 192.168.1.190 --dry-run --ui

UI plus logging dry run:

python scripts/run_full_teleop.py --robot-ip 192.168.1.190 --dry-run --ui --logging

Real send (only after staged checks and onsite safety verification):

python scripts/run_full_teleop.py --robot-ip 192.168.1.190 --move-to-ready --enable-send --confirm --ui

## Manual Validation Sequence

1. Pure dry pipeline check:

python scripts/check_stage6_pipeline_dry.py

2. Pico-only dry run:

python scripts/run_full_teleop.py --no-robot --dry-run --ui

3. Robot read-only plus dry-run:

python scripts/run_full_teleop.py --robot-ip 192.168.1.190 --dry-run --ui

4. Real send only after staged checks:

python scripts/run_full_teleop.py --robot-ip 192.168.1.190 --move-to-ready --enable-send --confirm --ui

## Known Limitations

- no workspace planner yet
- position-only teleoperation
- orientation remains frozen from calibration anchor
- gripper motion command path may remain future work
- UI is diagnostic-only
- logging is optional and disabled by default
