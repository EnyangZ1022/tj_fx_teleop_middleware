# Logging and Replay Notes (Stage 7)

## Stage 7 Purpose

Stage 7 adds optional logging, recording, and diagnostic replay infrastructure.

This stage does not add new robot motion behavior.

## Safety and Performance Principle

Logging must never compromise real-time control performance.

Key rules:

- logging has a global master switch
- logging is disabled by default
- runtime/control paths do not perform blocking disk I/O
- runtime/control paths do not flush files
- when queue is full, low-priority records can be dropped
- dropped count is tracked for diagnostics

## Recommended Modes

- normal low-risk operation: logging.enabled=false
- initial debug: event-only logging
- diagnostics: sampled frame/performance logging
- short controlled tests only: high-rate frame logging

## Format and Writer Design

- primary format: JSONL (`teleop_session.jsonl`)
- asynchronous background writer thread
- non-blocking enqueue from runtime paths
- batch writes with periodic flush

## Queue Full Behavior

- frame/performance records are low priority and can be dropped first
- event/error records use best-effort higher priority insertion
- logger never blocks caller waiting for queue space

## Dry-Run and Analysis Commands

Dry-run logging demo:

```bash
python scripts/logging_dry_run.py
```

Analyze a log file:

```bash
python scripts/analyze_teleop_log.py --input <jsonl>
```

Replay for analysis-only inspection:

```bash
python scripts/replay_teleop_log.py --input <jsonl>
```

## Replay Safety Scope

Replay in Stage 7 is analysis-only.

Replay does not send robot commands.

## Dependency Policy

Logging is optional and must never be required for teleoperation runtime to execute.
