# tj_fx_teleop_middleware

Lightweight middleware for PICO/XRoboToolkit based dual-arm teleoperation.

It receives PICO controller pose/button data, maps input into robot-side targets, applies calibration and safety checks, runs fixed-rate scheduling, calls IK, and routes commands through the robot SDK adapter.

Diagnostic UI and asynchronous logging are supported as optional tools.

## 1. Features

- PICO receiver and frame parser
- Teleop semantic mapping from raw PICO fields
- Axis-click calibration trigger (rising edge)
- Position-only teleoperation with frozen end-effector orientation (default)
- Safety gate with timeout, target jump, and velocity checks
- Fixed-rate scheduler (100 Hz / 50 Hz paths)
- Robot SDK read-only feedback and FK feedback
- Safe ready-pose startup utility
- Minimal robot SDK command adapter
- Optional async logging and replay tooling (disabled by default)
- Optional PySide6/PyQtGraph diagnostic UI (diagnostic-only)
- Experimental position+orientation teleop mode (explicit opt-in)
- Quaternion SO(3) Slerp low-pass orientation filter in position_orientation mode
- Robot command control mode selection (`joint_position` default, optional `joint_impedance`)
- Unified joint-limit mode selection (`reject` default, optional experimental `ramp`)

## 2. Project layout

- src/teleop/input/: PICO receiver and input mapping
- src/teleop/core/: common data structures
- src/teleop/transform/: calibration and coordinate transform
- src/teleop/safety/: safety state machine and safety gate
- src/teleop/control/: target buffer and command scheduler
- src/teleop/robot/: robot SDK, kinematics, startup, command adapter
- src/teleop/ui/: diagnostic UI
- src/teleop/logging/: optional async session logging
- configs/: YAML configuration files
- scripts/: manual test and run entrypoints
- docs/: detailed design notes

## 3. Installation

Use a clean Python 3.11 environment.

### Option A: conda

```bash
conda create -n teleop python=3.11 -y
conda activate teleop
pip install -U pip
pip install -r requirements.txt
```

### Option B: venv

Windows PowerShell:

```bash
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -U pip
pip install -r requirements.txt
```

Linux/macOS:

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -U pip
pip install -r requirements.txt
```

Notes:

- environment.yml exists, but it appears to be a base-environment export and is not recommended as the primary project setup path.
- Recommended setup is Python 3.11 + requirements.txt.
- Robot SDK dynamic libraries must be available under python_sdk/ or otherwise discoverable in runtime path.
- Kinematics config files should be available under assets/kinematics/.

## 4. Basic validation

```bash
pytest -q
python -m compileall src scripts tests
```

No-hardware diagnostics:

```bash
python scripts/check_coordinate_mapping.py
python scripts/check_stage6_pipeline_dry.py
```

These checks do not require PICO or robot hardware. They validate parsing, transforms, safety logic, scheduler behavior, and integration assumptions.

## 5. PICO setup and diagnostics

Recommended prerequisites:

- PICO 4 Ultra
- XRoboToolkit PICO APK (`assets/third_party/XRoboToolkit-PICO-1.1.1.apk`) installed on PICO
- PICO and PC on the same network
- Windows firewall allows Python networking

Frequency and stream quality:

```bash
python scripts/test_pico_frequency.py
```

This reports:

- receive FPS
- timestamp monotonicity issues
- left/right valid pose ratio
- zero-pose invalid counts

Live input inspection:

```bash
python scripts/inspect_pico_live.py
```

This prints:

- head/left/right pose
- controller quaternions
- trigger/grip/joystick/buttons

Button mapping in current middleware:

- grip: enable / deadman
- trigger: gripper input (movement enable is not trigger)
- axisClick: calibration request
- primaryButton / secondaryButton: auxiliary/reserved
- menuButton: unused for MVP calibration

## 6. Robot-side diagnostics

### Read robot feedback only

```bash
python scripts/read_robot_feedback_once.py --robot-ip 192.168.1.190
```

Behavior:

- connects to robot SDK
- clears errors where adapter supports it
- reads joint feedback
- runs FK to xyzabc
- does not send motion commands

### Move to ready pose

Dry-run:

```bash
python scripts/move_robot_to_ready_pose.py --robot-ip 192.168.1.190 --dry-run
```

Real startup movement:

```bash
python scripts/move_robot_to_ready_pose.py --robot-ip 192.168.1.190
```

Behavior:

- low speed / low acceleration startup path
- moves to configured ready pose
- requires explicit YES confirmation
- this is not teleoperation

Current script defaults:

- left: [90, -60, -90, -90, 0, 0, 0]
- right: [90, 60, -90, -90, 0, 0, 0]

Optional command-path preview utility:

```bash
python scripts/robot_command_dry_run.py --robot-ip 192.168.1.190 --dry-run --side both --delta-mm 2
```

## 7. Full teleoperation app

### Latest recommended command

**Windows:**
```bash
python scripts/run_full_teleop.py --robot-ip 192.168.1.190 --move-to-ready --enable-send --confirm --ui --side both --max-runtime-s 300 --enable-orientation --joint-limit-mode ramp --enable-win-high-res-timer --win-high-res-timer-ms 1 --disable-orientation-filter --control-mode joint_impedance --logging-mode full --pico-resample-mode predictive --ik-reference-mode last_sent --joint-ramp-profile balanced_100hz --safety-target-limit-mode clamp --safety-reacquire-mode position_offset --safety-reacquire-after-ms 1000 --safety-reacquire-error-mm 100
```

**Linux (Ubuntu 20.04/22.04):**
```bash
python scripts/run_full_teleop.py --robot-ip 192.168.1.190 --move-to-ready --enable-send --confirm --ui --side both --max-runtime-s 300 --enable-orientation --joint-limit-mode ramp --disable-orientation-filter --control-mode joint_impedance --logging-mode full --pico-resample-mode predictive --ik-reference-mode last_sent --joint-ramp-profile balanced_100hz --safety-target-limit-mode clamp --safety-reacquire-mode position_offset --safety-reacquire-after-ms 1000 --safety-reacquire-error-mm 100
```

Safety behavior:
- real send is disabled by default; `--enable-send` and `--confirm` are required
- operator must type `YES` to proceed
- `joint_impedance` real send requires `--move-to-ready`

### Dry-run / no-robot modes

```bash
# PICO-only (no robot SDK required)
python scripts/run_full_teleop.py --no-robot --dry-run --ui

# PICO + robot feedback, no command sending
python scripts/run_full_teleop.py --robot-ip 192.168.1.190 --dry-run --ui
```

### Single-arm selection

```bash
python scripts/run_full_teleop.py --robot-ip 192.168.1.190 --move-to-ready --enable-send --confirm --ui --side left
python scripts/run_full_teleop.py --robot-ip 192.168.1.190 --move-to-ready --enable-send --confirm --ui --side right
```

### Key options

| Option | Values | Default | Notes |
|--------|--------|---------|-------|
| `--teleop-mode` | `position_only`, `position_orientation` | `position_only` | `--enable-orientation` is a shorthand for `position_orientation` |
| `--control-mode` | `joint_position`, `joint_impedance` | `joint_position` | `joint_impedance` real send needs `--move-to-ready` |
| `--joint-limit-mode` | `reject`, `ramp` | `reject` | `ramp` clips joint deltas by `min(max_step, max_velocity * dt)` |
| `--pico-resample-mode` | `latest`, `predictive` | `latest` | predictive: causal position extrapolation only |
| `--rate-hz` | `50`, `100` | — | 50 Hz fallback if 100 Hz is unstable |
| `--disable-orientation-filter` | flag | off | disables SO(3) Slerp low-pass filter in `position_orientation` mode |
| `--orientation-filter-tau` | float | `0.02` | higher = smoother with more lag |
| `--enable-win-high-res-timer` | flag | off | Windows-only; improves scheduler precision |

Use `python scripts/run_full_teleop.py --help` to see all options.

## 8. How to operate teleoperation

Suggested operator flow:

1. Start PICO APK.
2. Start full teleop app.
3. Wait until PICO and robot feedback are connected.
4. Keep hands in a comfortable neutral pose.
5. Press axisClick to calibrate.
6. Keep hands still briefly after calibration.
7. Press and hold grip to enable motion.
8. Move slowly in a small workspace.
9. Release grip to pause.
10. Press Ctrl+C to stop if needed.

Important behavior:

- axisClick captures controller reference pose and robot FK feedback anchor.
- default mode is position_only: position follows relative controller displacement while end-effector orientation stays frozen at calibration orientation.
- trigger is not the motion-enable signal.
- Orientation tracking (`position_orientation`): `absolute_matrix` is the default algorithm. Orientation filter (SO(3) Slerp, `tau_s=0.02` default) is applied to controller quaternion before rotation-matrix conversion. Use `--disable-orientation-filter` to bypass. `--enable-orientation` is shorthand for `--teleop-mode position_orientation`.

## 9. Diagnostic UI

UI is optional and diagnostic-only.

- refresh around 20 Hz
- visualizes target/feedback/status
- does not send robot commands

Commands:

```bash
python scripts/run_teleop_ui_mock.py
python scripts/run_teleop_ui_from_log.py --input <jsonl>
```

## 10. Logging and replay

- logging is disabled by default
- async logging is optional
- logging should not block control loop

Dry-run logging:

```bash
python scripts/logging_dry_run.py
```

Analyze/replay:

```bash
python scripts/analyze_teleop_log.py --input <jsonl>
python scripts/replay_teleop_log.py --input <jsonl>
```

## 11. Safety notes

WARNING: real robot motion can cause injury or equipment damage. Use strict on-site safety process.

- Always test PICO-only first.
- Always test robot read-only feedback before motion.
- Confirm left/right arm mapping before motion.
- Use low speed and ready pose first.
- Start with one arm and tiny movement.
- Keep emergency stop reachable.
- Do not stand inside robot workspace.
- Do not use --enable-send unless ready for real robot motion.
- Do not use --control-mode joint_impedance for real send unless --move-to-ready is enabled.
- Recalibrate after PICO recenter, restart, or operator posture/orientation change.
- Release grip immediately if motion is unexpected.
- Use Ctrl+C and physical emergency stop when needed.
- UI is diagnostic-only.
- Logging is optional and disabled by default.
- If IK fails or motion stops, reduce range and return to ready pose.

## 12. Troubleshooting

PICO does not connect:

- check same Wi-Fi/LAN
- check PC IP
- check firewall rules
- verify Python listener ports
- use ipconfig/netstat for quick verification

Left/right mapping seems swapped:

- check configs/robot_sdk.yaml
- verify left_arm/right_arm mapping to SDK A/B

Robot moves in wrong direction:

- stop immediately and release grip
- re-check coordinate mapping assumptions
- recalibrate
- verify operator facing direction and PICO recenter state

UI appears empty:

- target/feedback points may be far from origin
- UI camera/grid defaults are in configs/ui.yaml
- run mock UI first to validate rendering path

IK failure:

- reduce workspace and rotation range
- return to ready pose
- check IK ZSP settings in configs/robot_command.yaml if needed

## 13. Useful documents

- docs/pico_interface_notes.md: PICO protocol and observed field behavior
- docs/teleop_mapping_notes.md: semantic mapping policy
- docs/calibration_transform_notes.md: calibration and coordinate transform details
- docs/safety_state_machine_notes.md: safety state logic
- docs/command_loop_notes.md: scheduler and target limiter notes
- docs/robot_sdk_readonly_notes.md: robot feedback adapter notes
- docs/robot_startup_ready_pose_notes.md: startup ready-pose safety flow
- docs/robot_command_adapter_notes.md: IK + command adapter behavior
- docs/ik_zsp_mode_notes.md: optional IK ZSP mode and fallback notes
- docs/logging_replay_notes.md: logging/replay usage
- docs/ui_visualization_notes.md: UI behavior and configuration
- docs/full_teleop_app_notes.md: full app runtime notes
- docs/orientation_tracking_notes.md: experimental orientation tracking mode