# Orientation Tracking Notes

## Scope

Orientation tracking is optional and does not change the default position-only behavior.

- position_only remains the default and production-safe baseline.
- position_orientation is explicit opt-in.
- all orientation computations are performed on SO(3) rotation matrices or quaternions.

## Modes

- position_only (default)
  - tracks xyz using calibration-relative displacement.
  - keeps robot abc frozen at calibration orientation.

- position_orientation (experimental)
  - tracks xyz with the same position path.
  - tracks orientation using selected algorithm.

## Orientation Algorithms

### 1) absolute_matrix (default)

This mode computes arm absolute orientation from controller quaternion using fixed frame maps, then composes a calibration offset for continuity.

Fixed matrices:

- T_L1_L (left arm)
- T_L1_R (right arm)
- T_w_to_pico
- T_pico_to_userworld

Runtime absolute orientation:

- R_abs_now = T_L1_side @ T_w_to_pico @ R_pico @ T_pico_to_userworld

Calibration continuity offset:

- R_offset = R_robot_anchor @ R_abs_anchor.T

Target orientation:

- R_target = R_offset @ R_abs_now

Safety limits are then applied on SO(3):

- clamp relative angle from anchor by max_total_angle_deg
- limit inter-frame step by max_step_angle_deg

Finally, SDK converter maps R_target -> abc.

### 2) relative_rotvec (fallback)

Legacy behavior preserved as fallback.

- q_delta computed from q_ref and q_now (world/local relative_mode)
- q_delta -> rotvec
- rotvec mapped by per-arm matrix
- scaled by rotation_scale
- clamped by max_total_angle_deg and max_step_angle_deg
- composed with robot anchor orientation
- SDK converter maps target matrix -> abc

## Calibration Data Stored Per Arm

- controller reference position
- controller reference quaternion
- robot reference position
- robot reference orientation abc
- robot reference orientation matrix (if converter available)
- controller absolute orientation matrix at calibration
- orientation offset matrix (if both matrices are available)

If offset data is unavailable and absolute_matrix is enabled, runtime falls back to R_target = R_abs_now for that arm.

## Defaults

- enabled: false
- orientation_algorithm: absolute_matrix
- use_calibration_offset: true
- relative_mode: world
- rotation_scale: 0.4
- max_total_angle_deg: 25
- max_step_angle_deg: 2

## CLI

Default position-only mode:

```bash
python scripts/run_full_teleop.py --robot-ip 192.168.1.190 --dry-run --ui
```

Enable orientation mode with default algorithm:

```bash
python scripts/run_full_teleop.py --robot-ip 192.168.1.190 --dry-run --ui --teleop-mode position_orientation
```

Choose fallback algorithm explicitly:

```bash
python scripts/run_full_teleop.py --robot-ip 192.168.1.190 --dry-run --ui --teleop-mode position_orientation --orientation-algorithm relative_rotvec
```

Shorthand for orientation mode:

```bash
python scripts/run_full_teleop.py --enable-orientation
```

## Safety Behavior

- Orientation mode failure on one side invalidates that side target with explicit reason.
- No direct abc subtraction/addition is used.
- Recalibration resets per-arm orientation tracking state for continuity.
