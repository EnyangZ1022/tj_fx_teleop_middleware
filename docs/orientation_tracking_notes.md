# Orientation Tracking Notes

## Scope

Stage 11 adds an explicit optional teleoperation mode for orientation tracking while preserving the verified position-only behavior as default.

## Modes

- position_only (default)
  - Tracks xyz position from controller relative displacement.
  - Freezes abc orientation at calibration-time robot feedback orientation.
  - This remains the stable default behavior.

- position_orientation (experimental)
  - Tracks xyz position with the same position mapping path.
  - Tracks orientation from controller relative quaternion.
  - Uses rotation-vector mapping and angle limiting.

## Calibration Data

axisClick calibration stores, per arm:

- controller reference position
- controller reference quaternion
- robot reference position
- robot reference orientation abc
- robot reference orientation rotation matrix when converter is available

## Orientation Algorithm

1. Compute relative controller quaternion:
   - world mode: q_delta = q_now * inverse(q_ref)
   - local mode: q_delta = inverse(q_ref) * q_now
2. Convert q_delta to rotation vector.
3. Map controller rotvec to robot rotvec by arm mapping matrix.
4. Apply rotation_scale.
5. Clamp total angle by max_total_angle_deg.
6. Limit per-step change by max_step_angle_deg.
7. Compose robot relative rotation with robot reference orientation.
8. Convert final orientation matrix back to abc with SDK kinematics converter.

No direct Euler subtraction is used.

## Measured Rotvec Mapping

- arm B (right): [z, -y, x]
- arm A (left): [-z, -y, x]

## Defaults

- enabled: false
- relative_mode: world
- rotation_scale: 0.4
- max_total_angle_deg: 25
- max_step_angle_deg: 2

## Safety Behavior

When orientation mode is enabled and orientation transform fails for a side, that side target is rejected with an explicit reason. The system does not silently send guessed orientation.

## CLI

Default (position-only):

```bash
python scripts/run_full_teleop.py --robot-ip 192.168.1.190 --move-to-ready --enable-send --confirm --ui
```

Orientation tracking (experimental):

```bash
python scripts/run_full_teleop.py --robot-ip 192.168.1.190 --move-to-ready --enable-send --confirm --ui --teleop-mode position_orientation
```

Optional shorthand:

```bash
python scripts/run_full_teleop.py --enable-orientation
```

## First Hardware Trial Recommendation

- Start with one arm only.
- Use small hand rotations.
- Keep rotation_scale low.
- Keep max_total_angle_deg low.
