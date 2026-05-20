# Robot SDK Read-Only Notes (Stage 6A)

## Stage 6A Purpose

Stage 6A adds a read-only robot SDK adapter for live robot feedback retrieval.

This stage does not move the robot. It only connects, validates feedback stream, and converts joint feedback to Cartesian feedback through FK.

## Connection Sequence

The read-only adapter follows this sequence:

1. connect to robot
2. clear arm errors
3. optionally disable SDK/local logs and kinematics logs
4. initialize kinematics
5. subscribe and verify frame updates
6. read q and convert to xyzabc via FK

## Feedback Conversion

Per arm:

- read joint feedback `q_deg` from SDK subscription
- run FK using kinematics SDK
- convert FK matrix to xyzabc

Conversion path:

`q_deg -> fk(mat4x4) -> xyzabc_mm_deg`

## Units

- joint angle: degree
- Cartesian position xyz: millimeter
- Cartesian orientation abc: degree
- time: second

## Arm Mapping

Project semantics use `left` and `right`.

SDK labels use `A` and `B`.

Mapping is configurable:

- default left -> `A`
- default right -> `B`

If hardware mapping is opposite, only configuration should be changed.

## Calibration Support

With Stage 6A, calibration can bind Pico controller pose to real robot FK feedback (`DualArmRobotFeedback`).

## Out of Scope (Stage 6A)

- IK command solving
- command sending to robot
- impedance mode
- fixed-rate SDK command loop
- pause/stop motion behavior
- UI
