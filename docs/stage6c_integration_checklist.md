# Stage 6C Integration Checklist

## A. XR Input Assumptions

- [ ] PICO data stream is received.
- [ ] Controller pose fields are valid.
- [ ] Head pose is not required for control logic.
- [ ] Coordinate convention is documented from measured data.
- [ ] Button mapping is documented and consistent.

## B. Reference-Relative Control

- [ ] Controller reference is captured at activation/deadman rising edge.
- [ ] Robot FK reference is captured at activation/calibration.
- [ ] Target uses reference-relative displacement formula.
- [ ] Release/pause does not integrate old target history.
- [ ] Recalibration resets reference anchors.

## C. Coordinate Mapping

- [ ] Right hand +forward maps to right SDK +X.
- [ ] Right hand +up maps to right SDK +Y.
- [ ] Right hand +right maps to right SDK +Z.
- [ ] Left hand +forward maps to left SDK +X.
- [ ] Left hand +up maps to left SDK -Y.
- [ ] Left hand +right maps to left SDK -Z.

## D. Unit Convention

- [ ] PICO meter to robot millimeter conversion is checked.
- [ ] FK xyz feedback is interpreted as millimeter.
- [ ] IK xyz input is millimeter.
- [ ] abc orientation uses degree.
- [ ] q joint values use degree.
- [ ] limits use mm/s or deg/s as appropriate.

## E. Startup and Ready Pose

- [ ] low-speed startup ratio is 20 / 20.
- [ ] position mode is set before ready pose move.
- [ ] ready pose is not all-zero joint vector.
- [ ] ready pose and IK reference are separate concepts.
- [ ] left ready pose: [90, -60, -90, -90, 0, 0, 0].
- [ ] right ready pose: [90, 60, -90, -90, 0, 0, 0].

## F. Safety

- [ ] no calibration means no motion.
- [ ] no enable/deadman means no motion.
- [ ] invalid pose means no motion.
- [ ] stale frame means no motion.
- [ ] target jump means no motion or clipping before SDK.
- [ ] IK failure means no motion.
- [ ] large q step means no motion.
- [ ] command adapter default is dry-run true.
- [ ] command adapter default command_enabled false.
- [ ] safety gate is not bypassed.

## G. SDK Command Path

- [ ] Stage 6B-pre ready pose passes before command adapter test.
- [ ] command adapter consumes DualArmCommandTarget only.
- [ ] command adapter does not read Pico directly.
- [ ] fixed IK reference is used as configured.
- [ ] previous IK output is not reused as dynamic IK reference.
