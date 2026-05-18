# tj_fx_teleop_middleware

A lightweight MVP middleware for Pico-based teleoperation of a fixed-base dual-arm robot.

## Overview

This repository focuses on turning Pico motion/controller input into clean internal data structures that can be consumed by later teleoperation stages.

## Architecture (MVP)

Pico input -> TeleopFrame -> coordinate transform -> filtering/safety -> robot SDK adapter

Stage 1 covers Pico input refactor and parser testability.

Stage 2 adds semantic mapping from PicoRawFrame to TeleopFrame for higher-level teleoperation modules.

## Pico Interface Notes

See `docs/pico_interface_notes.md` for currently confirmed protocol and coordinate assumptions.

## Teleop Mapping Notes

See `docs/teleop_mapping_notes.md` for Stage 2 semantic mapping and timing assumptions.
