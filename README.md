# tj_fx_teleop_middleware

A lightweight MVP middleware for Pico-based teleoperation of a fixed-base dual-arm robot.

## Overview

This repository focuses on turning Pico motion/controller input into clean internal data structures that can be consumed by later teleoperation stages.

## Architecture (MVP)

Pico input -> TeleopFrame -> coordinate transform -> filtering/safety -> robot SDK adapter

Prompt 1 only covers the Pico input refactor and parser testability.

## Pico Interface Notes

See `docs/pico_interface_notes.md` for currently confirmed protocol and coordinate assumptions.
