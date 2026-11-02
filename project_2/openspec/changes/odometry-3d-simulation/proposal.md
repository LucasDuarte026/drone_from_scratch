# Odometry 3D Simulation & IMU Driver Tester Proposal

## Why
To ensure the MPU-6050 / MPU-6000 attitude estimation algorithms and hardware communication are correctly dimensioned and validated prior to physical flight, an isolated visual verification and testing subsystem is required. This subsystem enables real-time 3D flight visualization driven strictly by telemetry streaming from a C driver tester communicating with the Arduino (or running dynamic mock simulations) while providing a live inspection chat and connection status indicator.

## What Changes
- **ADDED**: `odometry/tester.c` — C-based IMU driver tester with UART serial communication (`57600` bps), fallback synthetic mock flight simulation, modular test hook slots (`// TODO`), and real-time streaming to `odometry/telemetry.log`.
- **ADDED**: `odometry/telemetry_reader.py` — High-speed file tail reader maintaining an exact 10-position circular FIFO buffer and active connection state tracking (`Online` vs `Offline` based on `< 1.0s` heartbeat).
- **ADDED**: `odometry/test_telemetry_reader.py` — Automated Pytest verification suite certifying parsing robustness, FIFO eviction, and connection timeout behavior.
- **ADDED**: `odometry/telemetry_logger.py` — Dual-tier telemetry logging system supporting Standard 1 Hz heartbeat and Verbose (`-v`) per-sample forensic logging to stdout and `odometry/logs/sim_diagnostics.log`.
- **ADDED**: `odometry/sim_3d.py` — Real-time 3D visualization using Pygame and PyOpenGL:
  - Fictional drone model: central cuboid fuselage + 4 cylindrical diagonal arms + 4 vertical cylindrical motors.
  - Interactive orbital camera (mouse orbit, zoom, pan) with zero drone manipulation (attitude 100% telemetry-driven).
  - Home button returning to default isometric view.
  - Projection toggle button switching between Isometric (Orthographic) and Perspective.
  - Collapsible floating chat overlay streaming consumed telemetry lines from the 10-position buffer.
  - Live status indicator (Green circle = Online, Red circle = Offline) in the top-left corner.
  - 3D Odometry HUD card displaying Position (`POS 3D`), Velocity (`VEL 3D`), and Linear Acceleration (`LIN ACC`).
  - Prominent amber calibration warning modal overlay with live countdown and progress bar.
- **ADDED**: `odometry/drone_model.py` — 3D geometric components, RViz IMU coordinate triad, and `InertialPositionEstimator` with Zero-Velocity Update (ZUPT), $0.05g$ noise deadband, full-precision floating point kinematics, and diagnostics state snapshots.
- **MODIFIED**: `code/src/main.cpp` — Source-calibrated accelerometer biases (`acc_bias`), phase-synchronized 10-sample averaging latched at Tick 5, 4-decimal UART telemetry streaming, and visual built-in LED heartbeat (`led_toggle_interval = 10`).
- **ADDED**: `odometry/Makefile` — Compilation automation for the C tester.

## Capabilities
- ADDED: `specs/odometry-3d-simulation/spec.md` (Full normative requirements and GIVEN/WHEN/THEN test scenarios).

## Impact
Provides Layer 5 real-time telemetry inspection and validation tools without altering Stage 1–4 flight controller core firmware or pin allocations.
