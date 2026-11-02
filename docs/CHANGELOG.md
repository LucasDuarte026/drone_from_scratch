# Changelog

All notable changes to the **Autonomous Quadcopter Embedded Flight Controller (`drone`)** workspace will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to **Spec-Driven Development (SDD)**.

## [0.6.0] - 2026-11-02

### Added
- **Multi-Project Workspace Restructuring (`project_1/` & `project_2/`):**
  - Restructured repository into two distinct, complementary sub-projects:
    - **`project_1/`:** Full physical quadcopter test bench with dual ATmega328P microcontrollers, HC-12 433 MHz wireless RF link, 4x 28BYJ-48 stepper motors with ULN2003 drivers, and Pygame/PyOpenGL 3D visualizer (`envioadron.py`).
    - **`project_2/`:** PlatformIO firmware architecture, deterministic 250 Hz control loop, 2-tick 25 Hz serial telemetry, C driver tester, automated Pytest suite (22 unit tests), and odometry research.
- **Comprehensive Open-Source Documentation Hierarchy:**
  - **Root `README.md`:** Open-source badges, academic course metadata (SAA0356, Turma 201, EESC-USP, Prof. Dr. Glauco Caurin with USP profile and CNPq Lattes links), comparison matrix between Project 1 and Project 2, standard Mermaid system diagram, clickable HTML YouTube video demonstration card, and operational commands table.
  - **`project_1/README.md`:** Comprehensive hardware pinout tables, non-blocking stepper scheduler theory, proportional tilt compensation math ($adj = k \cdot \theta$), full mathematical audit and remediation of `example_01.ino` defects, RF protocol guide, YouTube video card, and physical bench image gallery.
  - **`project_2/README.md`:** Expanded with an in-depth mathematical analysis of inertial dead-reckoning odometry, discrete integration methods (Euler, Trapezoidal, ZUPT), $0.05g$ noise deadband, formal proof of quadratic error growth in double integration ($e_p(t) = \frac{1}{2}b_a t^2$), and theoretical justification for multi-sensor Kalman filtering (EKF/UKF).
- **Knowledge Gaps & Action Report (`docs/KNOWLEDGE_GAPS_REPORT.md`):**
  - Forensic audit documenting missing hardware datasheets, mechanical CAD files, project 2 video URLs, and future sensor options to guide Round 2 development.
- **Isolated OpenSpec Governance & Ephemeral Lifecycle Policy:**
  - Configured root `openspec/` for global release governance and `project_1/openspec/` for physical bench integration.
  - Enforced the invariant that all `openspec/` directories serve internal development and will be purged prior to final public open-source Git distribution.

### Changed
- **Updated `PROJECT.md`:** Linked `project_1/README.md` and updated multi-project topology.

## [0.5.0] - 2026-10-28

### Added
- **Zero-Velocity Update (ZUPT) & Noise Deadband (`project_2/odometry/drone_model.py`):**
  - Integrated physical stationary motion detection: when world-frame acceleration falls within the deadband ($|a_w| < 0.05g$) for $\ge 3$ consecutive samples (~120 ms), ZUPT exponentially bleeds residual velocity to zero (`vx *= 0.6` until $|v| < 0.01\text{ m/s} \implies 0.00\text{ m/s}$), solving velocity integration runaway from discrete deadband asymmetry upon manual stopping.
  - Configured $0.05g$ default noise deadband (`accel_deadband_g = 0.05`, `--deadband 0.05`) to reject stationary MEMS thermal drift and quantization noise.
- **Dual-Tier Telemetry & Diagnostics Logger (`project_2/odometry/telemetry_logger.py`):**
  - Implemented dual-destination logging emitting formatted console text with ANSI color tokens and persistent file output to `odometry/logs/sim_diagnostics.log`.
  - **Standard Mode (Default):** Captures lifecycle events and a 1 Hz system health heartbeat (`[INFO ] [HEARTBEAT] #...`).
  - **Verbose Mode (`-v` / `--verbose`):** Emits complete per-sample forensic diagnostic lines with $\Delta t$, clamping tags, and ZUPT activation tags.
- **Phase-Synchronized Latching & Built-in LED Heartbeat (`project_2/code/src/main.cpp`):**
  - Calibrated stationary accelerometer biases at the source during `calibrate_imu()`.
  - Accumulated 250 Hz accelerometer readings over 10 ticks and latched at **Tick 5** alongside attitude angles.
  - Added visual heartbeat toggling onboard LED (Pin 13) every 10 telemetry transmission cycles ($1.25\text{ Hz}$).

## [0.4.0] - 2026-10-27

### Added
- Anti-AI Slop Graphite UI Design System (`UITheme` in `odometry/sim_3d.py`).
- Hardware 4x MSAA and High-DPI typography.
- Standardized all simulator HUD labels in English.
- 6-Value Firmware Telemetry Stream emitting `Roll Pitch Yaw AccX AccY AccZ`.
