# Changelog

All notable changes to the **Arduino Quadcopter Flight Controller (`drone`)** project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to **Spec-Driven Development (SDD)**.

## [0.3.0] - 2026-10-22

### Added
- **Encapsulated Embedded Firmware Organization (`code/`):**
  - Reorganized embedded firmware structure cleanly under `code/` with `code/src/` (firmware logic) and `code/include/` (hardware configurations and mathematical transforms).
  - Configured `platformio.ini` with explicit `src_dir = code/src` and `include_dir = code/include`, complemented by root symlinks for universal CLI and IDE compatibility.
- **Stage 1 IMU Attitude Determination Firmware (`code/src/main.cpp`):**
  - Full bare-metal implementation of 400 kHz I2C MPU-6050/6000 driver with 0x68/0x69 autodetection and register verification (`0x6B`, `0x1B`, `0x1C`, `0x1A`).
  - 2000-sample stationary gyroscope bias calibration with LED `D12` toggle indicator.
  - Remediated out-of-bounds array indexing bugs from legacy prototype (`example_01.ino`), guaranteeing safe `0..2` array bounds.
  - Implemented body rate scaling ($65.5\text{ LSB/deg/s}$) and first-order IIR low-pass filter ($0.7 / 0.3$, Eq. 1–3).
  - Implemented non-linear Euler kinematic rate differential equations and integration over $\Delta t = 0.004\text{ s}$ (Eq. 4–7).
  - Implemented accelerometer gravity vector projection with domain clamping on `asin()` (Eq. 8–10).
  - Implemented complementary sensor fusion ($0.9996 / 0.0004$, Eq. 11–12) with initial sample attitude lock.
  - Enforced strict $4000\,\mu\text{s}$ (250 Hz) execution pacing with a $4050\,\mu\text{s}$ `D12` watchdog alert.
  - Streamed real-time decimated ($25\text{ Hz}$) serial attitude telemetry at 57600 baud (`Roll Pitch Yaw`), directly consumable by the Arduino Serial Plotter and the `odometry/` 3D visualizer.

## [0.2.0] - 2026-10-19

### Added
- **SDD & Dual-Role TDD Governance Directive:** Enforced mandatory OpenSpec specification matrices before any code authoring, paired with strict dual-role TDD (testing role writes failing Pytest suites first, builder role implements minimal code to pass) inside an isolated `.venv` environment across the entire project.
- **Odometry & Real-Time 3D Simulation Subsystem (`odometry/`):**
  - **`odometry/tester.c` & `odometry/Makefile`:** C driver testbed supporting UART serial communication (`/dev/ttyACM*`, `/dev/ttyUSB*` at 57600 baud) and standalone synthetic mock flight kinematics. Computes IIR rate filtering, Euler kinematics, and complementary fusion with extensible `// TODO` test hooks for live inspection.
  - **`odometry/telemetry_reader.py`:** Fast tail reader maintaining an exact 10-position FIFO circular buffer from `telemetry.log` and tracking live connection status (1.0s timeout).
  - **`odometry/sim_3d.py`:** Pygame + PyOpenGL 3D visualizer plotting a fictional quadcopter (central cuboid, 4 cylindrical arms, 4 vertical cylindrical motors) with free orbital mouse camera, Home button (zero isometric position), Isometric/Perspective toggle, live 10-position stream chat overlay, and top-left Online/Offline status indicator.
  - **`odometry/test_telemetry_reader.py` & `odometry/test_drone_model.py`:** Complete automated Pytest suite (9 tests, 100% green).
  - **OpenSpec Change (`openspec/changes/odometry-3d-simulation/`):** Full SDD matrix (`proposal.md`, `spec.md`, `design.md`, `tasks.md`).

## [0.1.0] - 2026-10-18


### Added
- Initialized OpenSpec (`openspec/config.yaml`, `openspec/specs/`, and `openspec/changes/stage-1-imu-implementation/`) for Spec-Driven Development.
- Created canonical Stage 1 (`stage-1-imu-attitude`) specification and open Stage 2 (`stage-2-rc-receiver`) and Stage 3/4 (`stage-3-motor-control`) specifications.
- Created Maestro-compliant human and AI agent SDD documentation under `docs/` (`docs/README.md`, `docs/system_purpose.md`, `docs/system_architecture.md`, `docs/CHANGELOG.md`) as well as root `PROJECT.md` and `README.md`.
- Conducted technical audit of `ref/DRONE_materialdeapoio/example_01.ino` against `tutorial_programacao_drone_especializacao.pdf`, documenting out-of-bounds array indexing, compilation errors, and missing kinematic equations.
- Initialized local Graphify knowledge graph (`graphify-out/`) and `VERSION` file (`0.1.0`).
- Locked in Stage 1 architectural decisions via `/grill-me`: Multi-environment PlatformIO (`platformio.ini`), 100% Appendix B global variable and equation fidelity (Eq. 1–12) with testable headers, configurable IMU axis polarity (`IMU_SIGN_X/Y/Z`), and 57600 bps Serial Plotter telemetry decimated to 25 Hz with a $4050\,\mu\text{s}$ `D12` overrun watchdog.
