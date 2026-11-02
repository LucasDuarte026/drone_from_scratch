# Changelog

All notable changes to the **Arduino Quadcopter Flight Controller (`drone`)** project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to **Spec-Driven Development (SDD)**.

## [0.5.0] - 2026-10-28

### Added
- **Zero-Velocity Update (ZUPT) & Noise Deadband (`odometry/drone_model.py`):**
  - Integrated physical stationary motion detection: when world-frame acceleration falls within the deadband ($|a_w| < 0.05g$) for $\ge 3$ consecutive samples (~120 ms), ZUPT exponentially bleeds residual velocity to zero (`vx *= 0.6` until $|v| < 0.01\text{ m/s} \implies 0.00\text{ m/s}$), solving velocity integration runaway from discrete deadband asymmetry upon manual stopping. Configurable via `zupt: bool = True` and CLI `--no-zupt`.
  - Configured $0.05g$ default noise deadband (`accel_deadband_g = 0.05`, `--deadband 0.05`) to reject stationary MEMS thermal drift and quantization noise.
  - Added `get_diagnostics_state()` returning a dictionary of body acceleration, world raw, world bias tare, linear acceleration, velocity, position, clamping state, and ZUPT activation.
- **Dual-Tier Telemetry & Diagnostics Logger (`odometry/telemetry_logger.py`):**
  - Implemented dual-destination logging emitting formatted console text with ANSI color tokens and persistent file output to `odometry/logs/sim_diagnostics.log`.
  - **Standard Mode (Default):** Captures lifecycle events and a 1 Hz system health heartbeat (`[INFO ] [HEARTBEAT] #...`).
  - **Verbose Mode (`-v` / `--verbose`):** Emits complete per-sample forensic diagnostic lines with $\Delta t$, clamping tags (`[CLAMP <0.05g]`), and ZUPT activation tags (`[ZUPT ACTIVE]`).
- **Calibration Warning Modal & Integration Pause (`odometry/sim_3d.py`):**
  - Renders a prominent amber modal overlay (`IMU CALIBRATION IN PROGRESS`, `KEEP DRONE COMPLETELY MOTIONLESS`) with live countdown timer and progress bar during the 3.0s calibration routine.
  - Completely pauses kinematic integration during calibration and auto-tares the post-calibration resting acceleration as world zero reference.
- **3D Velocity HUD Card Row (`odometry/sim_3d.py`):**
  - Expanded the top-left odometry card to display `VEL 3D (m/s): Vx +0.00 Vy +0.00 Vz +0.00` in soft technical blue (`UITheme.ACCENT_BLUE = (96, 165, 250)`) between position and linear acceleration.
- **Phase-Synchronized Latching & Built-in LED Heartbeat (`code/src/main.cpp`):**
  - Calibrated stationary accelerometer biases (`acc_bias[0..2]`) at the source during `calibrate_imu()`, ensuring resting readings equal $(0, 0, +4096\text{ LSB} = +1.00g)$ and eliminating attitude/acceleration offset mismatch.
  - Accumulated 250 Hz accelerometer readings over the 10-tick decimation window and latched the average at **Tick 5** alongside attitude angles, eliminating the 20 ms phase skew between attitude and acceleration.
  - Added a visual heartbeat toggling the onboard LED (Pin 13 / `LED_BUILTIN` / `PIN_LED_STATUS`) every 10 telemetry transmission cycles (`led_toggle_interval = 10`, $1.25\text{ Hz}$ blink) for immediate visual confirmation of firmware execution.
  - Emitted full 4-decimal acceleration telemetry over serial (`Serial.print(snap_ax_g, 4)`), preserving 100% of the raw 16-bit MPU-6050 resolution.

### Changed
- **Unconstrained Floating-Point Kinematics (`odometry/drone_model.py`, `odometry/sim_3d.py`):**
  - Removed the temporary 2-decimal truncation filter (`accel_decimals = None` by default), restoring continuous float64 precision across all kinematic calculations while preserving `--accel-decimals` for comparative benchmarking.
- **Sample-Synchronous Integration (`odometry/sim_3d.py`):**
  - Restricted `pos_estimator.update()` to execute strictly once per newly arrived serial packet at 25 Hz (`id(latest) != self._last_ingested_id`), eliminating duplicate-sample integration artifacts at 60 FPS.

## [0.4.0] - 2026-10-27

### Added
- **Anti-AI Slop Graphite UI Design System (`UITheme` in `odometry/sim_3d.py`):**
  - Synthesized design principles from the Maestro World Info Vault (`Impeccable` Anti-AI Slop rules), Agency UI/UX Personas (`design-ui-designer`, `design-ux-architect`), and `context7` (`Pygame` / `PyOpenGL`) to replace saturated neon borders and ASCII pseudo-icons with a professional 2-Bar Graphite Instrument Layout (`TOP_BAR_HEIGHT = 52px`, compact bottom dock `DOCK_PILL_HEIGHT = 24px`, `DOCK_PILL_WIDTH = 74px`, and `460px` 5-column telemetry drawer with `PANEL_ROW_LATEST` highlight).
  - Implemented High-DPI native typography hierarchy (`DejaVu Sans,Ubuntu` regular `14px`–`17px` for UI controls/labels and `DejaVu Sans Mono,Ubuntu Mono` `22px` bold tabular readouts for `ROLL`, `PITCH`, `YAW`).
  - Added hardware **4x MSAA** (`GL_MULTISAMPLEBUFFERS=1`, `GL_MULTISAMPLESAMPLES=4` with safe fallback), `GL_LINE_SMOOTH`, and vector 2D anti-aliased status indicators (`pygame.gfxdraw.aacircle` + `filled_circle`).
  - Added `format_sample_time()` to convert 13-digit Unix epoch timestamps and simulation milliseconds into relative session seconds (`12.4s`) in the right-hand telemetry drawer.
  - Standardized all simulator HUD labels, table columns (`SAMPLE`, `TIME`, `ROLL`, `PITCH`, `YAW`), layer dock pills (`Body`, `Arms`, `Motors`, `Axes`, `Grid`), and serial worker statuses in **English by default**, enforced by `test_english_only_ui_labels` in `odometry/tests/test_drone_model.py`.
  - Added TDD unit tests (`test_ui_theme_tokens`, `test_english_only_ui_labels`) in `odometry/tests/test_drone_model.py`.
- **RViz IMU Coordinate Triad, Layer Visibility & Direct Serial Ingestion (`odometry/`, `sim3d`):**
  - Added `IMUAxesSpec` and `ComponentVisibility` in `odometry/drone_model.py` to render an RViz-style 3-axis coordinate triad (`+X` Red, `+Y` Green, `+Z` Blue) at the IMU origin and toggle individual layers (`1`–`5`).
  - Added `SerialPlotPoller` and `parse_serial_plot_line()` in `odometry/telemetry_reader.py` for direct USB serial telemetry ingestion alongside the `./sim3d` CLI launcher (`--serialplot` / `--mock`).
- **6-Value Firmware Telemetry Stream (`code/src/main.cpp`, `code/include/drone_config.h`):**
  - Added `ACCEL_SENSITIVITY_LSB_G = 4096.0f` and expanded the 25 Hz serial stream to emit `Roll Pitch Yaw AccX AccY AccZ` (in degrees and $g$).

### Changed
- **Single Generic I2C Bus Scanner (`code/src/i2c_finder.cpp`):**
  - Generalized `i2c_finder.cpp` into a 100% device-agnostic I2C scanner (`0x01`..`0x7E` at 100 kHz with `A4`/`A5` electrical pull-up verification) and unified `platformio.ini` to `[env:uno]` and `[env:i2c_finder]`.
- **Canonical IMU Math Deduplication (`odometry/tester.c` & `odometry/Makefile`):**
  - Updated `odometry/tester.c` to include `code/include/imu_attitude.h` directly via `-I../code/include`, eliminating duplicate C filter implementations and unused `test_hook_*` stubs.

### Removed
- **Repository Cleanup (Graphify Dead-Node Audit):**
  - Removed redundant `code/src/alt_pin_scanner.cpp`, isolated 2D matplotlib script `odometry/plot_serial.py`, `compile_commands.json`, and empty PlatformIO boilerplate directories (`code/include/README`, `lib/`, `test/`).

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
