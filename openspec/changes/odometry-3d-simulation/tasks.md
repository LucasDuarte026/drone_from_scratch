# Implementation Tasks: Odometry 3D Simulation & Tester

## 1. Foundation, Specifications & TDD Test Harness
- [x] 1.1 Author OpenSpec delta specification (`proposal.md`, `spec.md`, `design.md`, `tasks.md`)
- [x] 1.2 Write TDD test suite for telemetry ingestion, 10-position FIFO buffer, and online/offline status (`odometry/tests/test_telemetry_reader.py`)
- [x] 1.3 Verify failing test suite (RED phase)
- [x] 1.4 Implement `odometry/telemetry_reader.py` to satisfy all tests (GREEN phase)
- [x] 1.5 Update project governance documents (`PROJECT.md`, `README.md`, `docs/`) with SDD + TDD directive
- [x] 1.6 Organize test suite into `odometry/tests/` subfolder and create persistent `odometry/logs/` directory

## 2. C Driver Tester & Serial Interface
- [x] 2.1 Implement `odometry/tester.c` with UART serial communication (`57600` baud) and fallback synthetic mock flight
- [x] 2.2 Implement IMU attitude math (IIR filter, Euler kinematics, complementary filter) with user test hook slots (`// TODO`)
- [x] 2.3 Create `odometry/Makefile` for host compilation
- [x] 2.4 Verify compilation and telemetry log generation into `odometry/logs/telemetry.log`

## 3. 3D Model & Kinematic Unit Tests
- [x] 3.1 Author unit test for drone geometric components and attitude quaternion/Euler conversions (`odometry/tests/test_drone_model.py`)
- [x] 3.2 Implement drone 3D geometry module (`odometry/drone_model.py`)

## 4. 3D Simulation & HUD Interface
- [x] 4.1 Implement Pygame + PyOpenGL viewport in `odometry/sim_3d.py`
- [x] 4.2 Implement orbital camera with mouse controls, Home button, and Isometric/Perspective toggle
- [x] 4.3 Implement collapsible floating chat overlay rendering the 10-position buffer stream
- [x] 4.4 Implement top-left Online (green) / Offline (red) indicator
- [x] 4.5 End-to-end integration verification
- [x] 4.6 Refactor `odometry/tester.c` to reuse canonical `code/include/imu_attitude.h` and remove unused hooks
- [x] 4.7 Implement Anti-AI Slop Graphite `UITheme` design system, High-DPI 2-bar instrument layout, 4x MSAA + `pygame.gfxdraw` anti-aliasing, and `format_sample_time()` with TDD tests in `odometry/tests/test_drone_model.py`
- [x] 4.8 Refactor Inertial Position Estimator with configurable `accel_decimals=2`, base kinematics (`method="base_euler"`), sample-synchronous integration in `sim_3d.py`, `VEL 3D (m/s)` HUD telemetry display, and source-calibrated accelerometer biases with phase-synchronized 10-sample decimation in Arduino firmware (`code/src/main.cpp`)
- [x] 4.9 Implement Zero-Velocity Update (ZUPT) in `InertialPositionEstimator` to solve velocity integration runaway upon stopping, with $0.05g$ stationary noise deadband, unconstrained float precision (`accel_decimals = None`), and `get_diagnostics_state()`
- [x] 4.10 Implement dual-tier logger (`odometry/telemetry_logger.py`), calibration warning modal overlay in `sim_3d.py`, 4-decimal UART output, and visual built-in LED heartbeat (`led_toggle_interval = 10`) in `code/src/main.cpp`

