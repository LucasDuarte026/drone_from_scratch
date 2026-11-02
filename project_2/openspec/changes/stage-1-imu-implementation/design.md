# Technical Design: Stage 1 IMU Attitude Determination

## Context & Constraints
- **Target MCU:** Arduino Uno (ATmega328P, 16 MHz, 2 KB SRAM, 32 KB Flash, 8-bit AVR without hardware FPU).
- **Inertial Sensor:** InvenSense MPU-6050 / MPU-6000 via I2C (`0x68`, 400 kHz Fast Mode via `TWBR = 12`).
- **Loop Timing Budget:** Strict $4000\,\mu\text{s}$ ($250\text{ Hz}$) cycle period enforced via `micros()`. Trigonometric operations (`sin`, `cos`, `tan`, `asin`, `sqrt`) and `Serial.print` calls must fit comfortably within $4000\,\mu\text{s}$.
- **Reference Specification:** `ref/DRONE_materialdeapoio/tutorial_programacao_drone_especializacao.pdf` (Section 2 & Appendices A.1, A.2, B) and `ref/DRONE_materialdeapoio/example_01.ino`.

## Goals & Non-Goals
- **Goals:**
  - Fix compilation errors and out-of-bounds array indexing present in `example_01.ino`.
  - Implement gyro sensitivity scaling ($65.5\text{ LSB}/(\text{deg/s})$ for $\pm 500\text{ dps}$) and first-order digital filtering ($0.7 / 0.3$).
  - Implement Euler kinematic integration (Eq. 4–7) and accelerometer tilt extraction (Eq. 8–10).
  - Fuse attitude via the $0.9996 / 0.0004$ complementary filter and output angles for verification.
- **Non-Goals:**
  - Stage 2 (PCINT receiver capture and EEPROM calibration) — deferred.
  - Stage 3 (ESC pulse generation and motor testing) — deferred.
  - Stage 4 (PID stabilization loop and battery voltage compensation) — deferred.

## Technical Decisions
1. **Array Indexing Remediation:**
   - `example_01.ino` declares `int acc_axis[3], gyro_axis[3];` and `double gyro_axis_cal[3];` (valid indices `0..2`), yet accesses indices `1..3` in `setup()` and `gyro_signalen()`, causing memory corruption of adjacent globals. We will standardize on clean `0..2` indexing (`0 = X/Roll`, `1 = Y/Pitch`, `2 = Z/Yaw`) while preserving the canonical global variable names from Appendix B (`acc_x`, `acc_y`, `acc_z`, `gyro_roll_input`, `gyro_pitch_input`, `gyro_yaw_input`, `angle_roll`, `angle_pitch`, etc.).
2. **Radian-to-Degree Conversion in Trigonometric Kinematics:**
   - AVR `math.h` functions (`sin`, `cos`, `tan`, `asin`) operate in radians, whereas the guide's Euler angles ($\phi, \theta, \psi$) and angular rates ($p, q, r$) are tracked in degrees and degrees/s. Explicit `DEG_TO_RAD` ($\pi / 180$) and `RAD_TO_DEG` ($180 / \pi$) conversions will be applied in Equations (5)–(10).
3. **Domain Clamping on `asin()`:**
   - Dynamic linear acceleration can briefly cause $|Acc_x| > Acc_{Total}$ or numerical rounding near $\pm 1.0$. Guarding or clamping the ratio to $[-1.0, 1.0]$ prevents `NaN` propagation into the complementary filter.

4. **Multi-Environment PlatformIO Architecture (`platformio.ini`) & `code/` Encapsulation:**
   - Embedded firmware code is cleanly encapsulated under `code/` (`code/src/` and `code/include/`), with `platformio.ini` configured via `src_dir = code/src` and `include_dir = code/include` (and root symlinks).
   - Each development stage (`[env:stage1_imu]`, `[env:stage2_rx]`, `[env:stage3_motors]`, `[env:stage4_flight]`) is configured as a dedicated PlatformIO build environment with its own entrypoint under `code/src/` while sharing common drivers and mathematical modules in `code/include/` and `code/src/`. A `[env:native]` environment enables fast host-side TDD unit testing of all attitude and control math.
5. **Configurable IMU Axis Polarity (`code/include/drone_config.h`):**
   - Axis sign constants (`IMU_SIGN_X`, `IMU_SIGN_Y`, `IMU_SIGN_Z`, defaulting to `+1`) are applied immediately after sensor reading to accommodate physical mounting inversions without altering kinematic equations.
6. **Decimated 25 Hz Telemetry & Overrun Watchdog:**
   - Serial remains at `57600` bps (`example_01.ino` standard), but Serial Plotter output is decimated to every 10th loop iteration ($25\text{ Hz}$) to prevent 64-byte UART TX buffer stalls. If `micros() - loop_timer > 4050`, pin `D12` is driven `HIGH` to flag a real-time overrun.
7. **Source-Calibrated Accelerometer Bias & Phase-Synchronized 10-Sample Averaging (`code/src/main.cpp`):**
   - During `calibrate_imu()`, the firmware computes both stationary gyroscope biases (`gyro_axis_cal[0..2]`) and accelerometer biases (`acc_bias[0..2]`) so that the resting pose directly yields `acc_x = 0`, `acc_y = 0`, `acc_z = +4096 LSB` ($+1.00g$) and zero tilt angles (`Roll = 0.00°`, `Pitch = 0.00°`).
   - Over each 10-sample decimation window ($250\text{ Hz} \to 25\text{ Hz}$), calibrated `acc_x, acc_y, acc_z` are accumulated (`acc_sum_x/y/z`); at Tick 5, the 10-sample boxcar average is latched into `snap_ax_g, snap_ay_g, snap_az_g` at the exact same instant `angle_roll, angle_pitch, angle_yaw` are transmitted, eliminating the 20 ms phase skew between attitude and acceleration while keeping per-tick UART formatting within the $4000\,\mu\text{s}$ loop budget.

## Resolved Questions (`/grill-me` Decisions)
1. **Build & Tooling Ecosystem:** Resolved — Full PlatformIO project (`platformio.ini` + `code/src/` + `code/include/` + `test/`) managed via CLI.
2. **Code Organization Across Stages:** Resolved — Multi-environment `platformio.ini` (`stage1_imu`, `stage2_rx`, `stage3_motors`, `stage4_flight`) preserving each stage's executable while sharing headers under `code/include/`.
3. **Appendix B Fidelity & Math Testability:** Resolved — 100% preservation of Appendix B global variable names and exact PDF equations (Eq. 1–12), remediating `[0..2]` array indexing bugs and isolating pure math functions in testable headers under `code/include/`.
4. **Hardware Mounting & Axis Polarity:** Resolved — Bench default orientation (`+1, +1, +1`) with polarity constants in `code/include/drone_config.h`.
5. **Serial Plotter vs. 4000 us Loop Budget:** Resolved — `57600` bps with 1-in-10 cycle decimation ($25\text{ Hz}$) and `D12` overrun warning at $> 4050\,\mu\text{s}$.
