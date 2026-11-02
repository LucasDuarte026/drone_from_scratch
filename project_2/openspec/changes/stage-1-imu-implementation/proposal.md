# Stage 1: IMU Attitude Determination & Prototype Remediation

## Why
The reference prototype (`ref/DRONE_materialdeapoio/example_01.ino`) provides a skeleton for reading raw MPU-6050/MPU-6000 registers over 400 kHz I2C and running a 250 Hz ($4000\,\mu\text{s}$) control loop, but it is incomplete (`//TODO: compute the angles`) and contains critical compilation and memory-safety bugs (undeclared `data` variable, missing semicolon on `intro()`, and 1-based array indexing `gyro_axis[1..3]` / `gyro_axis_cal[1..3]` on 3-element 0-indexed C arrays). We need a clean, Spec-Driven Stage 1 implementation that fixes these defects and implements the full mathematical pipeline from Section 2 of the technical guide (`tutorial_programacao_drone_especializacao.pdf`).

## What Changes
- **ADDED**: Complete Stage 1 Arduino sketch/firmware implementing:
  - Safe 0-indexed (or properly sized) array handling for `acc_axis`, `gyro_axis`, and `gyro_axis_cal`.
  - First-order IIR digital low-pass filter on angular rates ($p^{k+1} = 0.7\,p^k + 0.3\,p_{new}$).
  - Non-linear Euler kinematic rate transformation (Eq. 4–7) and forward Euler integration over $\Delta t = 0.004\text{ s}$.
  - Gravity vector projection from 3-axis accelerometer readings (Eq. 8–10) with domain protection for `asin()`.
  - Complementary filter sensor fusion ($\alpha = 0.9996$, $1 - \alpha = 0.0004$) for roll ($\phi$) and pitch ($\theta$).
  - Serial Plotter / Serial Monitor telemetry output for real-time attitude verification.

## Capabilities
- MODIFIED: `specs/stage-1-imu-attitude/spec.md` (Full normative requirements for Stage 1 IMU attitude estimation, calibration, and telemetry verification).

## Impact
Establishes the foundational Layer 1 (Hardware/I2C) and Layer 2/3 (HAL & Attitude Estimation) firmware for the quadcopter without modifying Stage 2 (RC Receiver) or Stage 3/4 (ESC/PID), which remain open for subsequent phases.
