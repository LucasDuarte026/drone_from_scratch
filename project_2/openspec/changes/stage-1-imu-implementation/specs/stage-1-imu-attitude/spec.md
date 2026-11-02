# stage-1-imu-attitude Delta Specification

## ADDED Requirements

### Requirement: Safe Array Indexing and Appendix B Variable Compliance
The system SHALL store and process 3-axis accelerometer and gyroscope measurements without out-of-bounds array accesses while preserving the global variable naming contract defined in Appendix B of the reference guide.

#### Scenario: Bias accumulation and subtraction across 3 axes
- **GIVEN** 3-element arrays `acc_axis[3]`, `gyro_axis[3]`, and `gyro_axis_cal[3]`
- **WHEN** `setup()` accumulates 2000 calibration samples and `gyro_signalen()` applies bias compensation
- **THEN** both routines SHALL access exclusively indices `0`, `1`, and `2` (`X`, `Y`, `Z`) with zero memory corruption of adjacent global variables.

### Requirement: Configurable IMU Axis Polarity
The system SHALL provide compile-time axis polarity multipliers (`IMU_SIGN_X`, `IMU_SIGN_Y`, `IMU_SIGN_Z`, defaulting to `+1`) in `code/include/drone_config.h` to align physical sensor mounting with the aircraft body coordinate frame ($+x$ nose, $+y$ right wing, $+z$ downward).

#### Scenario: Inverting an axis due to physical IMU mounting
- **GIVEN** `IMU_SIGN_X`, `IMU_SIGN_Y`, and `IMU_SIGN_Z` are configured in `code/include/drone_config.h`
- **WHEN** raw accelerometer and bias-compensated gyroscope measurements are processed
- **THEN** each axis measurement SHALL be multiplied by its corresponding sign constant before filtering and attitude computation.

### Requirement: Decimated Serial Plotter Telemetry and 4050 us Overrun Watchdog
The system SHALL operate UART Serial at `57600` bps, decimate Serial Plotter attitude output to once every 10 control cycles ($25\text{ Hz}$), and assert the warning LED (`D12`) if any cycle exceeds $4050\,\mu\text{s}$.

#### Scenario: Real-time attitude visualization without UART buffer stall
- **GIVEN** the 250 Hz loop computes updated `angle_roll` ($\phi$), `angle_pitch` ($\theta$), and `angle_yaw` ($\psi$)
- **WHEN** 10 control cycles have elapsed ($40\text{ ms}$ / $25\text{ Hz}$)
- **THEN** the system SHALL output the Euler angles to Serial for the Arduino Serial Plotter while keeping every cycle execution time below $4000\,\mu\text{s}$.

#### Scenario: Loop execution exceeds 4050 us budget
- **GIVEN** the main loop completes attitude and telemetry operations
- **WHEN** `micros() - loop_timer > 4050`
- **THEN** the system SHALL drive pin `D12` `HIGH` to signal a real-time deadline violation.
