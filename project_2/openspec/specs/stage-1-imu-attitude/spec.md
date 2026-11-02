# stage-1-imu-attitude Specification

## Purpose
Defines the normative hardware abstraction, sensor calibration, digital filtering, kinematic integration, and complementary sensor fusion requirements for **Stage 1: Aircraft Attitude Determination (IMU)** on the Arduino Uno (ATmega328P) platform using the MPU-6050 / MPU-6000 inertial measurement unit.

## Requirements

### Requirement: I2C Bus and IMU Register Initialization
The system SHALL initialize the I2C bus as master at 400 kHz (`TWBR = 12`) and configure the MPU-6050/MPU-6000 IMU at address `0x68` with waking power management, $\pm 500\text{ dps}$ gyroscope full-scale range, $\pm 8g$ accelerometer full-scale range, and a $\sim 43\text{ Hz}$ internal Digital Low-Pass Filter (DLPF).

#### Scenario: Successful IMU register configuration and verification
- **GIVEN** the Arduino Uno is powered on and connected via I2C (`A4/SDA`, `A5/SCL`) to the IMU at `0x68`
- **WHEN** `set_gyro_registers()` executes during `setup()`
- **THEN** the system SHALL write `0x00` to `PWR_MGMT_1` (`0x6B`), `0x08` to `GYRO_CONFIG` (`0x1B`), `0x10` to `ACCEL_CONFIG` (`0x1C`), verify that reading `0x1B` returns `0x08`, and write `0x03` to `CONFIG` (`0x1A`).

#### Scenario: IMU register verification failure
- **GIVEN** `set_gyro_registers()` reads back register `0x1B` from address `0x68`
- **WHEN** the returned byte is not equal to `0x08`
- **THEN** the system SHALL set the warning LED pin (`D12`) to `HIGH` and halt execution in an infinite loop.

### Requirement: Gyroscope Stationary Bias Calibration
The system SHALL compute stationary gyroscope bias offsets across all three body axes ($x, y, z$) by averaging 2000 consecutive raw readings during `setup()` before entering the main flight loop.

#### Scenario: 2000-sample startup calibration loop
- **GIVEN** IMU registers are initialized and `cal_int` is not equal to `2000`
- **WHEN** the calibration loop runs for 2000 iterations with a $3\text{ ms}$ delay between samples
- **THEN** the system SHALL toggle LED `D12` every 125 samples, accumulate raw gyroscope readings for roll, pitch, and yaw axes without out-of-bounds array indexing, divide each sum by 2000, and subtract the resulting bias from subsequent readings once `cal_int == 2000`.

### Requirement: First-Order Digital Low-Pass Filter on Angular Rates
The system SHALL attenuate high-frequency gyroscope noise on the body angular rates ($p, q, r$ in degrees per second) using a first-order IIR digital filter with coefficients $0.7$ (previous state) and $0.3$ (new sample).

#### Scenario: Angular rate filtering per control cycle
- **GIVEN** bias-compensated gyroscope raw values are converted to degrees per second ($\text{LSB} / 65.5$ for $\pm 500\text{ dps}$)
- **WHEN** a new 250 Hz cycle executes
- **THEN** the system SHALL update each axis angular rate according to:
  $$p^{k+1} = 0.7\,p^k + 0.3\,p_{new}$$
  $$q^{k+1} = 0.7\,q^k + 0.3\,q_{new}$$
  $$r^{k+1} = 0.7\,r^k + 0.3\,r_{new}$$

### Requirement: Euler Angle Kinematic Integration from Gyroscope
The system SHALL compute the Euler angle rates ($\dot{\phi}_{gyro}, \dot{\theta}_{gyro}, \dot{\psi}_{gyro}$) from the filtered body angular rates ($p^{k+1}, q^{k+1}, r^{k+1}$) and numerically integrate them over the $\Delta t = 0.004\text{ s}$ ($250\text{ Hz}$) period.

#### Scenario: Kinematic transformation and forward Euler integration
- **GIVEN** the previous Euler angles ($\phi^k, \theta^k, \psi^k$) and current filtered body rates ($p^{k+1}, q^{k+1}, r^{k+1}$)
- **WHEN** the gyroscope attitude update executes
- **THEN** the system SHALL evaluate the kinematic rate equations:
  $$\phi^{k+1}_{gyro} = \phi^k + (p^{k+1} + q^{k+1}\tan\theta^k\sin\phi^k + r^{k+1}\tan\theta^k\cos\phi^k)\,\Delta t$$
  $$\theta^{k+1}_{gyro} = \theta^k + (q^{k+1}\cos\phi^k - r^{k+1}\sin\phi^k)\,\Delta t$$
  $$\psi^{k+1}_{gyro} = \psi^k + (q^{k+1}\sin\phi^k\sec\theta^k + r^{k+1}\cos\phi^k\sec\theta^k)\,\Delta t$$

### Requirement: Accelerometer Source Bias Calibration and Phase-Synchronized Telemetry Output
The system SHALL compute stationary accelerometer biases (`acc_bias[0..2]`) at the source during IMU calibration such that resting acceleration equals $(0, 0, +4096\text{ LSB} = +1.00g)$, accumulate accelerations across the 10-tick decimation window, latch the averaged values at Tick 5 concurrently with attitude angles, emit 4-decimal precision values (`Roll Pitch Yaw AccX AccY AccZ`) at 25 Hz, and toggle the onboard status LED (`PIN_LED_STATUS` / Pin 13) every `led_toggle_interval` telemetry cycles.

#### Scenario: Source-calibrated resting acceleration
- **GIVEN** the drone is stationary and level during `calibrate_imu()`
- **WHEN** calibration finishes
- **THEN** stationary body accelerations SHALL read $Acc_x = 0$, $Acc_y = 0$, and $Acc_z = +4096\text{ LSB}$ ($+1.00g$), aligning attitude angles ($\phi = 0^\circ, \theta = 0^\circ$) and acceleration vectors at the source.

#### Scenario: Phase-synchronized 10-tick decimation and visual heartbeat
- **GIVEN** the 250 Hz control loop executing with decimation factor 10
- **WHEN** Tick 5 occurs
- **THEN** the system SHALL latch the averaged acceleration across the decimation window and transmit attitude angles over serial, followed by the latched acceleration vector at Tick 10 with 4-decimal precision, and toggle the status LED state every `led_toggle_interval = 10` cycles ($1.25\text{ Hz}$ heartbeat).
The system SHALL estimate low-frequency roll ($\phi_{acc}$) and pitch ($\theta_{acc}$) Euler angles from the 3-axis accelerometer projections relative to the total acceleration magnitude.

#### Scenario: Stationary or quasi-static tilt estimation
- **GIVEN** raw 3-axis accelerometer readings $(Acc_x, Acc_y, Acc_z)$
- **WHEN** the total acceleration vector magnitude $Acc_{Total} = \sqrt{Acc_x^2 + Acc_y^2 + Acc_z^2}$ is non-zero and within valid physical bounds ($|Acc_x| < Acc_{Total}$ and $|Acc_y| < Acc_{Total}$)
- **THEN** the system SHALL compute:
  $$\phi_{acc}^{k+1} = \arcsin\left(\frac{Acc_y}{Acc_{Total}}\right)$$
  $$\theta_{acc}^{k+1} = -\arcsin\left(\frac{Acc_x}{Acc_{Total}}\right)$$
  expressed in degrees according to the body coordinate frame ($x$ forward to nose, $y$ right wing, $z$ downward to Earth).

### Requirement: Complementary Filter Sensor Fusion and 250 Hz Loop Timing
The system SHALL fuse high-frequency gyroscope Euler integration with low-frequency accelerometer tilt estimates using a complementary filter ($\alpha = 0.9996$, $1 - \alpha = 0.0004$) and enforce a strict $4000\,\mu\text{s}$ ($250\text{ Hz}$) loop period.

#### Scenario: First iteration attitude initialization
- **GIVEN** `first_angle` is `false` on the first execution of `loop()`
- **WHEN** initial accelerometer angles $\phi_{acc}$ and $\theta_{acc}$ are computed
- **THEN** the system SHALL initialize $\phi = \phi_{acc}$, $\theta = \theta_{acc}$, and set `first_angle = true`.

#### Scenario: Steady-state complementary filter fusion and loop pacing
- **GIVEN** `first_angle` is `true`
- **WHEN** the attitude update completes during `loop()`
- **THEN** the system SHALL compute:
  $$\phi^{k+1} = 0.9996\,(\phi^k + \dot{\phi}_{gyro}^{k+1}\Delta t) + 0.0004\,\phi_{acc}^{k+1}$$
  $$\theta^{k+1} = 0.9996\,(\theta^k + \dot{\theta}_{gyro}^{k+1}\Delta t) + 0.0004\,\theta_{acc}^{k+1}$$
  and wait until `micros() - loop_timer >= 4000` before starting the next cycle.
