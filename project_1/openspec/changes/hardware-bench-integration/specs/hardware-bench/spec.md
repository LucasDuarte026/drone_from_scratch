# Specification: Project 1 Hardware Bench Integration & Prototype Remediation

## ADDED Requirements

### Requirement: Dual-Arduino Hardware Architecture & Pin Assignments
The system SHALL operate across two dedicated ATmega328P microcontrollers:
1. **Arduino 1 (Flight Bench Unit)**:
   - I2C Interface: SDA on `A4`, SCL on `A5` (MPU-6050 @ `0x68`, 400 kHz clock).
   - Stepper Motor 1 (Front Right / FD): Pins `2, 3, 4, 5`.
   - Stepper Motor 2 (Rear Right / TD): Pins `6, 7, 8, 9`.
   - Stepper Motor 3 (Rear Left / TI): Pins `10, 11, 12, 13`.
   - Stepper Motor 4 (Front Left / FI): Pins `A0, A1, A2, A3`.
   - Hardware UART: Pin `0` (RX), Pin `1` (TX) @ 115200 bps.
2. **Arduino 2 (Wireless Remote Controller)**:
   - Analog Potentiometer: Pin `A0` (throttle control 0–1023 mapped to 0–100%).
   - HC-12 433 MHz RF Transceiver: Pin `10` (RX), Pin `11` (TX) via `SoftwareSerial` @ 115200 bps.
   - Hardware UART: Pin `0` (RX), Pin `1` (TX) @ 115200 bps for PC serial bridging.

#### Scenario: Transmitting Remote Throttle Commands
- **GIVEN** the remote controller operating with potentiometer set to a new level ($P \ge 2$)
- **WHEN** the ADC read detects a change $\ge 1\%$
- **THEN** Arduino 2 SHALL transmit `S1\n` followed by `P<value>\n` over HC-12 to Arduino 1.

---

### Requirement: Prototype Defect Remediation (`example_01.ino`)
The firmware in `arduino1dron.ino` SHALL correct all four architectural defects identified in the reference `example_01.ino`:
1. **Array Indexing Fix**: All gyro and calibration arrays (`gyro_axis`, `gyro_cal`) MUST use zero-based indexing (`0, 1, 2`), eliminating out-of-bounds indexing (`gyro_axis[3]`) that corrupts SRAM.
2. **Pin Initialization**: All output pins (`MOTOR_PINS`, status LEDs) MUST be initialized explicitly with `pinMode(pin, OUTPUT)`.
3. **Kinematic Integration**: Angular rate measurements MUST be scaled by $65.5\text{ LSB/(deg/s)}$, filtered through a first-order digital filter ($0.7 / 0.3$), and integrated into Euler angles using the non-linear transformation:
   $$\dot{\phi} = p + q\tan\theta\sin\phi + r\tan\theta\cos\phi$$
   $$\dot{\theta} = q\cos\phi - r\sin\phi$$
   $$\dot{\psi} = (q\sin\phi + r\cos\phi)\sec\theta$$
4. **Sensor Fusion**: Accelerometer tilt angles ($\phi_{acc} = \arcsin(a_y / a_{total})$, $\theta_{acc} = -\arcsin(a_x / a_{total})$) MUST be fused with gyro integration via a $0.9996 / 0.0004$ complementary filter.

#### Scenario: Verifying Attitude Stability Under Hand Tilt
- **GIVEN** Arduino 1 calibrated and running its 250 Hz control loop
- **WHEN** the drone platform is tilted forward by $+15^\circ$
- **THEN** the firmware SHALL output an attitude of $\approx +15^\circ$ without divergence or zero-drift decay.

---

### Requirement: Differential Tilt Compensation Motor Control
The firmware SHALL compute individual motor speeds using proportional tilt compensation:
$$adj_{pitch} = k \cdot \theta, \quad adj_{roll} = k \cdot \phi \quad (k = 2.0)$$
$$pwr_1 = P_{global} + adj_{pitch} - adj_{roll} \quad (\text{M1: Front Right})$$
$$pwr_2 = P_{global} - adj_{pitch} - adj_{roll} \quad (\text{M2: Rear Right})$$
$$pwr_3 = P_{global} - adj_{pitch} + adj_{roll} \quad (\text{M3: Rear Left})$$
$$pwr_4 = P_{global} + adj_{pitch} + adj_{roll} \quad (\text{M4: Front Left})$$
Power levels SHALL be clamped to $[0, 100]\%$, and non-zero powers mapped to step intervals between $5000\,\mu\text{s}$ (1%) and $1000\,\mu\text{s}$ (100%).

#### Scenario: Tilting Forward (Pitch Positive)
- **GIVEN** motors enabled ($S=1$) with $P_{global} = 50\%$ and platform tilted nose-down ($\theta > 0$)
- **WHEN** motor power is calculated
- **THEN** front motors M1 and M4 SHALL increase power while rear motors M2 and M3 SHALL decrease power, creating a counter-torque to level the platform.

---

### Requirement: Ephemeral OpenSpec Governance
All artifacts inside `project_1/openspec/` SHALL serve as internal development contracts only:
1. The permanent technical documentation for Project 1 SHALL reside in `project_1/README.md`.
2. This `project_1/openspec/` directory SHALL be excluded from the final open-source public Git release.

#### Scenario: OpenSpec Removal for Release
- **GIVEN** a public Git release tag being created
- **WHEN** files are packaged
- **THEN** `project_1/openspec/` SHALL be removed, leaving `project_1/README.md` as the self-contained subsystem documentation.
