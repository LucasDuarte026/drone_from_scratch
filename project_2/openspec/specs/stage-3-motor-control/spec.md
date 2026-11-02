# stage-3-motor-control Specification

## Purpose
Defines the planned normative requirements for **Stage 3: Motor Testing & Arm/Disarm Safety State Machine** and **Stage 4: PID Flight Stabilization & ESC Pulse Generation** (currently open/deferred).

## Requirements

### Requirement: Motor Arm and Disarm State Machine
The system SHALL prevent unintended motor spin-up by enforcing a two-step left-stick (`THRO` + `RUDD`) arming sequence controlled by the `start` state variable (`0 = disarmed`, `1 = arming ready`, `2 = armed`).

#### Scenario: Arming the quadcopter motors
- **GIVEN** `start == 0` and all four ESC outputs are held at $1000\,\mu\text{s}$
- **WHEN** the pilot moves `THRO < 1050` and `RUDD < 1050` (`start = 1`) and subsequently returns `RUDD > 1450` while keeping `THRO < 1050`
- **THEN** the system SHALL transition to `start = 2`, initialize attitude angles to accelerometer readings (`angle_pitch = angle_pitch_acc`, `angle_roll = angle_roll_acc`), reset PID integral and derivative memories to zero, and set minimum ESC idle pulse to $1100\,\mu\text{s}$.

#### Scenario: Disarming the quadcopter motors
- **GIVEN** `start == 2` (motors armed)
- **WHEN** the pilot moves `THRO < 1050` and `RUDD > 1950`
- **THEN** the system SHALL transition immediately to `start = 0` and command $1000\,\mu\text{s}$ to all four ESCs.

### Requirement: Synchronous 250 Hz ESC Pulse Generation on PORTD
The system SHALL generate $1000\text{–}2000\,\mu\text{s}$ PWM pulses for ESC 1 (`D4`, FR), ESC 2 (`D5`, RR), ESC 3 (`D6`, RL), and ESC 4 (`D7`, FL) via direct `PORTD` register manipulation synchronized to the $4000\,\mu\text{s}$ ($250\text{ Hz}$) control loop.

#### Scenario: Generating ESC pulses within 4000 us loop budget
- **GIVEN** control calculations are complete and `micros() - loop_timer >= 4000`
- **WHEN** a new $250\text{ Hz}$ cycle begins
- **THEN** the system SHALL set `PORTD |= B11110000` (pins 4, 5, 6, 7 HIGH simultaneously) and clear each corresponding bit (`PORTD &= ...`) as its individual pulse expiration timestamp (`esc_1..4 + loop_timer`) is reached.
