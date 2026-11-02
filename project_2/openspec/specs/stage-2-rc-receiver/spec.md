# stage-2-rc-receiver Specification

## Purpose
Defines the planned normative requirements for **Stage 2: RC Receiver Pulse Width Measurement & EEPROM Calibration** (currently open/deferred until Stage 1 is completed and verified).

## Requirements

### Requirement: Pin Change Interrupt (PCINT) PWM Capture
The system SHALL capture PWM pulse widths ($1000\,\mu\text{s}$ to $2000\,\mu\text{s}$ nominal at $45\text{ Hz}$) across 4 radio receiver channels connected to `D8` (THRO), `D9` (AILE), `D10` (ELEV), and `D11` (RUDD) using hardware Pin Change Interrupts (`PCIE0`, `PCINT0..3`).

#### Scenario: Rising and falling edge pulse timing on PINB
- **GIVEN** `PCICR` and `PCMSK0` are configured for `PCINT0..3` in `setup()`
- **WHEN** a state change occurs on pins `D8`–`D11` triggering `ISR(PCINT0_vect)`
- **THEN** the ISR SHALL record the rising edge timestamp in `micros()` and compute the high-pulse duration on the falling edge into `receiver_input_channel_1..4`.

### Requirement: Stick Endpoint Calibration and EEPROM Persistence
The system SHALL guide the operator through stick endpoint and center identification during calibration and store 16-bit integer values (center, low, high) in EEPROM addresses `0` through `23`.

#### Scenario: Saving and reading 2-byte channel calibration bounds
- **GIVEN** the user moves THRO, AILE, ELEV, and RUDD sticks to center, minimum, and maximum positions during setup calibration
- **WHEN** calibration finishes
- **THEN** the system SHALL write 2-byte little-endian integers to EEPROM addresses `0..7` (Center), `8..15` (Low), and `16..23` (High) and normalize live readings linearly to $1000\text{–}2000\,\mu\text{s}$ with $1500\,\mu\text{s}$ center.
