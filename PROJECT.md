# 🚁 Project Manifest: Arduino Quadcopter Flight Controller (`drone`)

Standard interface manifest for the `drone` autonomous sub-project within the Maestro ecosystem.

---

## 📌 Project Identity & Metadata

- **Project Name:** Arduino Quadcopter Flight Controller (`drone`)
- **Directory:** `projects/drone/`
- **Domain Tags:** `#drone` `#quadcopter` `#arduino` `#atmega328p` `#mpu6050` `#mpu6000` `#imu` `#pid-control` `#embedded-c`
- **Status:** Active (Stage 1: IMU Attitude Determination in Progress)
- **Version Control Scope:** Independent repository / sub-project with encapsulated embedded firmware in `code/` (`code/src/`, `code/include/`), and isolated `docs/`, `openspec/`, `odometry/`, `ref/`, and `graphify-out/`.

---

## 🎯 Purpose & Phased Roadmap

The **Arduino Quadcopter Flight Controller (`drone`)** repository develops a deterministic, 250 Hz ($4000\,\mu\text{s}$ loop) flight stabilization system on the Arduino Uno (ATmega328P), structured around the EESC-USP specialization guide (*Angélico, Daud Filho, Bidinotto, July 2025*) and the YMFC-3D/AL hardware schematic:

1. **Stage 1 — Aircraft Attitude Determination (IMU) [ACTIVE]:**
   - 400 kHz I2C register setup and verification for MPU-6050 / MPU-6000 (`0x68`).
   - 2000-sample stationary gyroscope bias calibration (`gyro_axis_cal`).
   - First-order IIR digital low-pass filter on angular rates ($p, q, r$): $0.7 / 0.3$.
   - Euler angle kinematic rate transformation and numerical integration ($\Delta t = 0.004\text{ s}$).
   - 3-axis accelerometer gravity vector projection ($\phi_{acc}, \theta_{acc}$).
   - Complementary filter sensor fusion ($\alpha = 0.9996$, $1 - \alpha = 0.0004$) and Serial Plotter verification.
2. **Stage 2 — RC Receiver PWM Capture & EEPROM Calibration [OPEN / DEFERRED]:**
   - Pin Change Interrupts (`PCINT0..3` on `D8`–`D11`) for 4-channel radio pulse width capture (`THRO`, `AILE`, `ELEV`, `RUDD`).
   - Interactive stick limit/center calibration persisted in EEPROM (addresses `0`–`23`) and linear normalization ($1000\text{–}2000\,\mu\text{s}$).
3. **Stage 3 — Motor Testing & Arm/Disarm State Machine [OPEN / DEFERRED]:**
   - EEPROM verification on startup and individual/simultaneous motor test routines (`1`–`5`: FR, RR, RL, FL, ALL).
   - Left-stick (`THRO` + `RUDD`) safety arming/disarming state machine (`start = 0, 1, 2`).
4. **Stage 4 — PID Flight Stabilization & Battery Compensation [OPEN / DEFERRED]:**
   - Cascaded attitude-to-rate setpoint conversion ($\pm 8\,\mu\text{s}$ deadband, max tilt $32.8^\circ$, max rate $164^\circ/\text{s}$) and 3-axis PID controller with $\pm 400\,\mu\text{s}$ integral/output saturation.
   - 3S LiPo battery voltage monitoring on `A0` with diode drop compensation, $0.92 / 0.08$ digital filter, low-voltage warning (`D12` LED at $< 10.0\text{ V}$), and ESC pulse thrust compensation.
   - Synchronous 250 Hz hardware-timed pulse generation on `PORTD` (`D4`–`D7`).

---

## 📐 Spec-Driven Development (SDD) & Dual-Role TDD Governance

This project is governed by strict **Spec-Driven Development (SDD)** with OpenSpec and **Test-Driven Development (TDD)** using an isolated Python/Pytest test stack:

1. **Matrix Specification Prerequisite (SDD First):**
   - No production code (C or Python) SHALL ever be written without a prior normative specification matrix (`proposal.md`, `specs/*.md` with EARS & `GIVEN / WHEN / THEN`, `design.md`, `tasks.md`) under `openspec/`.
2. **Dual-Role Builder/Tester Discipline (Strict TDD):**
   - Distinct roles are maintained: one role designs and authors rigorous failing test harnesses (RED), and another implements the minimal production code to pass (GREEN), followed by clean refactoring.
3. **Automated Python Test Stack:**
   - Every hardware driver, kinematic algorithm, telemetry pipeline, and mathematical transformation is mapped to automated, deterministic tests in Python (`pytest` inside an isolated `.venv`).
4. **Odometry & Real-Time 3D Simulation Subsystem (`odometry/`):**
   - Real-time 3D telemetry visualization (Pygame + PyOpenGL) consuming a 10-position FIFO buffer from `odometry/telemetry.log`, fed by an IMU C driver tester (`odometry/tester.c`) for continuous live inspection and dimensioning validation.

---

## 📐 Spec-Driven Development (SDD) & OpenSpec Integration

- **Human & Agent SDD Hub:** [docs/README.md](docs/README.md)
- **System Purpose & Roadmap:** [docs/system_purpose.md](docs/system_purpose.md)
- **Bottom-Up System Architecture:** [docs/system_architecture.md](docs/system_architecture.md)
- **Canonical OpenSpec Specifications:** [openspec/specs/](openspec/specs/)
- **Active OpenSpec Changes:**
  - Stage 1 IMU Firmware: [openspec/changes/stage-1-imu-implementation/](openspec/changes/stage-1-imu-implementation/)
  - Odometry 3D Simulation & Tester: [openspec/changes/odometry-3d-simulation/](openspec/changes/odometry-3d-simulation/)

---

## 🔄 Graphify Integration

- **Local AST & Semantic Graph:** [graphify-out/graph.json](graphify-out/graph.json)
- **Audit Report:** [graphify-out/GRAPH_REPORT.md](graphify-out/GRAPH_REPORT.md)

