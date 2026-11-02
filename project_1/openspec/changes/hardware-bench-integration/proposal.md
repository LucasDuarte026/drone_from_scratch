# Proposal: Project 1 Hardware Bench Integration & Prototype Remediation

## Why
`project_1` represents the physical embedded quadcopter test bench developed for course SAA0356 (Sistemas Embarcados para Veículos Aéreos) at EESC-USP. The system integrates dual Arduino microcontrollers, an MPU-6050 6-axis IMU, an HC-12 wireless RF transceiver pair, and four 28BYJ-48 unipolar stepper motors driven by ULN2003 Darlington arrays.

The original course reference code (`ref/DRONE_materialdeapoio/example_01.ino`) contained critical defects: out-of-bounds array access leading to SRAM corruption, missing pin modes, and incomplete attitude kinematics. Project 1 resolves these issues, implements a complete attitude determination pipeline with differential tilt motor compensation, and streams telemetry to a custom Pygame/PyOpenGL 3D visualizer (`envioadron.py`).

Prior to this specification, `project_1` had no documentation. This proposal codifies its hardware pinout, communication protocols, motor scheduling algorithms, and mathematical corrections into a standardized specification and subsystem README.

## What Changes
- Specify hardware architecture, pin assignments, and electrical requirements for Arduino 1 (flight unit) and Arduino 2 (wireless remote controller).
- Specify the non-blocking stepper motor driver and proportional tilt compensation control law.
- Specify the serial telemetry protocol and RF command interface (`S<state>`, `P<power>`).
- Document the mathematical defects found in `example_01.ino` and specify their exact remediation in `arduino1dron.ino`.
- Author `project_1/README.md` including a clickable HTML YouTube video card, transcription context, wiring tables, and image gallery.

## Capabilities
- `ADDED`: Hardware bench integration specification (`specs/hardware-bench/spec.md`).
- `ADDED`: Stepper motor timing, non-blocking scheduler, and active tilt compensation requirements.
- `ADDED`: Reference prototype audit and mathematical remediation contract.

## Impact
- Files Created:
  - `project_1/README.md`
  - `project_1/openspec/*`
- Build/Runtime Impact: Pure documentation and specification; zero modification to existing `.ino` and `.py` code.
