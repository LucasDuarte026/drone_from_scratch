# Implementation Tasks: Global Documentation & Governance

## 1. Specification & Governance Setup
- [x] 1.1 Establish root `openspec/` with `config.yaml`, `proposal.md`, `spec.md`, `design.md`, and `tasks.md`
- [x] 1.2 Codify OpenSpec ephemeral lifecycle rule across all specifications
- [x] 1.3 Establish `project_1/openspec/` subsystem specification

## 2. Subsystem Documentation Authoring
- [x] 2.1 Author `project_1/README.md`:
  - [x] Hardware component inventory and wiring pinout tables (Arduino 1, ULN2003, stepper motors, HC-12, potentiometer)
  - [x] Stepper motor timing, non-blocking scheduler, and differential tilt compensation math
  - [x] Full audit of `example_01.ino` defects (array out-of-bounds, missing pinMode, missing kinematics) and Project 1 fixes
  - [x] Pygame/PyOpenGL 3D visualizer guide (`envioadron.py`)
  - [x] Clickable HTML video card for YouTube demonstration (`eWvYoFlVz2w`) and transcription context
  - [x] Image gallery referencing `platform.jpeg` and `exploded_hardware_connections.jpeg`
- [x] 2.2 Author `project_2/README.md`:
  - [x] PlatformIO build environment, firmware architecture, and 250 Hz control loop
  - [x] 2-tick phase-synchronized 25 Hz telemetry streaming
  - [x] Standalone C driver tester and automated Pytest test suite (22/22 passing tests)
  - [x] Dedicated Odometry & Inertial Navigation section (Euler vs. Trapezoidal vs. ZUPT, $0.05g$ deadband)
  - [x] Mathematical analysis of quadratic error growth in double integration ($\iint a\,dt^2$) and justification for Kalman filter sensor fusion
  - [x] Placeholder cards for future 3D simulation videos and UI screenshots

## 3. Master README & Knowledge Gaps Report
- [x] 3.1 Author Root `README.md`:
  - [x] Open-source shields.io badges (C++, Python, Arduino, PlatformIO, License, Pytest)
  - [x] Academic metadata (Course SAA0356, Turma 201, EESC-USP, Prof. Glauco Caurin with USP + Lattes links, team members)
  - [x] Comparative matrix between Project 1 (hardware bench integration) and Project 2 (firmware architecture & odometry research)
  - [x] Top-level standard Mermaid architecture diagram
  - [x] Featured multimedia section with YouTube video card and Project 2 video placeholder
  - [x] Repository file tree and verified operational command table
- [x] 3.2 Author `docs/KNOWLEDGE_GAPS_REPORT.md` (Action Report of missing datasheets, CAD, and assets for next round)
- [x] 3.3 Validate internal relative markdown links, Pytest execution, and firmware compilation
