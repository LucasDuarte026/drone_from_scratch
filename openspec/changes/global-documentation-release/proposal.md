# Proposal: Global Open-Source Documentation & Multi-Project Governance

## Why
The drone workspace contains two complementary embedded projects created for the SAA0356 course (Sistemas Embarcados para Veículos Aéreos) at EESC-USP:
1. `project_1`: A physical quadcopter test bench featuring dual Arduino Uno/Nano nodes communicating over HC-12 433 MHz RF, actuating 4 stepper motors with ULN2003 drivers via differential tilt compensation, and streaming telemetry to a Pygame/PyOpenGL 3D visualizer.
2. `project_2`: A firmware architecture and 3D odometry simulation platform developed under Spec-Driven Development (SDD) and Test-Driven Development (TDD), featuring deterministic 250 Hz control, 2-tick serial decimation, and research into inertial position estimation (ZUPT, deadband, quadratic integration drift).

Previously, the repository lacked a unified open-source documentation hierarchy. Project 1 had zero formal documentation, while Project 2 possessed deep internal architecture specs but required structured public exposure and rigorous odometry analysis. This proposal establishes a three-tier documentation structure (Root, Project 1, Project 2), formalizes academic attribution, embeds interactive multimedia cards, and enforces the ephemeral lifecycle of internal OpenSpec folders.

## What Changes
- Author comprehensive Root `README.md` introducing the system, team, academic links, badges, comparative architecture, and multimedia demo.
- Author `project_1/README.md` detailing hardware wiring, stepper motor scheduling, RF communications, mathematical remediation of `example_01.ino`, and video demonstration.
- Author `project_2/README.md` detailing firmware architecture, PlatformIO toolchain, automated Pytest harness, and an in-depth mathematical analysis of odometry divergence and Kalman filtering.
- Codify the OpenSpec Ephemeral Lifecycle rule: all `openspec/` directories serve internal development and will be purged before final public release, with permanent knowledge retained in `docs/` and READMEs.
- Formulate `docs/KNOWLEDGE_GAPS_REPORT.md` identifying missing datasheets, media assets, and parameters for future iterations.

## Capabilities
- `ADDED`: Global documentation architecture specification (`specs/documentation-governance/spec.md`).
- `ADDED`: Multi-project comparative presentation and academic compliance matrix.
- `ADDED`: Ephemeral lifecycle enforcement policy for SDD specifications.

## Impact
- Files Created/Modified:
  - `README.md`
  - `project_1/README.md`
  - `project_2/README.md`
  - `docs/KNOWLEDGE_GAPS_REPORT.md`
  - `openspec/config.yaml`
  - `openspec/changes/global-documentation-release/*`
- Build/Runtime Impact: Zero runtime performance overhead; documentation and metadata only.
