# Architectural Decision Records (ADRs)

This directory maintains the **Architectural Decision Records** for Project 2 (High-Precision IMU Odometry & 3D Telemetry Platform), following the **Gate 2: Lightweight ADR** standard of Maestro's `my_auto_document` skill.

---

## 📋 ADR Index

| ADR ID | Title | Date | Status | Impact Area |
| :--- | :--- | :---: | :---: | :--- |
| [ADR-0001](/home/luska/Documents/projects/Maestro/projects/drone/project_2/docs/adr/0001-gravity-isolation-and-static-calibration.md) | Gravity Isolation and Static Accelerometer Bias Calibration | 2026-10-28 | **Accepted** | C++ Firmware (`code/src/main.cpp`) |
| [ADR-0002](/home/luska/Documents/projects/Maestro/projects/drone/project_2/docs/adr/0002-dual-rate-decimation-and-uart-telemetry.md) | Dual-Rate Phase-Synchronized 10-Tick Decimation (250 Hz to 25 Hz) | 2026-10-28 | **Accepted** | UART Telemetry & Timing |
| [ADR-0003](/home/luska/Documents/projects/Maestro/projects/drone/project_2/docs/adr/0003-zero-velocity-update-zupt-and-noise-deadband.md) | Zero-Velocity Update (ZUPT) and $0.05g$ Noise Deadband | 2026-10-28 | **Accepted** | Kinematics (`odometry/drone_model.py`) |
| [ADR-0004](/home/luska/Documents/projects/Maestro/projects/drone/project_2/docs/adr/0004-dual-tier-telemetry-logging-architecture.md) | Dual-Tier Telemetry Logging Architecture (Standard vs Verbose) | 2026-10-28 | **Accepted** | Observability (`odometry/telemetry_logger.py`) |

---

## 🏛️ ADR Format Standard

All ADRs follow the standard 4-section architecture:
1. **Context & Problem Statement**: Motivation, physical or software constraints, observed failure modes.
2. **Alternatives Considered**: Evaluated designs and why alternative options were discarded.
3. **Decision**: Chosen architecture and physical implementation details.
4. **Consequences & Trade-offs**: Positive operational outcomes and negative trade-offs.
