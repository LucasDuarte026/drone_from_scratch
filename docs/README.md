# Central Project Documentation (`docs/`)

> **Purpose of This Directory:**  
> The `docs/` directory is the human- and AI-readable architectural core of the **Arduino Quadcopter Flight Controller (`drone`)** repository. While `openspec/` stores formal normative contracts (`SHALL`/`MUST` with `GIVEN/WHEN/THEN` scenarios), `docs/` explains **why the project exists, how the physical hardware and mathematical equations connect, and how the 4-stage incremental development progresses**.
>
> This directory is maintained under **Zero Doc-Drift** for two primary audiences:
> 1. **Humans (Engineers / Researchers / Pilots):** To quickly grasp the physical wiring, coordinate frames, control theory equations, and stage progression without parsing raw register code.
> 2. **Autonomous AI Agents (Maestro & Subagents):** As the primary onboarding and context-recovery hub before proposing OpenSpec changes, writing firmware, or auditing real-time constraints.

---

## 📂 Documentation Structure

| File / Directory | Primary Purpose | Mandatory Reading For |
| :--- | :--- | :--- |
| 📄 **[system_purpose.md](system_purpose.md)** | **The "Why" & "What" (Product & Roadmap).** Defines the educational and engineering objectives, the 4 incremental implementation stages, safety boundaries, and actors. | Onboarding, stage scoping, and `/grill-me` requirement refinement. |
| 📄 **[system_architecture.md](system_architecture.md)** | **The "How It Works" (Hardware & Math Architecture).** Maps the 5-layer Bottom-Up stack, Arduino Uno pinout, MPU-6050/6000 registers, Euler kinematics, complementary filter, PID equations, Appendix B globals, and prototype bug audit. | Writing firmware, verifying mathematical formulas, and debugging hardware/timing issues. |
| 📄 **[CHANGELOG.md](CHANGELOG.md)** | **The "History" (Evolution Log).** Chronological record of all architectural decisions, OpenSpec proposals, and stage completions. | Auditing repository progress across sessions. |
| 📁 **[../ref/DRONE_materialdeapoio/](../ref/DRONE_materialdeapoio/)** | **The "Reference Baseline" (Datasheets & Guide).** Contains the EESC-USP guide (`tutorial_programacao_drone_especializacao.pdf`), `example_01.ino` prototype, `YMFC_schematic.pdf`, and MPU-6000 datasheets. | Cross-checking original equations and hardware specifications. |

---

## 📐 Integration with Spec-Driven Development (SDD) & Dual-Role TDD

This project operates strictly under **Bottom-Up Spec-Driven Development (SDD)** paired with **Dual-Role Test-Driven Development (TDD)**:

1. **Matrix Specification Prerequisite:** Every subsystem must have an OpenSpec delta spec under `openspec/changes/` before implementation. Requirements are written in EARS notation with executable `GIVEN / WHEN / THEN` scenarios.
2. **Dual-Agent Role Specialization:**
   - **Agent Tester Role:** Creates the Python pytest suite first, mapping directly to the scenario contracts and verifying failure (RED).
   - **Agent Builder Role:** Writes the minimal production code in C/Python to satisfy the tests (GREEN), and refactors cleanly.
3. **Automated Python Test Rig (`.venv`):** High-control unit and integration tests run inside the Python virtual environment for all telemetry decoding, ring buffers, kinematic transforms, and driver mocking.
4. **Interactive Telemetry & Odometry (`odometry/`):** Real-time 3D simulation and C driver tester (`odometry/tester.c` + `odometry/sim_3d.py`) to provide visual inspection and dimensioning verification during development.
5. **Stage Isolation:** Stage 1 (IMU Attitude Determination) is the active focus. Stage 2 (RC Receiver PWM & EEPROM) and Stages 3–4 (Motor Testing & PID Control) are documented at the architectural level and kept open for subsequent development rounds.
6. **English-Only UI & Documentation Standard:** All system documentation, OpenSpec specifications, source code comments, serial telemetry messages, and graphical user interfaces (including the 3D simulator HUD in `odometry/sim_3d.py`) MUST use **English as the mandatory default language** (`test_english_only_ui_labels`).

