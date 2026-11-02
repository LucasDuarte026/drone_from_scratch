# Action Report: Knowledge Gaps & Information Request (Round 1)

> **Document Type:** Engineering Investigation & Action Report  
> **Course:** SAA0356 - Sistemas Embarcados para Veículos Aéreos (Turma 201) — EESC-USP  
> **Target Audience:** Master / Project Maintainer (Lucas Duarte & Team)  
> **Status:** Active — Pending Information for Round 2  

---

## 1. Executive Summary

During the initial engineering audit, knowledge graph construction (`/graphify`), and documentation authoring across `project_1` and `project_2`, the codebase and reference prototype (`example_01.ino`) were analyzed in detail. While the firmware logic, mathematical filters, communication protocols, and simulation code are fully understood and verified, several physical, hardware, and multimedia details require additional information from the team to complete future development stages and open-source publication.

This report summarizes these knowledge gaps and provides an actionable checklist of items to supply in the next iteration.

---

## 2. Hardware Specifications & Electrical Datasheets

| Item | Current Assumption / Status | Information Needed from Team | Priority |
| :--- | :--- | :--- | :---: |
| **Stepper Motor Gearbox Ratio** | Assumed standard 28BYJ-48 5V unipolar stepper with 1:64 reduction (approx. 2048 or 4096 half-steps/rev). | Confirm exact model, nominal voltage (5V vs 12V), and whether motors operate in full-step (2048 steps/rev) or half-step. | Medium |
| **Bench Power Supply Parameters** | 4 stepper motors driven simultaneously via ULN2003 can draw up to $1.2\text{ A}$ at stall. | Clarify bench power supply voltage (e.g., 5V 2A external DC vs. USB rail) and whether grounds are unified with Arduino GND. | Medium |
| **Mechanical Frame & CAD Files** | Physical bench observed in `platform.jpeg` uses wooden cross-arms and central breadboard mounting. | Provide any 3D models (STL / STEP) or dimension drawings of motor mounts and central frame for inclusion in `project_1/cad/`. | Low |
| **Complete Electrical Schematics** | Fritzing image `exploded_hardware_connections.jpeg` illustrates wiring. | Provide native KiCad or Fritzing project files (`.fzz` / `.kicad_sch`) for version-controlled vector export into documentation. | Medium |
| **HC-12 Module Configuration** | Default 9600 bps AT command state vs. firmware configured 115200 bps (`SoftwareSerial hc12(10, 11)`). | Confirm if HC-12 was pre-programmed using `AT+B115200` or `FU3` mode, or if default factory baud was modified. | High |

---

## 3. Multimedia Assets & Video Placeholders

| Asset | Target Location | Description & Action Required | Priority |
| :--- | :--- | :--- | :---: |
| **Project 2 Video Demonstration** | `project_2/README.md` and Root `README.md` | Provide YouTube URL or video link demonstrating the Project 2 real-time 3D simulation (`sim_3d.py`) and live MPU-6050 serial tracking. | High |
| **Project 2 Simulation Screenshots** | `project_2/images/sim_3d_visualizer/` | Capture 3 to 4 high-resolution screenshots of `sim_3d.py`: (1) Default isometric attitude, (2) Pitch/Roll tilt with laser ground projection, (3) ZUPT active indicator, (4) Verbose HUD telemetry. | High |
| **Project 2 Physical Hardware Photos** | `project_2/images/hardware/` | Photographs of the Arduino Uno + MPU-6050 prototype wiring and D12/D13 status LED indicators. | Medium |

---

## 4. Flight Envelope Parameters & Research Aspirations

| Dimension | Scope in Project 2 | Technical Clarification Needed | Priority |
| :--- | :--- | :--- | :---: |
| **Physical Frame Target** | Currently simulated with $1.8\text{ m}$ virtual arm dimensions in `drone_model.py`. | Provide real target airframe specifications for flight testing (e.g., F450 wheelbase: 450 mm, target all-up-weight: 900–1200 g). | Medium |
| **Actuator & ESC Selection** | Stage 3 OpenSpec targets 250 Hz PWM on `PORTD` (pins D4–D7). | Specify target BLDC motors (e.g., 2212 920KV), ESC protocol (standard 1000–2000 $\mu$s PWM vs OneShot125), and LiPo battery chemistry (3S 11.1V vs 4S 14.8V). | High |
| **Radio Control (RC) Protocol** | Stage 2 OpenSpec targets `PCINT0..3` on pins D8–D11. | Confirm remote transmitter model (e.g., FlySky FS-i6, Radiolink, or custom Arduino RF) and pulse range. | High |
| **Positioning Fusion Sensors** | Section 4 of `project_2/README.md` documents quadratic divergence of IMU and recommends Kalman filtering. | Which external sensors are planned for navigation fusion? (Options: Barometer BMP280, ToF LiDAR VL53L1X, Optical Flow PMW3901, or GPS Ublox Neo-6M). | High |

---

## 5. Action Checklist for Round 2

- [ ] 1. Provide YouTube URL for Project 2 video demonstration.
- [ ] 2. Supply 3-4 screenshots for `project_2/images/sim_3d_visualizer/`.
- [ ] 3. Confirm HC-12 AT baud rate programming procedure.
- [ ] 4. Confirm target BLDC motor/ESC specifications for Stage 3 flight controller.
- [ ] 5. Confirm planned positioning sensor suite for Extended Kalman Filter (EKF) implementation.
