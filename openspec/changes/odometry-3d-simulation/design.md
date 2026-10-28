# Technical Design: Odometry 3D Simulation & C Driver Tester

## Context & Architecture
- **Rendering Stack:** Python 3.12, Pygame 2.6, PyOpenGL 3.1.
- **Hardware Interface:** C binary (`odometry/tester`) interacting with Arduino serial port (`/dev/ttyACM*`, `/dev/ttyUSB*` @ 57600 baud) or generating autonomous synthetic harmonic flight kinematics.
- **Data Conduit:** File-based real-time tail polling on `odometry/telemetry.log`.
- **Testing Governance:** Strict TDD using Pytest in `.venv`, with dual-role builder/tester discipline.

## Goals & Non-Goals
- **Goals:**
  - Provide a visual 3D verification harness for the quadcopter's attitude estimation math.
  - Draw a fictional drone comprising: central cuboid + 4 cylindrical arms + 4 vertical cylindrical motors.
  - Non-manipulable drone (attitude 100% driven by telemetry file).
  - Orbital camera controls, Home button (zero isometric position), and Isometric/Perspective toggle.
  - Top-left Online (green) / Offline (red) status indicator with 1.0s timeout.
  - Collapsible floating chat window showing the 10-position live telemetry buffer.
  - C driver tester with serial communication and user test hook slots (`// TODO`).
- **Non-Goals:**
  - Flight path planner or GPS waypoint tracking (deferred to autonomous navigation phases).
  - Direct PWM actuation of real ESCs from Python (safety boundary).

## Technical Decisions
1. **Pygame + PyOpenGL Pipeline with Hardware Anti-Aliasing:**
   - Provides native hardware-accelerated OpenGL 3D rendering with GLU quadrics for cylinders and cuboids, coupled with 2D Pygame surface rendering for the instrument HUD.
   - **4x MSAA & Line Smoothing (`context7` PyOpenGL / Pygame):** Requests `pygame.GL_MULTISAMPLEBUFFERS = 1` and `pygame.GL_MULTISAMPLESAMPLES = 4` before `pygame.display.set_mode()` (with automatic fallback if unsupported by the GPU driver), and enables `glEnable(GL_MULTISAMPLE)`, `glEnable(GL_LINE_SMOOTH)`, and `glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)`.
   - **2D Vector Anti-Aliasing (`pygame.gfxdraw`):** Uses `gfxdraw.aacircle` and `gfxdraw.filled_circle` to render smooth status indicator dots with a subtle translucent halo.
2. **Anti-AI Slop Graphite Design System (`UITheme`):**
   - Guided by the **Impeccable Anti-AI Slop principles** (`UI_Component_Registries_and_Design_Systems.md`) and **UI/UX Architect token architecture**:
     - Eliminates saturated neon borders (`1px cyan/green`), `bold=True` on every font, and ASCII pseudo-icons (`[✓]`, `[ ]`, `⌂`, `▼`).
     - Centralizes semantic color tokens in `UITheme`: deep graphite canvas `(13, 15, 18)`, translucent carbon bars `(20, 23, 28, 235)`, subtle borders `(46, 52, 64)`, soft dividers `(36, 41, 50)`, muted uppercase labels `(138, 148, 166)`, high-contrast primary values `(240, 244, 248)`, and distinct interactive pill states (`BTN_IDLE`, `BTN_HOVER`, `BTN_ACTIVE` slate-blue `(38, 92, 168)`).
     - Consolidates floating boxes into a **2-Bar Instrument Layout (English-Only UI Standard)**:
       - **Top Instrument Bar (`52px` height):** Left clickable status & serial-release pill (`Online · ttyUSBx (57600) [S]`, `Port Free · ttyUSBx [S]`, `Online · Simulation`, `Offline · No Signal`), center 3-column Attitude Readout (`ROLL`, `PITCH`, `YAW`), and right action pills (`Reset`, `Calibrate`, `Perspective`/`Isometric`, `Telemetry`).
       - **Bottom Centered Layer Dock (`24px` compact pill height, `74px` width):** 5 compact pill toggles (`[1] Body`, `[2] Arms`, `[3] Motors`, `[4] Axes`, `[5] Grid`) with hover and active states, plus bottom navigation hint (`Left Drag: Orbit · Right Drag: Pan · Scroll: Zoom`).
       - **Right-Hand Telemetry Drawer (`460px` width):** Structured 5-column tabular log view (`SAMPLE`, `TIME`, `ROLL`, `PITCH`, `YAW`) with latest-row highlight (`PANEL_ROW_LATEST`) and relative session time formatting (`format_sample_time()`).
     - **High-DPI Typography Hierarchy:** Native Linux geometric sans (`DejaVu Sans,Ubuntu`, `14px`–`17px`) for UI labels and tabular monospace (`DejaVu Sans Mono,Ubuntu Mono`, `22px` bold for primary readouts, `16px` for telemetry table) to prevent horizontal jitter at 25 Hz.
3. **File-Based Telemetry Tail Buffer & Relative Time Normalization:**
   - Inode-aware polling on `odometry/logs/telemetry.log` avoids full file re-reads and handles file truncations gracefully.
   - Fixed-size `collections.deque(maxlen=10)` enforces strict FIFO capacity.
   - `format_sample_time()` normalizes both zero-based simulation timestamps and 13-digit Unix epoch timestamps into readable session seconds (`12.4s`).
4. **Canonical C Math Reuse (`code/include/imu_attitude.h`):**
   - `odometry/tester.c` compiles with `-I../code/include` to directly `#include "imu_attitude.h"`, sharing the exact `imu_filter_rate()`, `imu_calc_euler_rates()`, `imu_calc_acc_angles()`, and `imu_fuse_angle()` implementation with the ATmega328P firmware.
5. **Inertial Position Estimator Architecture & Evolutionary Method Catalog (`odometry/drone_model.py`):**
   - **Sample-Synchronous Integration:** `sim_3d.py` invokes `pos_estimator.update()` strictly once per newly arrived telemetry sample (`latest.timestamp != self._last_ingested_ts`) rather than on every 60 FPS render frame, preventing duplicate-sample integration artifacts.
   - **Configurable Decimal Precision (`accel_decimals: int = 2`):** Both body-frame inputs `(acc_x, acc_y, acc_z)` and world-frame linear accelerations `(aw_x, aw_y, aw_z)` are rounded to `round(val, self.accel_decimals)` (default `2` decimal places = `0.01g` resolution).
   - **Active Base Model (`method="base_euler"`, `accel_deadband_g=0.05`, `vel_damping=1.0`):**
     - Rotates body acceleration into the world frame via $R(\phi, \theta, \psi)$, subtracts $(0, 0, 1.0g)$, rounds to `accel_decimals`, applies default deadband (`0.05g` to reject stationary MEMS sensor noise and thermal drift), and integrates velocity $\vec{v}[k] = \vec{v}[k-1] + \vec{a}_{world}[k] \cdot 9.81 \cdot \Delta t$ and position $\vec{p}[k] = \vec{p}[k-1] + \vec{v}[k] \cdot \Delta t$.
   - **Dual-Tier Telemetry & Diagnostics Logging (`odometry/telemetry_logger.py`):**
     - **Standard Mode (Default):** Logs operational events and a 1 Hz heartbeat containing attitude, body acceleration, world linear acceleration, velocity, and position.
     - **Verbose Mode (`-v` / `--verbose`):** Logs every single incoming telemetry packet with detailed diagnostics (sample count, dt, raw body acc, world raw, bias tare, deadband clamping tag, net linear acc, velocity, position) to stdout and `odometry/logs/sim_diagnostics.log`.
   - **State-Machine Calibration & Tare Management:**
     - Pauses kinematic integration for 3.0s during Arduino calibration routine, then automatically zeroes 3D position and captures post-calibration resting world acceleration as tare reference.
   - **Preserved Evolutionary Methods (Saved in `InertialPositionEstimator` for Future Testing):**
     - `method="trapezoidal"`: Integrates velocity and position using the trapezoidal rule $\frac{a_{k-1} + a_k}{2}\Delta t$ and $\frac{v_{k-1} + v_k}{2}\Delta t$ to reduce first-order phase truncation error during hand-held acceleration/braking lobes.
     - `method="zupt"`: Combines trapezoidal integration with configurable `accel_deadband_g` (e.g., `0.04g`), `vel_damping` (e.g., `0.95`), adaptive world bias tare, and Zero-Velocity Update (ZUPT) clamping when acceleration remains within the deadband for $N$ consecutive samples.
   - **3-Row Odometry Readout Card (`POS 3D`, `VEL 3D`, `LIN ACC`):** Displays 3D position `(X, Y, Z)` in meters, 3D linear velocity `(Vx, Vy, Vz)` in $\text{m/s}$, and linear acceleration `(Ax, Ay, Az)` in $g$ in the top-left instrument panel.
