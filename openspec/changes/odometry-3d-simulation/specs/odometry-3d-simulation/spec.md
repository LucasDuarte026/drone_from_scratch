# odometry-3d-simulation Specification

## ADDED Requirements

### Requirement: 10-Position Telemetry Buffer and Online/Offline Heartbeat
The telemetry ingestion subsystem SHALL maintain an exact 10-position circular FIFO buffer from `odometry/telemetry.log` and track online/offline status based on an activity timeout of 1.0 second.

#### Scenario: Telemetry buffering and FIFO eviction
- **GIVEN** a newly instantiated `TelemetryBuffer` with capacity 10
- **WHEN** 15 valid telemetry records are streamed sequentially
- **THEN** the buffer SHALL retain exactly the 10 most recent records in chronological order and discard the 5 oldest records.

#### Scenario: Online to Offline transition upon data stall
- **GIVEN** an active telemetry stream setting status to `Online`
- **WHEN** no new records arrive for greater than 1.0 second
- **THEN** the status SHALL transition to `Offline` and display a red indicator icon.

### Requirement: Fictional Quadcopter 3D Model Representation
The 3D renderer SHALL represent the quadcopter as a central cuboid fuselage, 4 diagonal cylindrical arms in an X-configuration, and 4 vertical cylindrical motors at the tips of the arms.

#### Scenario: Drone geometry composition
- **GIVEN** the 3D scene is rendered
- **WHEN** the drone model is drawn
- **THEN** the renderer SHALL draw 1 central cuboid, 4 cylindrical arm shafts, and 4 vertical cylindrical motor housings.

### Requirement: Orbital Camera Control and Isometric/Perspective Toggle
The system SHALL provide interactive orbital camera navigation around the drone, a Home reset button, and an Isometric/Perspective projection toggle button.

#### Scenario: Reset camera to isometric Home
- **GIVEN** the user has rotated, panned, or zoomed the camera
- **WHEN** the Home button is clicked
- **THEN** the camera orientation SHALL reset to azimuth $45^\circ$, elevation $30^\circ$, and default zoom distance.

#### Scenario: Toggle projection between Perspective and Isometric
- **GIVEN** the 3D viewport is rendered in perspective mode
- **WHEN** the projection toggle button is clicked
- **THEN** the camera projection SHALL switch to orthographic isometric projection.

### Requirement: Real-Time Stream Chat and Non-Manipulable Drone Attitude
The system SHALL reflect drone attitude strictly from the latest telemetry record without direct keyboard/mouse attitude control, and provide a collapsible floating chat displaying the 10-position stream.

#### Scenario: Telemetry attitude response
- **GIVEN** new telemetry lines containing Roll, Pitch, and Yaw arrive
- **WHEN** the 3D frame is rendered
- **THEN** the drone model SHALL rotate strictly according to the received angles without user manual attitude override.

#### Scenario: Toggle collapsible chat overlay
- **GIVEN** the floating telemetry chat is visible
- **WHEN** the user clicks the chat toggle button
- **THEN** the chat window SHALL collapse/expand, showing or hiding the 10 real-time records.

### Requirement: RViz IMU Axes Triad and Component Visibility Controls
The system SHALL render an RViz-style 3-axis coordinate triad (X: Red, Y: Green, Z: Blue) centered at the drone IMU origin $(0, 0, 0)$, and provide an interactive visibility menu allowing individual toggling of the central cuboid, arms, motors, IMU axes, and ground grid.

#### Scenario: RViz IMU coordinate triad rendering
- **GIVEN** the drone model is rendered and `show_imu_axes` is enabled
- **WHEN** the 3D attitude transformation is applied
- **THEN** the renderer SHALL draw orthogonal colored coordinate shafts and arrowheads at the body origin conforming to the body coordinate frame.

#### Scenario: Individual component visibility toggle
- **GIVEN** the visibility control panel or keyboard shortcuts (`1`–`5`)
- **WHEN** a component visibility toggle is triggered for fuselage, arms, motors, RViz axes, or grid
- **THEN** the renderer SHALL dynamically include or exclude only the selected component from the scene.

### Requirement: Graphite Instrument Layout, UITheme Design Tokens, and Hardware Anti-Aliasing
The 3D simulator SHALL organize its HUD into a 2-bar Graphite Instrument Layout governed by semantic `UITheme` tokens (free of saturated neon borders or ASCII pseudo-icons), format sample timestamps as relative session seconds, and enable hardware 4x MSAA and vector 2D anti-aliasing.

#### Scenario: UITheme semantic tokens and relative session time formatting
- **GIVEN** the `UITheme` design token class and `format_sample_time()` helper
- **WHEN** UI surfaces, interactive pills, and telemetry drawer timestamps (including 13-digit epoch timestamps) are rendered
- **THEN** all colors and High-DPI dimensions (`TOP_BAR_HEIGHT = 52`, `DOCK_PILL_HEIGHT = 24`, `DOCK_PILL_WIDTH = 74`) SHALL conform to `UITheme` tokens and timestamps SHALL display as relative session seconds (`0.0s`, `12.4s`).

#### Scenario: Hardware 4x MSAA and gfxdraw anti-aliased indicators
- **GIVEN** the OpenGL context and 2D HUD renderer initialization
- **WHEN** `init_gl()` and the status pill indicator execute
- **THEN** the viewport SHALL request 4x MSAA (`GL_MULTISAMPLEBUFFERS=1`, `GL_MULTISAMPLESAMPLES=4` with safe fallback) plus `GL_LINE_SMOOTH`, and render status dots using `pygame.gfxdraw.aacircle` and `pygame.gfxdraw.filled_circle`.

#### Scenario: English-only user interface standard
- **GIVEN** all 2D HUD controls, table headers, dock toggles, and serial worker status messages
- **WHEN** the simulator renders any text or status string
- **THEN** all labels and messages SHALL be strictly in English (`SAMPLE`, `TIME`, `ROLL`, `PITCH`, `YAW`, `Body`, `Arms`, `Motors`, `Axes`, `Grid`, `Reset`, `Calibrate`, `Perspective`/`Isometric`, `Telemetry`, `Port Free`).

### Requirement: Zero-Velocity Update (ZUPT), Noise Deadband, and Unconstrained Precision Odometry
The `InertialPositionEstimator` SHALL support unconstrained floating-point acceleration precision (`accel_decimals = None` by default, configurable), apply a default $0.05g$ stationary noise deadband (`accel_deadband_g = 0.05`), execute sample-synchronous integration strictly once per new serial telemetry packet, and apply Zero-Velocity Update (ZUPT) to extinguish residual velocity whenever the device remains stationary for 3 or more consecutive samples (~120 ms).

#### Scenario: Full-precision base Euler integration with noise deadband
- **GIVEN** an `InertialPositionEstimator` initialized with `accel_decimals=None`, `accel_deadband_g=0.05`, and `method="base_euler"`
- **WHEN** incoming body accelerations have magnitudes below $0.05g$ at rest
- **THEN** the estimator SHALL clamp linear world acceleration to exactly $0.0g$, preventing stationary drift.

#### Scenario: Zero-Velocity Update (ZUPT) velocity settling
- **GIVEN** an active movement that resulted in non-zero velocity $\vec{v} > 0$
- **WHEN** the drone stops moving and linear acceleration remains within the deadband for 3 or more consecutive samples
- **THEN** ZUPT SHALL exponentially decay residual velocity until $|v| < 0.01\text{ m/s}$ and clamp to exactly $0.00\text{ m/s}$.

### Requirement: Calibration Warning Modal & Operational State Machine
The 3D simulator SHALL display a prominent amber warning overlay during the 3.0s calibration routine instructing the user to keep the device completely motionless, pause kinematic integration during this window, and automatically tare post-calibration resting acceleration as the world zero reference upon completion.

#### Scenario: Calibration warning display and integration pause
- **GIVEN** IMU calibration is triggered via `R` or upon simulator launch
- **WHEN** the 3.0s calibration window is active
- **THEN** the HUD SHALL render the modal card with `IMU CALIBRATION IN PROGRESS`, `KEEP DRONE COMPLETELY MOTIONLESS`, a live countdown, and a progress bar, while kinematic position integration remains paused.

#### Scenario: Post-calibration auto-tare and position reset
- **GIVEN** the calibration countdown reaches 0.0s
- **WHEN** the calibration state machine transitions to inactive
- **THEN** the simulator SHALL reset 3D position to $(0, 0, H_0)$, zero velocity, and capture the current resting world acceleration as the tare bias reference.

### Requirement: Dual-Tier Telemetry and Diagnostics Logging
The simulation subsystem SHALL provide a dual-tier logger (`odometry/telemetry_logger.py`) outputting to stdout and `odometry/logs/sim_diagnostics.log`, supporting a high-level 1 Hz heartbeat in Standard mode and a per-sample diagnostic breakdown (including `dt`, `[CLAMP]`, and `[ZUPT ACTIVE]` tags) in Verbose mode (`-v`).

#### Scenario: Standard mode periodic heartbeat
- **GIVEN** the simulator running without the `-v` flag
- **WHEN** telemetry is streaming at 25 Hz
- **THEN** the logger SHALL emit operational lifecycle events and a compact 1 Hz heartbeat summarizing attitude, acceleration, velocity, and position.

#### Scenario: Verbose mode per-sample forensic stream
- **GIVEN** the simulator launched with `-v` or `--verbose`
- **WHEN** each telemetry sample arrives
- **THEN** the logger SHALL output a detailed diagnostic line including sample counter, inter-packet timing $\Delta t$, raw body acceleration, world acceleration, tare offset, deadband clamp status, velocity, and position.

