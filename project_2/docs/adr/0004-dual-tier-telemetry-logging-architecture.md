# ADR-0004: Dual-Tier Telemetry Logging Architecture (Standard vs Verbose)

- **Status:** Accepted
- **Date:** 2026-10-28
- **Context:** `project_2/odometry/telemetry_logger.py`, `project_2/odometry/sim_3d.py`

---

## 1. Context & Problem Statement

Diagnosing sensor fusion, decimation jitter, deadband clamping, and ZUPT state transitions during live 3D simulation was hindered by lack of visibility. Standard console prints at $25\text{ Hz}$ flooded the terminal, obscuring fatal errors and degrading GUI framerate. Conversely, silent execution hid whether runaway velocity was caused by incorrect gravity removal, negative deceleration truncation, or serial baud buffer corruption.

A structured logging system was required that could:
1. Provide a clean, human-readable 1 Hz operational heartbeat by default.
2. Provide granular, per-frame mathematical diagnostics when invoked with `--verbose` (`-v`).
3. Persist diagnostic dumps to disk (`odometry/logs/sim_diagnostics.log`) without impacting the 60 FPS PyOpenGL rendering loop.

---

## 2. Alternatives Considered

1. **Python Standard `logging` Module with Default Formatter:**
   - *Pros:* Built-in standard library.
   - *Cons:* Multi-line formatters and synchronous disk I/O on every frame induce rendering stutter; difficult to dynamically toggle between 1 Hz summary heartbeats and 25 Hz micro-diagnostics without complex filters.
2. **Raw `print()` Statements with Debug Flags:**
   - *Pros:* Zero overhead.
   - *Cons:* No persistent log rotation; no timestamps; no causal tracing.
3. **Dedicated `TelemetryLogger` Class with Dual Operational Tiers (Chosen):**
   - *Pros:* Decouples 1 Hz human heartbeat from high-frequency per-frame diagnostic logs; writes structured diagnostics to `odometry/logs/sim_diagnostics.log`; integrates cleanly with simulator CLI `--verbose`.

---

## 3. Decision

1. **Architecture (`odometry/telemetry_logger.py`):**
   - Implemented `TelemetryLogger(verbose=False, log_file="odometry/logs/sim_diagnostics.log")`.
   - **Tier 1: Standard Mode (`verbose=False`):**
     - Emits a concise single-line heartbeat once every 1.0 second:
       `[ODOM HEARTBEAT] Euler: (R:+1.2, P:-0.4, Y:45.0) | Pos: (X:0.00, Y:0.00, Z:0.00) | Vel: (0.00, 0.00, 0.00) | Acc: (0.00, 0.00, 0.00) | Frames: 250`
   - **Tier 2: Verbose Mode (`verbose=True` / `-v` CLI flag):**
     - Logs every single $25\text{ Hz}$ frame with microsecond timestamps, raw body acceleration, world acceleration, integration step `dt`, deadband state (`[CLAMP]`), and ZUPT state (`[ZUPT ACTIVE]`):
       `[2026-10-28 15:30:12.124] FRAME #250 | dt=0.0401s | Euler=(1.20, -0.40, 45.00) | Acc_b=(0.012, -0.005, 0.998) | Acc_w=(0.000, 0.000, 0.000) [CLAMP] | Vel=(0.000, 0.000, 0.000) [ZUPT ACTIVE] | Pos=(0.000, 0.000, 0.000)`
2. **File Persistence:**
   - Verbose traces are streamed to `odometry/logs/sim_diagnostics.log`, ensuring forensic analysis can occur post-flight without terminal flood.

---

## 4. Consequences & Trade-offs

### Positive
- Clear real-time visibility during calibration, motionless rest, and aggressive translational movements.
- Live verification of ZUPT triggering and noise deadband clipping.
- Persistent file logs allow reviewing drift trajectories and verifying numerical stability offline.

### Negative / Trade-offs
- In verbose mode, disk I/O generates $\approx 100\text{ KB/min}$ of log data. Log rotation or manual cleanup is recommended for extended sessions.
