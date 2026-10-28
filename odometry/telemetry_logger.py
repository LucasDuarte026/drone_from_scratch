"""
Telemetry and Diagnostics Logger for Drone 3D Odometry Simulator.

Provides dual-tier logging:
- Standard Mode: Human-readable lifecycle events and a 1 Hz heartbeat of the system.
- Verbose Mode (-v / --verbose): In-depth, per-sample physical diagnostic stream
  capturing raw body acceleration, attitude rotation, world linear acceleration,
  tare offsets, deadband clamping, timing jitter (dt), and kinematics integration.

Outputs simultaneously to stdout (with ANSI color highlighting) and a persistent
diagnostic log file for post-flight or live agent forensic inspection.
"""

import os
import sys
import time
from typing import Optional, Tuple


class TelemetryLogger:
    # ANSI color tokens for terminal readability
    CLR_RESET = "\033[0m"
    CLR_MUTED = "\033[90m"
    CLR_INFO = "\033[36m"     # Cyan
    CLR_DEBUG = "\033[34m"    # Blue
    CLR_WARN = "\033[33m"     # Yellow
    CLR_SUCCESS = "\033[32m"  # Green
    CLR_ERROR = "\033[31m"    # Red
    CLR_BOLD = "\033[1m"

    def __init__(
        self,
        verbose: bool = False,
        log_file: Optional[str] = "odometry/logs/sim_diagnostics.log",
        heartbeat_interval: float = 1.0,
        enable_stdout: bool = True
    ):
        self.verbose: bool = bool(verbose)
        self.log_file: Optional[str] = log_file
        self.heartbeat_interval: float = float(heartbeat_interval)
        self.enable_stdout: bool = enable_stdout

        self._last_heartbeat_mono: float = 0.0
        self._sample_counter: int = 0
        self._file_handle = None
        self._is_tty: bool = enable_stdout and hasattr(sys.stdout, "isatty") and sys.stdout.isatty()

        if self.log_file:
            os.makedirs(os.path.dirname(os.path.abspath(self.log_file)), exist_ok=True)
            try:
                self._file_handle = open(self.log_file, "a", encoding="utf-8", buffering=1)
                banner = (
                    f"\n--- SESSION START [{time.strftime('%Y-%m-%d %H:%M:%S')}] "
                    f"mode={'VERBOSE' if self.verbose else 'STANDARD'} ---\n"
                )
                self._file_handle.write(banner)
            except OSError as e:
                print(f"[WARN] Failed to open diagnostics log file {self.log_file}: {e}")

    def close(self):
        if self._file_handle:
            try:
                self._file_handle.write(f"--- SESSION END [{time.strftime('%Y-%m-%d %H:%M:%S')}] ---\n")
                self._file_handle.close()
            except OSError:
                pass
            self._file_handle = None

    def _format_timestamp(self) -> str:
        t = time.time()
        msec = int((t % 1) * 1000)
        return f"{time.strftime('%H:%M:%S', time.localtime(t))}.{msec:03d}"

    def log_event(self, level: str, message: str):
        """Log a high-level lifecycle or operational event."""
        ts = self._format_timestamp()
        lvl = level.upper()

        # Terminal colored formatting
        if self._is_tty:
            color = self.CLR_INFO
            if lvl in ("WARN", "WARNING"):
                color = self.CLR_WARN
            elif lvl in ("ERROR", "CRITICAL"):
                color = self.CLR_ERROR
            elif lvl in ("SUCCESS", "OK"):
                color = self.CLR_SUCCESS
            elif lvl == "DEBUG":
                color = self.CLR_DEBUG
            line_term = f"{self.CLR_MUTED}[{ts}]{self.CLR_RESET} {color}[{lvl:<5}]{self.CLR_RESET} {message}"
        else:
            line_term = f"[{ts}] [{lvl:<5}] {message}"

        if self.enable_stdout:
            print(line_term, flush=True)

        if self._file_handle:
            clean_line = f"[{ts}] [{lvl:<5}] {message}\n"
            self._file_handle.write(clean_line)

    def log_sample(
        self,
        dt: float,
        roll: float,
        pitch: float,
        yaw: float,
        acc_body: Tuple[float, float, float],
        acc_world_raw: Tuple[float, float, float],
        acc_world_bias: Tuple[float, float, float],
        lin_acc: Tuple[float, float, float],
        vel: Tuple[float, float, float],
        pos: Tuple[float, float, float],
        deadband: float = 0.05,
        clamped: bool = False,
        zupt_active: bool = False,
    ):
        """
        Record a telemetry sample.
        In Verbose mode, logs per-sample details on every incoming packet.
        In Standard mode, accumulates and triggers a 1.0s heartbeat.
        """
        self._sample_counter += 1
        now_mono = time.monotonic()
        ts = self._format_timestamp()

        bx, by, bz = acc_body
        wx, wy, wz = acc_world_raw
        tx, ty, tz = acc_world_bias
        ax, ay, az = lin_acc
        vx, vy, vz = vel
        px, py, pz = pos

        # In Verbose mode, print complete line for every single sample
        if self.verbose:
            clamp_tag = f" [CLAMP <{deadband:0.2f}g]" if clamped else ""
            zupt_tag = " [ZUPT ACTIVE]" if zupt_active else ""
            msg = (
                f"[#{self._sample_counter:05d} dt={dt*1000.0:4.1f}ms] "
                f"Att: R={roll:+6.2f}° P={pitch:+6.2f}° Y={yaw:+6.2f}° | "
                f"BodyAcc: ({bx:+5.2f}, {by:+5.2f}, {bz:+5.2f})g | "
                f"WorldRaw: ({wx:+5.2f}, {wy:+5.2f}, {wz:+5.2f})g | "
                f"LinAcc: ({ax:+5.2f}, {ay:+5.2f}, {az:+5.2f})g{clamp_tag} | "
                f"Vel: ({vx:+5.2f}, {vy:+5.2f}, {vz:+5.2f})m/s{zupt_tag} | "
                f"Pos: ({px:+5.2f}, {py:+5.2f}, {pz:+5.2f})m"
            )
            self.log_event("DEBUG", msg)

        # Standard 1 Hz Heartbeat
        if now_mono - self._last_heartbeat_mono >= self.heartbeat_interval:
            self._last_heartbeat_mono = now_mono
            hb_msg = (
                f"[HEARTBEAT] #{self._sample_counter} | "
                f"R: {roll:+5.1f}° P: {pitch:+5.1f}° Y: {yaw:+5.1f}° | "
                f"Acc(b): ({bx:+4.2f}, {by:+4.2f}, {bz:+4.2f})g | "
                f"LinAcc(w): ({ax:+4.2f}, {ay:+4.2f}, {az:+4.2f})g | "
                f"Vel: ({vx:+5.2f}, {vy:+5.2f}, {vz:+5.2f})m/s | "
                f"Pos: ({px:+5.2f}, {py:+5.2f}, {pz:+4.2f})m"
            )
            if not self.verbose:
                self.log_event("INFO", hb_msg)
