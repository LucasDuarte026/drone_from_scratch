"""Telemetry reader, circular buffer and serial ingestion for real-time IMU visualization."""

import os
import re
import time
import glob
import threading
from collections import deque
from dataclasses import dataclass
from typing import Optional, List


DEFAULT_PIO_INI = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "platformio.ini"))


def read_pio_port(pio_ini_path: str = DEFAULT_PIO_INI) -> str:
    """Read the default serial port (`porta = ...`) from platformio.ini."""
    if not os.path.exists(pio_ini_path):
        return "/dev/ttyUSB0"
    try:
        with open(pio_ini_path, "r", encoding="utf-8") as f:
            for line in f:
                m = re.match(r"^\s*porta\s*=\s*(\S+)", line)
                if m:
                    return m.group(1).strip()
    except OSError:
        pass
    return "/dev/ttyUSB0"


def update_pio_port(new_port: str, pio_ini_path: str = DEFAULT_PIO_INI) -> bool:
    """Update `porta = <new_port>` in platformio.ini when a valid port is selected or detected."""
    if not new_port or not os.path.exists(pio_ini_path):
        return False
    try:
        with open(pio_ini_path, "r", encoding="utf-8") as f:
            content = f.read()
        new_content, count = re.subn(
            r"^(\s*porta\s*=\s*)\S+",
            rf"\g<1>{new_port}",
            content,
            flags=re.MULTILINE
        )
        if count > 0:
            if new_content != content:
                with open(pio_ini_path, "w", encoding="utf-8") as f:
                    f.write(new_content)
            return True
    except OSError:
        pass
    return False


def find_serial_port(preferred_port: Optional[str] = None) -> Optional[str]:
    """Auto-detect connected Arduino USB serial port, prioritizing preferred_port and ttyUSB0/ttyUSB1."""
    if preferred_port and os.path.exists(preferred_port):
        return preferred_port
    for candidate in ("/dev/ttyUSB0", "/dev/ttyUSB1"):
        if os.path.exists(candidate):
            return candidate
    tty_usb = sorted(glob.glob("/dev/ttyUSB*"))
    if tty_usb:
        return tty_usb[0]
    tty_acm = sorted(glob.glob("/dev/ttyACM*"))
    if tty_acm:
        return tty_acm[0]
    by_id = glob.glob("/dev/serial/by-id/*")
    if by_id:
        return by_id[0]
    return None


@dataclass
class TelemetryRecord:
    timestamp: int
    roll: float
    pitch: float
    yaw: float
    acc_x: float
    acc_y: float
    acc_z: float
    raw_text: str

    @classmethod
    def from_line(cls, line: str) -> Optional["TelemetryRecord"]:
        """Parse a telemetry string line into a TelemetryRecord."""
        stripped = line.strip()
        if not stripped:
            return None

        # Try comma-separated first, then semicolon, then whitespace
        delimiters = [",", ";", None]
        for sep in delimiters:
            if sep is not None:
                tokens = [p.strip() for p in stripped.split(sep) if p.strip()]
            else:
                tokens = [p.strip() for p in stripped.split() if p.strip()]

            if len(tokens) >= 3:
                try:
                    vals = [float(t) for t in tokens]
                    now_ms = int(time.time() * 1000)

                    if len(vals) == 3:
                        # Format: roll, pitch, yaw (e.g. Serial Plotter output from Arduino)
                        return cls(
                            timestamp=now_ms,
                            roll=vals[0],
                            pitch=vals[1],
                            yaw=vals[2],
                            acc_x=0.0,
                            acc_y=0.0,
                            acc_z=1.0,
                            raw_text=stripped
                        )
                    elif len(vals) == 4:
                        # Format: timestamp, roll, pitch, yaw
                        return cls(
                            timestamp=int(vals[0]),
                            roll=vals[1],
                            pitch=vals[2],
                            yaw=vals[3],
                            acc_x=0.0,
                            acc_y=0.0,
                            acc_z=1.0,
                            raw_text=stripped
                        )
                    elif len(vals) == 6:
                        # Format: roll, pitch, yaw, ax, ay, az
                        return cls(
                            timestamp=now_ms,
                            roll=vals[0],
                            pitch=vals[1],
                            yaw=vals[2],
                            acc_x=vals[3],
                            acc_y=vals[4],
                            acc_z=vals[5],
                            raw_text=stripped
                        )
                    elif len(vals) >= 7:
                        # Format: timestamp, roll, pitch, yaw, ax, ay, az (canonical log format)
                        return cls(
                            timestamp=int(vals[0]),
                            roll=vals[1],
                            pitch=vals[2],
                            yaw=vals[3],
                            acc_x=vals[4],
                            acc_y=vals[5],
                            acc_z=vals[6],
                            raw_text=stripped
                        )
                except ValueError:
                    continue

        return None


class TelemetryBuffer:
    """Maintains a fixed-size FIFO buffer (10 items) and tracks online/offline state."""

    def __init__(self, capacity: int = 10, offline_timeout: float = 1.0):
        self.capacity = capacity
        self.offline_timeout = offline_timeout
        self._buffer: deque[TelemetryRecord] = deque(maxlen=capacity)
        self._last_received_time: float = 0.0
        self._file_offset: int = 0
        self._last_file_inode: Optional[int] = None
        self._lock = threading.Lock()

    def push_line(self, line: str) -> Optional[TelemetryRecord]:
        """Parse and append a raw line into the buffer."""
        record = TelemetryRecord.from_line(line)
        if record is not None:
            with self._lock:
                self._buffer.append(record)
                self._last_received_time = time.monotonic()
            return record
        return None

    def is_online(self) -> bool:
        """Return True if new data arrived within offline_timeout seconds."""
        with self._lock:
            if self._last_received_time == 0.0:
                return False
            return (time.monotonic() - self._last_received_time) < self.offline_timeout

    def get_latest(self) -> Optional[TelemetryRecord]:
        """Return the most recent record or None if buffer is empty."""
        with self._lock:
            if self._buffer:
                return self._buffer[-1]
            return None

    def get_history(self) -> List[TelemetryRecord]:
        """Return a snapshot list of current buffer contents (oldest to newest)."""
        with self._lock:
            return list(self._buffer)

    def poll_file(self, filepath: str) -> int:
        """
        Poll the file for newly appended lines (tailing).
        Returns the number of new records successfully parsed.
        """
        if not os.path.exists(filepath):
            return 0

        try:
            stat = os.stat(filepath)
            # If file was truncated or replaced (new inode or smaller size)
            if self._last_file_inode != stat.st_ino or stat.st_size < self._file_offset:
                self._last_file_inode = stat.st_ino
                self._file_offset = 0

            with open(filepath, "r", encoding="utf-8", errors="replace") as f:
                f.seek(self._file_offset)
                new_lines = f.readlines()
                self._file_offset = f.tell()

            count = 0
            for line in new_lines:
                rec = self.push_line(line)
                if rec is not None:
                    count += 1
            return count
        except (OSError, IOError):
            return 0


class SerialTelemetryWorker:
    """Background worker daemon thread reading serial port directly into TelemetryBuffer."""

    def __init__(
        self,
        buffer: TelemetryBuffer,
        port: Optional[str] = None,
        baudrate: int = 57600,
        log_file: Optional[str] = None,
        pio_ini_path: str = DEFAULT_PIO_INI,
        auto_scan: Optional[bool] = None
    ):
        self.buffer = buffer
        self.pio_ini_path = pio_ini_path
        self.auto_scan = (port is None) if auto_scan is None else auto_scan
        self.port = port or read_pio_port(self.pio_ini_path)
        self.baudrate = baudrate
        self.log_file = log_file
        self.running = False
        self.user_connected = True
        self._reconnect_requested = False
        self._thread: Optional[threading.Thread] = None
        self._connected = False
        self._error_msg: Optional[str] = None
        self._active_port: Optional[str] = None
        self._trying_port: Optional[str] = self.port
        self._scan_idx: int = 0
        self._log_fp = None
        self._cmd_lock = threading.Lock()
        self._pending_commands: deque[bytes] = deque()

    def start(self):
        if self.running:
            return
        self.running = True
        if self.log_file:
            log_dir = os.path.dirname(os.path.abspath(self.log_file))
            os.makedirs(log_dir, exist_ok=True)
            self._log_fp = open(self.log_file, "a", encoding="utf-8")
        self._thread = threading.Thread(target=self._run, daemon=True, name="SerialTelemetryWorker")
        self._thread.start()

    def stop(self):
        self.running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._log_fp:
            try:
                self._log_fp.close()
            except Exception:
                pass
            self._log_fp = None

    def set_user_connected(self, connected: bool):
        """Connect or disconnect serial port from UI to free/claim /dev/ttyUSBx."""
        self.user_connected = bool(connected)
        if not self.user_connected:
            self._connected = False
            self._active_port = None
            self._error_msg = "Porta liberada pelo usuário"
        else:
            self._reconnect_requested = True

    def toggle_user_connected(self) -> bool:
        """Toggle user_connected state and return new state."""
        self.set_user_connected(not self.user_connected)
        return self.user_connected

    def select_port(self, port: Optional[str] = None, auto_scan: bool = False):
        """Select a specific serial port or enable auto-scan between USB0/USB1 and sync platformio.ini."""
        self.auto_scan = auto_scan
        if port:
            self.port = port
            self._trying_port = port
            update_pio_port(port, self.pio_ini_path)
        self.user_connected = True
        self._reconnect_requested = True

    def send_command(self, cmd: str):
        """Queue a serial command string (e.g. 'C\\n') to be written to the Arduino."""
        data = cmd.encode("utf-8") if isinstance(cmd, str) else bytes(cmd)
        with self._cmd_lock:
            self._pending_commands.append(data)

    def request_calibration(self):
        """Queue the 'C\\n' recalibration command for the Arduino IMU firmware."""
        self.send_command("C\n")

    def has_pending_commands(self) -> bool:
        with self._cmd_lock:
            return len(self._pending_commands) > 0

    def pop_pending_command(self) -> Optional[bytes]:
        with self._cmd_lock:
            if self._pending_commands:
                return self._pending_commands.popleft()
            return None

    def is_connected(self) -> bool:
        return self._connected and self.user_connected

    @property
    def active_port(self) -> Optional[str]:
        return self._active_port or self._trying_port or self.port

    @property
    def status_description(self) -> str:
        if not self.user_connected:
            return f"DESCONECTADO ({os.path.basename(self.port or 'USB')} LIVRE)"
        if self._connected:
            return f"{self.active_port} @ {self.baudrate}"
        trying = self._trying_port or self.port or "ttyUSB0/USB1"
        return f"Tentando {trying}..."

    def _next_candidate_port(self) -> str:
        if not self.auto_scan:
            return self.port
        candidates = []
        for p in (self.port, "/dev/ttyUSB0", "/dev/ttyUSB1"):
            if p and p not in candidates:
                candidates.append(p)
        for p in candidates:
            if os.path.exists(p):
                return p
        chosen = candidates[self._scan_idx % len(candidates)]
        self._scan_idx += 1
        return chosen

    def _run(self):
        try:
            import serial
        except ImportError:
            self._connected = False
            self._error_msg = "pyserial not installed"
            return

        while self.running:
            if not self.user_connected:
                self._connected = False
                self._active_port = None
                time.sleep(0.15)
                continue

            self._reconnect_requested = False
            target_port = self._next_candidate_port()
            self._trying_port = target_port

            if not target_port or not os.path.exists(target_port):
                self._connected = False
                self._active_port = None
                self._error_msg = f"Tentando {target_port or 'ttyUSB0'}..."
                time.sleep(0.6)
                continue

            try:
                self._active_port = target_port
                with serial.Serial(target_port, self.baudrate, timeout=0.25) as ser:
                    self._connected = True
                    self._error_msg = None
                    self.port = target_port
                    update_pio_port(target_port, self.pio_ini_path)
                    ser.reset_input_buffer()

                    while self.running and self.user_connected and not self._reconnect_requested:
                        cmd_bytes = self.pop_pending_command()
                        if cmd_bytes is not None:
                            ser.write(cmd_bytes)
                            ser.flush()
                            ser.reset_input_buffer()

                        raw_line = ser.readline()
                        if not raw_line:
                            continue
                        line_str = raw_line.decode("utf-8", errors="replace").strip()
                        if not line_str:
                            continue

                        record = self.buffer.push_line(line_str)
                        if record and self._log_fp:
                            self._log_fp.write(
                                f"{record.timestamp},{record.roll:.2f},{record.pitch:.2f},{record.yaw:.2f},"
                                f"{record.acc_x:.3f},{record.acc_y:.3f},{record.acc_z:.3f}\n"
                            )
                            self._log_fp.flush()
                self._connected = False
            except Exception as e:
                self._connected = False
                self._error_msg = str(e)
                time.sleep(0.6)
