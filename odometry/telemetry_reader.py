"""Telemetry reader and circular buffer for real-time IMU visualization."""

import os
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional, List

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

        # Format 1: CSV format: timestamp_ms,roll,pitch,yaw,acc_x,acc_y,acc_z
        parts = [p.strip() for p in stripped.split(",") if p.strip()]
        if len(parts) >= 7:
            try:
                ts = int(float(parts[0]))
                r = float(parts[1])
                p = float(parts[2])
                y = float(parts[3])
                ax = float(parts[4])
                ay = float(parts[5])
                az = float(parts[6])
                return cls(
                    timestamp=ts,
                    roll=r,
                    pitch=p,
                    yaw=y,
                    acc_x=ax,
                    acc_y=ay,
                    acc_z=az,
                    raw_text=stripped
                )
            except ValueError:
                pass

        # Format 2: Space separated format
        parts_space = [p.strip() for p in stripped.split() if p.strip()]
        if len(parts_space) >= 7:
            try:
                ts = int(float(parts_space[0]))
                r = float(parts_space[1])
                p = float(parts_space[2])
                y = float(parts_space[3])
                ax = float(parts_space[4])
                ay = float(parts_space[5])
                az = float(parts_space[6])
                return cls(
                    timestamp=ts,
                    roll=r,
                    pitch=p,
                    yaw=y,
                    acc_x=ax,
                    acc_y=ay,
                    acc_z=az,
                    raw_text=stripped
                )
            except ValueError:
                pass

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

    def push_line(self, line: str) -> Optional[TelemetryRecord]:
        """Parse and append a raw line into the buffer."""
        record = TelemetryRecord.from_line(line)
        if record is not None:
            self._buffer.append(record)
            self._last_received_time = time.monotonic()
            return record
        return None

    def is_online(self) -> bool:
        """Return True if new data arrived within offline_timeout seconds."""
        if self._last_received_time == 0.0:
            return False
        return (time.monotonic() - self._last_received_time) < self.offline_timeout

    def get_latest(self) -> Optional[TelemetryRecord]:
        """Return the most recent record or None if buffer is empty."""
        if self._buffer:
            return self._buffer[-1]
        return None

    def get_history(self) -> List[TelemetryRecord]:
        """Return a snapshot list of current buffer contents (oldest to newest)."""
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
