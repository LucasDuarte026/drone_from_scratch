import time
import os
import pytest
from odometry.telemetry_reader import TelemetryBuffer, TelemetryRecord

def test_telemetry_record_parsing():
    line = "12345,12.5,-5.2,180.0,0.05,-0.02,0.98"
    record = TelemetryRecord.from_line(line)
    assert record is not None
    assert record.timestamp == 12345
    assert record.roll == pytest.approx(12.5)
    assert record.pitch == pytest.approx(-5.2)
    assert record.yaw == pytest.approx(180.0)
    assert record.acc_x == pytest.approx(0.05)
    assert record.acc_y == pytest.approx(-0.02)
    assert record.acc_z == pytest.approx(0.98)
    assert record.raw_text == line

def test_telemetry_record_invalid_line():
    assert TelemetryRecord.from_line("corrupted,line,with,missing,data") is None
    assert TelemetryRecord.from_line("") is None
    assert TelemetryRecord.from_line("   \n") is None

def test_buffer_exact_ten_positions():
    buf = TelemetryBuffer(capacity=10, offline_timeout=1.0)
    assert len(buf.get_history()) == 0

    for i in range(15):
        line = f"{i*100},{float(i)},0.0,0.0,0.0,0.0,1.0"
        buf.push_line(line)
        assert len(buf.get_history()) <= 10

    history = buf.get_history()
    assert len(history) == 10
    # The oldest element in buffer should be index 5, latest index 14
    assert history[0].timestamp == 500
    assert history[-1].timestamp == 1400

    latest = buf.get_latest()
    assert latest is not None
    assert latest.timestamp == 1400

def test_buffer_online_offline_status():
    buf = TelemetryBuffer(capacity=10, offline_timeout=0.2)
    assert not buf.is_online()  # initially offline

    buf.push_line("1000,0.0,0.0,0.0,0.0,0.0,1.0")
    assert buf.is_online()

    time.sleep(0.25)
    assert not buf.is_online()

    buf.push_line("1250,1.0,0.0,0.0,0.0,0.0,1.0")
    assert buf.is_online()

def test_tail_file_reading(tmp_path):
    log_file = tmp_path / "telemetry.log"
    log_file.write_text("100,0.0,0.0,0.0,0.0,0.0,1.0\n")

    buf = TelemetryBuffer(capacity=10, offline_timeout=1.0)
    buf.poll_file(str(log_file))

    assert len(buf.get_history()) == 1
    assert buf.get_latest().timestamp == 100

    # Append new lines
    with open(str(log_file), "a") as f:
        f.write("200,1.0,2.0,3.0,0.0,0.0,1.0\n")
        f.write("300,4.0,5.0,6.0,0.0,0.0,1.0\n")

    buf.poll_file(str(log_file))
    assert len(buf.get_history()) == 3
    assert buf.get_latest().timestamp == 300
