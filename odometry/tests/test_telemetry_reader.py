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

def test_telemetry_record_serialplot_3values():
    # 3 values space-separated: "Roll Pitch Yaw"
    line = "15.40 -3.25 45.10"
    record = TelemetryRecord.from_line(line)
    assert record is not None
    assert record.roll == pytest.approx(15.40)
    assert record.pitch == pytest.approx(-3.25)
    assert record.yaw == pytest.approx(45.10)
    assert record.acc_z == pytest.approx(1.0)

    # 3 values comma-separated: "Roll,Pitch,Yaw"
    line_comma = "15.40,-3.25,45.10"
    record_comma = TelemetryRecord.from_line(line_comma)
    assert record_comma is not None
    assert record_comma.roll == pytest.approx(15.40)
    assert record_comma.pitch == pytest.approx(-3.25)
    assert record_comma.yaw == pytest.approx(45.10)

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


def test_telemetry_record_6values_serial():
    line = "12.50 -4.20 90.00 0.125 -0.250 0.985"
    rec = TelemetryRecord.from_line(line)
    assert rec is not None
    assert rec.roll == pytest.approx(12.50)
    assert rec.pitch == pytest.approx(-4.20)
    assert rec.yaw == pytest.approx(90.00)
    assert rec.acc_x == pytest.approx(0.125)
    assert rec.acc_y == pytest.approx(-0.250)
    assert rec.acc_z == pytest.approx(0.985)


def test_platformio_ini_port_sync(tmp_path):
    from odometry.telemetry_reader import read_pio_port, update_pio_port

    pio_ini = tmp_path / "platformio.ini"
    pio_ini.write_text(
        "[platformio]\ndefault_envs = uno\n\n[common]\nporta = /dev/ttyUSB0\n\n[env:uno]\nupload_port = ${common.porta}\n"
    )

    assert read_pio_port(str(pio_ini)) == "/dev/ttyUSB0"

    ok = update_pio_port("/dev/ttyUSB1", str(pio_ini))
    assert ok is True
    assert read_pio_port(str(pio_ini)) == "/dev/ttyUSB1"
    content = pio_ini.read_text()
    assert "porta = /dev/ttyUSB1" in content
    assert "upload_port = ${common.porta}" in content


def test_serial_worker_connect_disconnect_control():
    from odometry.telemetry_reader import SerialTelemetryWorker

    buf = TelemetryBuffer(capacity=10)
    worker = SerialTelemetryWorker(buf, port="/dev/ttyUSB0", baudrate=57600)
    assert worker.user_connected is True

    worker.set_user_connected(False)
    assert worker.user_connected is False
    assert worker.is_connected() is False
    assert "DESCONECTADO" in worker.status_description.upper()

    worker.set_user_connected(True)
    assert worker.user_connected is True


def test_serial_worker_calibration_command():
    from odometry.telemetry_reader import SerialTelemetryWorker

    buf = TelemetryBuffer(capacity=10)
    worker = SerialTelemetryWorker(buf, port="/dev/ttyUSB0", baudrate=57600)
    assert worker.has_pending_commands() is False

    worker.request_calibration()
    assert worker.has_pending_commands() is True
    cmd = worker.pop_pending_command()
    assert cmd == b"C\n"
    assert worker.has_pending_commands() is False


