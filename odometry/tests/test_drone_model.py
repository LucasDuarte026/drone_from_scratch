import pytest
import math
from odometry.drone_model import DroneDimensions, DroneGeometry, euler_to_matrix

def test_drone_dimensions_initialization():
    dims = DroneDimensions()
    assert dims.body_size > 0
    assert dims.arm_length > dims.body_size
    assert dims.arm_radius > 0
    assert dims.motor_radius > dims.arm_radius
    assert dims.motor_height > 0

def test_drone_arm_endpoints():
    dims = DroneDimensions(arm_length=2.0)
    geom = DroneGeometry(dims)
    arms = geom.get_arm_endpoints()
    assert len(arms) == 4
    
    # 4 arms at 45, 135, 225, 315 degrees
    expected_dist = 2.0
    for arm in arms:
        dist = math.sqrt(arm["end_x"]**2 + arm["end_y"]**2)
        assert dist == pytest.approx(expected_dist, rel=1e-3)
        assert arm["end_z"] == pytest.approx(0.0)
        assert "label" in arm

def test_euler_to_matrix_identity():
    # Zero angles should yield identity matrix
    mat = euler_to_matrix(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0)
    assert len(mat) == 16
    assert mat[0] == pytest.approx(1.0)
    assert mat[5] == pytest.approx(1.0)
    assert mat[10] == pytest.approx(1.0)
    assert mat[15] == pytest.approx(1.0)
    assert mat[1] == pytest.approx(0.0)
    assert mat[2] == pytest.approx(0.0)

def test_euler_to_matrix_pure_yaw():
    # 90 deg yaw rotation about Z axis
    mat = euler_to_matrix(roll_deg=0.0, pitch_deg=0.0, yaw_deg=90.0)
    assert mat[0] == pytest.approx(0.0, abs=1e-5)
    assert mat[1] == pytest.approx(1.0, abs=1e-5)
    assert mat[4] == pytest.approx(-1.0, abs=1e-5)
    assert mat[5] == pytest.approx(0.0, abs=1e-5)

def test_visibility_options():
    from odometry.drone_model import VisibilityOptions
    vis = VisibilityOptions()
    # Default: all components visible
    assert vis.show_body is True
    assert vis.show_arms is True
    assert vis.show_motors is True
    assert vis.show_imu_axes is True
    assert vis.show_grid is True

    vis.toggle("body")
    assert vis.show_body is False
    vis.toggle("body")
    assert vis.show_body is True

    vis.toggle("imu_axes")
    assert vis.show_imu_axes is False

def test_imu_axes_vectors():
    from odometry.drone_model import get_imu_axes_vectors
    axes = get_imu_axes_vectors(axis_length=1.5)
    assert "x" in axes
    assert "y" in axes
    assert "z" in axes
    # X axis points along +X (Nose / Red)
    assert axes["x"]["dir"] == (1.5, 0.0, 0.0)
    assert axes["x"]["color"] == (1.0, 0.15, 0.15, 1.0)
    # Y axis points along +Y (Right wing / Green)
    assert axes["y"]["dir"] == (0.0, 1.5, 0.0)
    assert axes["y"]["color"] == (0.15, 0.95, 0.25, 1.0)
    # Z axis points along +Z (Down/Up / Blue)
    assert axes["z"]["dir"] == (0.0, 0.0, 1.5)
    assert axes["z"]["color"] == (0.2, 0.5, 1.0, 1.0)


def test_ui_theme_tokens():
    from odometry.sim_3d import UITheme, format_sample_time
    # Neutral graphite background and surfaces (no neon cyan borders)
    assert len(UITheme.BG_CLEAR) == 4
    assert UITheme.BG_CLEAR[0] < 0.12 and UITheme.BG_CLEAR[1] < 0.12 and UITheme.BG_CLEAR[2] < 0.15
    assert UITheme.BORDER[0] < 80 and UITheme.BORDER[1] < 80 and UITheme.BORDER[2] < 95
    assert UITheme.TEXT_PRIMARY[0] > 220
    assert UITheme.TEXT_MUTED[0] < 180
    assert UITheme.STATUS_ONLINE != UITheme.STATUS_OFFLINE
    assert UITheme.TOP_BAR_HEIGHT >= 50
    assert UITheme.DOCK_PILL_HEIGHT >= 24
    # Epoch ms timestamps should format compactly in seconds
    assert len(format_sample_time(1793129963582, 1793129960000)) <= 8


def test_english_only_ui_labels():
    from pathlib import Path

    sim_src = (Path(__file__).resolve().parent.parent / "sim_3d.py").read_text(encoding="utf-8")
    reader_src = (Path(__file__).resolve().parent.parent / "telemetry_reader.py").read_text(encoding="utf-8")

    # Mandatory English UI strings
    for eng_label in (
        '"SAMPLE"', '"TIME"', '"Isometric"', '"Perspective"', '"Calibrate"',
        '"Telemetry"', '"Body"', '"Arms"', '"Motors"', '"Axes"', '"Grid"',
        '"Waiting for telemetry data..."', '"Left Drag: Orbit   ·   Right Drag: Pan   ·   Scroll: Zoom"'
    ):
        assert eng_label in sim_src

    # Prohibited Portuguese UI strings
    for pt_label in (
        "AMOSTRA", "TEMPO", "Isométrica", "Perspectiva", "Calibrar",
        "Telemetria", "Corpo", "Braços", "Motores", "Eixos", "Grade",
        "Porta Livre", "Sem sinal", "Aguardando dados", "DESCONECTADO"
    ):
        assert pt_label not in sim_src
        assert pt_label not in reader_src




def test_inertial_position_estimator_stationary_and_tilted_gravity():
    from odometry.drone_model import InertialPositionEstimator

    est = InertialPositionEstimator(default_height=1.0)
    assert est.pos == (0.0, 0.0, 1.0)
    assert est.vel == (0.0, 0.0, 0.0)

    # 1. Stationary flat (1g along body Z) -> zero linear acceleration, stays at (0, 0, 1.0)
    for _ in range(10):
        est.update(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0, acc_x=0.0, acc_y=0.0, acc_z=1.0, dt=0.04)
    assert est.pos[0] == pytest.approx(0.0, abs=1e-4)
    assert est.pos[1] == pytest.approx(0.0, abs=1e-4)
    assert est.pos[2] == pytest.approx(1.0, abs=1e-4)

    # 2. Tilted 30 deg Roll, stationary: gravity projects as ay = sin(30)=0.5, az = cos(30)=0.8660254
    ay_tilt = math.sin(math.radians(30.0))
    az_tilt = math.cos(math.radians(30.0))
    est.update(roll_deg=30.0, pitch_deg=0.0, yaw_deg=0.0, acc_x=0.0, acc_y=ay_tilt, acc_z=az_tilt, dt=0.04)
    assert est.lin_acc[0] == pytest.approx(0.0, abs=1e-2)
    assert est.lin_acc[1] == pytest.approx(0.0, abs=1e-2)
    assert est.lin_acc[2] == pytest.approx(0.0, abs=1e-2)


def test_inertial_position_estimator_motion_and_zero_button():
    from odometry.drone_model import InertialPositionEstimator

    est = InertialPositionEstimator(default_height=1.2)
    # Apply forward acceleration (+0.5g along X while level)
    for _ in range(10):
        est.update(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0, acc_x=0.5, acc_y=0.0, acc_z=1.0, dt=0.04)

    assert est.pos[0] > 0.02
    assert est.vel[0] > 0.05

    # Click ZERAR: resets only position to (0, 0, default_height) and velocity to (0, 0, 0)
    est.zero_position()
    assert est.pos == (0.0, 0.0, pytest.approx(1.2))
    assert est.vel == (0.0, 0.0, 0.0)


def test_fading_trajectory_trail():
    from odometry.drone_model import TrajectoryTrail

    trail = TrajectoryTrail(max_points=100, fade_duration=2.0, min_dist=0.01)
    trail.add_point(0.0, 0.0, 1.0, now=10.0)
    trail.add_point(0.5, 0.0, 1.0, now=11.0)
    trail.add_point(1.0, 0.0, 1.0, now=11.8)

    active = trail.get_active_points(now=12.0)
    # Point at t=10.0 has age 2.0s -> expired, remaining 2 points have alpha in (0, 1]
    assert len(active) == 2
    assert active[0]["alpha"] == pytest.approx(0.5, abs=0.05)
    assert active[1]["alpha"] == pytest.approx(0.9, abs=0.05)

    trail.clear()
    assert len(trail.get_active_points(now=12.0)) == 0


def test_inertial_estimator_decimals_and_saved_methods():
    from pathlib import Path
    from odometry.drone_model import InertialPositionEstimator
    from odometry.telemetry_logger import TelemetryLogger

    # 1. Default base_euler mode: accel_decimals=None (unfiltered), accel_deadband_g=0.05, vel_damping=1.0
    est = InertialPositionEstimator(default_height=1.0)
    assert est.accel_decimals is None
    assert est.accel_deadband_g == pytest.approx(0.05)
    assert est.vel_damping == pytest.approx(1.0)
    assert est.method == "base_euler"

    # Small stationary noise 0.024g is filtered by deadband (0.05g)
    est.update(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0, acc_x=0.024, acc_y=0.0, acc_z=1.0, dt=0.04)
    assert est.lin_acc[0] == pytest.approx(0.0)
    assert est.vel[0] == pytest.approx(0.0)
    assert est.pos[0] == pytest.approx(0.0)
    assert est.last_was_clamped is True

    # Real movement above deadband (0.08g) integrates normally
    est.update(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0, acc_x=0.08, acc_y=0.0, acc_z=1.0, dt=0.04)
    assert est.lin_acc[0] == pytest.approx(0.08)
    assert est.vel[0] == pytest.approx(0.08 * 9.81 * 0.04)
    assert est.pos[0] == pytest.approx(est.vel[0] * 0.04)

    # Stationary ZUPT test: when motion stops (acc_x = 0), velocity decays to 0.0
    for _ in range(8):
        est.update(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0, acc_x=0.0, acc_y=0.0, acc_z=1.0, dt=0.04)
    assert est.vel[0] == pytest.approx(0.0)

    # 2. Configurable precision (e.g., 3 decimals)
    est3 = InertialPositionEstimator(default_height=1.0, accel_decimals=3, accel_deadband_g=0.0)
    est3.update(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0, acc_x=0.126, acc_y=0.0, acc_z=1.0, dt=0.04)
    assert est3.lin_acc[0] == pytest.approx(0.126)

    # 3. Saved evolutionary methods ('trapezoidal' and 'zupt')
    est_trap = InertialPositionEstimator(default_height=1.0, method="trapezoidal", accel_deadband_g=0.0)
    est_trap.update(0.0, 0.0, 0.0, acc_x=0.10, acc_y=0.0, acc_z=1.0, dt=0.04)
    est_trap.update(0.0, 0.0, 0.0, acc_x=0.20, acc_y=0.0, acc_z=1.0, dt=0.04)
    assert est_trap.vel[0] > 0.0

    est_zupt = InertialPositionEstimator(
        default_height=1.0, method="zupt", accel_deadband_g=0.04, vel_damping=0.95
    )
    est_zupt.update(0.0, 0.0, 0.0, acc_x=0.02, acc_y=0.0, acc_z=1.0, dt=0.04)
    assert est_zupt.lin_acc[0] == pytest.approx(0.0)

    # 4. Velocity line & CLI verbose flag displayed/supported in sim_3d.py
    sim_src = (Path(__file__).resolve().parent.parent / "sim_3d.py").read_text(encoding="utf-8")
    assert "VEL 3D" in sim_src
    assert "--verbose" in sim_src
    assert "--deadband" in sim_src
    assert "IMU CALIBRATION IN PROGRESS" in sim_src
    assert "KEEP DRONE COMPLETELY MOTIONLESS" in sim_src

    # 5. Telemetry logger test (Standard and Verbose modes)
    test_log_path = Path(__file__).resolve().parent.parent / "logs" / "test_diag.log"
    logger = TelemetryLogger(verbose=True, log_file=str(test_log_path), enable_stdout=False)
    logger.log_event("INFO", "Test event")
    logger.log_sample(
        dt=0.04,
        roll=0.0, pitch=0.0, yaw=0.0,
        acc_body=(0.0, 0.0, 1.0),
        acc_world_raw=(0.0, 0.0, 0.0),
        acc_world_bias=(0.0, 0.0, 0.0),
        lin_acc=(0.0, 0.0, 0.0),
        vel=(0.0, 0.0, 0.0),
        pos=(0.0, 0.0, 1.0),
        deadband=0.05,
        clamped=True
    )
    logger.close()
    assert test_log_path.exists()
    content = test_log_path.read_text(encoding="utf-8")
    assert "Test event" in content
    assert "CLAMP" in content
    test_log_path.unlink()




