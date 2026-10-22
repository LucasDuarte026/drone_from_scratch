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

