"""Drone 3D model geometry, inertial position estimation, and OpenGL rendering routines."""

import math
import time
from collections import deque
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional
from OpenGL.GL import *


@dataclass
class DroneDimensions:
    body_size: float = 0.8       # Central cuboid width/length
    body_height: float = 0.25    # Central cuboid thickness
    arm_length: float = 1.8      # Distance from center to motor
    arm_radius: float = 0.05     # Cylinder arm thickness
    motor_radius: float = 0.16   # Vertical cylinder motor radius
    motor_height: float = 0.25   # Vertical cylinder motor height

class DroneGeometry:
    def __init__(self, dims: DroneDimensions = None):
        self.dims = dims or DroneDimensions()

    def get_arm_endpoints(self) -> List[Dict[str, float]]:
        """Return the coordinates for the 4 motor locations in Quad-X configuration."""
        d = self.dims.arm_length / math.sqrt(2.0)
        return [
            {"id": 1, "label": "M1 (FR)", "end_x": d,  "end_y": d,  "end_z": 0.0, "is_front": True},
            {"id": 2, "label": "M2 (RR)", "end_x": -d, "end_y": d,  "end_z": 0.0, "is_front": False},
            {"id": 3, "label": "M3 (RL)", "end_x": -d, "end_y": -d, "end_z": 0.0, "is_front": False},
            {"id": 4, "label": "M4 (FL)", "end_x": d,  "end_y": -d, "end_z": 0.0, "is_front": True},
        ]

def euler_to_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> List[float]:
    """
    Compute 4x4 column-major OpenGL transformation matrix from Roll, Pitch, Yaw in degrees.
    Rotation order: Yaw (Z) * Pitch (Y) * Roll (X).
    """
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)

    # Combined rotation matrix R = Rz(y) * Ry(p) * Rx(r)
    # Col 0
    m00 = cy * cp
    m10 = sy * cp
    m20 = -sp
    m30 = 0.0

    # Col 1
    m01 = cy * sp * sr - sy * cr
    m11 = sy * sp * sr + cy * cr
    m21 = cp * sr
    m31 = 0.0

    # Col 2
    m02 = cy * sp * cr + sy * sr
    m12 = sy * sp * cr - cy * sr
    m22 = cp * cr
    m32 = 0.0

    # Col 3
    m03 = 0.0
    m13 = 0.0
    m23 = 0.0
    m33 = 1.0

    # OpenGL expects 16-element column-major array
    return [
        m00, m10, m20, m30,
        m01, m11, m21, m31,
        m02, m12, m22, m32,
        m03, m13, m23, m33
    ]


def draw_cuboid(width: float, length: float, height: float):
    """Draw a 3D solid cuboid centered at the local origin using OpenGL quads."""
    from OpenGL.GL import (
        glBegin, glEnd, glVertex3f, glNormal3f, GL_QUADS
    )

    w = width / 2.0
    l = length / 2.0
    h = height / 2.0

    glBegin(GL_QUADS)
    # Top Face (+Z)
    glNormal3f(0.0, 0.0, 1.0)
    glVertex3f(-w, -l,  h)
    glVertex3f( w, -l,  h)
    glVertex3f( w,  l,  h)
    glVertex3f(-w,  l,  h)

    # Bottom Face (-Z)
    glNormal3f(0.0, 0.0, -1.0)
    glVertex3f(-w, -l, -h)
    glVertex3f(-w,  l, -h)
    glVertex3f( w,  l, -h)
    glVertex3f( w, -l, -h)

    # Front Face (+X)
    glNormal3f(1.0, 0.0, 0.0)
    glVertex3f( w, -l, -h)
    glVertex3f( w,  l, -h)
    glVertex3f( w,  l,  h)
    glVertex3f( w, -l,  h)

    # Back Face (-X)
    glNormal3f(-1.0, 0.0, 0.0)
    glVertex3f(-w, -l, -h)
    glVertex3f(-w, -l,  h)
    glVertex3f(-w,  l,  h)
    glVertex3f(-w,  l, -h)

    # Right Face (+Y)
    glNormal3f(0.0, 1.0, 0.0)
    glVertex3f(-w,  l, -h)
    glVertex3f(-w,  l,  h)
    glVertex3f( w,  l,  h)
    glVertex3f( w,  l, -h)

    # Left Face (-Y)
    glNormal3f(0.0, -1.0, 0.0)
    glVertex3f(-w, -l, -h)
    glVertex3f( w, -l, -h)
    glVertex3f( w, -l,  h)
    glVertex3f(-w, -l,  h)
    glEnd()


def draw_cylinder(radius: float, length: float, slices: int = 16):
    """Draw a cylinder aligned along the +Z axis from Z=0 to Z=length."""
    from OpenGL.GL import (
        glBegin, glEnd, glVertex3f, glNormal3f, GL_QUAD_STRIP, GL_TRIANGLE_FAN
    )

    glBegin(GL_QUAD_STRIP)
    for i in range(slices + 1):
        theta = 2.0 * math.pi * i / slices
        nx = math.cos(theta)
        ny = math.sin(theta)
        x = radius * nx
        y = radius * ny
        glNormal3f(nx, ny, 0.0)
        glVertex3f(x, y, 0.0)
        glVertex3f(x, y, length)
    glEnd()

    # Caps
    for z, nz in [(0.0, -1.0), (length, 1.0)]:
        glBegin(GL_TRIANGLE_FAN)
        glNormal3f(0.0, 0.0, nz)
        glVertex3f(0.0, 0.0, z)
        for i in range(slices + 1):
            theta = 2.0 * math.pi * i / slices * (-1 if nz < 0 else 1)
            glVertex3f(radius * math.cos(theta), radius * math.sin(theta), z)
        glEnd()


@dataclass
class VisibilityOptions:
    """Toggles for individual visual components of the drone simulation."""
    show_body: bool = True
    show_arms: bool = True
    show_motors: bool = True
    show_imu_axes: bool = True
    show_grid: bool = True
    show_trail: bool = True
    show_ground_laser: bool = True

    def toggle(self, component: str):
        comp = component.lower().strip()
        if comp in ("body", "fuselage", "central"):
            self.show_body = not self.show_body
        elif comp in ("arms", "hastes", "braços", "bracos"):
            self.show_arms = not self.show_arms
        elif comp in ("motors", "motores"):
            self.show_motors = not self.show_motors
        elif comp in ("imu_axes", "axes", "rviz", "eixos"):
            self.show_imu_axes = not self.show_imu_axes
        elif comp in ("grid", "solo", "grade"):
            self.show_grid = not self.show_grid
        elif comp in ("trail", "rastro"):
            self.show_trail = not self.show_trail
        elif comp in ("ground_laser", "laser", "altura"):
            self.show_ground_laser = not self.show_ground_laser


class InertialPositionEstimator:
    """
    Estimates 3D position (X, Y, Z) relative to Ground Reference Zero (Z = 0.0)
    by rotating body-frame accelerometer readings into the world frame,
    compensating gravity (1.0g), and integrating with deadband + ZUPT damping.
    """

    def __init__(
        self,
        default_height: float = 1.0,
        accel_deadband_g: float = 0.035,
        vel_damping: float = 0.90,
        scale_mps2: float = 9.81,
        max_radius: float = 8.0,
        max_height: float = 6.0
    ):
        self.default_height: float = max(0.0, float(default_height))
        self.accel_deadband_g: float = accel_deadband_g
        self.vel_damping: float = vel_damping
        self.scale_mps2: float = scale_mps2
        self.max_radius: float = max_radius
        self.max_height: float = max_height
        self.enabled: bool = True

        self.pos: Tuple[float, float, float] = (0.0, 0.0, self.default_height)
        self.vel: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.lin_acc: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    def zero_position(self):
        """
        Reset ONLY the 3D position back to (0.0, 0.0, default_height) and velocity to (0, 0, 0),
        leaving IMU attitude (Roll, Pitch, Yaw) untouched.
        """
        self.pos = (0.0, 0.0, self.default_height)
        self.vel = (0.0, 0.0, 0.0)
        self.lin_acc = (0.0, 0.0, 0.0)

    def set_default_height(self, height: float):
        """Adjust default reference hover height H0 above the Z=0 ground plane."""
        old_h = self.default_height
        self.default_height = max(0.1, min(self.max_height, float(height)))
        px, py, pz = self.pos
        new_pz = max(0.0, min(self.max_height, pz + (self.default_height - old_h)))
        self.pos = (px, py, new_pz)

    def update(
        self,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
        acc_x: float,
        acc_y: float,
        acc_z: float,
        dt: float = 0.04
    ) -> Tuple[float, float, float]:
        """Update world-frame linear acceleration, velocity, and 3D position."""
        dt = max(0.001, min(0.2, float(dt)))
        mat = euler_to_matrix(roll_deg, pitch_deg, yaw_deg)

        # Rotate body acceleration into world frame: a_world = R * a_body
        aw_x = mat[0] * acc_x + mat[4] * acc_y + mat[8] * acc_z
        aw_y = mat[1] * acc_x + mat[5] * acc_y + mat[9] * acc_z
        aw_z = mat[2] * acc_x + mat[6] * acc_y + mat[10] * acc_z - 1.0

        # Deadband filter for sensor noise
        if abs(aw_x) < self.accel_deadband_g:
            aw_x = 0.0
        if abs(aw_y) < self.accel_deadband_g:
            aw_y = 0.0
        if abs(aw_z) < self.accel_deadband_g:
            aw_z = 0.0

        self.lin_acc = (aw_x, aw_y, aw_z)

        if not self.enabled:
            return self.pos

        vx, vy, vz = self.vel
        px, py, pz = self.pos

        # Zero-Velocity Update (ZUPT): fast damping when accelerometer is stationary
        if aw_x == 0.0 and aw_y == 0.0 and aw_z == 0.0:
            vx *= 0.65
            vy *= 0.65
            vz *= 0.65
            if abs(vx) < 1e-3:
                vx = 0.0
            if abs(vy) < 1e-3:
                vy = 0.0
            if abs(vz) < 1e-3:
                vz = 0.0
        else:
            vx = (vx + aw_x * self.scale_mps2 * dt) * self.vel_damping
            vy = (vy + aw_y * self.scale_mps2 * dt) * self.vel_damping
            vz = (vz + aw_z * self.scale_mps2 * dt) * self.vel_damping

        px = max(-self.max_radius, min(self.max_radius, px + vx * dt))
        py = max(-self.max_radius, min(self.max_radius, py + vy * dt))
        pz = max(0.0, min(self.max_height, pz + vz * dt))
        if pz == 0.0 and vz < 0.0:
            vz = 0.0

        self.vel = (vx, vy, vz)
        self.pos = (px, py, pz)
        return self.pos


class TrajectoryTrail:
    """Maintains a time-fading 3D trajectory trail of drone center positions."""

    def __init__(self, max_points: int = 220, fade_duration: float = 4.0, min_dist: float = 0.015):
        self.max_points = max_points
        self.fade_duration = max(0.1, float(fade_duration))
        self.min_dist = min_dist
        self._points: deque[Tuple[float, float, float, float]] = deque(maxlen=max_points)

    def add_point(self, x: float, y: float, z: float, now: Optional[float] = None):
        t = time.monotonic() if now is None else float(now)
        if self._points:
            lx, ly, lz, _ = self._points[-1]
            dist = math.sqrt((x - lx) ** 2 + (y - ly) ** 2 + (z - lz) ** 2)
            if dist < self.min_dist:
                return
        self._points.append((float(x), float(y), float(z), t))

    def get_active_points(self, now: Optional[float] = None) -> List[Dict[str, float]]:
        t = time.monotonic() if now is None else float(now)
        while self._points and (t - self._points[0][3]) >= self.fade_duration:
            self._points.popleft()

        result: List[Dict[str, float]] = []
        for x, y, z, pt_time in self._points:
            age = max(0.0, t - pt_time)
            if age < self.fade_duration:
                alpha = max(0.0, min(1.0, 1.0 - (age / self.fade_duration)))
                result.append({"x": x, "y": y, "z": z, "alpha": alpha})
        return result

    def clear(self):
        self._points.clear()


def draw_trajectory_trail(points: List[Dict[str, float]]):
    """Render fading 3D trajectory ribbon/line strip in world coordinates."""
    if len(points) < 2:
        return
    glDisable(GL_LIGHTING)
    glLineWidth(2.8)
    glBegin(GL_LINE_STRIP)
    for pt in points:
        alpha = pt["alpha"]
        glColor4f(0.0, 0.92, 1.0, alpha * 0.85)
        glVertex3f(pt["x"], pt["y"], pt["z"])
    glEnd()
    glEnable(GL_LIGHTING)


def draw_ground_drop_laser(x: float, y: float, z: float):
    """Draw vertical laser line and ground shadow projection from (x, y, z) to ground (x, y, 0.0)."""
    glDisable(GL_LIGHTING)
    glLineWidth(1.8)
    glBegin(GL_LINES)
    glColor4f(1.0, 0.78, 0.15, 0.75)
    glVertex3f(x, y, z)
    glColor4f(1.0, 0.78, 0.15, 0.25)
    glVertex3f(x, y, 0.0)
    glEnd()

    # Ground target shadow ring at (x, y, 0.005)
    glBegin(GL_LINE_LOOP)
    glColor4f(0.0, 0.95, 0.85, 0.65)
    segments = 24
    radius = 0.22
    for i in range(segments):
        ang = 2.0 * math.pi * i / segments
        glVertex3f(x + radius * math.cos(ang), y + radius * math.sin(ang), 0.005)
    glEnd()

    # Crosshair at ground projection
    glBegin(GL_LINES)
    glColor4f(0.0, 0.95, 0.85, 0.5)
    glVertex3f(x - 0.12, y, 0.005)
    glVertex3f(x + 0.12, y, 0.005)
    glVertex3f(x, y - 0.12, 0.005)
    glVertex3f(x, y + 0.12, 0.005)
    glEnd()
    glEnable(GL_LIGHTING)


def get_imu_axes_vectors(axis_length: float = 1.5) -> Dict[str, Dict]:
    """Return dictionary of RViz-style IMU coordinate vectors for unit testing and drawing."""
    return {
        "x": {"dir": (axis_length, 0.0, 0.0), "color": (1.0, 0.15, 0.15, 1.0), "name": "X (Roll/Nose)"},
        "y": {"dir": (0.0, axis_length, 0.0), "color": (0.15, 0.95, 0.25, 1.0), "name": "Y (Pitch/Starboard)"},
        "z": {"dir": (0.0, 0.0, axis_length), "color": (0.2, 0.5, 1.0, 1.0), "name": "Z (Yaw/Down)"},
    }


def draw_imu_axes(axis_length: float = 1.5, shaft_radius: float = 0.035, tip_radius: float = 0.07, tip_length: float = 0.22):
    """Draw RViz-style RGB coordinate axes at the drone IMU center (0, 0, 0)."""
    glDisable(GL_LIGHTING)
    
    # 1. Central origin marker (small white/silver sphere/cube representing IMU chip)
    glColor4f(0.9, 0.9, 0.9, 1.0)
    draw_cuboid(0.12, 0.12, 0.05)

    # 2. X Axis (Red - Nose / Roll)
    glColor4f(1.0, 0.15, 0.15, 1.0)
    glLineWidth(4.0)
    glBegin(GL_LINES)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(axis_length, 0.0, 0.0)
    glEnd()

    # X Arrowhead Cone
    glPushMatrix()
    glTranslatef(axis_length, 0.0, 0.0)
    glRotatef(90.0, 0.0, 1.0, 0.0)
    draw_cylinder(tip_radius, tip_length, slices=12)
    glPopMatrix()

    # 3. Y Axis (Green - Right Wing / Pitch)
    glColor4f(0.15, 0.95, 0.25, 1.0)
    glLineWidth(4.0)
    glBegin(GL_LINES)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, axis_length, 0.0)
    glEnd()

    # Y Arrowhead Cone
    glPushMatrix()
    glTranslatef(0.0, axis_length, 0.0)
    glRotatef(-90.0, 1.0, 0.0, 0.0)
    draw_cylinder(tip_radius, tip_length, slices=12)
    glPopMatrix()

    # 4. Z Axis (Blue - Vertical / Yaw)
    glColor4f(0.2, 0.5, 1.0, 1.0)
    glLineWidth(4.0)
    glBegin(GL_LINES)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, 0.0, axis_length)
    glEnd()

    # Z Arrowhead Cone
    glPushMatrix()
    glTranslatef(0.0, 0.0, axis_length)
    draw_cylinder(tip_radius, tip_length, slices=12)
    glPopMatrix()

    glEnable(GL_LIGHTING)

