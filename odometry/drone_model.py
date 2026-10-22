"""Drone 3D model geometry and OpenGL rendering routines."""

import math
from dataclasses import dataclass
from typing import List, Dict
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

