#!/usr/bin/env python3
"""
Real-Time 3D Drone Odometry & Attitude Simulator
Built with Pygame + PyOpenGL.
"""

import sys
import os

# Automatically add repository root to sys.path
_CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_CURRENT_DIR, ".."))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)
if _CURRENT_DIR not in sys.path:
    sys.path.insert(1, _CURRENT_DIR)

import math
import time
from typing import Optional
import pygame
import pygame.gfxdraw
from pygame.locals import (
    QUIT, KEYDOWN, K_ESCAPE, K_h, K_p, K_c, K_r, K_s, K_z, K_f,
    MOUSEBUTTONDOWN, MOUSEBUTTONUP, MOUSEMOTION, VIDEORESIZE
)

from OpenGL.GL import *
from OpenGL.GLU import *

try:
    from odometry.telemetry_reader import (
        TelemetryBuffer, SerialTelemetryWorker, find_serial_port
    )
    from odometry.drone_model import (
        DroneDimensions, DroneGeometry, draw_cuboid, draw_cylinder,
        euler_to_matrix, VisibilityOptions, draw_imu_axes,
        InertialPositionEstimator, TrajectoryTrail,
        draw_trajectory_trail, draw_ground_drop_laser
    )
    from odometry.telemetry_logger import TelemetryLogger
except ImportError:
    from telemetry_reader import (
        TelemetryBuffer, SerialTelemetryWorker, find_serial_port
    )
    from drone_model import (
        DroneDimensions, DroneGeometry, draw_cuboid, draw_cylinder,
        euler_to_matrix, VisibilityOptions, draw_imu_axes,
        InertialPositionEstimator, TrajectoryTrail,
        draw_trajectory_trail, draw_ground_drop_laser
    )
    from telemetry_logger import TelemetryLogger


DEFAULT_LOG_FILE = "odometry/logs/telemetry.log"
FALLBACK_LOG_FILE = "odometry/telemetry.log"
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 820


class UITheme:
    """Neutral graphite design tokens (anti-slop technical instrument aesthetic)."""
    BG_CLEAR = (0.07, 0.08, 0.10, 1.0)          # #121419 deep graphite 3D viewport
    PANEL_BG = (22, 25, 32, 232)                # Translucent dark slate surface
    PANEL_ROW_LATEST = (38, 46, 60, 225)        # Subtle highlight row for latest sample
    BTN_IDLE = (28, 32, 40, 220)                # Default button fill
    BTN_HOVER = (44, 50, 64, 240)               # Hover button fill
    BTN_ACTIVE = (56, 68, 90, 248)              # Active toggle fill
    BORDER = (56, 62, 76, 215)                  # 1px subtle neutral border
    BORDER_ACTIVE = (98, 120, 158, 240)         # 1px steel active border
    DIVIDER = (48, 54, 68, 200)                 # Internal hairline separator
    TEXT_PRIMARY = (244, 246, 250)              # Crisp off-white for values & active items
    TEXT_SECONDARY = (205, 212, 224)            # Secondary text
    TEXT_MUTED = (145, 154, 170)                # Muted labels & key hints
    ACCENT_BLUE = (96, 165, 250)                # Soft technical blue for velocity
    STATUS_ONLINE = (16, 185, 129)              # Emerald green (#10B981)
    STATUS_OFFLINE = (239, 68, 68)              # Coral red (#EF4444)

    # Layout Sizing Tokens (High-DPI friendly)
    TOP_BAR_HEIGHT = 52
    DOCK_PILL_HEIGHT = 24
    DOCK_PILL_WIDTH = 74


def format_sample_time(timestamp_ms: int, base_ms: int = 0) -> str:
    """Format timestamp into a compact, readable seconds string (e.g. '12.4s')."""
    if timestamp_ms > 1_000_000_000:
        rel_ms = max(0, timestamp_ms - base_ms) if base_ms > 0 else (timestamp_ms % 100000)
        return f"{rel_ms / 1000.0:6.1f}s"
    return f"{timestamp_ms / 1000.0:6.1f}s"


class CameraState:
    def __init__(self):
        self.is_isometric = False
        self.follow_drone = False
        self.reset_home()

    def reset_home(self):
        """Set to default isometric zero view above the Z=0 ground plane."""
        self.azimuth = 45.0      # deg
        self.elevation = 28.0    # deg
        self.distance = 6.5      # units
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.target_z = 0.75


class Sim3DApp:
    def __init__(
        self,
        mode_or_log: str = "serial",
        port: str = None,
        baudrate: int = 57600,
        log_path: str = DEFAULT_LOG_FILE,
        mode: str = None,
        accel_decimals: Optional[int] = None,
        accel_deadband: float = 0.05,
        int_method: str = "base_euler",
        zupt: bool = True,
        verbose: bool = False,
    ):
        if mode is not None:
            self.mode = mode
            self.log_path = log_path
        elif mode_or_log in ("serial", "mock"):
            self.mode = mode_or_log
            self.log_path = log_path
        elif mode_or_log.endswith(".log") or os.path.sep in mode_or_log:
            self.mode = "mock"
            self.log_path = mode_or_log
        else:
            self.mode = "serial"
            self.log_path = log_path

        if self.log_path == DEFAULT_LOG_FILE and not os.path.exists(self.log_path) and os.path.exists(FALLBACK_LOG_FILE):
            self.log_path = FALLBACK_LOG_FILE

        self.port = port
        self.baudrate = baudrate
        self.buffer = TelemetryBuffer(capacity=10, offline_timeout=1.0)
        self.serial_worker: Optional[SerialTelemetryWorker] = None
        self.mock_process = None
        self._session_base_ms: int = 0

        if self.mode == "serial":
            self.serial_worker = SerialTelemetryWorker(
                buffer=self.buffer,
                port=self.port,
                baudrate=self.baudrate,
                log_file=self.log_path
            )
            self.serial_worker.start()
        elif self.mode == "mock":
            tester_bin = os.path.join(_PROJECT_ROOT, "odometry", "tester")
            if os.path.exists(tester_bin) and os.access(tester_bin, os.X_OK):
                import subprocess
                try:
                    self.mock_process = subprocess.Popen(
                        [tester_bin, "-m", "-o", self.log_path],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL
                    )
                except Exception as e:
                    print(f"[WARN] Could not auto-start mock tester: {e}")

        self.dims = DroneDimensions()
        self.geom = DroneGeometry(self.dims)
        self.visibility = VisibilityOptions()
        self.pos_estimator = InertialPositionEstimator(
            default_height=1.0,
            accel_decimals=accel_decimals,
            method=int_method,
            accel_deadband_g=accel_deadband,
            zupt=zupt,
        )
        self.logger = TelemetryLogger(
            verbose=verbose,
            log_file="odometry/logs/sim_diagnostics.log"
        )
        self.trail = TrajectoryTrail(max_points=240, fade_duration=4.0, min_dist=0.015)
        self.trail.add_point(*self.pos_estimator.pos)
        self._last_ingested_id: Optional[int] = None
        self._last_sample_mono: float = time.monotonic()
        self._last_physics_time: float = time.monotonic()
        self.layer_buttons = {}

        # Camera & Projection
        self.camera = CameraState()
        self.mouse_left_down = False
        self.mouse_right_down = False
        self.last_mouse_pos = (0, 0)

        # Attitude interpolation (filtered for 60 FPS smoothness)
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0

        # UI State
        self.chat_expanded = True
        self._calibrating_duration: float = 3.0
        self._is_calibrating: bool = True
        self._calibrating_until: float = time.monotonic() + self._calibrating_duration
        self.width = WINDOW_WIDTH
        self.height = WINDOW_HEIGHT

        # Initialize Pygame & OpenGL with 4x MSAA anti-aliasing
        pygame.init()
        pygame.font.init()
        pygame.display.set_caption("Drone 3D Telemetry")
        self._create_gl_window(self.width, self.height)

        # 2D HUD Surface & High-Readability Typography Hierarchy
        self.hud_surface = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
        sans_fonts = "DejaVu Sans,Ubuntu,Liberation Sans,Arial,sans-serif"
        mono_fonts = "DejaVu Sans Mono,Ubuntu Mono,Liberation Mono,Consolas,monospace"

        self.font_label = pygame.font.SysFont(sans_fonts, 14, bold=True)
        self.font_ui = pygame.font.SysFont(sans_fonts, 17, bold=False)
        self.font_ui_bold = pygame.font.SysFont(sans_fonts, 17, bold=True)
        self.font_dock = pygame.font.SysFont(sans_fonts, 11, bold=False)
        self.font_dock_bold = pygame.font.SysFont(sans_fonts, 11, bold=True)
        self.font_dock_key = pygame.font.SysFont(sans_fonts, 10, bold=True)
        self.font_value = pygame.font.SysFont(mono_fonts, 22, bold=True)
        self.font_mono = pygame.font.SysFont(mono_fonts, 16, bold=False)
        self.font_mono_bold = pygame.font.SysFont(mono_fonts, 16, bold=True)

        self.init_gl()

    def _create_gl_window(self, width: int, height: int):
        flags = pygame.OPENGL | pygame.DOUBLEBUF | pygame.RESIZABLE
        try:
            pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLEBUFFERS, 1)
            pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLESAMPLES, 4)
            self.screen = pygame.display.set_mode((width, height), flags)
        except pygame.error:
            pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLEBUFFERS, 0)
            pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLESAMPLES, 0)
            self.screen = pygame.display.set_mode((width, height), flags)

    def init_gl(self):
        glClearColor(*UITheme.BG_CLEAR)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glShadeModel(GL_SMOOTH)
        glEnable(GL_NORMALIZE)

        # Smooth anti-aliased lines and multisampling
        glEnable(GL_LINE_SMOOTH)
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
        try:
            glEnable(GL_MULTISAMPLE)
        except Exception:
            pass

        # Lighting setup
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

        glLightfv(GL_LIGHT0, GL_POSITION, (5.0, 5.0, 10.0, 1.0))
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.38, 0.39, 0.42, 1.0))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.88, 0.89, 0.92, 1.0))
        glLightfv(GL_LIGHT0, GL_SPECULAR, (0.9, 0.9, 0.95, 1.0))

        # Hardware-accelerated 2D HUD Texture
        self.hud_texture = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, self.hud_texture)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
        glBindTexture(GL_TEXTURE_2D, 0)

    def setup_projection(self):
        glViewport(0, 0, self.width, self.height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect = float(self.width) / float(self.height if self.height > 0 else 1)

        if self.camera.is_isometric:
            ortho_size = self.camera.distance * 0.45
            glOrtho(-ortho_size * aspect, ortho_size * aspect, -ortho_size, ortho_size, 0.1, 100.0)
        else:
            gluPerspective(45.0, aspect, 0.1, 100.0)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

    def update_camera(self):
        rad_az = math.radians(self.camera.azimuth)
        rad_el = math.radians(self.camera.elevation)

        if self.camera.follow_drone:
            dx, dy, dz = self.pos_estimator.pos
            tx = dx + self.camera.pan_x
            ty = dy + self.camera.pan_y
            tz = dz
        else:
            tx = self.camera.pan_x
            ty = self.camera.pan_y
            tz = self.camera.target_z

        cam_x = tx + self.camera.distance * math.cos(rad_el) * math.sin(rad_az)
        cam_y = ty + self.camera.distance * math.cos(rad_el) * math.cos(rad_az)
        cam_z = tz + self.camera.distance * math.sin(rad_el)

        gluLookAt(
            cam_x, cam_y, cam_z,
            tx, ty, tz,
            0.0, 0.0, 1.0
        )

    def draw_ground_grid(self):
        """Draws ground reference plane at Z = 0.0 with Landing Pad and X (+Red), Y (+Green), Z (+Blue) origin axes."""
        glDisable(GL_LIGHTING)
        glLineWidth(1.0)
        glBegin(GL_LINES)
        glColor4f(0.18, 0.20, 0.24, 0.55)

        grid_size = 5.5
        step = 0.5
        x = -grid_size
        while x <= grid_size + 1e-4:
            glVertex3f(x, -grid_size, 0.0)
            glVertex3f(x,  grid_size, 0.0)
            glVertex3f(-grid_size, x, 0.0)
            glVertex3f( grid_size, x, 0.0)
            x += step
        glEnd()

        # Landing Pad Circle at Ground Origin (0, 0, 0)
        glLineWidth(1.8)
        glBegin(GL_LINE_LOOP)
        glColor4f(0.25, 0.55, 0.85, 0.55)
        pad_r = 0.60
        for i in range(36):
            ang = 2.0 * math.pi * i / 36.0
            glVertex3f(pad_r * math.cos(ang), pad_r * math.sin(ang), 0.002)
        glEnd()

        # Origin Axis pointers at Z = 0.0
        glLineWidth(2.0)
        glBegin(GL_LINES)
        glColor4f(0.85, 0.25, 0.25, 0.85)
        glVertex3f(0.0, 0.0, 0.004)
        glVertex3f(1.2, 0.0, 0.004)

        glColor4f(0.20, 0.78, 0.38, 0.85)
        glVertex3f(0.0, 0.0, 0.004)
        glVertex3f(0.0, 1.2, 0.004)

        glColor4f(0.25, 0.50, 0.90, 0.85)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 0.7)
        glEnd()

        glEnable(GL_LIGHTING)

    def draw_drone(self):
        dx, dy, dz = self.pos_estimator.pos

        # World-space fading 3D trajectory trail
        if self.visibility.show_trail:
            draw_trajectory_trail(self.trail.get_active_points())

        # Vertical laser & target shadow from drone center (dx, dy, dz) to floor (dx, dy, 0.0)
        if self.visibility.show_ground_laser:
            draw_ground_drop_laser(dx, dy, dz)

        glPushMatrix()

        # 1. Translate drone to 3D position relative to ground reference (Z = 0.0)
        glTranslatef(dx, dy, dz)

        # 2. Apply Attitude (Euler Rotation Matrix)
        rot_mat = euler_to_matrix(self.current_roll, self.current_pitch, self.current_yaw)
        glMultMatrixf(rot_mat)

        # 1. Central Cuboid (Fuselage)
        if self.visibility.show_body:
            glColor4f(0.24, 0.27, 0.33, 1.0)
            draw_cuboid(self.dims.body_size, self.dims.body_size, self.dims.body_height)

            # Nose direction chevron on top of body (+X)
            glDisable(GL_LIGHTING)
            glLineWidth(2.5)
            glColor4f(0.92, 0.94, 0.98, 0.9)
            glBegin(GL_LINES)
            z_top = (self.dims.body_height / 2.0) + 0.01
            glVertex3f(0.0, -0.14, z_top)
            glVertex3f(self.dims.body_size * 0.42, 0.0, z_top)
            glVertex3f(0.0,  0.14, z_top)
            glVertex3f(self.dims.body_size * 0.42, 0.0, z_top)
            glEnd()
            glEnable(GL_LIGHTING)

        # 2. RViz-style IMU Coordinate Axes at Origin (0,0,0)
        if self.visibility.show_imu_axes:
            draw_imu_axes(axis_length=1.35)

        # 3. 4 Cylindrical Arms + 4 Vertical Motors
        arm_endpoints = self.geom.get_arm_endpoints()
        arm_len = self.dims.arm_length

        for arm in arm_endpoints:
            ex = arm["end_x"]
            ey = arm["end_y"]
            angle_xy = math.degrees(math.atan2(ey, ex))
            is_front = arm["is_front"]

            if self.visibility.show_arms:
                glPushMatrix()
                glRotatef(angle_xy, 0.0, 0.0, 1.0)
                glRotatef(90.0, 0.0, 1.0, 0.0)
                if is_front:
                    glColor4f(0.32, 0.46, 0.62, 1.0)  # Steel blue front arms
                else:
                    glColor4f(0.36, 0.38, 0.43, 1.0)  # Matte carbon rear arms
                draw_cylinder(self.dims.arm_radius, arm_len)
                glPopMatrix()

            if self.visibility.show_motors:
                glPushMatrix()
                glTranslatef(ex, ey, -self.dims.motor_height / 2.0)
                if is_front:
                    glColor4f(0.12, 0.72, 0.50, 1.0)  # Emerald front motors
                else:
                    glColor4f(0.82, 0.38, 0.24, 1.0)  # Terracotta rear motors
                draw_cylinder(self.dims.motor_radius, self.dims.motor_height)
                glPopMatrix()

        glPopMatrix()

    def _draw_aa_dot(self, x: int, y: int, radius: int, color: tuple):
        """Draw a smooth anti-aliased circle using pygame.gfxdraw."""
        pygame.gfxdraw.aacircle(self.hud_surface, x, y, radius, color)
        pygame.gfxdraw.filled_circle(self.hud_surface, x, y, radius, color)

    def _draw_pill_button(
        self,
        rect: pygame.Rect,
        label: str,
        mouse_pos: tuple,
        is_active: bool = False,
        shortcut: str = None,
        compact: bool = False
    ):
        """Render a clean, modern pill button with hover and active states."""
        is_hovered = rect.collidepoint(mouse_pos)
        if is_active:
            bg = UITheme.BTN_ACTIVE
            border = UITheme.BORDER_ACTIVE
            fg = UITheme.TEXT_PRIMARY
            font = self.font_dock_bold if compact else self.font_ui_bold
        elif is_hovered:
            bg = UITheme.BTN_HOVER
            border = UITheme.BORDER
            fg = UITheme.TEXT_PRIMARY
            font = self.font_dock if compact else self.font_ui
        else:
            bg = UITheme.BTN_IDLE
            border = UITheme.BORDER
            fg = UITheme.TEXT_SECONDARY
            font = self.font_dock if compact else self.font_ui

        radius = 5 if compact else 8
        pygame.draw.rect(self.hud_surface, bg, rect, border_radius=radius)
        pygame.draw.rect(self.hud_surface, border, rect, width=1, border_radius=radius)

        lbl_surf = font.render(label, True, fg)
        if shortcut:
            key_font = self.font_dock_key if compact else self.font_label
            gap = 4 if compact else 8
            key_surf = key_font.render(shortcut, True, UITheme.TEXT_MUTED)
            total_w = key_surf.get_width() + gap + lbl_surf.get_width()
            start_x = rect.x + (rect.width - total_w) // 2
            self.hud_surface.blit(key_surf, (start_x, rect.y + (rect.height - key_surf.get_height()) // 2))
            self.hud_surface.blit(
                lbl_surf,
                (start_x + key_surf.get_width() + gap, rect.y + (rect.height - lbl_surf.get_height()) // 2)
            )
        else:
            self.hud_surface.blit(
                lbl_surf,
                (rect.x + (rect.width - lbl_surf.get_width()) // 2, rect.y + (rect.height - lbl_surf.get_height()) // 2)
            )

    def toggle_serial_connection(self):
        """Connect or disconnect serial port from UI to free /dev/ttyUSBx for PlatformIO upload."""
        if self.serial_worker:
            self.serial_worker.toggle_user_connected()

    def zero_drone_position(self):
        """Reset ONLY 3D position to (0, 0, H0), zero velocity, and tare residual world accel bias."""
        self.pos_estimator.zero_position(tare_bias=True)
        self.trail.clear()
        self.trail.add_point(*self.pos_estimator.pos)
        bx, by, bz = self.pos_estimator.world_bias
        self.logger.log_event("INFO", f"Zeroed 3D position (0,0,{self.pos_estimator.default_height:0.1f}m) | Captured tare bias: ({bx:+0.2f}, {by:+0.2f}, {bz:+0.2f})g")

    def recalibrate_imu(self):
        """Send 'C' recalibration signal to Arduino over Serial, zero attitude, and pause integration during routine."""
        if self.serial_worker:
            self.serial_worker.request_calibration()
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self._is_calibrating = True
        self._calibrating_duration = 3.0
        self._calibrating_until = time.monotonic() + self._calibrating_duration
        self.logger.log_event("WARN", "IMU Calibration triggered... Keep drone completely motionless for 3 seconds!")

    def render_hud_overlay(self):
        """Draws a minimal, unified 2-bar HUD: Top Instrument Bar + Centered Bottom Dock."""
        self.hud_surface.fill((0, 0, 0, 0))
        mouse_pos = pygame.mouse.get_pos()
        top_y = 18
        bar_h = UITheme.TOP_BAR_HEIGHT
        is_calibrating = time.monotonic() < self._calibrating_until

        # =====================================================================
        # 1. TOP-LEFT: Clickable Status & Serial Release Pill (Shortcut: S)
        # =====================================================================
        is_online = self.buffer.is_online()
        user_disconnected = (self.mode == "serial" and self.serial_worker and not self.serial_worker.user_connected)
        if is_calibrating:
            status_color = (245, 180, 40)
            status_text = "Calibrating IMU..."
        elif user_disconnected:
            status_color = (245, 175, 40)
            port_name = os.path.basename(self.serial_worker.port or "ttyUSB0")
            status_text = f"Port Free  ·  {port_name} [S]"
        else:
            status_color = UITheme.STATUS_ONLINE if is_online else UITheme.STATUS_OFFLINE
            if is_online:
                if self.mode == "serial" and self.serial_worker:
                    port_name = os.path.basename(self.serial_worker.active_port) if self.serial_worker.active_port else "Serial"
                    status_text = f"Online  ·  {port_name} ({self.baudrate}) [S]"
                else:
                    status_text = "Online  ·  Simulation"
            else:
                if self.mode == "serial" and self.serial_worker and self.serial_worker.active_port:
                    port_name = os.path.basename(self.serial_worker.active_port)
                    status_text = f"Trying  ·  {port_name} [S]"
                else:
                    status_text = "Offline  ·  No Signal"

        status_surf = self.font_ui_bold.render(status_text, True, UITheme.TEXT_PRIMARY if is_online else UITheme.TEXT_SECONDARY)
        status_w = status_surf.get_width() + 56
        status_rect = pygame.Rect(20, top_y, status_w, bar_h)
        self.btn_status_rect = status_rect
        status_bg = UITheme.BTN_HOVER if status_rect.collidepoint(mouse_pos) else UITheme.PANEL_BG
        pygame.draw.rect(self.hud_surface, status_bg, status_rect, border_radius=10)
        pygame.draw.rect(self.hud_surface, UITheme.BORDER, status_rect, width=1, border_radius=10)

        self._draw_aa_dot(status_rect.x + 22, top_y + bar_h // 2, 7, status_color)
        self.hud_surface.blit(status_surf, (status_rect.x + 38, top_y + (bar_h - status_surf.get_height()) // 2))

        # =====================================================================
        # 2. TOP-CENTER: Unified Attitude & Altitude Readout Pill (Roll | Pitch | Yaw | Alt Z)
        # =====================================================================
        latest = self.buffer.get_latest()
        r_deg = latest.roll if latest else self.current_roll
        p_deg = latest.pitch if latest else self.current_pitch
        y_deg = latest.yaw if latest else self.current_yaw
        px, py, pz = self.pos_estimator.pos
        vx, vy, vz = self.pos_estimator.vel
        ax, ay, az = self.pos_estimator.lin_acc

        col_w = 122
        att_w = col_w * 4
        att_x = max(status_rect.right + 14, (self.width - att_w) // 2 - 20)
        att_rect = pygame.Rect(att_x, top_y, att_w, bar_h)
        pygame.draw.rect(self.hud_surface, UITheme.PANEL_BG, att_rect, border_radius=10)
        pygame.draw.rect(self.hud_surface, UITheme.BORDER, att_rect, width=1, border_radius=10)

        axes_readout = [
            ("ROLL", f"{r_deg:+5.1f}°"),
            ("PITCH", f"{p_deg:+5.1f}°"),
            ("YAW", f"{y_deg:+5.1f}°"),
            ("ALT Z", f"{pz:4.2f}m"),
        ]
        for i, (axis_lbl, val_str) in enumerate(axes_readout):
            cx = att_x + i * col_w
            if i > 0:
                pygame.draw.line(
                    self.hud_surface, UITheme.DIVIDER,
                    (cx, top_y + 10), (cx, top_y + bar_h - 10), 1
                )
            lbl_s = self.font_label.render(axis_lbl, True, UITheme.TEXT_MUTED)
            val_s = self.font_ui_bold.render(val_str, True, UITheme.TEXT_PRIMARY)
            self.hud_surface.blit(lbl_s, (cx + 10, top_y + (bar_h - lbl_s.get_height()) // 2))
            self.hud_surface.blit(val_s, (cx + col_w - val_s.get_width() - 10, top_y + (bar_h - val_s.get_height()) // 2))

        # =====================================================================
        # 2B. TOP-LEFT SUB-ROW: Physical Odometry Controls (Z Zero | R Calibrate | F Camera) + 3D Readout
        # =====================================================================
        sub_y = top_y + bar_h + 10
        sub_h = 36
        self.btn_zero_rect = pygame.Rect(20, sub_y, 98, sub_h)
        self.btn_cal_rect = pygame.Rect(self.btn_zero_rect.right + 8, sub_y, 118, sub_h)
        self.btn_cam_rect = pygame.Rect(self.btn_cal_rect.right + 8, sub_y, 104, sub_h)

        cal_label = "Calibrating" if is_calibrating else "Calibrate"
        cam_label = "Follow" if self.camera.follow_drone else "Static"
        self._draw_pill_button(self.btn_zero_rect, "Zero", mouse_pos, is_active=False, shortcut="Z")
        self._draw_pill_button(self.btn_cal_rect, cal_label, mouse_pos, is_active=is_calibrating, shortcut="R")
        self._draw_pill_button(self.btn_cam_rect, cam_label, mouse_pos, is_active=self.camera.follow_drone, shortcut="F")

        odom_rect = pygame.Rect(20, sub_y + sub_h + 8, 344, 66)
        pygame.draw.rect(self.hud_surface, UITheme.PANEL_BG, odom_rect, border_radius=8)
        pygame.draw.rect(self.hud_surface, UITheme.BORDER, odom_rect, width=1, border_radius=8)
        pos_line = f"POS 3D (m)  : X {px:+5.2f}   Y {py:+5.2f}   Z {pz:4.2f}"
        vel_line = f"VEL 3D (m/s): Vx {vx:+5.2f}  Vy {vy:+5.2f}  Vz {vz:+5.2f}"
        acc_line = f"LIN ACC (g) : Ax {ax:+5.2f}  Ay {ay:+5.2f}  Az {az:+5.2f}"
        self.hud_surface.blit(self.font_dock_bold.render(pos_line, True, UITheme.TEXT_PRIMARY), (32, odom_rect.y + 8))
        self.hud_surface.blit(self.font_dock_bold.render(vel_line, True, UITheme.ACCENT_BLUE), (32, odom_rect.y + 26))
        self.hud_surface.blit(self.font_dock.render(acc_line, True, UITheme.TEXT_MUTED), (32, odom_rect.y + 44))

        # =====================================================================
        # 3. TOP-RIGHT: View & Telemetry Drawer Controls (H Reset | P Persp | C Telemetry)
        # =====================================================================
        btn_gap = 8
        chat_btn_w = 138
        proj_btn_w = 144
        home_btn_w = 98

        right_edge = self.width - 20
        self.btn_chat_rect = pygame.Rect(right_edge - chat_btn_w, top_y, chat_btn_w, bar_h)
        self.btn_proj_rect = pygame.Rect(self.btn_chat_rect.left - btn_gap - proj_btn_w, top_y, proj_btn_w, bar_h)
        self.btn_home_rect = pygame.Rect(self.btn_proj_rect.left - btn_gap - home_btn_w, top_y, home_btn_w, bar_h)

        proj_label = "Isometric" if self.camera.is_isometric else "Perspective"
        self._draw_pill_button(self.btn_home_rect, "Reset", mouse_pos, is_active=False, shortcut="H")
        self._draw_pill_button(self.btn_proj_rect, proj_label, mouse_pos, is_active=self.camera.is_isometric, shortcut="P")
        self._draw_pill_button(self.btn_chat_rect, "Telemetry", mouse_pos, is_active=self.chat_expanded, shortcut="C")

        # Collapsible Telemetry Drawer
        if self.chat_expanded:
            drawer_w = 460
            drawer_x = self.width - drawer_w - 20
            drawer_y = top_y + bar_h + 10
            history = self.buffer.get_history()
            if history and self._session_base_ms == 0:
                self._session_base_ms = history[0].timestamp

            row_h = 28
            rows_count = max(1, len(history))
            drawer_h = 46 + rows_count * row_h + 12

            drawer_rect = pygame.Rect(drawer_x, drawer_y, drawer_w, drawer_h)
            pygame.draw.rect(self.hud_surface, UITheme.PANEL_BG, drawer_rect, border_radius=10)
            pygame.draw.rect(self.hud_surface, UITheme.BORDER, drawer_rect, width=1, border_radius=10)

            # Table Columns Definition: (Header, X-Anchor, Align)
            # Align 'left' anchors at x; Align 'right' aligns the right edge of header & values at x.
            columns = [
                ("SAMPLE", drawer_x + 18, "left"),
                ("TIME",   drawer_x + 168, "right"),
                ("ROLL",   drawer_x + 258, "right"),
                ("PITCH",  drawer_x + 350, "right"),
                ("YAW",    drawer_x + drawer_w - 18, "right"),
            ]

            for col_title, col_x, align in columns:
                hdr_surf = self.font_label.render(col_title, True, UITheme.TEXT_MUTED)
                hx = col_x if align == "left" else (col_x - hdr_surf.get_width())
                self.hud_surface.blit(hdr_surf, (hx, drawer_y + 13))

            pygame.draw.line(
                self.hud_surface, UITheme.DIVIDER,
                (drawer_x + 14, drawer_y + 38), (drawer_x + drawer_w - 14, drawer_y + 38), 1
            )

            if history:
                y_row = drawer_y + 45
                for idx, rec in enumerate(history):
                    is_last = (idx == len(history) - 1)
                    if is_last:
                        hl_rect = pygame.Rect(drawer_x + 8, y_row - 2, drawer_w - 16, row_h - 2)
                        pygame.draw.rect(self.hud_surface, UITheme.PANEL_ROW_LATEST, hl_rect, border_radius=6)

                    t_str = format_sample_time(rec.timestamp, self._session_base_ms).strip()
                    cells = [
                        f"#{idx+1:02d}",
                        t_str,
                        f"{rec.roll:+.1f}°",
                        f"{rec.pitch:+.1f}°",
                        f"{rec.yaw:+.1f}°",
                    ]
                    row_font = self.font_mono_bold if is_last else self.font_mono
                    row_color = UITheme.TEXT_PRIMARY if is_last else UITheme.TEXT_SECONDARY

                    for cell_text, (_, col_x, align) in zip(cells, columns):
                        cell_surf = row_font.render(cell_text, True, row_color)
                        cx = col_x if align == "left" else (col_x - cell_surf.get_width())
                        self.hud_surface.blit(cell_surf, (cx, y_row + 2))

                    y_row += row_h
            else:
                empty_s = self.font_ui.render("Waiting for telemetry data...", True, UITheme.TEXT_MUTED)
                self.hud_surface.blit(empty_s, (drawer_x + 18, drawer_y + 46))

        # =====================================================================
        # 4. BOTTOM-CENTER: Floating Layer Visibility Dock (Compact 50%)
        # =====================================================================
        layer_items = [
            ("body", "Body", "1", self.visibility.show_body),
            ("arms", "Arms", "2", self.visibility.show_arms),
            ("motors", "Motors", "3", self.visibility.show_motors),
            ("imu_axes", "Axes", "4", self.visibility.show_imu_axes),
            ("grid", "Grid", "5", self.visibility.show_grid),
            ("trail", "Trail", "6", self.visibility.show_trail),
            ("ground_laser", "Laser", "7", self.visibility.show_ground_laser),
        ]

        pill_w = UITheme.DOCK_PILL_WIDTH
        pill_h = UITheme.DOCK_PILL_HEIGHT
        pill_gap = 4
        dock_pad = 4
        dock_w = len(layer_items) * pill_w + (len(layer_items) - 1) * pill_gap + dock_pad * 2
        dock_h = pill_h + dock_pad * 2
        dock_x = (self.width - dock_w) // 2
        dock_y = self.height - dock_h - 28

        dock_rect = pygame.Rect(dock_x, dock_y, dock_w, dock_h)
        pygame.draw.rect(self.hud_surface, UITheme.PANEL_BG, dock_rect, border_radius=8)
        pygame.draw.rect(self.hud_surface, UITheme.BORDER, dock_rect, width=1, border_radius=8)

        bx = dock_x + dock_pad
        by = dock_y + dock_pad
        self.layer_buttons = {}
        for key, label, shortcut, is_active in layer_items:
            rect = pygame.Rect(bx, by, pill_w, pill_h)
            self.layer_buttons[key] = rect
            self._draw_pill_button(rect, label, mouse_pos, is_active=is_active, shortcut=shortcut, compact=True)
            bx += pill_w + pill_gap

        # Subtle bottom hint
        hints = "Left Drag: Orbit   ·   Right Drag: Pan   ·   Scroll: Zoom"
        hint_surf = self.font_label.render(hints, True, UITheme.TEXT_MUTED)
        self.hud_surface.blit(hint_surf, ((self.width - hint_surf.get_width()) // 2, self.height - 26))

        # =====================================================================
        # 4B. CALIBRATION WARNING MODAL (Active until calibration completes)
        # =====================================================================
        if is_calibrating:
            remaining = max(0.0, self._calibrating_until - time.monotonic())
            cal_duration = getattr(self, "_calibrating_duration", 3.0)
            progress = max(0.0, min(1.0, 1.0 - (remaining / (cal_duration if cal_duration > 0 else 3.0))))

            card_w = min(600, self.width - 40)
            card_h = 138
            card_x = (self.width - card_w) // 2
            card_y = top_y + bar_h + 18

            card_rect = pygame.Rect(card_x, card_y, card_w, card_h)
            # Translucent dark slate modal backdrop with amber warning border
            pygame.draw.rect(self.hud_surface, (18, 22, 28, 246), card_rect, border_radius=12)
            pygame.draw.rect(self.hud_surface, (245, 180, 40, 230), card_rect, width=2, border_radius=12)

            # Warning header badge + countdown
            badge_surf = self.font_label.render("IMU CALIBRATION IN PROGRESS", True, (245, 180, 40))
            self.hud_surface.blit(badge_surf, (card_x + 24, card_y + 16))

            countdown_str = f"{remaining:3.1f}s remaining"
            countdown_surf = self.font_mono_bold.render(countdown_str, True, UITheme.TEXT_SECONDARY)
            self.hud_surface.blit(countdown_surf, (card_x + card_w - countdown_surf.get_width() - 24, card_y + 16))

            # Primary instruction banner
            title_surf = self.font_ui_bold.render("KEEP DRONE COMPLETELY MOTIONLESS", True, UITheme.TEXT_PRIMARY)
            self.hud_surface.blit(title_surf, (card_x + 24, card_y + 44))

            # Operational guidance subtext
            desc1_surf = self.font_dock.render("Keep device static on a flat surface without rotation or acceleration.", True, UITheme.TEXT_SECONDARY)
            self.hud_surface.blit(desc1_surf, (card_x + 24, card_y + 72))
            desc2_surf = self.font_dock.render("Measuring stationary gyro and accelerometer reference biases...", True, UITheme.TEXT_MUTED)
            self.hud_surface.blit(desc2_surf, (card_x + 24, card_y + 90))

            # Progress bar
            pbar_bg = pygame.Rect(card_x + 24, card_y + card_h - 18, card_w - 48, 6)
            pygame.draw.rect(self.hud_surface, (36, 42, 54, 200), pbar_bg, border_radius=3)
            fill_w = int((card_w - 48) * progress)
            if fill_w > 0:
                pbar_fill = pygame.Rect(card_x + 24, card_y + card_h - 18, fill_w, 6)
                pygame.draw.rect(self.hud_surface, (245, 180, 40), pbar_fill, border_radius=3)

        # =====================================================================
        # 5. Composite 2D HUD Surface onto OpenGL Viewport
        # =====================================================================
        data = pygame.image.tobytes(self.hud_surface, "RGBA", True)
        glBindTexture(GL_TEXTURE_2D, self.hud_texture)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, self.width, self.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data)

        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        glOrtho(0, self.width, 0, self.height, -1, 1)

        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()

        glDisable(GL_DEPTH_TEST)
        glDisable(GL_LIGHTING)
        glEnable(GL_TEXTURE_2D)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glColor4f(1.0, 1.0, 1.0, 1.0)

        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 0.0); glVertex2f(0.0, 0.0)
        glTexCoord2f(1.0, 0.0); glVertex2f(float(self.width), 0.0)
        glTexCoord2f(1.0, 1.0); glVertex2f(float(self.width), float(self.height))
        glTexCoord2f(0.0, 1.0); glVertex2f(0.0, float(self.height))
        glEnd()

        glDisable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, 0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)

        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        glPopMatrix()

    def run(self):
        clock = pygame.time.Clock()
        running = True

        while running:
            now_mono = time.monotonic()
            dt = max(0.005, min(0.1, now_mono - self._last_physics_time))
            self._last_physics_time = now_mono

            # Calibration state machine transition
            if self._is_calibrating and now_mono >= self._calibrating_until:
                self._is_calibrating = False
                self.zero_drone_position()
                self.logger.log_event("OK", "IMU calibration routine complete on board! Position zeroed & tare captured.")

            # 1. Ingest new telemetry (only poll log file in mock mode; serial worker pushes directly)
            if self.mode == "mock":
                self.buffer.poll_file(self.log_path)
            latest = self.buffer.get_latest()
            if latest:
                self.target_roll = latest.roll
                self.target_pitch = latest.pitch
                self.target_yaw = latest.yaw

                if id(latest) != self._last_ingested_id:
                    sample_dt = (
                        max(0.01, min(0.15, now_mono - self._last_sample_mono))
                        if self._last_ingested_id is not None
                        else 0.04
                    )
                    self._last_sample_mono = now_mono
                    self._last_ingested_id = id(latest)

                    if self.buffer.is_online() and not self._is_calibrating:
                        self.pos_estimator.update(
                            roll_deg=latest.roll,
                            pitch_deg=latest.pitch,
                            yaw_deg=latest.yaw,
                            acc_x=latest.acc_x,
                            acc_y=latest.acc_y,
                            acc_z=latest.acc_z,
                            dt=sample_dt
                        )
                        self.trail.add_point(*self.pos_estimator.pos, now=now_mono)
                        diag = self.pos_estimator.get_diagnostics_state()
                        self.logger.log_sample(
                            dt=sample_dt,
                            roll=latest.roll,
                            pitch=latest.pitch,
                            yaw=latest.yaw,
                            acc_body=diag["body_acc"],
                            acc_world_raw=diag["world_raw"],
                            acc_world_bias=diag["world_bias"],
                            lin_acc=diag["lin_acc"],
                            vel=diag["vel"],
                            pos=diag["pos"],
                            deadband=diag["deadband"],
                            clamped=diag["clamped"],
                            zupt_active=diag.get("zupt_active", False),
                        )

            # Smooth interpolation (0.25 factor for 60 FPS responsiveness)
            self.current_roll  += (self.target_roll  - self.current_roll)  * 0.25
            self.current_pitch += (self.target_pitch - self.current_pitch) * 0.25
            self.current_yaw   += (self.target_yaw   - self.current_yaw)   * 0.25

            # 2. Event Handling
            for event in pygame.event.get():
                if event.type == QUIT:
                    running = False
                elif event.type == VIDEORESIZE:
                    self.width, self.height = event.size
                    self._create_gl_window(self.width, self.height)
                    self.hud_surface = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
                elif event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        running = False
                    elif event.key == K_h:
                        self.camera.reset_home()
                    elif event.key == K_z:
                        self.zero_drone_position()
                    elif event.key == K_f:
                        self.camera.follow_drone = not self.camera.follow_drone
                    elif event.key == K_s:
                        self.toggle_serial_connection()
                    elif event.key == K_r:
                        self.recalibrate_imu()
                    elif event.key == K_p:
                        self.camera.is_isometric = not self.camera.is_isometric
                    elif event.key == K_c:
                        self.chat_expanded = not self.chat_expanded
                    elif event.key in (pygame.K_1, pygame.K_KP1):
                        self.visibility.toggle("body")
                    elif event.key in (pygame.K_2, pygame.K_KP2):
                        self.visibility.toggle("arms")
                    elif event.key in (pygame.K_3, pygame.K_KP3):
                        self.visibility.toggle("motors")
                    elif event.key in (pygame.K_4, pygame.K_KP4):
                        self.visibility.toggle("imu_axes")
                    elif event.key in (pygame.K_5, pygame.K_KP5):
                        self.visibility.toggle("grid")
                    elif event.key in (pygame.K_6, pygame.K_KP6):
                        self.visibility.toggle("trail")
                    elif event.key in (pygame.K_7, pygame.K_KP7):
                        self.visibility.toggle("ground_laser")
                elif event.type == MOUSEBUTTONDOWN:
                    if event.button == 1:
                        mouse_pos = event.pos
                        layer_clicked = False
                        for comp_key, rect in self.layer_buttons.items():
                            if rect.collidepoint(mouse_pos):
                                self.visibility.toggle(comp_key)
                                layer_clicked = True
                                break

                        if layer_clicked:
                            pass
                        elif hasattr(self, "btn_status_rect") and self.btn_status_rect.collidepoint(mouse_pos):
                            self.toggle_serial_connection()
                        elif hasattr(self, "btn_home_rect") and self.btn_home_rect.collidepoint(mouse_pos):
                            self.camera.reset_home()
                        elif hasattr(self, "btn_zero_rect") and self.btn_zero_rect.collidepoint(mouse_pos):
                            self.zero_drone_position()
                        elif hasattr(self, "btn_cal_rect") and self.btn_cal_rect.collidepoint(mouse_pos):
                            self.recalibrate_imu()
                        elif hasattr(self, "btn_cam_rect") and self.btn_cam_rect.collidepoint(mouse_pos):
                            self.camera.follow_drone = not self.camera.follow_drone
                        elif hasattr(self, "btn_proj_rect") and self.btn_proj_rect.collidepoint(mouse_pos):
                            self.camera.is_isometric = not self.camera.is_isometric
                        elif hasattr(self, "btn_chat_rect") and self.btn_chat_rect.collidepoint(mouse_pos):
                            self.chat_expanded = not self.chat_expanded
                        else:
                            self.mouse_left_down = True
                            self.last_mouse_pos = event.pos
                    elif event.button == 3:
                        self.mouse_right_down = True
                        self.last_mouse_pos = event.pos
                    elif event.button == 4:
                        self.camera.distance = max(1.5, self.camera.distance - 0.4)
                    elif event.button == 5:
                        self.camera.distance = min(25.0, self.camera.distance + 0.4)
                elif event.type == MOUSEBUTTONUP:
                    if event.button == 1:
                        self.mouse_left_down = False
                    elif event.button == 3:
                        self.mouse_right_down = False
                elif event.type == MOUSEMOTION:
                    dx = event.pos[0] - self.last_mouse_pos[0]
                    dy = event.pos[1] - self.last_mouse_pos[1]
                    self.last_mouse_pos = event.pos

                    if self.mouse_left_down:
                        self.camera.azimuth += dx * 0.4
                        self.camera.elevation = max(-89.0, min(89.0, self.camera.elevation + dy * 0.4))
                    elif self.mouse_right_down:
                        factor = self.camera.distance * 0.002
                        self.camera.pan_x -= dx * factor
                        self.camera.pan_y -= dy * factor

            # 3. 3D OpenGL Rendering
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            self.setup_projection()
            self.update_camera()

            if self.visibility.show_grid:
                self.draw_ground_grid()
            self.draw_drone()

            # 4. HUD 2D Overlay
            self.render_hud_overlay()

            pygame.display.flip()
            clock.tick(60)

        # Cleanup background worker & processes
        self.logger.close()
        if self.serial_worker:
            self.serial_worker.stop()
        if self.mock_process:
            self.mock_process.terminate()
            try:
                self.mock_process.wait(timeout=1.0)
            except Exception:
                self.mock_process.kill()

        pygame.quit()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
        prog="sim3d",
        description="Drone 3D Real-Time Odometry & Telemetry Simulator"
    )
    parser.add_argument(
        "--serialplot", "--serial", "-s",
        dest="mode",
        action="store_const",
        const="serial",
        default="serial",
        help="Capture telemetry directly from USB Serial (Default)"
    )
    parser.add_argument(
        "--mock", "-m",
        dest="mode",
        action="store_const",
        const="mock",
        help="Use mock C kinematics simulation (odometry/tester -m)"
    )
    parser.add_argument(
        "--port", "-p",
        type=str,
        default=None,
        help="Serial port (e.g. /dev/ttyUSB0, /dev/ttyACM0). Auto-detected if omitted."
    )
    parser.add_argument(
        "--baud", "-b",
        type=int,
        default=57600,
        help="Baud rate (default: 57600)"
    )
    parser.add_argument(
        "--log", "-l",
        type=str,
        default=DEFAULT_LOG_FILE,
        help=f"Target telemetry log path (default: {DEFAULT_LOG_FILE})"
    )
    parser.add_argument(
        "--accel-decimals",
        type=int,
        default=None,
        help="Decimal places for acceleration rounding before integration (default: None = full precision)"
    )
    parser.add_argument(
        "--deadband", "--accel-deadband",
        type=float,
        default=0.05,
        help="Acceleration deadband threshold in g (default: 0.05g to reject MEMS noise)"
    )
    parser.add_argument(
        "--int-method",
        type=str,
        choices=["base_euler", "trapezoidal", "zupt"],
        default="base_euler",
        help="Kinematic integration method: base_euler (default), trapezoidal, or zupt"
    )
    parser.add_argument(
        "--no-zupt",
        dest="zupt",
        action="store_false",
        default=True,
        help="Disable Zero-Velocity Update (ZUPT) stationary velocity damping"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Enable live per-sample verbose diagnostic logging to stdout & diagnostics file"
    )

    args = parser.parse_args()
    app = Sim3DApp(
        mode=args.mode,
        port=args.port,
        baudrate=args.baud,
        log_path=args.log,
        accel_decimals=args.accel_decimals,
        accel_deadband=args.deadband,
        int_method=args.int_method,
        zupt=args.zupt,
        verbose=args.verbose,
    )
    app.run()
