#!/usr/bin/env python3
"""
Real-Time 3D Drone Odometry & Attitude Simulator
Built with Pygame + PyOpenGL.

Features:
- Fictional drone model: central cuboid + 4 cylindrical arms + 4 vertical cylindrical motors.
- 10-position FIFO tail buffer reading 'odometry/telemetry.log'.
- Orbital camera (left drag: orbit, right drag: pan, wheel: zoom).
- Non-manipulable drone (attitude 100% driven by telemetry file).
- 'Home' button: returns camera to default isometric view.
- 'Projection' toggle: Isometric (Orthographic) vs Perspective.
- Top-left Online (green) / Offline (red) status indicator (< 1.0s timeout).
- Collapsible floating HUD chat displaying the 10-position telemetry stream.
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
import pygame
from pygame.locals import (
    QUIT, KEYDOWN, K_ESCAPE, K_h, K_p, K_c,
    MOUSEBUTTONDOWN, MOUSEBUTTONUP, MOUSEMOTION, VIDEORESIZE
)

from OpenGL.GL import *
from OpenGL.GLU import *


try:
    from odometry.telemetry_reader import TelemetryBuffer
    from odometry.drone_model import (
        DroneDimensions, DroneGeometry, draw_cuboid, draw_cylinder,
        euler_to_matrix, VisibilityOptions, draw_imu_axes
    )
except ImportError:
    from telemetry_reader import TelemetryBuffer
    from drone_model import (
        DroneDimensions, DroneGeometry, draw_cuboid, draw_cylinder,
        euler_to_matrix, VisibilityOptions, draw_imu_axes
    )



DEFAULT_LOG_FILE = "odometry/logs/telemetry.log"
FALLBACK_LOG_FILE = "odometry/telemetry.log"
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 800


class CameraState:
    def __init__(self):
        self.reset_home()
        self.is_isometric = False
        self.pan_x = 0.0
        self.pan_y = 0.0

    def reset_home(self):
        """Set to default isometric zero view."""
        self.azimuth = 45.0      # deg
        self.elevation = 30.0    # deg
        self.distance = 6.0      # units
        self.pan_x = 0.0
        self.pan_y = 0.0

class Sim3DApp:
    def __init__(self, log_path: str = DEFAULT_LOG_FILE):

        if log_path == DEFAULT_LOG_FILE and not os.path.exists(log_path) and os.path.exists(FALLBACK_LOG_FILE):
            self.log_path = FALLBACK_LOG_FILE
        else:
            self.log_path = log_path
        self.buffer = TelemetryBuffer(capacity=10, offline_timeout=1.0)


        self.dims = DroneDimensions()
        self.geom = DroneGeometry(self.dims)
        self.visibility = VisibilityOptions()
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
        self.width = WINDOW_WIDTH
        self.height = WINDOW_HEIGHT

        # Initialize Pygame & OpenGL
        pygame.init()
        pygame.font.init()
        pygame.display.set_caption("Drone 3D Real-Time Odometry & Telemetry Simulator")
        self.screen = pygame.display.set_mode((self.width, self.height), pygame.OPENGL | pygame.DOUBLEBUF | pygame.RESIZABLE)

        # 2D HUD Surface
        self.hud_surface = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
        self.font_main = pygame.font.SysFont("monospace", 14, bold=True)
        self.font_title = pygame.font.SysFont("sans-serif", 15, bold=True)
        self.font_btn = pygame.font.SysFont("sans-serif", 13, bold=True)

        self.init_gl()

    def init_gl(self):
        glClearColor(0.08, 0.10, 0.14, 1.0) # Deep slate aerospace background
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glShadeModel(GL_SMOOTH)
        glEnable(GL_NORMALIZE)

        # Lighting setup
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

        glLightfv(GL_LIGHT0, GL_POSITION, (5.0, 5.0, 10.0, 1.0))
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.35, 0.35, 0.40, 1.0))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.85, 0.85, 0.90, 1.0))
        glLightfv(GL_LIGHT0, GL_SPECULAR, (1.0, 1.0, 1.0, 1.0))

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
            # Orthographic Isometric Projection
            ortho_size = self.camera.distance * 0.45
            glOrtho(-ortho_size * aspect, ortho_size * aspect, -ortho_size, ortho_size, 0.1, 100.0)
        else:
            # Perspective Projection
            gluPerspective(45.0, aspect, 0.1, 100.0)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

    def update_camera(self):
        rad_az = math.radians(self.camera.azimuth)
        rad_el = math.radians(self.camera.elevation)

        cam_x = self.camera.distance * math.cos(rad_el) * math.sin(rad_az) + self.camera.pan_x
        cam_y = self.camera.distance * math.cos(rad_el) * math.cos(rad_az) + self.camera.pan_y
        cam_z = self.camera.distance * math.sin(rad_el)

        gluLookAt(
            cam_x, cam_y, cam_z,
            self.camera.pan_x, self.camera.pan_y, 0.0,
            0.0, 0.0, 1.0
        )

    def draw_ground_grid(self):
        """Draws subtle ground reference plane with X (+Red) and Y (+Green) axes."""
        glDisable(GL_LIGHTING)
        glLineWidth(1.0)
        glBegin(GL_LINES)
        glColor4f(0.18, 0.22, 0.28, 0.6)

        grid_size = 5.0
        step = 0.5
        x = -grid_size
        while x <= grid_size + 1e-4:
            glVertex3f(x, -grid_size, -1.0)
            glVertex3f(x,  grid_size, -1.0)
            glVertex3f(-grid_size, x, -1.0)
            glVertex3f( grid_size, x, -1.0)
            x += step
        glEnd()

        # Origin Axis pointers
        glLineWidth(2.5)
        glBegin(GL_LINES)
        # X: Nose / Forward (Red)
        glColor4f(0.9, 0.2, 0.2, 0.9)
        glVertex3f(0.0, 0.0, -0.98)
        glVertex3f(1.5, 0.0, -0.98)
        # Y: Right Wing (Green)
        glColor4f(0.2, 0.8, 0.3, 0.9)
        glVertex3f(0.0, 0.0, -0.98)
        glVertex3f(0.0, 1.5, -0.98)
        # Z: Downward / Upward (Blue)
        glColor4f(0.2, 0.4, 0.9, 0.9)
        glVertex3f(0.0, 0.0, -0.98)
        glVertex3f(0.0, 0.0, -0.2)
        glEnd()

        glEnable(GL_LIGHTING)

    def draw_drone(self):
        glPushMatrix()

        # Apply Attitude (Euler Rotation Matrix)
        rot_mat = euler_to_matrix(self.current_roll, self.current_pitch, self.current_yaw)
        glMultMatrixf(rot_mat)

        # 1. Central Cuboid (Fuselage)
        if self.visibility.show_body:
            glColor4f(0.25, 0.28, 0.35, 1.0) # Sleek graphite body
            draw_cuboid(self.dims.body_size, self.dims.body_size, self.dims.body_height)

            # Nose indicator on top of body (Arrow pointing forward along +X)
            glDisable(GL_LIGHTING)
            glLineWidth(3.0)
            glColor4f(0.0, 0.95, 0.85, 1.0) # Cyan arrow for front
            glBegin(GL_LINES)
            z_top = (self.dims.body_height / 2.0) + 0.01
            glVertex3f(0.0, -0.15, z_top)
            glVertex3f(self.dims.body_size * 0.45, 0.0, z_top)
            glVertex3f(0.0,  0.15, z_top)
            glVertex3f(self.dims.body_size * 0.45, 0.0, z_top)
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

            # Draw Arm Cylinder
            if self.visibility.show_arms:
                glPushMatrix()
                glRotatef(angle_xy, 0.0, 0.0, 1.0)
                glRotatef(90.0, 0.0, 1.0, 0.0) # Align cylinder along +X
                if is_front:
                    glColor4f(0.20, 0.65, 0.70, 1.0) # Front arms cyan-tinted
                else:
                    glColor4f(0.40, 0.42, 0.46, 1.0) # Rear arms carbon-gray
                draw_cylinder(self.dims.arm_radius, arm_len)
                glPopMatrix()

            # Draw Vertical Motor Cylinder at Arm Extremity
            if self.visibility.show_motors:
                glPushMatrix()
                glTranslatef(ex, ey, -self.dims.motor_height / 2.0)
                if is_front:
                    glColor4f(0.0, 0.85, 0.45, 1.0) # Front motors vivid green
                else:
                    glColor4f(0.85, 0.35, 0.20, 1.0) # Rear motors warm amber
                draw_cylinder(self.dims.motor_radius, self.dims.motor_height)
                glPopMatrix()

        glPopMatrix()


    def render_hud_overlay(self):
        """Draws 2D overlay: Status indicator, Home & Projection buttons, and Collapsible Chat."""
        self.hud_surface.fill((0, 0, 0, 0))

        # -------------------------------------------------------------
        # 1. Top-Left: Online/Offline Status Indicator
        # -------------------------------------------------------------
        is_online = self.buffer.is_online()
        status_color = (40, 220, 90) if is_online else (240, 50, 60)
        status_text = "STATUS: ONLINE (STREAMING)" if is_online else "STATUS: OFFLINE (NO SIGNAL)"

        # Background badge
        badge_rect = pygame.Rect(15, 15, 270, 38)
        pygame.draw.rect(self.hud_surface, (20, 25, 35, 210), badge_rect, border_radius=8)
        pygame.draw.rect(self.hud_surface, (50, 60, 80, 220), badge_rect, width=1, border_radius=8)

        # Status Circle
        pygame.draw.circle(self.hud_surface, status_color, (33, 34), 8)
        pygame.draw.circle(self.hud_surface, (255, 255, 255, 120), (33, 34), 10, width=1)

        # Status Label
        lbl_surf = self.font_title.render(status_text, True, (240, 245, 250))
        self.hud_surface.blit(lbl_surf, (50, 25))

        # Current Attitude Badge below status
        latest = self.buffer.get_latest()
        att_rect = pygame.Rect(15, 60, 270, 48)
        pygame.draw.rect(self.hud_surface, (15, 20, 28, 190), att_rect, border_radius=6)
        r_deg = latest.roll if latest else self.current_roll
        p_deg = latest.pitch if latest else self.current_pitch
        y_deg = latest.yaw if latest else self.current_yaw
        att_str1 = f"ROLL: {r_deg:+6.1f}°  PITCH: {p_deg:+6.1f}°"
        att_str2 = f"YAW : {y_deg:+6.1f}°  BUFFER: 10 POS"
        self.hud_surface.blit(self.font_main.render(att_str1, True, (0, 230, 210)), (24, 66))
        self.hud_surface.blit(self.font_main.render(att_str2, True, (180, 200, 220)), (24, 86))

        # -------------------------------------------------------------
        # 2. Top-Center Controls: HOME & PROJECTION Buttons
        # -------------------------------------------------------------
        # Home Button
        self.btn_home_rect = pygame.Rect(300, 15, 100, 36)
        pygame.draw.rect(self.hud_surface, (30, 38, 52, 220), self.btn_home_rect, border_radius=6)
        pygame.draw.rect(self.hud_surface, (70, 90, 120, 240), self.btn_home_rect, width=1, border_radius=6)
        self.hud_surface.blit(self.font_btn.render("⌂ HOME", True, (240, 245, 250)), (322, 24))

        # Projection Button
        proj_label = "PROJ: ISOMETRIC" if self.camera.is_isometric else "PROJ: PERSPECTIVE"
        self.btn_proj_rect = pygame.Rect(410, 15, 170, 36)
        pygame.draw.rect(self.hud_surface, (30, 38, 52, 220), self.btn_proj_rect, border_radius=6)
        pygame.draw.rect(self.hud_surface, (70, 90, 120, 240), self.btn_proj_rect, width=1, border_radius=6)
        self.hud_surface.blit(self.font_btn.render(proj_label, True, (240, 245, 250)), (425, 24))

        # -------------------------------------------------------------
        # 3. Top-Right: Collapsible Real-Time Stream Chat
        # -------------------------------------------------------------
        chat_width = 460
        chat_x = self.width - chat_width - 15

        # Chat Toggle Button
        toggle_label = "▼ TELEMETRY STREAM CHAT" if self.chat_expanded else "▲ SHOW TELEMETRY CHAT"
        self.btn_chat_rect = pygame.Rect(chat_x, 15, chat_width, 36)
        pygame.draw.rect(self.hud_surface, (25, 32, 45, 230), self.btn_chat_rect, border_radius=6)
        pygame.draw.rect(self.hud_surface, (0, 200, 220, 180), self.btn_chat_rect, width=1, border_radius=6)
        self.hud_surface.blit(self.font_btn.render(toggle_label, True, (0, 240, 255)), (chat_x + 15, 24))

        if self.chat_expanded:
            chat_height = 270
            chat_box = pygame.Rect(chat_x, 56, chat_width, chat_height)
            pygame.draw.rect(self.hud_surface, (12, 16, 24, 235), chat_box, border_radius=8)
            pygame.draw.rect(self.hud_surface, (45, 55, 75, 200), chat_box, width=1, border_radius=8)

            # Chat Header
            hdr = "BUFFER (LAST 10 RAW SAMPLES):"
            self.hud_surface.blit(self.font_main.render(hdr, True, (130, 150, 180)), (chat_x + 12, 64))

            # Stream records
            history = self.buffer.get_history()
            y_offset = 86
            for idx, rec in enumerate(history):
                # Clean format for user viewing
                text = f"[{idx+1:02d}] t={rec.timestamp:05d}ms | R:{rec.roll:+5.1f}° P:{rec.pitch:+5.1f}° Y:{rec.yaw:+5.1f}°"
                color = (0, 255, 200) if idx == len(history) - 1 else (190, 205, 225)
                self.hud_surface.blit(self.font_main.render(text, True, color), (chat_x + 12, y_offset))
                y_offset += 17

            if not history:
                self.hud_surface.blit(self.font_main.render("Waiting for telemetry stream in 'odometry/logs/telemetry.log'...", True, (240, 100, 100)), (chat_x + 12, 95))

        # -------------------------------------------------------------
        # 4. Bottom-Left: RViz & Component Visibility Menu
        # -------------------------------------------------------------
        menu_y = self.height - 72
        self.hud_surface.blit(self.font_main.render("COMPONENTES & RVIZ (VISIBILIDADE):", True, (0, 230, 255)), (15, menu_y - 20))

        layer_items = [
            ("body", "1:CORPO", self.visibility.show_body),
            ("arms", "2:HASTES", self.visibility.show_arms),
            ("motors", "3:MOTORES", self.visibility.show_motors),
            ("imu_axes", "4:RVIZ EIXOS", self.visibility.show_imu_axes),
            ("grid", "5:GRADE", self.visibility.show_grid),
        ]

        bx = 15
        self.layer_buttons = {}
        for key, label, is_active in layer_items:
            bw = 118
            rect = pygame.Rect(bx, menu_y, bw, 28)
            self.layer_buttons[key] = rect
            
            # Active vs inactive styling
            bg_col = (18, 55, 42, 230) if is_active else (25, 28, 35, 190)
            border_col = (0, 240, 140, 240) if is_active else (65, 75, 85, 180)
            text_col = (0, 255, 170) if is_active else (135, 145, 155)
            prefix = "[✓] " if is_active else "[ ] "

            pygame.draw.rect(self.hud_surface, bg_col, rect, border_radius=5)
            pygame.draw.rect(self.hud_surface, border_col, rect, width=1, border_radius=5)
            self.hud_surface.blit(self.font_btn.render(prefix + label, True, text_col), (bx + 8, menu_y + 6))
            bx += bw + 8

        # Bottom Hints Bar
        hints = "Orbit: Left Drag | Pan: Right Drag | Zoom: Wheel | H: Home | P: Proj | C: Chat | 1-5: Visibilidade"
        self.hud_surface.blit(self.font_main.render(hints, True, (100, 120, 150)), (15, self.height - 20))


        # Render 2D surface over OpenGL using textured orthographic quad
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
            # 1. Ingest new telemetry from file
            self.buffer.poll_file(self.log_path)
            latest = self.buffer.get_latest()
            if latest:
                self.target_roll = latest.roll
                self.target_pitch = latest.pitch
                self.target_yaw = latest.yaw

            # Smooth interpolation (0.2 factor for 60 FPS responsiveness)
            self.current_roll  += (self.target_roll  - self.current_roll)  * 0.25
            self.current_pitch += (self.target_pitch - self.current_pitch) * 0.25
            self.current_yaw   += (self.target_yaw   - self.current_yaw)   * 0.25

            # 2. Event Handling
            for event in pygame.event.get():
                if event.type == QUIT:
                    running = False
                elif event.type == VIDEORESIZE:
                    self.width, self.height = event.size
                    self.screen = pygame.display.set_mode((self.width, self.height), pygame.OPENGL | pygame.DOUBLEBUF | pygame.RESIZABLE)
                    self.hud_surface = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
                elif event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        running = False
                    elif event.key == K_h:
                        self.camera.reset_home()
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
                elif event.type == MOUSEBUTTONDOWN:
                    if event.button == 1: # Left click
                        # Check HUD button clicks
                        mouse_pos = event.pos
                        layer_clicked = False
                        if hasattr(self, 'layer_buttons'):
                            for comp_key, rect in self.layer_buttons.items():
                                if rect.collidepoint(mouse_pos):
                                    self.visibility.toggle(comp_key)
                                    layer_clicked = True
                                    break

                        if layer_clicked:
                            pass
                        elif hasattr(self, 'btn_home_rect') and self.btn_home_rect.collidepoint(mouse_pos):
                            self.camera.reset_home()
                        elif hasattr(self, 'btn_proj_rect') and self.btn_proj_rect.collidepoint(mouse_pos):
                            self.camera.is_isometric = not self.camera.is_isometric
                        elif hasattr(self, 'btn_chat_rect') and self.btn_chat_rect.collidepoint(mouse_pos):
                            self.chat_expanded = not self.chat_expanded
                        else:
                            self.mouse_left_down = True
                            self.last_mouse_pos = event.pos
                    elif event.button == 3: # Right click (Pan)
                        self.mouse_right_down = True
                        self.last_mouse_pos = event.pos
                    elif event.button == 4: # Wheel Up (Zoom in)
                        self.camera.distance = max(1.5, self.camera.distance - 0.4)
                    elif event.button == 5: # Wheel Down (Zoom out)
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
                        # Orbit
                        self.camera.azimuth += dx * 0.4
                        self.camera.elevation = max(-89.0, min(89.0, self.camera.elevation - dy * 0.4))
                    elif self.mouse_right_down:
                        # Pan
                        factor = self.camera.distance * 0.002
                        self.camera.pan_x -= dx * factor
                        self.camera.pan_y += dy * factor

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

        pygame.quit()

if __name__ == "__main__":
    log_file = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LOG_FILE
    app = Sim3DApp(log_file)
    app.run()
