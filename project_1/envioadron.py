#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import re
import sys
import threading
import time

import pygame
import serial
import serial.tools.list_ports
from pygame.locals import (DOUBLEBUF, OPENGL, QUIT, KEYDOWN, K_ESCAPE, K_z, K_y,
                           K_p, K_c, MOUSEMOTION, MOUSEWHEEL)
from OpenGL.GL import *
from OpenGL.GLU import *

DRAW_ROLL_SIGN = 1.0
DRAW_PITCH_SIGN = -1.0
DRAW_YAW_SIGN = -1.0

WIDTH, HEIGHT = 1100, 720

class RFController(threading.Thread):
    def __init__(self, port, baud):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.roll = self.pitch = self.yaw = self.alt = 0.0
        self.m1 = self.m2 = self.m3 = self.m4 = 0
        self.connected = False
        self.running = True
        self.rate = 0
        self._count = 0
        self._t0 = time.time()
        self.status = "Esperando conexion RF (Potenciometro)..."

    def run(self):
        while self.running:
            try:
                with serial.Serial(self.port, self.baud, timeout=0.1) as ser:
                    ser.reset_input_buffer()
                    self.connected = True
                    buffer_text = ""
                    
                    while self.running:
                        try:
                            char = ser.read().decode(errors="ignore")
                            if char:
                                buffer_text += char
                                if char == '\n':
                                    raw = buffer_text.strip()
                                    buffer_text = ""
                                    updated = False
                                    
                                    m = re.search(r"Roll:\s*([-+]?\d*\.?\d+)", raw)
                                    if m:
                                        try:
                                            v = float(m.group(1))
                                            if -90 <= v <= 90: self.roll = v; updated = True
                                        except: pass

                                    m = re.search(r"Pitch:\s*([-+]?\d*\.?\d+)", raw)
                                    if m:
                                        try:
                                            v = float(m.group(1))
                                            if -90 <= v <= 90: self.pitch = v; updated = True
                                        except: pass

                                    m = re.search(r"Yaw:\s*([-+]?\d*\.?\d+)", raw)
                                    if m:
                                        try:
                                            v = float(m.group(1))
                                            if -180 <= v <= 180: self.yaw = v; updated = True
                                        except: pass

                                    m = re.search(r"Alt:\s*([-+]?\d*\.?\d+)", raw)
                                    if m:
                                        try:
                                            self.alt = float(m.group(1))
                                            updated = True
                                        except: pass

                                    for i in range(1, 5):
                                        m = re.search(rf"M{i}:\s*(\d+)", raw)
                                        if m:
                                            try:
                                                v = int(m.group(1))
                                                if 0 <= v <= 100:
                                                    setattr(self, f"m{i}", v)
                                                    updated = True
                                            except: pass

                                    if updated:
                                        self.status = "Telemetria RF Activa"
                                        self._count += 1
                                        now = time.time()
                                        if now - self._t0 >= 1.0:
                                            self.rate = self._count
                                            self._count = 0
                                            self._t0 = now
                                    elif raw and len(raw) > 5 and "Roll" not in raw and "M1" not in raw:
                                        self.status = raw[:70]
                        except Exception:
                            pass
            except (serial.SerialException, OSError):
                self.connected = False
                self.status = f"Buscando base en {self.port}..."
                time.sleep(1.0)

def draw_box(sx, sy, sz):
    x, y, z = sx / 2.0, sy / 2.0, sz / 2.0
    glBegin(GL_QUADS)
    glNormal3f(0, 0, 1)
    glVertex3f(-x, -y, z); glVertex3f(x, -y, z); glVertex3f(x, y, z); glVertex3f(-x, y, z)
    glNormal3f(0, 0, -1)
    glVertex3f(-x, y, -z); glVertex3f(x, y, -z); glVertex3f(x, -y, -z); glVertex3f(-x, -y, -z)
    glNormal3f(1, 0, 0)
    glVertex3f(x, -y, -z); glVertex3f(x, y, -z); glVertex3f(x, y, z); glVertex3f(x, -y, z)
    glNormal3f(-1, 0, 0)
    glVertex3f(-x, -y, z); glVertex3f(-x, y, z); glVertex3f(-x, y, -z); glVertex3f(-x, -y, -z)
    glNormal3f(0, 1, 0)
    glVertex3f(-x, y, z); glVertex3f(x, y, z); glVertex3f(x, y, -z); glVertex3f(-x, y, -z)
    glNormal3f(0, -1, 0)
    glVertex3f(-x, -y, -z); glVertex3f(x, -y, -z); glVertex3f(x, -y, z); glVertex3f(-x, -y, z)
    glEnd()

def draw_cylinder_between(quad, p1, p2, r1, r2=None, slices=14):
    if r2 is None: r2 = r1
    vx, vy, vz = p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    if length < 1e-6: return
    glPushMatrix()
    glTranslatef(p1[0], p1[1], p1[2])
    ax, ay = -vy, vx
    ang = math.degrees(math.acos(max(-1.0, min(1.0, vz / length))))
    if abs(ax) > 1e-9 or abs(ay) > 1e-9: glRotatef(ang, ax, ay, 0.0)
    elif vz < 0: glRotatef(180.0, 1.0, 0.0, 0.0)
    gluCylinder(quad, r1, r2, length, slices, 1)
    glPopMatrix()

ARM = 1.25  

# MAPPING SPATIAL: (sx, sy)
# M1: Adelante Izquierda (-1, 1)
# M2: Abajo Izquierda   (-1, -1)
# M3: Abajo Derecha     (1, -1)
# M4: Adelante Derecha   (1, 1)
MOTORS = [(1, 1), (-1, 1), (-1, -1), (1, -1)]  

def draw_drone(quad, prop_angles, motor_speeds):
    glColor3f(0.88, 0.88, 0.90)
    glPushMatrix()
    glScalef(1.0, 0.95, 0.34)
    gluSphere(quad, 0.62, 32, 24)
    glPopMatrix()
    glColor3f(0.96, 0.96, 0.98)
    glPushMatrix()
    glTranslatef(0.0, 0.0, 0.19)
    draw_box(0.52, 0.52, 0.07)
    glPopMatrix()

    for i, (sx, sy) in enumerate(MOTORS):
        is_left = (sx < 0)  # Motores izquierdos (flecha naranja) llevan capuchón naranja
        
        glColor3f(0.80, 0.80, 0.83)
        draw_cylinder_between(quad, (0, 0, 0.0), (sx * ARM, sy * ARM, 0.0), 0.10, 0.07)
        if is_left: glColor3f(0.95, 0.45, 0.10)
        else: glColor3f(0.25, 0.25, 0.28)
        glPushMatrix()
        glTranslatef(sx * ARM, sy * ARM, -0.12)
        gluCylinder(quad, 0.17, 0.17, 0.30, 20, 1)
        glTranslatef(0, 0, 0.30)
        gluDisk(quad, 0.0, 0.17, 20, 1)
        glPopMatrix()

    glColor3f(0.82, 0.82, 0.85)
    for sy in (-1, 1):
        draw_cylinder_between(quad, (0.35, sy * 0.25, -0.12), (0.46, sy * 0.58, -0.98), 0.04)
        draw_cylinder_between(quad, (-0.35, sy * 0.25, -0.12), (-0.46, sy * 0.58, -0.98), 0.04)
        draw_cylinder_between(quad, (0.46, sy * 0.58, -0.98), (-0.46, sy * 0.58, -0.98), 0.035)

    glColor3f(0.22, 0.22, 0.26)
    glPushMatrix()
    glTranslatef(0.42, 0.0, -0.30)
    draw_box(0.26, 0.26, 0.24)
    glPopMatrix()
    glColor3f(0.04, 0.04, 0.07)
    glPushMatrix()
    glTranslatef(0.56, 0.0, -0.30)
    gluSphere(quad, 0.075, 14, 14)
    glPopMatrix()

    glDisable(GL_LIGHTING)
    for i, (sx, sy) in enumerate(MOTORS):
        glPushMatrix()
        glTranslatef(sx * ARM, sy * ARM, 0.22)
        glRotatef(prop_angles[i], 0, 0, 1)
        glColor3f(0.10, 0.10, 0.12)
        glBegin(GL_QUADS)
        glVertex3f(-0.78, -0.05, 0); glVertex3f(0.78, -0.05, 0)
        glVertex3f(0.78, 0.05, 0);   glVertex3f(-0.78, 0.05, 0)
        glEnd()
        glColor3f(0.6, 0.6, 0.65)
        glBegin(GL_QUADS)
        glVertex3f(-0.07, -0.07, 0.01); glVertex3f(0.07, -0.07, 0.01)
        glVertex3f(0.07, 0.07, 0.01);   glVertex3f(-0.07, 0.07, 0.01)
        glEnd()
        glPopMatrix()
    glEnable(GL_LIGHTING)

    glDisable(GL_LIGHTING)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glDepthMask(GL_FALSE)
    for i, (sx, sy) in enumerate(MOTORS):
        alpha = min(0.25, motor_speeds[i] * 0.003)
        if alpha > 0:
            glColor4f(0.7, 0.8, 1.0, alpha)
            glPushMatrix()
            glTranslatef(sx * ARM, sy * ARM, 0.225)
            gluDisk(quad, 0.0, 0.78, 36, 1)
            glPopMatrix()
    glDepthMask(GL_TRUE)
    glDisable(GL_BLEND)
    glEnable(GL_LIGHTING)

def draw_grid():
    glDisable(GL_LIGHTING)
    y = -1.6
    glBegin(GL_LINES)
    for i in range(-8, 9):
        c = 0.22 if i % 4 else 0.35
        glColor3f(c, c, c + 0.04)
        glVertex3f(i, y, -8); glVertex3f(i, y, 8)
        glVertex3f(-8, y, i); glVertex3f(8, y, i)
    glEnd()
    glEnable(GL_LIGHTING)

def draw_text(font, text, x, y, color=(230, 230, 235)):
    try:
        surf = font.render(text, True, color).convert_alpha()
        w, h = surf.get_size()
        
        if hasattr(pygame.image, "tobytes"):
            data = pygame.image.tobytes(surf, "RGBA", False)
        else:
            data = pygame.image.tostring(surf, "RGBA", False)
            
        tex = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, tex)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data)
        
        glEnable(GL_TEXTURE_2D)
        glColor4f(1.0, 1.0, 1.0, 1.0)
        
        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 0.0); glVertex2f(x, y)
        glTexCoord2f(1.0, 0.0); glVertex2f(x + w, y)
        glTexCoord2f(1.0, 1.0); glVertex2f(x + w, y + h)
        glTexCoord2f(0.0, 1.0); glVertex2f(x, y + h)
        glEnd()
        
        glDisable(GL_TEXTURE_2D)
        
        try:
            glDeleteTextures(tex)
        except:
            glDeleteTextures(1, [tex])
    except Exception:
        pass

def draw_rect(x, y, w, h, color):
    glDisable(GL_TEXTURE_2D)
    glBegin(GL_QUADS)
    glColor4f(color[0]/255.0, color[1]/255.0, color[2]/255.0, 1.0)
    glVertex2f(x, y); glVertex2f(x+w, y)
    glVertex2f(x+w, y+h); glVertex2f(x, y+h)
    glEnd()

def draw_hud(font, big_font, reader, roll, pitch, yaw, alt, show_yaw):
    glDisable(GL_LIGHTING)
    glDisable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    
    glMatrixMode(GL_PROJECTION)
    glPushMatrix()
    glLoadIdentity()
    gluOrtho2D(0, WIDTH, HEIGHT, 0) 
    glMatrixMode(GL_MODELVIEW)
    glPushMatrix()
    glLoadIdentity()

    draw_text(big_font, "Roll  %+7.2f deg" % roll, 20, 20, (255, 120, 120))
    draw_text(big_font, "Pitch %+7.2f deg" % pitch, 20, 50, (120, 255, 140))
    draw_text(big_font, "Yaw   %+7.2f deg" % yaw if show_yaw else "Yaw   (oculto)", 20, 80, (130, 170, 255))
    draw_text(big_font, "Alt   %+7.2f m  " % alt, 20, 110, (240, 240, 100))
    draw_text(font, f"Estado: {reader.status} ({reader.rate} Hz)", 20, HEIGHT - 30, (170, 175, 190))

    panel_x = WIDTH - 250
    draw_rect(panel_x, 0, 250, HEIGHT, (30, 30, 35))
    draw_text(big_font, "MOTORES", panel_x + 75, 30, (255, 255, 255))
    
    motors = [reader.m1, reader.m2, reader.m3, reader.m4]
    labels = ["M1 (FI)", "M2 (TI)", "M3 (TD)", "M4 (FD)"]
    
    for i in range(4):
        y_base = 80 + i * 50
        draw_text(font, labels[i], panel_x + 10, y_base, (200, 200, 200))
        draw_rect(panel_x + 85, y_base, 100, 15, (60, 60, 60))
        draw_rect(panel_x + 85, y_base, motors[i], 15, (50, 200, 50) if motors[i] > 0 else (200, 50, 50))
        draw_text(font, f"{motors[i]}%", panel_x + 195, y_base, (255, 255, 255))

    glPopMatrix()
    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)
    
    glDisable(GL_BLEND)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM9")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    reader = RFController(args.port, args.baud)
    reader.start()

    pygame.init()
    pygame.display.set_caption("Estacion Base RF (Con Potenciometro)")
    pygame.display.set_mode((WIDTH, HEIGHT), DOUBLEBUF | OPENGL)
    glViewport(0, 0, WIDTH, HEIGHT)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, WIDTH / float(HEIGHT), 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)

    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glEnable(GL_COLOR_MATERIAL)
    glLightfv(GL_LIGHT0, GL_AMBIENT, (0.35, 0.35, 0.38, 1.0))
    glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.85, 0.85, 0.85, 1.0))

    quad = gluNewQuadric()
    font = pygame.font.SysFont("consolas,menlo,monospace", 16)
    big_font = pygame.font.SysFont("consolas,menlo,monospace", 22, bold=True)

    cam_yaw, cam_pitch, cam_dist = 205.0, 22.0, 7.5
    disp_roll = disp_pitch = disp_yaw = 0.0
    off_roll = off_pitch = off_yaw = 0.0
    show_yaw = False
    
    prop_angles = [0.0, 0.0, 0.0, 0.0]

    clock = pygame.time.Clock()
    running = True

    while running:
        dt = clock.tick(60) / 1000.0
        mouse_x, mouse_y = pygame.mouse.get_pos()

        for ev in pygame.event.get():
            if ev.type == QUIT: running = False
            elif ev.type == KEYDOWN:
                if ev.key == K_ESCAPE: running = False
                elif ev.key == K_z: off_roll, off_pitch, off_yaw = reader.roll, reader.pitch, reader.yaw
            elif ev.type == MOUSEWHEEL:
                if mouse_x < WIDTH - 250:
                    cam_dist = max(3.0, min(25.0, cam_dist - ev.y * 0.5))
        
        if pygame.mouse.get_pressed()[0] and mouse_x < WIDTH - 250:
            rel_x, rel_y = pygame.mouse.get_rel()
            cam_yaw -= rel_x * 0.4
            cam_pitch = max(-85.0, min(85.0, cam_pitch + rel_y * 0.4))
        else:
            pygame.mouse.get_rel() 

        t_roll, t_pitch = reader.roll - off_roll, reader.pitch - off_pitch
        t_yaw = (reader.yaw - off_yaw) if show_yaw else 0.0

        k = min(1.0, dt * 6.0) 
        
        disp_roll += (t_roll - disp_roll) * k
        disp_pitch += (t_pitch - disp_pitch) * k
        disp_yaw += (t_yaw - disp_yaw) * k
        
        motor_speeds = [reader.m1, reader.m2, reader.m3, reader.m4]
        for i in range(4):
            prop_angles[i] = (prop_angles[i] + (motor_speeds[i] * 30.0) * dt) % 360.0

        glClearColor(0.06, 0.07, 0.10, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        cy, cp = math.radians(cam_yaw), math.radians(cam_pitch)
        eye = (cam_dist * math.cos(cp) * math.cos(cy), cam_dist * math.sin(cp), cam_dist * math.cos(cp) * math.sin(cy))
        gluLookAt(eye[0], eye[1], eye[2], 0, 0, 0, 0, 1, 0)
        glLightfv(GL_LIGHT0, GL_POSITION, (0.4, 1.0, 0.6, 0.0))

        draw_grid()

        glPushMatrix()
        glTranslatef(0.0, 0.0, 0.0) 
        glRotatef(-90.0, 1, 0, 0)                      
        glRotatef(DRAW_YAW_SIGN * disp_yaw, 0, 0, 1)
        glRotatef(DRAW_PITCH_SIGN * disp_pitch, 0, 1, 0)
        glRotatef(DRAW_ROLL_SIGN * disp_roll, 1, 0, 0)
        draw_drone(quad, prop_angles, motor_speeds)
        glPopMatrix()

        draw_hud(font, big_font, reader, disp_roll, disp_pitch, disp_yaw, reader.alt, show_yaw)

        pygame.display.flip()

    if hasattr(reader, "running"): reader.running = False
    pygame.quit()

if __name__ == "__main__":
    main()