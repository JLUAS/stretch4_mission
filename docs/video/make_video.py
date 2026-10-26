#!/usr/bin/env python3
"""Explainer video for stretch4_mission (Spanish, 1920x1080, 30 fps, no audio).

    python3 make_video.py                 # -> ../explicacion_stretch4_mission.mp4 (H.264) and .webm (VP9)
    python3 make_video.py --preview       # -> preview/*.png, a few key frames per scene

Edit the texts in the scene functions and re-run. Needs Pillow, ffmpeg (libx264) and the Ubuntu font.
Colors: Hello Robot palette. No logos.
"""

import argparse
import math
import subprocess
from functools import lru_cache
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

W, H, FPS = 1920, 1080, 30
HERE = Path(__file__).resolve().parent
OUT = HERE.parent / 'explicacion_stretch4_mission.mp4'

NAVY, PINK, BLUE, TEAL = (0x12, 0x28, 0x37), (0xFB, 0xCB, 0xDA), (0x05, 0x50, 0xB3), (0x0A, 0x68, 0x8A)
WHITE, BG = (0xFF, 0xFF, 0xFF), (0xF6, 0xF8, 0xFA)
GREY, LINE, SOFT = (0x5D, 0x66, 0x75), (0xD9, 0xDD, 0xE4), (0xE8, 0xEE, 0xF5)
AUTHORS = 'José Luis  ·  Samuel'

FONTS = {'R': '/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf', 'M': '/usr/share/fonts/truetype/ubuntu/Ubuntu-M.ttf',
         'B': '/usr/share/fonts/truetype/ubuntu/Ubuntu-B.ttf', 'L': '/usr/share/fonts/truetype/ubuntu/Ubuntu-L.ttf',
         'C': '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf',
         # fallback for symbols Ubuntu lacks (→ ▾ ✓)
         'SR': '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 'SB': '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'}
SYMBOLS = set('→▾✓')


@lru_cache(maxsize=None)
def font(weight, size):
    return ImageFont.truetype(FONTS[weight], size)


def ease(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3 - 2 * x)


def mix(a, b, k):
    return tuple(round(a[i] + (b[i] - a[i]) * k) for i in range(3))


class Frame:
    """Drawing on solid backgrounds: fading = blending a color toward what is under it (fast, no layers)."""

    def __init__(self, t, bg):
        self.t = t
        self.img = Image.new('RGB', (W, H), bg)
        self.d = ImageDraw.Draw(self.img)

    def k(self, start, dur=0.6):
        return ease((self.t - start) / dur)

    def text(self, xy, s, size, color, under, start=0.0, weight='R', anchor='la', dy=22):
        a = self.k(start)
        if a > 0:
            if SYMBOLS & set(s) and weight != 'C':
                weight, size = ('SB' if weight in 'BM' else 'SR'), round(size * 0.92)
            self.d.text((xy[0], xy[1] + (1 - a) * dy), s, font=font(weight, size), fill=mix(under, color, a), anchor=anchor)

    def para(self, xy, s, size, color, under, start=0.0, maxw=800, weight='R', spacing=1.38):
        y = xy[1]
        for line in wrap(s, size, weight, maxw):
            self.text((xy[0], y), line, size, color, under, start, weight)
            y += size * spacing
        return y

    def box(self, rect, fill, under, start=0.0, radius=18, outline=None, width=3, dy=22):
        a = self.k(start)
        if a <= 0:
            return
        x0, y0, x1, y1 = rect
        off = (1 - a) * dy
        self.d.rounded_rectangle((x0, y0 + off, x1, y1 + off), radius, fill=mix(under, fill, a),
                                 outline=mix(under, outline, a) if outline else None, width=width)

    def arrow(self, p0, p1, color, under, start=0.0, width=5, dur=0.5):
        a = self.k(start, dur)
        if a <= 0:
            return
        x = p0[0] + (p1[0] - p0[0]) * a
        y = p0[1] + (p1[1] - p0[1]) * a
        c = mix(under, color, min(1, a * 2))
        self.d.line((p0, (x, y)), fill=c, width=width)
        ang = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        hs = 16
        pts = [(x, y), (x - hs * math.cos(ang - 0.45), y - hs * math.sin(ang - 0.45)),
               (x - hs * math.cos(ang + 0.45), y - hs * math.sin(ang + 0.45))]
        self.d.polygon(pts, fill=c)

    def circle(self, c, r, fill, under, start=0.0, outline=None, width=3):
        a = self.k(start)
        if a > 0:
            self.d.ellipse((c[0] - r, c[1] - r, c[0] + r, c[1] + r), fill=mix(under, fill, a) if fill else None,
                           outline=mix(under, outline, a) if outline else None, width=width)


def wrap(s, size, weight, maxw):
    f, out, line = font(weight, size), [], ''
    for word in s.split():
        trial = f'{line} {word}'.strip()
        if f.getlength(trial) <= maxw or not line:
            line = trial
        else:
            out.append(line)
            line = word
    return out + ([line] if line else [])


# ---------------------------------------------------------------- shared chrome

def chrome(f, number, title, subtitle=None):
    f.text((120, 92), f'{number:02d}', 30, PINK, BG, 0, 'B')
    f.d.rectangle((120, 136, 168, 142), fill=PINK)
    f.text((190, 82), title, 52, NAVY, BG, 0.05, 'B')
    if subtitle:
        f.text((190, 150), subtitle, 28, GREY, BG, 0.2, 'R')


def footer(f, progress):
    f.d.rectangle((0, H - 56, W, H), fill=NAVY)
    f.d.text((120, H - 28), AUTHORS, font=font('M', 22), fill=PINK, anchor='lm')
    f.d.text((W - 120, H - 28), 'Stretch 4 · misiones multi-política', font=font('R', 22), fill=WHITE, anchor='rm')
    f.d.rectangle((0, H - 60, W, H - 56), fill=LINE)
    f.d.rectangle((0, H - 60, round(W * progress), H - 56), fill=BLUE)


def card(f, rect, title, body, start, accent=BLUE, title_size=34, body_size=26, under=BG):
    f.box(rect, WHITE, under, start, outline=LINE)
    f.box((rect[0], rect[1], rect[0] + 10, rect[3]), accent, under, start, radius=4)
    f.text((rect[0] + 40, rect[1] + 32), title, title_size, NAVY, WHITE, start + 0.15, 'B')
    f.para((rect[0] + 40, rect[1] + 32 + title_size * 1.6), body, body_size, GREY, WHITE, start + 0.3,
           maxw=rect[2] - rect[0] - 80)


def node(f, rect, label, state, start, size=26):
    """state: pending | active | done"""
    fill, text_c, outline = {'pending': (WHITE, GREY, LINE), 'active': (BLUE, WHITE, BLUE),
                             'done': (WHITE, TEAL, TEAL)}[state]
    f.box(rect, fill, BG, start, radius=14, outline=outline, width=4)
    cx, cy = (rect[0] + rect[2]) / 2, (rect[1] + rect[3]) / 2
    a = f.k(start + 0.1)
    if a > 0:
        lines = wrap(label, size, 'M', rect[2] - rect[0] - 30)
        y = cy - (len(lines) - 1) * size * 0.65
        for line in lines:
            f.d.text((cx, y), line, font=font('M', size), fill=mix(fill, text_c, a), anchor='mm')
            y += size * 1.3


# ---------------------------------------------------------------- scenes

def s_title(f, t):
    f.d.rectangle((0, 0, W, H), fill=NAVY)
    f.text((160, 300), 'STRETCH 4  ·  ROS 2 JAZZY  ·  LEROBOT', 30, PINK, NAVY, 0.2, 'M')
    f.text((160, 370), 'Misiones autónomas', 104, WHITE, NAVY, 0.45, 'B')
    f.text((160, 490), 'multi-política', 104, PINK, NAVY, 0.7, 'B')
    a = f.k(1.1, 0.8)
    f.d.rectangle((160, 640, 160 + 520 * a, 646), fill=BLUE)
    f.text((160, 680), 'Navegación Nav2 + políticas entrenadas en una sola misión segura', 36, WHITE, NAVY, 1.3, 'L')
    f.text((160, 820), AUTHORS, 44, PINK, NAVY, 1.9, 'M')
    # decorative path: robot dot travelling to three goals
    pts = [(1450, 840), (1560, 640), (1700, 520), (1640, 330)]
    for i, p in enumerate(pts[1:], 1):
        f.circle(p, 16, None, NAVY, 2.2 + i * 0.25, outline=PINK, width=4)
        f.text((p[0] + 28, p[1] - 18), str(i), 30, PINK, NAVY, 2.2 + i * 0.25, 'B')
    seg = max(0.0, (t - 3.2) / 1.0)
    i, frac = min(int(seg), 2), min(seg - min(int(seg), 2), 1.0)
    p0, p1 = pts[i], pts[i + 1]
    pos = (p0[0] + (p1[0] - p0[0]) * ease(frac), p0[1] + (p1[1] - p0[1]) * ease(frac))
    f.circle(pos, 26, BLUE, NAVY, 2.0)


def s_problem(f, t):
    chrome(f, 1, 'El reto', 'Dos sistemas que ya funcionan por separado')
    card(f, (120, 250, 900, 560), 'Navegación Nav2 (Hello Robot)',
         'Mapa guardado, localización AMCL y evasión de obstáculos. Se usa tal cual: no se reimplementa nada.',
         0.5, BLUE, 38, 31)
    card(f, (1020, 250, 1800, 560), 'Políticas LeRobot (difusión)',
         'Tareas entrenadas, como pick & place de un frasco, que se ejecutan a través del bridge ROS 2 '
         '/lerobot/start · status · stop.', 1.2, TEAL, 38, 31)
    f.arrow((510, 600), (840, 700), NAVY, BG, 2.4)
    f.arrow((1410, 600), (1080, 700), NAVY, BG, 2.4)
    f.box((440, 700, 1480, 850), NAVY, BG, 3.0, radius=20)
    f.text((960, 745), 'Objetivo: una misión que llegue al lugar correcto', 34, WHITE, NAVY, 3.3, 'M', anchor='mm')
    f.text((960, 800), 'y ejecute cada política, en orden y de forma segura', 34, PINK, NAVY, 3.5, 'M', anchor='mm')
    f.box((120, 900, 1800, 975), PINK, BG, 5.0, radius=14)
    f.text((960, 938), 'Regla de oro: Nav2 y una política nunca controlan el robot al mismo tiempo', 30, NAVY, PINK,
           5.3, 'B', anchor='mm')


def s_architecture(f, t):
    chrome(f, 2, 'Arquitectura', 'Un paquete pequeño, stretch4_mission, sobre la infraestructura existente')
    f.box((120, 250, 560, 400), NAVY, BG, 0.4)
    f.text((340, 300), 'UI web', 38, WHITE, NAVY, 0.6, 'B', anchor='mm')
    f.text((340, 350), 'mapa · misiones · estado', 24, PINK, NAVY, 0.7, anchor='mm')
    f.arrow((560, 325), (700, 325), NAVY, BG, 1.2)
    f.box((700, 250, 1220, 400), BLUE, BG, 1.4)
    f.text((960, 300), 'mission_controller', 38, WHITE, BLUE, 1.6, 'B', anchor='mm')
    f.text((960, 350), 'máquina de estados por paso', 24, WHITE, BLUE, 1.7, anchor='mm')
    targets = [((120, 520, 640, 720), 'Nav2', 'NavigateToPose · AMCL · evasión de obstáculos', TEAL, 2.4),
               ((700, 520, 1220, 720), 'stretch_driver', 'FollowJointTrajectory: retraer brazo y subir lift', TEAL, 3.0),
               ((1280, 520, 1800, 720), 'Bridge LeRobot', '/lerobot/start · status · stop + parámetros por política',
                TEAL, 3.6)]
    for rect, title, body, color, st in targets:
        cx = (rect[0] + rect[2]) / 2
        f.arrow((960, 400), (cx, rect[1]), NAVY, BG, st - 0.3)
        card(f, rect, title, body, st, color, 32, 24)
    f.arrow((1540, 720), (1540, 820), NAVY, BG, 4.4)
    f.box((1280, 820, 1800, 960), WHITE, BG, 4.6, outline=LINE)
    f.text((1540, 865), 'Servidor de inferencia', 30, NAVY, WHITE, 4.8, 'B', anchor='mm')
    f.text((1540, 915), '192.168.1.101:8080', 26, GREY, WHITE, 4.9, 'C', anchor='mm')
    f.para((120, 800), 'La UI lanza dos procesos: la sesión de navegación (Nav2 + bridge, se queda corriendo) '
                       'y una misión a la vez sobre ella.', 28, GREY, BG, 5.6, maxw=1060)
    f.para((120, 900), 'Solo interfaces existentes: acciones y servicios de ROS 2.', 28, NAVY, BG, 6.4, maxw=1060,
           weight='M')


def s_mission(f, t):
    chrome(f, 3, 'Qué es una misión', 'Un mapa + pasos en orden. Cada paso = una política')
    f.box((120, 250, 820, 960), NAVY, BG, 0.4)
    code = ['name: limpiar_mesa', 'map: .../maps/lab.yaml', 'steps:', '  - policy: abrir_gabinete',
            '    goal: {x: 1.2, y: 0.4, yaw: 1.57}', "    start_pose: ''", '  - policy: tomar_esponja',
            '    goal: {x: 2.0, y: 0.9, yaw: 0.0}', '  - policy: limpiar_mesa', '    goal: {x: 2.6, y: 1.5, yaw: -1.57}']
    for i, line in enumerate(code):
        color = PINK if line.strip().startswith(('name', 'map', 'steps')) else (WHITE if 'policy' in line else (0xB8, 0xC7, 0xD6))
        f.text((170, 300 + i * 56), line, 27, color, NAVY, 0.8 + i * 0.12, 'C')
    f.text((170, 880), 'missions/limpiar_mesa.yaml', 22, (0x8F, 0xA3, 0xB5), NAVY, 2.0, 'C')
    steps = [('1', 'Abrir gabinete'), ('2', 'Tomar esponja'), ('3', 'Limpiar mesa')]
    for i, (n, name) in enumerate(steps):
        y0, st = 250 + i * 190, 2.4 + i * 0.6
        f.box((900, y0, 1800, y0 + 160), WHITE, BG, st, outline=LINE)
        f.circle((960, y0 + 80), 34, BLUE, WHITE, st + 0.1)
        f.text((960, y0 + 80), n, 34, WHITE, BLUE, st + 0.2, 'B', anchor='mm', dy=0)
        f.text((1020, y0 + 32), name, 34, NAVY, WHITE, st + 0.2, 'B')
        f.text((1020, y0 + 88), 'meta de navegación (x, y, yaw)  ·  pose inicial del brazo', 25, GREY, WHITE, st + 0.35)
    f.box((900, 840, 1800, 960), PINK, BG, 4.6, radius=14)
    f.para((930, 862), 'Cada política vive en config/policies.yaml: tarea, modelo, pose inicial y tiempo máximo. '
                       'La misión se las pasa al bridge en cada paso.', 25, NAVY, PINK, 4.9, maxw=840)


PHASES = [('Retraer brazo', 'El brazo telescópico entra por completo. El gripper y el lift no se tocan, '
                            'así que un objeto que ya se tomó sigue sujeto.'),
          ('Verificar pose', 'Se compara la pose actual (TF map → base_link) con la meta. Si ya está dentro de '
                             'tolerancia, la navegación se salta.'),
          ('Navegar', 'Meta NavigateToPose a Nav2, que evita obstáculos. Antes se ajusta su verificador de meta '
                      'a 0.10 m y 0.08 rad.'),
          ('Detener + subir lift', 'Espera a que la base esté quieta, verifica 0.25 m y 0.15 rad, y sube el lift '
                                   'al máximo para bajar a la pose inicial desde arriba.'),
          ('Pose inicial + política', 'El bridge ejecuta stretch4_pose goto y arranca la política. La misión '
                                      'consulta /lerobot/status hasta ver STOPPED.')]


def s_state_machine(f, t):
    chrome(f, 4, 'La máquina de estados', 'Lo mismo para cada paso, siempre en este orden')
    phase_t = [2.5 + 4.2 * i for i in range(5)]      # step 1 walkthrough
    fast = phase_t[-1] + 4.2                          # steps 2 and 3 run quickly
    for i, (n, name) in enumerate([('1', 'Abrir gabinete'), ('2', 'Tomar esponja'), ('3', 'Limpiar mesa')]):
        x0 = 120 + i * 400
        done = t > fast + 1.0 * (i + 1) if i else t > fast
        active = (i == 0 and t <= fast) or (i and fast + 1.0 * i < t <= fast + 1.0 * (i + 1))
        state = 'active' if active else ('done' if done else 'pending')
        node(f, (x0, 220, x0 + 360, 290), f'{n} · {name}', state, 0.3 + i * 0.15, 26)
    end_k = f.k(fast + 3.1)
    if end_k > 0:
        f.box((1340, 220, 1800, 290), TEAL, BG, fast + 3.1, radius=14)
        f.text((1570, 255), 'COMPLETED', 30, WHITE, TEAL, fast + 3.2, 'B', anchor='mm', dy=0)
    xs = [120 + i * 344 for i in range(5)]
    for i, (name, _) in enumerate(PHASES):
        if t >= fast:
            state = 'done'
        elif t >= phase_t[i]:
            state = 'active' if i == 4 or t < phase_t[i + 1] else 'done'
        else:
            state = 'pending'
        node(f, (xs[i], 360, xs[i] + 300, 500), name, state, 0.8 + i * 0.2, 28)
        if i < 4:
            f.arrow((xs[i] + 302, 430), (xs[i] + 342, 430), GREY, BG, 1.6 + i * 0.1, width=4)
    current = next((i for i in range(4, -1, -1) if phase_t[i] <= t < fast), None)
    if current is not None:
        st = phase_t[current]
        f.box((120, 560, 1800, 800), WHITE, BG, st, outline=BLUE, width=4)
        f.text((170, 600), f'{current + 1}. {PHASES[current][0]}', 40, BLUE, WHITE, st + 0.1, 'B')
        f.para((170, 670), PHASES[current][1], 32, NAVY, WHITE, st + 0.25, maxw=1580)
    elif t >= fast:
        f.box((120, 560, 1800, 800), WHITE, BG, fast, outline=TEAL, width=4)
        f.text((170, 600), 'Paso terminado → siguiente paso', 40, TEAL, WHITE, fast + 0.1, 'B')
        f.para((170, 670), 'La siguiente navegación empieza solo cuando el bridge reportó STOPPED. '
                           '"Terminado" significa que el proceso de la política acabó, no que la tarea salió bien.',
               32, NAVY, WHITE, fast + 0.25, maxw=1580)
    f.box((120, 850, 1800, 950), PINK, BG, 2.0, radius=14)
    f.text((960, 900), 'Cualquier falla → FAILED: no se ejecuta nada después y el robot queda detenido',
           30, NAVY, PINK, 2.2, 'M', anchor='mm')


def s_safety(f, t):
    chrome(f, 5, 'Seguridad y control', 'Quién mueve el robot en cada momento')
    items = [('Secuencial', 'La política solo arranca cuando Nav2 terminó y la base está quieta. '
                            'La siguiente navegación, solo tras STOPPED.', BLUE),
             ('Brazo adentro al navegar', 'Antes de cada navegación el brazo se retrae; el objeto sigue en el gripper.', TEAL),
             ('Lift arriba antes de la pose', 'stretch4_pose goto interpola todas las articulaciones sin revisar '
                                              'colisiones: desde arriba, solo baja.', BLUE),
             ('Se mide, no se supone', 'Se verifica la posición medida de cada articulación; la simulación reporta '
                                       'éxito antes de que el lift se mueva.', TEAL)]
    for i, (title, body, color) in enumerate(items):
        x0, y0 = 120 + (i % 2) * 860, 240 + (i // 2) * 290
        card(f, (x0, y0, x0 + 820, y0 + 250), title, body, 0.5 + i * 0.9, color, 34, 27)
    f.box((120, 840, 1800, 960), NAVY, BG, 4.4, radius=16)
    f.text((960, 880), 'El software no es un sistema de seguridad', 32, PINK, NAVY, 4.6, 'B', anchor='mm')
    f.text((960, 925), 'Siempre una persona con el paro físico a la mano', 28, WHITE, NAVY, 4.8, 'R', anchor='mm')


def s_ui(f, t):
    chrome(f, 6, 'La interfaz web', 'http://<robot>:8090 · todo el flujo desde el navegador')
    under = BG
    # panel 1: navigation
    f.box((120, 220, 560, 560), WHITE, under, 0.3, outline=LINE)
    f.text((150, 245), '1 · Navegación', 28, NAVY, WHITE, 0.4, 'B')
    f.box((150, 300, 530, 350), BG, WHITE, 0.5, radius=8, outline=LINE, width=2)
    f.text((168, 313), 'lab.yaml  ▾', 24, NAVY, BG, 0.6)
    nav_on = t > 2.5
    f.box((150, 370, 400, 420), TEAL if nav_on else BLUE, WHITE, 0.6, radius=8)
    f.text((275, 395), 'navegación activa' if nav_on else 'Iniciar navegación', 22, WHITE,
           TEAL if nav_on else BLUE, 0.7, 'M', anchor='mm', dy=0)
    loc = 'localizando…' if 4.5 < t < 7.5 else ('localizado ✓' if t >= 7.5 else 'Localizar (global)')
    f.box((150, 440, 400, 490), PINK if t >= 7.5 else BG, WHITE, 0.7, radius=8, outline=LINE, width=2)
    f.text((275, 465), loc, 22, NAVY, PINK if t >= 7.5 else BG, 0.8, 'M', anchor='mm', dy=0)
    # map
    mx0, my0, mx1, my1 = 600, 220, 1800, 560
    f.box((mx0, my0, mx1, my1), WHITE, under, 0.4, outline=LINE)
    if f.k(0.6) > 0:
        a = f.k(0.6)
        f.d.rectangle((mx0 + 30, my0 + 30, mx1 - 30, my1 - 30), fill=mix(WHITE, BG, a))
        f.d.rectangle((mx0 + 30, my0 + 30, mx1 - 30, my1 - 30), outline=mix(WHITE, NAVY, a), width=6)
        f.d.rectangle((1000, 440, 1360, 530), fill=mix(WHITE, NAVY, a))       # table
        f.d.rectangle((1500, 250, 1770, 300), fill=mix(WHITE, NAVY, a))       # cabinet
    goals = [(1560, 360, -1.57), (1180, 390, 1.57), (820, 400, 0.0)]
    show_goals = t > 9.0
    for i, (gx, gy, yaw) in enumerate(goals):
        if show_goals:
            st = 9.0 + i * 0.5
            f.arrow((gx, gy), (gx + 46 * math.cos(yaw), gy - 46 * math.sin(yaw)), PINK if i else BLUE, BG, st, 6, 0.3)
            f.text((gx + 14, gy - 40), str(i + 1), 26, NAVY, BG, st, 'B', dy=0)
    # robot motion: spin while localizing, then visit the goals
    rx, ry, ryaw = 760, 330, 0.0
    if 4.5 < t < 7.5:
        ryaw = (t - 4.5) * 2.1
    legs = [(13.0, 15.5), (16.5, 19.0), (20.0, 22.5)]
    path = [(760, 330)] + [(g[0], g[1]) for g in goals]
    for i, (a0, a1) in enumerate(legs):
        if t >= a0:
            k = ease((t - a0) / (a1 - a0))
            rx = path[i][0] + (path[i + 1][0] - path[i][0]) * k
            ry = path[i][1] + (path[i + 1][1] - path[i][1]) * k
            ryaw = math.atan2(-(path[i + 1][1] - path[i][1]), path[i + 1][0] - path[i][0]) if k < 1 else goals[i][2]
    if f.k(1.0) > 0:
        f.d.ellipse((rx - 30, ry - 30, rx + 30, ry + 30), outline=BLUE, width=6)
        f.d.line(((rx, ry), (rx + 46 * math.cos(ryaw), ry - 46 * math.sin(ryaw))), fill=BLUE, width=6)
    # panel 2: missions
    f.box((120, 590, 560, 900), WHITE, under, 0.5, outline=LINE)
    f.text((150, 615), '2 · Misiones', 28, NAVY, WHITE, 0.6, 'B')
    if t > 10.5:
        f.text((150, 680), 'limpiar_mesa', 28, NAVY, WHITE, 10.5, 'M')
        f.text((150, 722), 'abrir → esponja → limpiar', 22, GREY, WHITE, 10.6)
        f.box((150, 770, 290, 820), BLUE, WHITE, 11.0, radius=8)
        f.text((220, 795), 'Iniciar', 24, WHITE, BLUE, 11.1, 'M', anchor='mm', dy=0)
    else:
        f.text((150, 680), '+ Nueva misión', 26, BLUE, WHITE, 0.8, 'M')
        f.text((150, 730), 'paso: política + pose inicial', 22, GREY, WHITE, 0.9)
        f.box((150, 780, 470, 830), BG, WHITE, 9.0, radius=8, outline=BLUE, width=3)
        f.text((310, 805), 'Usar pose del robot', 22, BLUE, BG, 9.1, 'M', anchor='mm', dy=0)
    # state machine panel
    f.box((600, 590, 1800, 900), WHITE, under, 0.6, outline=LINE)
    f.text((630, 615), 'Máquina de estados en vivo', 28, NAVY, WHITE, 0.7, 'B')
    for i in range(3):
        x0 = 630 + i * 390
        a0, a1 = legs[i]
        done = t > a1 + 0.8
        active = a0 - 0.6 <= t <= a1 + 0.8
        f.box((x0, 670, x0 + 360, 870), WHITE, WHITE, 0.8, radius=12,
              outline=TEAL if done else (BLUE if active else LINE), width=4)
        f.text((x0 + 20, 688), ['1. abrir_gabinete', '2. tomar_esponja', '3. limpiar_mesa'][i], 23, NAVY, WHITE, 0.9, 'B')
        for j in range(5):
            pt = a0 - 0.6 + j * ((a1 + 0.8) - (a0 - 0.6)) / 5
            st = 'done' if t > pt + ((a1 + 1.4) - a0) / 5 or done else ('active' if t >= pt else 'pending')
            color = {'done': TEAL, 'active': BLUE, 'pending': LINE}[st]
            y = 730 + j * 27
            f.circle((x0 + 30, y + 10), 7, color, WHITE, 0.9)
            f.text((x0 + 48, y), ['Retraer brazo', 'Verificar pose', 'Navegar', 'Subir lift', 'Pose + política'][j],
                   19, BLUE if st == 'active' else GREY, WHITE, 0.9, 'M' if st == 'active' else 'R', dy=0)
    # captions
    steps = [(0.0, '1 · Elige el mapa e inicia la navegación (Nav2 + bridge)'),
             (4.0, '2 · Localización global: AMCL reparte hipótesis y el robot gira hasta saber dónde está'),
             (8.5, '3 · Crea la misión: cada paso con su política y "Usar pose del robot" como meta'),
             (11.5, '4 · Inicia y sigue la máquina de estados en vivo; Detener = Ctrl-C')]
    cap = [s for s in steps if t >= s[0]][-1]
    f.box((120, 930, 1800, 1000), NAVY, BG, cap[0], radius=12)
    f.text((960, 965), cap[1], 28, WHITE, NAVY, cap[0] + 0.15, 'M', anchor='mm', dy=0)


def s_validation(f, t):
    chrome(f, 7, 'Validación', 'Probado en simulación MuJoCo y con un primer intento en el robot')
    stats = [('3', 'políticas en una misión, completada de principio a fin'), ('14', 'pruebas automáticas'),
             ('0', 'cambios a Nav2 y al driver')]
    for i, (n, label) in enumerate(stats):
        x0 = 120 + i * 570
        f.box((x0, 240, x0 + 530, 440), NAVY, BG, 0.4 + i * 0.4)
        f.text((x0 + 40, 262), n, 96, PINK, NAVY, 0.6 + i * 0.4, 'B')
        f.para((x0 + 40, 380), label, 24, WHITE, NAVY, 0.8 + i * 0.4, maxw=450)
    f.text((120, 490), 'Lo que encontramos y corregimos', 34, NAVY, BG, 2.0, 'B')
    fixes = [('Robot real', 'Nav2 aceptaba 0.34 rad de error de orientación → se ajusta su verificador de meta en tiempo de ejecución.'),
             ('Robot real', 'El Spin de Nav2 se pasaba unos 0.23 rad → se descartó para alinear.'),
             ('Simulación', 'Con pose inicial en el archivo, la misión se bloqueaba esperando a Nav2 → se localiza antes.'),
             ('Simulación', 'La acción reporta éxito antes de mover el lift → se verifica la posición medida.')]
    for i, (where, text) in enumerate(fixes):
        y0, st = 560 + i * 92, 2.6 + i * 0.7
        f.box((120, y0, 1800, y0 + 78), WHITE, BG, st, radius=12, outline=LINE)
        f.box((140, y0 + 18, 330, y0 + 60), TEAL if where == 'Simulación' else BLUE, WHITE, st + 0.1, radius=21)
        f.text((235, y0 + 39), where, 22, WHITE, TEAL if where == 'Simulación' else BLUE, st + 0.15, 'M', anchor='mm', dy=0)
        f.text((360, y0 + 22), text, 26, NAVY, WHITE, st + 0.2)


def s_next(f, t):
    chrome(f, 8, 'Siguiente paso: el robot real', 'Lo que falta antes de una misión completa en Stretch 4')
    todo = [('Aplicar el parche del bridge', 'Agrega el parámetro start_pose; sin definirlo, se comporta igual que hoy.'),
            ('Registrar las políticas', 'Abrir gabinete, tomar esponja y limpiar mesa en policies.yaml, '
                                        'cada una con su pose inicial guardada con stretch4_pose save.'),
            ('Probar la localización global', 'En la simulación no se puede probar: su lidar solo se ve a sí mismo.'),
            ('Confirmar el driver real', 'Nombres de articulaciones (wrist_extension, lift_joint) y altura máxima del lift.')]
    for i, (title, body) in enumerate(todo):
        y0, st = 240 + i * 175, 0.5 + i * 0.9
        f.box((120, y0, 1800, y0 + 150), WHITE, BG, st, outline=LINE)
        f.box((150, y0 + 45, 210, y0 + 105), WHITE, WHITE, st + 0.1, radius=10, outline=BLUE, width=4)
        f.text((250, y0 + 28), title, 34, NAVY, WHITE, st + 0.15, 'B')
        f.para((250, y0 + 80), body, 26, GREY, WHITE, st + 0.3, maxw=1500)


def s_end(f, t):
    f.d.rectangle((0, 0, W, H), fill=NAVY)
    f.text((W / 2, 380), 'Gracias', 120, WHITE, NAVY, 0.3, 'B', anchor='mm')
    a = f.k(0.8, 0.8)
    f.d.rectangle((W / 2 - 260 * a, 470, W / 2 + 260 * a, 476), fill=BLUE)
    f.text((W / 2, 560), AUTHORS, 56, PINK, NAVY, 1.0, 'M', anchor='mm')
    f.text((W / 2, 660), 'stretch4_mission  ·  ~/tec_workspace', 32, (0xB8, 0xC7, 0xD6), NAVY, 1.5, 'C', anchor='mm')
    f.text((W / 2, 720), 'Stretch 4 · ROS 2 Jazzy · Nav2 · LeRobot', 30, WHITE, NAVY, 1.8, 'L', anchor='mm')


SCENES = [(s_title, 7, False), (s_problem, 12, True), (s_architecture, 13, True), (s_mission, 12, True),
          (s_state_machine, 30, True), (s_safety, 12, True), (s_ui, 26, True), (s_validation, 13, True),
          (s_next, 11, True), (s_end, 6, False)]
FADE = 0.5
TOTAL = sum(d for _, d, _ in SCENES)


def render_scene(i, t):
    fn, dur, has_chrome = SCENES[i]
    f = Frame(t, BG)
    fn(f, t)
    if has_chrome:
        start = sum(d for _, d, _ in SCENES[:i])
        footer(f, (start + t) / TOTAL)
    return f.img


def frames():
    for i, (_, dur, _) in enumerate(SCENES):
        for n in range(int(dur * FPS)):
            t = n / FPS
            img = render_scene(i, t)
            if i + 1 < len(SCENES) and t > dur - FADE:  # crossfade into the next scene
                img = Image.blend(img, render_scene(i + 1, 0.0), ease((t - (dur - FADE)) / FADE))
            yield img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--preview', action='store_true')
    args = ap.parse_args()
    if args.preview:
        out = HERE / 'preview'
        out.mkdir(exist_ok=True)
        for i, (fn, dur, _) in enumerate(SCENES):
            for t in (dur * 0.35, dur - FADE - 0.05):
                render_scene(i, t).save(out / f'{i:02d}_{fn.__name__}_{t:04.1f}.png')
        print(f'previews in {out}')
        return
    cmd = ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}',
           '-r', str(FPS), '-i', '-', '-c:v', 'libx264', '-preset', 'medium', '-crf', '18', '-pix_fmt', 'yuv420p',
           '-movflags', '+faststart', str(OUT)]
    enc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    for n, img in enumerate(frames()):
        enc.stdin.write(img.tobytes())
        if n % (FPS * 10) == 0:
            print(f'{n / FPS:5.0f}s / {TOTAL}s', flush=True)
    enc.stdin.close()
    if enc.wait() != 0:
        raise SystemExit('ffmpeg failed')
    print(f'wrote {OUT} ({TOTAL}s)')
    # VP9 copy: plays in Ubuntu's default player (GStreamer) without the extra H.264 codec package
    webm = OUT.with_suffix('.webm')
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-i', str(OUT), '-c:v', 'libvpx-vp9', '-b:v', '0',
                    '-crf', '30', '-row-mt', '1', '-deadline', 'good', '-cpu-used', '4', '-an', str(webm)], check=True)
    print(f'wrote {webm}')


if __name__ == '__main__':
    main()
