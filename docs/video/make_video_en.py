#!/usr/bin/env python3
"""English explainer (120 s, 1920x1080, captions instead of a voice-over).

    docker run ... shots.py   # first: renders render3d/frames/<shot>/ (see render3d/shots.py)
    python3 make_video_en.py            # -> ../stretch4_mission_orchestration_en.mp4 + .webm
    python3 make_video_en.py --preview  # -> preview_en/*.png, key frames only

3D shots: Hello Robot's Stretch 4 MuJoCo model, kinematic playback (conceptual animation).
The dashboard in scene 3 is conceptual; the UI in scene 8 is a real screenshot from a MuJoCo simulation run.
"""

import argparse
import json
import math
import subprocess
from functools import lru_cache
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from make_video import FPS, H, W, Frame, ease, font, mix, wrap

HERE = Path(__file__).resolve().parent
FRAMES = HERE / 'render3d' / 'frames'
ASSETS = HERE / 'assets'
OUT = HERE.parent / 'stretch4_mission_orchestration_en.mp4'

NAVY, PINK, WHITE = (0x12, 0x28, 0x37), (0xFB, 0xCB, 0xDA), (0xFF, 0xFF, 0xFF)
CYAN, BLUE, BG = (0x5C, 0xCF, 0xE0), (0x05, 0x50, 0xB3), (0xF6, 0xF8, 0xFA)
GREY, LINE, SOFT, NAVY2 = (0x5D, 0x66, 0x75), (0xD9, 0xDD, 0xE4), (0xB8, 0xC7, 0xD6), (0x1C, 0x3A, 0x4E)
OKG, BAD = (0x4C, 0xC0, 0x8A), (0xE5, 0x5B, 0x5B)
AUTHORS = 'José Luis  ·  Samuel'
# goals solved in render3d/shots.py (printed when it runs)
GOALS = [(-0.689, 0.813, math.pi), (-0.879, 0.813, math.pi), (0.901, -1.187, math.pi)]


def k(t, start, dur=0.6):
    return ease((t - start) / dur)


def k_out(t, end, dur=0.5):
    return 1 - ease((t - (end - dur)) / dur)


# ---------------------------------------------------------------- overlay on photographic frames

class Layer:
    """RGBA overlays (shapes, then text) composited over a background image."""

    def __init__(self, base, t):
        self.base, self.t = base.convert('RGBA'), t
        self.shapes = Image.new('RGBA', (W, H), (0, 0, 0, 0))
        self.texts = Image.new('RGBA', (W, H), (0, 0, 0, 0))
        self.ds, self.dt = ImageDraw.Draw(self.shapes), ImageDraw.Draw(self.texts)

    def rect(self, box, color, alpha, radius=14, outline=None, width=3):
        if alpha > 0:
            self.ds.rounded_rectangle(box, radius, fill=(*color, round(255 * alpha)),
                                      outline=(*outline, round(255 * alpha)) if outline else None, width=width)

    def circle(self, c, r, color, alpha):
        if alpha > 0:
            self.ds.ellipse((c[0] - r, c[1] - r, c[0] + r, c[1] + r), fill=(*color, round(255 * alpha)))

    def text(self, xy, s, size, color, alpha=1.0, weight='R', anchor='la'):
        if alpha > 0:
            if set('→✓▾') & set(s) and weight != 'C':
                weight = 'SB' if weight in 'BM' else 'SR'
            self.dt.text(xy, s, font=font(weight, size), fill=(*color, round(255 * alpha)), anchor=anchor)

    def para(self, xy, s, size, color, alpha, maxw, weight='R', spacing=1.35):
        y = xy[1]
        for line in wrap(s, size, weight, maxw):
            self.text((xy[0], y), line, size, color, alpha, weight)
            y += size * spacing
        return y

    def result(self):
        return Image.alpha_composite(Image.alpha_composite(self.base, self.shapes), self.texts).convert('RGB')


def captions(L, t, chunks, cx=W / 2, maxw=1500):
    for start, end, text in chunks:
        if start <= t < end:
            a = min(k(t, start, 0.3), k_out(t, end, 0.3))
            lines = wrap(text, 36, 'M', maxw)
            h = 26 + len(lines) * 50
            y0 = H - 60 - h
            w = max(font('M', 36).getlength(s) for s in lines) + 70
            L.rect((cx - w / 2, y0, cx + w / 2, y0 + h), NAVY, 0.84 * a, radius=12)
            for i, line in enumerate(lines):
                L.text((cx, y0 + 13 + 25 + i * 50), line, 36, WHITE, a, 'M', 'mm')


def tag(L, text, color=PINK, alpha=1.0):
    w = font('B', 20).getlength(text) + 36
    L.rect((40, 34, 40 + w, 74), NAVY, 0.82 * alpha, radius=20)
    L.text((58, 54), text, 20, color, alpha, 'B', 'lm')


def scene_chip(L, number, title, alpha=1.0):
    L.rect((40, 88, 40 + font('B', 30).getlength(title) + 110, 140), WHITE, 0.92 * alpha, radius=12)
    L.text((62, 114), f'{number:02d}', 26, BLUE, alpha, 'B', 'lm')
    L.text((112, 114), title, 30, NAVY, alpha, 'B', 'lm')


def pin(L, xy, text, alpha, color=WHITE):
    if xy is None or alpha <= 0:
        return
    x, y = xy
    w = font('M', 24).getlength(text) + 28
    L.rect((x - w / 2, y - 22, x + w / 2, y + 22), NAVY, 0.8 * alpha, radius=22)
    L.text((x, y), text, 24, color, alpha, 'M', 'mm')


@lru_cache(maxsize=8)
def labels(shot):
    return json.loads((FRAMES / shot / 'labels.json').read_text())


def frame3d(shot, t):
    n = min(int(t * FPS), len(labels(shot)) - 1)
    return Image.open(FRAMES / shot / f'{n:05d}.jpg').convert('RGB'), dict((a, b) for a, b in labels(shot)[n])


# ---------------------------------------------------------------- scenes (t is local time)

def s1_intro(t):
    img, lab = frame3d('intro', t)
    L = Layer(img, t)
    # title over a navy veil, 0-4.8 s
    a = k_out(t, 5.0, 0.6)
    if a > 0:
        L.rect((0, 0, W, H), NAVY, 0.72 * a, radius=0)
        L.text((140, 330), 'STRETCH 4  ·  ROS 2  ·  NAV2  ·  LEROBOT', 28, PINK, a * k(t, 0.2), 'M')
        L.text((140, 390), 'Stretch 4 —', 84, WHITE, a * k(t, 0.4), 'B')
        L.text((140, 490), 'Autonomous Mission Orchestration', 84, WHITE, a * k(t, 0.6), 'B')
        L.text((140, 590), 'with Nav2 and LeRobot', 84, PINK, a * k(t, 0.8), 'B')
        L.rect((140, 718, 140 + 560 * k(t, 1.0, 0.8), 724), CYAN, a, radius=2)
        L.text((140, 760), AUTHORS, 40, WHITE, a * k(t, 1.3), 'M')
    # the two capabilities, shown apart
    b = k(t, 5.4)
    if b > 0:
        for x0, title, sub, color in [(80, 'ROS 2 Nav2', 'Autonomous navigation', CYAN),
                                      (1380, 'LeRobot', 'Learned manipulation', PINK)]:
            L.rect((x0, 600, x0 + 460, 760), NAVY, 0.86 * b, radius=16)
            L.rect((x0, 600, x0 + 10, 760), color, b, radius=4)
            L.text((x0 + 40, 645), title, 40, WHITE, b, 'B')
            L.text((x0 + 40, 705), sub, 30, color, b, 'M')
        c = k(t, 8.0)
        L.rect((W / 2 - 190, 655, W / 2 + 190, 705), NAVY, 0.7 * c, radius=25)
        L.text((W / 2, 680), 'separate subsystems', 30, WHITE, c * 0.95, 'M', 'mm')
        L.ds.line((560, 680, 765, 680), fill=(*WHITE, round(160 * c)), width=3)
        L.ds.line((1155, 680, 1360, 680), fill=(*WHITE, round(160 * c)), width=3)
        for name in ('Stretch 4', 'cabinet', 'table'):
            pin(L, lab.get(name), name, k(t, 6.0) * k_out(t, 12.0))
        tag(L, 'CONCEPTUAL ANIMATION · Stretch 4 MuJoCo model', alpha=b)
    captions(L, t, [(0.5, 6.0, 'Mobile robots can navigate autonomously, and learned policies can perform manipulation tasks.'),
                    (6.0, 12.0, 'But combining both into a reliable, sequential workflow requires coordination.')])
    return L.result()


def box2d(f, rect, title, sub, start, accent, fill=NAVY2, under=NAVY, tsize=30, ssize=22):
    f.box(rect, fill, under, start, radius=14, outline=accent, width=3)
    cx = (rect[0] + rect[2]) / 2
    f.text((cx, rect[1] + 38), title, tsize, WHITE, fill, start + 0.15, 'B', anchor='mm')
    y = rect[1] + 80
    for line in wrap(sub, ssize, 'R', rect[2] - rect[0] - 40):
        f.text((cx, y), line, ssize, SOFT, fill, start + 0.3, 'R', anchor='mm')
        y += ssize * 1.35


def pulse(f, p0, p1, t, start, dur, color):
    """A dot travelling along a connection = a command being sent."""
    u = (t - start) / dur
    if 0 <= u <= 1:
        x, y = p0[0] + (p1[0] - p0[0]) * ease(u), p0[1] + (p1[1] - p0[1]) * ease(u)
        f.d.ellipse((x - 9, y - 9, x + 9, y + 9), fill=color)


def s2_architecture(t):
    f = Frame(t, NAVY)
    f.text((W / 2, 70), 'System architecture', 46, WHITE, NAVY, 0.1, 'B', anchor='mm')
    f.text((W / 2, 118), 'ROS 2 Jazzy · existing actions and services, no new interfaces', 26, SOFT, NAVY, 0.3, anchor='mm')
    ctrl = (610, 160, 1310, 270)
    f.box(ctrl, NAVY2, NAVY, 0.5, radius=16, outline=PINK, width=4)
    f.text((960, 198), 'stretch4_mission — Mission Controller', 34, WHITE, NAVY2, 0.65, 'B', anchor='mm')
    f.text((960, 240), 'sequences every step · one subsystem in control at a time', 24, PINK, NAVY2, 0.8, anchor='mm')
    Y0, Y1 = 380, 530
    cols = [((80, Y0, 500, Y1), 'Web Interface', 'Mission configuration and monitoring', CYAN, 1.4),
            ((540, Y0, 960, Y1), 'Nav2', 'Localization · path planning · obstacle avoidance', CYAN, 1.8),
            ((1000, Y0, 1420, Y1), 'Stretch Driver', 'Base and joint control (arm retract, lift)', CYAN, 2.2),
            ((1460, Y0, 1840, Y1), 'LeRobot Bridge', 'Policy execution · /lerobot/start · status · stop', PINK, 2.6)]
    for rect, title, sub, accent, st in cols:
        cx = (rect[0] + rect[2]) / 2
        f.arrow((960 + (cx - 960) * 0.3, 270), (cx, rect[1]), SOFT, NAVY, st - 0.2, width=4)
        box2d(f, rect, title, sub, st, accent)
    # robot + inference
    robot = (540, 690, 1420, 790)
    f.box(robot, NAVY2, NAVY, 3.2, radius=16, outline=SOFT, width=3)
    f.text((980, 725), 'Stretch 4', 32, WHITE, NAVY2, 3.35, 'B', anchor='mm')
    f.text((980, 765), 'mobile base · lift · telescopic arm · wrist · gripper', 22, SOFT, NAVY2, 3.45, anchor='mm')
    f.arrow((750, Y1), (750, 690), SOFT, NAVY, 3.4, width=4)
    f.text((765, 600), 'cmd_vel', 22, SOFT, NAVY, 3.6, 'C')
    f.arrow((1210, Y1), (1210, 690), SOFT, NAVY, 3.6, width=4)
    f.text((1225, 600), 'trajectories', 22, SOFT, NAVY, 3.8, 'C')
    client = (1460, 570, 1840, 650)
    f.arrow((1650, Y1), (1650, 570), SOFT, NAVY, 4.0, width=4)
    box2d(f, client, 'Inference client', 'launched by the bridge', 4.1, PINK, tsize=26, ssize=20)
    f.arrow((1500, 650), (1420, 735), SOFT, NAVY, 4.5, width=4)
    f.text((1395, 672), 'policy commands', 20, SOFT, NAVY, 4.7, anchor='rm')
    server = (1460, 790, 1840, 870)
    f.box(server, NAVY, NAVY, 5.0, radius=14, outline=PINK, width=2)
    f.text((1650, 818), 'LeRobot inference server', 24, WHITE, NAVY, 5.1, 'B', anchor='mm')
    f.text((1650, 850), 'separate GPU machine · 192.168.1.101:8080', 17, SOFT, NAVY, 5.2, 'C', anchor='mm')
    f.arrow((1700, 650), (1700, 790), PINK, NAVY, 5.3, width=3)
    f.arrow((1600, 790), (1600, 650), PINK, NAVY, 5.5, width=3)
    # command flow: one subsystem at a time
    if t > 6.5:
        u = (t - 6.5) % 4.5
        pulse(f, (960, 270), (750, Y0), u, 0.0, 0.9, CYAN)
        pulse(f, (960, 270), (1210, Y0), u, 1.5, 0.9, CYAN)
        pulse(f, (960, 270), (1650, Y0), u, 3.0, 0.9, PINK)
        f.text((1330, 215), 'one command flow at a time', 24, PINK, NAVY, 6.5, 'M', anchor='lm')
    L = Layer(f.img, t)
    captions(L, t, [(0.5, 10.0, "We developed stretch4_mission, a ROS 2 orchestration layer that coordinates Hello Robot's "
                                 'existing navigation stack, robot driver, and LeRobot policy execution'),
                    (10.0, 16.0, 'without rewriting them.')])
    return L.result()


@lru_cache(maxsize=1)
def map_image(scale):
    hm = np.load(HERE / 'render3d' / 'heightmap.npy')      # rows: y -3.0.. (0.05 m), cols: x -2.2..
    ys, xs = np.arange(hm.shape[0]) * 0.05 - 3.0 + 0.025, np.arange(hm.shape[1]) * 0.05 - 2.2 + 0.025
    X, Y = np.meshgrid(xs, ys)
    inside = (np.abs(X) < 1.95) & (np.abs(Y) < 2.74)
    occ = ((hm > 0.05) & (hm < 1.5)) | ((X > -1.94) & (X < -1.5) & (Y > 0.5) & (Y < 1.3))
    img = np.full(hm.shape + (3,), 205, np.uint8)            # unknown
    img[inside] = 250
    img[inside & occ] = NAVY
    edge = inside & ~(np.roll(inside, 1, 0) & np.roll(inside, -1, 0) & np.roll(inside, 1, 1) & np.roll(inside, -1, 1))
    img[edge] = NAVY
    return Image.fromarray(img[::-1]).resize((hm.shape[1] * scale, hm.shape[0] * scale), Image.NEAREST)


def s3_mission(t):
    f = Frame(t, BG)
    f.text((80, 100), 'Defining a mission', 46, NAVY, BG, 0.1, 'B')
    f.text((560, 116), 'A map + an ordered sequence of navigation goals and learned policies', 26, GREY, BG, 0.3)
    # map panel
    scale = 5
    img = map_image(scale)
    mx, my = 140, 210
    f.box((mx - 20, my - 20, mx + img.width + 20, my + img.height + 20), WHITE, BG, 0.4, radius=14, outline=LINE)
    if k(t, 0.6) > 0:
        f.img.paste(Image.blend(Image.new('RGB', img.size, WHITE), img, k(t, 0.6)), (mx, my))
    px = lambda x, y: (mx + (x + 2.2) / 0.05 * scale, my + img.height - (y + 3.0) / 0.05 * scale)  # noqa: E731
    f.text((mx, my + img.height + 32), 'occupancy grid of the room · 5 cm cells', 22, GREY, BG, 0.8)
    names = ['Open Cabinet', 'Pick Up Sponge', 'Clean Table']
    for i, (gx, gy, yaw) in enumerate(GOALS):
        st = 1.4 + i * 0.9
        a = k(t, st)
        if a > 0:
            u, v = px(gx, gy)
            off = (-38, -40) if i == 0 else ((-38, 40) if i == 1 else (40, 0))
            c = mix(WHITE, BLUE if i == 1 else PINK if i == 0 else CYAN, a)
            f.d.ellipse((u - 12, v - 12, u + 12, v + 12), fill=c)
            u2, v2 = u + 46 * math.cos(yaw) * a, v - 46 * math.sin(yaw) * a
            f.d.line((u, v, u2, v2), fill=mix(WHITE, NAVY, a), width=5)
            f.d.polygon([(u2 - 10, v2), (u2 + 2, v2 - 8), (u2 + 2, v2 + 8)], fill=mix(WHITE, NAVY, a))
            f.d.ellipse((u + off[0] - 18, v + off[1] - 18, u + off[0] + 18, v + off[1] + 18), fill=mix(WHITE, NAVY, a))
            f.d.text((u + off[0], v + off[1]), str(i + 1), font=font('B', 22), fill=mix(NAVY, WHITE, a), anchor='mm')
    sx, sy = px(1.25, 1.55)
    if k(t, 1.0) > 0:
        f.d.ellipse((sx - 14, sy - 14, sx + 14, sy + 14), outline=mix(WHITE, BLUE, k(t, 1.0)), width=4)
        f.text((sx - 22, sy), 'robot', 20, BLUE, WHITE, 1.0, 'M', anchor='rm', dy=0)
    # steps
    x0 = 820
    f.text((x0, 200), 'Mission: “Clean the Table”', 34, NAVY, BG, 0.8, 'B')
    for i, ((gx, gy, yaw), name) in enumerate(zip(GOALS, names)):
        y0, st = 262 + i * 212, 1.6 + i * 1.0
        accent = [PINK, BLUE, CYAN][i]
        f.box((x0, y0, 1840, y0 + 190), WHITE, BG, st, radius=14, outline=LINE)
        f.box((x0, y0, x0 + 10, y0 + 190), accent, BG, st, radius=4)
        f.text((x0 + 40, y0 + 22), f'{i + 1}.  Navigate → {name}', 32, NAVY, WHITE, st + 0.1, 'B')
        rows = [('Navigation goal', f'x {gx:+.2f} m · y {gy:+.2f} m · yaw {yaw:.2f} rad'),
                ('Policy', ['open_cabinet', 'pick_up_sponge', 'clean_table'][i]),
                ('Initial arm pose', ['open_cabinet.json', 'pick_up_sponge.json', 'clean_table.json'][i])]
        for j, (kname, val) in enumerate(rows):
            f.text((x0 + 40, y0 + 78 + j * 36), kname, 24, GREY, WHITE, st + 0.25 + j * 0.1, 'M')
            f.text((x0 + 300, y0 + 78 + j * 36), val, 24, NAVY, WHITE, st + 0.25 + j * 0.1, 'C')
    L = Layer(f.img, t)
    tag(L, 'CONCEPTUAL VIEW · the real web UI appears at the end')
    captions(L, t, [(0.5, 7.0, 'A mission combines a map with an ordered sequence of navigation goals and learned policies.'),
                    (7.0, 13.0, 'Each step specifies where the robot must go and which task it must execute.')])
    return L.result()


def s4_navigation(t):
    img, lab = frame3d('navigation', t)
    L = Layer(img, t)
    tag(L, 'CONCEPTUAL ANIMATION · Stretch 4 MuJoCo model')
    a = k(t, 0.3)
    cw = font('SB', 30).getlength('Nav2 → Autonomous Navigation') + 60
    L.rect((W - 40 - cw, 34, W - 40, 110), NAVY, 0.85 * a, radius=14)
    L.text((W - 40 - cw + 30, 72), 'Nav2 → Autonomous Navigation', 32, WHITE, a, 'B', 'lm')
    stages = [(1.2, 5.0, 'AMCL localization: pose hypotheses converge on the robot'),
              (5.0, 6.4, 'Nav2 plans a route on the map'),
              (6.4, 13.2, 'The robot follows the planned path; Nav2 avoids obstacles'),
              (13.2, 15.0, 'Goal reached: the controller verifies the final pose')]
    for s0, s1, text in stages:
        if s0 <= t < s1:
            b = min(k(t, s0, 0.3), k_out(t, s1, 0.3))
            w = font('M', 28).getlength(text) + 50
            L.rect((W - 40 - w, 126, W - 40, 180), WHITE, 0.92 * b, radius=12)
            L.text((W - 40 - w + 25, 153), text, 28, NAVY, b, 'M', 'lm')
    if t > 13.2:
        b = k(t, 13.3)
        L.rect((W - 640, 200, W - 40, 340), NAVY, 0.88 * b, radius=14)
        L.text((W - 610, 232), 'Mission controller check', 26, PINK, b, 'B')
        L.text((W - 610, 274), 'position error ≤ 0.25 m   ✓', 26, WHITE, b, 'M')
        L.text((W - 610, 310), 'heading error ≤ 0.15 rad   ✓', 26, WHITE, b, 'M')
    for name in ('AMCL', 'goal 1'):
        pin(L, lab.get(name), name, 1.0, CYAN if name == 'AMCL' else PINK)
    captions(L, t, [(0.3, 6.5, 'Nav2 handles localization, route planning, and navigation.'),
                    (6.5, 15.0, 'The mission controller verifies that the robot reaches the required position and '
                                'orientation before manipulation can begin.')])
    return L.result()


PHASES = [('Retract Arm', 'Telescopic arm fully in. Gripper and lift are not commanded: the sponge stays held.'),
          ('Verify Pose', 'Current pose vs. goal 3 (TF map → base_link). Already there? Navigation is skipped.'),
          ('Navigate', 'NavigateToPose goal to Nav2, which plans the route and avoids obstacles.'),
          ('Stop & Raise Lift', 'Wait until the base is stationary, check the arrival tolerance, raise the lift to the top.'),
          ('Execute Policy', 'The bridge moves to the initial arm pose, starts the LeRobot policy; '
                             'the controller polls /lerobot/status until STOPPED.')]


def smooth_keys(t, ks):
    if t <= ks[0][0]:
        return ks[0][1]
    for (t0, v0), (t1, v1) in zip(ks, ks[1:]):
        if t <= t1:
            return v0 + (v1 - v0) * ease((t - t0) / (t1 - t0))
    return ks[-1][1]


def s5_pipeline(t):
    img, lab = frame3d('pipeline', t)
    L = Layer(img, t)
    L.rect((40, 160, 660, 940), NAVY, 0.86 * k(t, 0.1), radius=18)
    L.text((80, 200), 'MISSION STEP 3 · CLEAN TABLE', 22, PINK, k(t, 0.2), 'B')
    L.text((80, 236), 'Five-phase pipeline', 36, WHITE, k(t, 0.3), 'B')
    current = min(int(t / 4.8), 4)
    for i, (name, _) in enumerate(PHASES):
        y0 = 300 + i * 118
        a = k(t, 0.4 + i * 0.12)
        done, active = i < current, i == current
        color = PINK if active else (OKG if done else SOFT)
        L.rect((80, y0, 620, y0 + 96), BLUE if active else NAVY2, 0.95 * a, radius=12,
               outline=PINK if active else None, width=3)
        L.circle((122, y0 + 48), 20, color, a)
        L.text((122, y0 + 48), '✓' if done else str(i + 1), 22, NAVY, a, 'B', 'mm')
        L.text((160, y0 + 30), name, 28, WHITE, a, 'B')
        L.text((160, y0 + 66), 'done' if done else ('active' if active else 'pending'), 20, color, a, 'M')
        if i < 4:
            L.ds.line((122, y0 + 96, 122, y0 + 118), fill=(*SOFT, round(200 * a)), width=3)
    # detail card for the active phase, top right
    b = k(t, current * 4.8 + 0.1, 0.4)
    L.rect((700, 34, 1880, 178), WHITE, 0.94 * b, radius=14)
    L.text((730, 56), f'{current + 1}. {PHASES[current][0]}', 30, BLUE, b, 'B')
    L.para((730, 100), PHASES[current][1], 24, NAVY, b, maxw=1120)
    # measured values (the controller trusts measured joints, not the action result)
    reading = None
    if current == 0:
        reading = f'measured arm extension  {smooth_keys(t, [(0.3, 0.40), (4.0, 0.0)]):.2f} m'
    elif current == 3:
        reading = f'base speed 0.00 m/s ✓   measured lift  {smooth_keys(t, [(15.4, 0.92), (18.6, 1.20)]):.2f} m'
    elif current == 4:
        reading = '/lerobot/status:  ' + ('RUNNING' if t > 20.6 else 'starting (stretch4_pose goto)…')
    if reading:
        L.rect((700, 192, 700 + font('C', 24).getlength(reading) + 50, 240), NAVY, 0.85 * b, radius=10)
        L.text((725, 216), reading, 24, CYAN, b, 'C', 'lm')
    pin(L, lab.get('goal 3'), 'goal 3', 1.0, PINK)
    tag(L, 'CONCEPTUAL ANIMATION · policy motion is illustrative')
    captions(L, t, [(0.3, 13.0, "Every mission step follows five controlled phases: retract the arm, verify the robot's pose, "
                                'navigate if necessary, stop and prepare the lift, and finally execute the trained policy.'),
                    (13.0, 24.0, 'The next step begins only after policy execution has stopped.')], cx=1280, maxw=1100)
    return L.result()


MISSION_T = [(0.0, 2.6, 0, 'NAV2'), (2.6, 3.2, 0, 'CONTROLLER'), (3.2, 6.1, 0, 'POLICY'), (6.1, 6.6, 1, 'CONTROLLER'),
             (6.6, 7.4, 1, 'NAV2'), (7.4, 8.0, 1, 'CONTROLLER'), (8.0, 10.9, 1, 'POLICY'), (10.9, 11.6, 2, 'CONTROLLER'),
             (11.6, 14.0, 2, 'NAV2'), (14.0, 14.5, 2, 'CONTROLLER'), (14.5, 18.0, 2, 'POLICY')]


def s6_mission(t):
    img, lab = frame3d('mission', t)
    L = Layer(img, t)
    tag(L, 'CONCEPTUAL ANIMATION · time-compressed ×4')
    seg = next((s for s in MISSION_T if s[0] <= t < s[1]), MISSION_T[-1])
    step, who = seg[2], seg[3]
    names = ['Open Cabinet', 'Pick Up Sponge', 'Clean Table']
    L.rect((40, 90, 560, 160), WHITE, 0.94, radius=12)
    L.text((66, 125), f'Step {step + 1}/3 · {names[step]}', 32, NAVY, 1.0, 'B', 'lm')
    lanes = [('NAV2', 'Nav2'), ('CONTROLLER', 'Controller: arm / lift'), ('POLICY', 'LeRobot policy')]
    x = W - 40
    for key, label in reversed(lanes):
        on = key == who
        w = font('B', 24).getlength(label) + 60
        L.rect((x - w, 40, x, 92), (PINK if key == 'POLICY' else CYAN) if on else NAVY, 0.9, radius=26)
        L.text((x - w / 2, 66), label, 24, NAVY if on else SOFT, 1.0, 'B', 'mm')
        x -= w + 14
    L.text((W - 40, 112), 'exactly one in control at any time', 22, WHITE, 0.9, 'M', 'ra')
    for g in ('1', '2', '3'):
        pin(L, lab.get(g), g, 1.0, PINK)
    captions(L, t, [(0.3, 9.0, 'The result is a unified workflow: navigate, manipulate, transition safely between behaviors,'),
                    (9.0, 18.0, 'and continue to the next task without requiring an operator to trigger each step individually.')])
    return L.result()


def s7_exclusive(t):
    f = Frame(t, NAVY)
    f.text((W / 2, 80), 'Exclusive control and failure handling', 46, WHITE, NAVY, 0.1, 'B', anchor='mm')
    x0, x1 = 480, 1820
    lanes = [('NAV2 CONTROL', 260, CYAN), ('LEROBOT POLICY CONTROL', 400, PINK)]
    for name, y, color in lanes:
        f.text((60, y + 22), name, 24, color, NAVY, 0.3, 'B')
        f.box((x0, y, x1, y + 70), NAVY2, NAVY, 0.3, radius=10)
    blocks = [(0.00, 0.17, 0), (0.21, 0.36, 1), (0.40, 0.48, 0), (0.52, 0.66, 1), (0.70, 0.82, 0)]
    play = min(1.0, max(0.0, (t - 0.8) / 4.2))
    stop_at = 0.76      # STOP pressed during the last navigation
    for b0, b1, lane in blocks:
        if play <= b0:
            continue
        e = min(b1, play, stop_at)
        y = lanes[lane][1]
        xa, xb = x0 + (x1 - x0) * b0, x0 + (x1 - x0) * e
        if xb > xa:
            f.d.rounded_rectangle((xa, y + 8, xb, y + 62), 8, fill=lanes[lane][2])
    if play > 0:
        xp = x0 + (x1 - x0) * min(play, stop_at)
        f.d.line((xp, 230, xp, 490), fill=WHITE, width=2)
    f.text((x0, 520), 'Windows alternate and never overlap: the next one starts only after the previous one has stopped.',
           26, SOFT, NAVY, 1.5, 'M')
    if t > 5.2:
        xs = x0 + (x1 - x0) * stop_at
        f.box((xs - 70, 180, xs + 70, 226), BAD, NAVY, 5.2, radius=10)
        f.text((xs, 203), 'STOP', 26, WHITE, BAD, 5.3, 'B', anchor='mm', dy=0)
        f.text((xs - 90, 203), 'cancels the active Nav2 goal, arm move or policy', 24, WHITE, NAVY, 5.5, 'M', anchor='rm', dy=0)
    # failure path
    fy = 620
    f.text((80, fy), 'On failure', 30, WHITE, NAVY, 6.5, 'B')
    chain = [('Navigate', SOFT), ('Nav2 aborts', BAD), ('FAILED', BAD), ('next policy not started', SOFT)]
    x = 80
    for i, (label, color) in enumerate(chain):
        st = 6.8 + i * 0.6
        w = font('B', 28).getlength(label) + 60
        f.box((x, fy + 60, x + w, fy + 130), NAVY2 if color == SOFT else color, NAVY, st, radius=12,
              outline=SOFT if color == SOFT else None, width=2)
        f.text((x + w / 2, fy + 95), label, 28, WHITE, NAVY2 if color == SOFT else color, st + 0.1, 'B', anchor='mm', dy=0)
        if i < 3:
            f.arrow((x + w + 8, fy + 95), (x + w + 62, fy + 95), SOFT, NAVY, st + 0.3, width=4)
        x += w + 70
    f.box((80, 820, 1840, 900), PINK, NAVY, 9.0, radius=14)
    f.text((960, 860), 'Not a certified safety system — physical emergency-stop supervision remains essential',
           28, NAVY, PINK, 9.2, 'B', anchor='mm', dy=0)
    L = Layer(f.img, t)
    captions(L, t, [(0.3, 5.0, 'A central rule prevents navigation and manipulation policies from controlling the robot simultaneously.'),
                    (5.0, 9.0, 'Failed operations stop the mission, while cancellation can interrupt active commands.'),
                    (9.0, 12.0, 'Physical emergency-stop supervision remains essential.')])
    return L.result()


@lru_cache(maxsize=1)
def ui_shot():
    im = Image.open(ASSETS / 'ui_completed_mission.png').convert('RGB')
    im = im.crop((40, 55, im.width - 40, 1085))
    return im.resize((820, round(im.height * 820 / im.width)), Image.LANCZOS)


def s8_final(t):
    if t < 6.6:
        f = Frame(t, BG)
        f.text((70, 34), 'Web interface and current status', 44, NAVY, BG, 0.1, 'B')
        ui = ui_shot()
        a = k(t, 0.3)
        f.box((60, 140, 60 + ui.width + 20, 140 + ui.height + 20), WHITE, BG, 0.3, radius=12, outline=LINE)
        if a > 0:
            f.img.paste(Image.blend(Image.new('RGB', ui.size, WHITE), ui, a), (70, 150))
        f.text((70, 104), 'Actual web UI · screenshot from a MuJoCo simulation run (stand-in policies)',
               22, BLUE, BG, 0.6, 'M')
        x0 = 1080
        inset = frame3d('mission', 15.5)[0].resize((780, 439), Image.LANCZOS)
        b = k(t, 1.0)
        f.box((x0 - 10, 140, x0 + 790, 591), WHITE, BG, 1.0, radius=12, outline=LINE)
        if b > 0:
            f.img.paste(Image.blend(Image.new('RGB', inset.size, WHITE), inset, b), (x0, 146))
        f.text((x0, 604), 'Stretch 4 MuJoCo model (illustration)', 22, GREY, BG, 1.2, 'M')
        f.box((x0 - 10, 648, x0 + 790, 880), NAVY, BG, 1.8, radius=14)
        f.text((x0 + 20, 668), 'Validation status', 28, PINK, NAVY, 2.0, 'B')
        rows = [('✓', 'Full 3-step mission, end to end', 'MuJoCo simulation', OKG),
                ('•', 'Navigation + pose check, first runs', 'physical Stretch 4 · used to tune tolerances', SOFT),
                ('…', 'Full 3-step mission', 'physical robot: pending', PINK)]
        for i, (mark, what, where, color) in enumerate(rows):
            y = 714 + i * 54
            f.text((x0 + 20, y), mark, 28, color, NAVY, 2.3 + i * 0.3, 'SB')
            f.text((x0 + 64, y), what, 24, WHITE, NAVY, 2.3 + i * 0.3, 'M')
            f.text((x0 + 64, y + 30), where, 20, SOFT, NAVY, 2.3 + i * 0.3)
        img = f.img
    else:
        f = Frame(t - 6.6, NAVY)
        f.text((W / 2, 420), 'Stretch 4 Mission Orchestration', 80, WHITE, NAVY, 0.2, 'B', anchor='mm')
        f.text((W / 2, 520), 'ROS 2 · Nav2 · LeRobot · Autonomous Mobile Manipulation', 36, PINK, NAVY, 0.6, 'M', anchor='mm')
        a = k(t - 6.6, 0.9, 0.8)
        f.d.rectangle((W / 2 - 300 * a, 590, W / 2 + 300 * a, 595), fill=CYAN)
        f.text((W / 2, 660), AUTHORS, 40, WHITE, NAVY, 1.2, 'M', anchor='mm')
        img = f.img
    L = Layer(img, t)
    captions(L, t, [(0.3, 6.6, 'Validated end-to-end in simulation, the architecture provides a foundation for coordinating '
                                'learned manipulation and autonomous navigation on the Stretch 4.')], cx=W / 2, maxw=1500)
    return L.result()


SCENES = [(s1_intro, 12), (s2_architecture, 16), (s3_mission, 13), (s4_navigation, 15), (s5_pipeline, 24),
          (s6_mission, 18), (s7_exclusive, 12), (s8_final, 10)]
FADE = 0.5
TOTAL = sum(d for _, d in SCENES)


def frames():
    for i, (fn, dur) in enumerate(SCENES):
        nxt = None
        for n in range(int(dur * FPS)):
            t = n / FPS
            img = fn(t)
            if i + 1 < len(SCENES) and t > dur - FADE:
                nxt = nxt or SCENES[i + 1][0](0.0)
                img = Image.blend(img, nxt, ease((t - (dur - FADE)) / FADE))
            yield img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--preview', action='store_true')
    args = ap.parse_args()
    if args.preview:
        out = HERE / 'preview_en'
        out.mkdir(exist_ok=True)
        for i, (fn, dur) in enumerate(SCENES):
            for t in (dur * 0.3, dur * 0.7, dur - FADE - 0.05):
                fn(t).save(out / f'{i:02d}_{fn.__name__}_{t:04.1f}.png')
        print(f'previews in {out}')
        return
    cmd = ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}', '-r', str(FPS),
           '-i', '-', '-c:v', 'libx264', '-preset', 'medium', '-crf', '18', '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
           str(OUT)]
    enc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    for n, img in enumerate(frames()):
        enc.stdin.write(img.tobytes())
        if n % (FPS * 10) == 0:
            print(f'{n / FPS:5.0f}s / {TOTAL}s', flush=True)
    enc.stdin.close()
    if enc.wait() != 0:
        raise SystemExit('ffmpeg failed')
    webm = OUT.with_suffix('.webm')
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-i', str(OUT), '-c:v', 'libvpx-vp9', '-b:v', '0', '-crf', '30',
                    '-row-mt', '1', '-deadline', 'good', '-cpu-used', '4', '-an', str(webm)], check=True)
    print(f'wrote {OUT} and {webm} ({TOTAL}s)')


if __name__ == '__main__':
    main()
