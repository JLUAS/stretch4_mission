#!/usr/bin/env python3
"""3D shots for the English explainer: Hello Robot's Stretch 4 MuJoCo model, kinematic playback.

Runs inside the stretch4-jazzy-sim image (MuJoCo 3.5, EGL):
  docker run --rm --runtime=nvidia -e MUJOCO_GL=egl -v $PWD:/out stretch4-jazzy-sim:local \
      bash -lc 'cd /out && python3 shots.py [--preview]'
Writes /out/frames/<shot>/00000.jpg ... and /out/frames/<shot>/labels.json (2D label positions).

CONCEPTUAL ANIMATION: joint values are set directly (no physics, no controllers). The robot uses its
real geometry and joint ranges; grasp poses are solved from its measured kinematics.
"""

import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image
from stretch4_mujoco.stretch4_mujoco_simulator import Stretch4MujocoSimulator as Sim

W, H, FPS = 1920, 1080, 30
PREVIEW = '--preview' in sys.argv
OUT = Path('/out/frames' if not PREVIEW else '/out/preview3d')
PINK = [0.984, 0.796, 0.855]
NAVY = [0.071, 0.157, 0.216]
CYAN = [0.25, 0.80, 0.90]

# ---------------------------------------------------------------- scene

DRAWER_CLOSED = np.array([-1.72, 0.9, 0.65])   # drawer body origin (world) when closed
HANDLE_LOCAL_X = 0.27
SPONGE_IN_DRAWER = np.array([0.02, 0.0, -0.038])
SPONGE_HALF = (0.05, 0.035, 0.022)


def build():
    Sim.get_robot_xml_path()  # Stretch 4 MJCF generated from its URDF, as the sim driver does
    spec = mujoco.MjSpec.from_file(Sim.get_scene_xml_path())
    for key in list(spec.keys):  # bodies are added below, so the stored keyframes no longer fit
        spec.delete(key)
    wb = spec.worldbody
    box, cyl = mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_CYLINDER
    white, front, dark = [0.93, 0.94, 0.96, 1], PINK + [1], NAVY + [1]
    wb.add_light(pos=[0.5, -0.5, 2.9], dir=[0, 0.2, -1], diffuse=[0.55] * 3, specular=[0.1] * 3, castshadow=True)
    wb.add_light(pos=[-1.2, 1.5, 2.6], dir=[0.3, -0.3, -1], diffuse=[0.35] * 3, castshadow=False)

    cab = wb.add_body(name='cabinet', pos=[-1.72, 0.9, 0.0])
    for size, pos, rgba in [((0.22, 0.40, 0.27), (0, 0, 0.27), white),            # lower carcass
                            ((0.23, 0.41, 0.02), (0, 0, 0.77), white),            # top
                            ((0.22, 0.02, 0.115), (0, 0.39, 0.655), white),       # sides of the drawer bay
                            ((0.22, 0.02, 0.115), (0, -0.39, 0.655), white),
                            ((0.01, 0.40, 0.115), (-0.215, 0, 0.655), white),     # back
                            ((0.004, 0.37, 0.235), (0.222, 0, 0.275), [0.85, 0.87, 0.9, 1]),  # lower door panel
                            ((0.01, 0.05, 0.01), (0.232, 0, 0.45), dark)]:        # lower door knob
        cab.add_geom(type=box, size=size, pos=pos, rgba=rgba)
    dr = cab.add_body(name='drawer', pos=list(DRAWER_CLOSED - [-1.72, 0.9, 0.0]))
    dr.add_joint(name='drawer_slide', type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[1, 0, 0], range=[0, 0.35])
    for size, pos, rgba in [((0.20, 0.34, 0.01), (0, 0, -0.07), white), ((0.012, 0.37, 0.085), (0.212, 0, 0), front),
                            ((0.01, 0.34, 0.07), (-0.19, 0, 0), white), ((0.20, 0.01, 0.07), (0, 0.34, 0), white),
                            ((0.20, 0.01, 0.07), (0, -0.34, 0), white),
                            ((0.025, 0.008, 0.008), (0.245, 0, 0.05), dark), ((0.025, 0.008, 0.008), (0.245, 0, -0.05), dark)]:
        dr.add_geom(type=box, size=size, pos=pos, rgba=rgba)
    dr.add_geom(type=cyl, size=[0.012, 0.06, 0], pos=[HANDLE_LOCAL_X, 0, 0], rgba=dark)   # vertical bar handle

    sp = wb.add_body(name='sponge', pos=[0, 0, -1])
    sp.add_freejoint()
    sp.add_geom(type=box, size=[SPONGE_HALF[0], SPONGE_HALF[1], 0.015], pos=[0, 0, -0.007], rgba=[0.98, 0.80, 0.20, 1])
    sp.add_geom(type=box, size=[SPONGE_HALF[0], SPONGE_HALF[1], 0.007], pos=[0, 0, 0.015], rgba=[0.15, 0.55, 0.30, 1])

    m = spec.compile()
    m.vis.global_.offwidth, m.vis.global_.offheight = W, H
    m.vis.quality.shadowsize = 4096
    m.vis.quality.offsamples = 8
    m.vis.headlight.ambient[:] = [0.32] * 3
    m.vis.headlight.diffuse[:] = [0.35] * 3
    return m


m = build()
d = mujoco.MjData(m)
Q = {n: m.joint(n).qposadr[0] for n in
     ['floating_base', 'lift_joint', 'arm_l1_joint', 'arm_l2_joint', 'arm_l3_joint', 'arm_l4_joint', 'wrist_yaw_joint',
      'wrist_pitch_joint', 'wrist_roll_joint', 'gripper_finger_right_joint', 'gripper_finger_left_joint', 'drawer_slide']}
SPONGE_Q = m.joint(m.body('sponge').jntadr[0]).qposadr[0]
BASE_Z = 0.0565


def quat_yaw(yaw):
    return np.array([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])


def tip():
    return (d.body('gripper_fingertip_right_link').xpos + d.body('gripper_fingertip_left_link').xpos) / 2


def apply(s):
    d.qpos[:] = m.qpos0
    x, y, yaw = s['base']
    d.qpos[Q['floating_base']:Q['floating_base'] + 7] = [x, y, BASE_Z, *quat_yaw(yaw)]
    d.qpos[Q['lift_joint']] = s['lift']
    for i in range(1, 5):
        d.qpos[Q[f'arm_l{i}_joint']] = s['arm'] / 4
    d.qpos[Q['wrist_yaw_joint']], d.qpos[Q['wrist_pitch_joint']], d.qpos[Q['wrist_roll_joint']] = s['wrist']
    d.qpos[Q['gripper_finger_right_joint']] = d.qpos[Q['gripper_finger_left_joint']] = s['grip']
    d.qpos[Q['drawer_slide']] = s['drawer']
    mujoco.mj_kinematics(m, d)
    if s['sponge'] == 'drawer':
        pos, yaw_s = d.body('drawer').xpos + SPONGE_IN_DRAWER, 0.0
    elif s['sponge'] == 'grip':
        pos, yaw_s = tip() + [0, 0, 0.004], yaw + s['wrist'][0]
    else:
        pos, yaw_s = np.array(s['sponge']), yaw
    d.qpos[SPONGE_Q:SPONGE_Q + 7] = [*pos, *quat_yaw(yaw_s)]
    mujoco.mj_forward(m, d)


# Fingertip offset in the base frame for a wrist pitch, at lift = 0 and arm = 0 (affine in lift and arm).
def tip_offset(pitch):
    apply(dict(base=(0, 0, 0), lift=0.0, arm=0.0, wrist=(0, pitch, 0), grip=0.0, drawer=0, sponge=(9, 9, -1)))
    return tip() - [0, 0, 0]


C0 = {0.0: tip_offset(0.0), 1.57: tip_offset(1.57)}


def solve(target, pitch, arm, yaw=math.pi):
    """Base (x, y) and lift that put the fingertips on `target` with this arm extension and pitch."""
    c = C0[pitch]
    rot = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    bxy = np.array(target[:2]) - rot @ np.array([c[0] + arm, c[1]])
    return (float(bxy[0]), float(bxy[1])), float(target[2] - c[2])


def table_top():
    gid = np.zeros(1, dtype=np.int32)
    apply(dict(base=(9, 9, 0), lift=0, arm=0, wrist=(0, 0, 0), grip=0, drawer=0, sponge=(9, 9, -1)))
    return 2.5 - mujoco.mj_ray(m, d, np.array([0.3, -1.1, 2.5]), np.array([0, 0, -1.0]), None, 1, -1, gid)


TABLE_Z = table_top()
HANDLE = DRAWER_CLOSED + [HANDLE_LOCAL_X, 0, 0]
OPEN = 0.25
SPONGE_OPEN = DRAWER_CLOSED + [OPEN, 0, 0] + SPONGE_IN_DRAWER
(G1, LIFT_HANDLE) = solve(HANDLE, 0.0, 0.30)
(G2, LIFT_SPONGE) = solve(SPONGE_OPEN + [0, 0, 0.008], 1.57, 0.40)
WIPE = np.array([0.35, -1.1, TABLE_Z + SPONGE_HALF[2] + 0.002])
(G3, LIFT_WIPE) = solve(WIPE, 1.57, 0.38)
START = (1.25, 1.55, -math.pi / 2)
YAW = math.pi
print(f'table top {TABLE_Z:.3f}  G1 {np.round(G1, 3)} lift {LIFT_HANDLE:.3f}  G2 {np.round(G2, 3)} lift {LIFT_SPONGE:.3f}'
      f'  G3 {np.round(G3, 3)} lift {LIFT_WIPE:.3f}')

# ---------------------------------------------------------------- animation helpers


def ss(x):
    x = min(1.0, max(0.0, x))
    return x * x * (3 - 2 * x)


def keys(t, ks):
    """Piecewise smoothstep through (time, value) keys."""
    if t <= ks[0][0]:
        return ks[0][1]
    for (t0, v0), (t1, v1) in zip(ks, ks[1:]):
        if t <= t1:
            k = ss((t - t0) / (t1 - t0)) if t1 > t0 else 1.0
            return (np.asarray(v0, float) + (np.asarray(v1, float) - np.asarray(v0, float)) * k).tolist() \
                if isinstance(v0, (tuple, list)) else v0 + (v1 - v0) * k
    return ks[-1][1]


def spline(points, n=200):
    """Catmull-Rom through points -> dense polyline."""
    p = [np.array(points[0])] + [np.array(q) for q in points] + [np.array(points[-1])]
    out = []
    for i in range(1, len(p) - 2):
        for u in np.linspace(0, 1, n // (len(p) - 3), endpoint=False):
            out.append(0.5 * ((2 * p[i]) + (-p[i - 1] + p[i + 1]) * u + (2 * p[i - 1] - 5 * p[i] + 4 * p[i + 1] - p[i + 2]) * u ** 2
                              + (-p[i - 1] + 3 * p[i] - 3 * p[i + 1] + p[i + 2]) * u ** 3))
    out.append(np.array(points[-1]))
    return np.array(out)


def along(poly, s):
    seg = np.linalg.norm(np.diff(poly, axis=0), axis=1)
    cum = np.concatenate([[0], np.cumsum(seg)])
    target = s * cum[-1]
    i = min(np.searchsorted(cum, target) - 1, len(seg) - 1)
    i = max(i, 0)
    k = (target - cum[i]) / seg[i] if seg[i] > 0 else 0
    return poly[i] + (poly[i + 1] - poly[i]) * k


def ang(a0, a1, k):
    diff = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
    return a0 + diff * k


P_S_G1 = spline([START[:2], (0.35, 1.15), G1])
P_G1_G2 = spline([G1, G2], 20)
P_G2_G3 = spline([G2, (-0.2, 0.35), (1.15, -0.15), (1.2, -0.75), G3])


def drive(t, t0, t1, poly, yaw0, yaw1):
    k = ss((t - t0) / (t1 - t0))
    x, y = along(poly, k)
    return (float(x), float(y), ang(yaw0, yaw1, ss(min(1, k * 1.6))))


STOW = dict(lift=0.35, arm=0.0, wrist=(0, 0, 0), grip=0.0)

# ---------------------------------------------------------------- per-shot state


def state(**kw):
    s = dict(base=START, lift=0.35, arm=0.0, wrist=(0.0, 0.0, 0.0), grip=0.0, drawer=0.0, sponge='drawer')
    s.update(kw)
    return s


def step1_policy(t, t0):
    """Open drawer: pre-grasp start pose -> extend to handle -> close -> pull (arm retracts, drawer slides) -> release."""
    u = t - t0
    arm = keys(u, [(0, 0.0), (1.0, 0.30), (1.5, 0.30), (2.6, 0.30 - OPEN), (2.9, 0.30 - OPEN)])
    grip = keys(u, [(0, 0.5), (1.0, 0.5), (1.4, 0.12), (2.6, 0.12), (2.9, 0.5)])
    drawer = keys(u, [(1.5, 0.0), (2.6, OPEN)])
    return dict(lift=LIFT_HANDLE, arm=arm, grip=grip, drawer=drawer, wrist=(0, 0, 0))


def step2_policy(t, t0):
    """Pick sponge, top-down: pre-grasp above the drawer -> lower -> close -> lift out."""
    u = t - t0
    lift = keys(u, [(0, LIFT_SPONGE + 0.2), (1.0, LIFT_SPONGE + 0.2), (1.6, LIFT_SPONGE), (2.2, LIFT_SPONGE), (2.9, LIFT_SPONGE + 0.25)])
    arm = keys(u, [(0, 0.0), (1.0, 0.40)])
    grip = keys(u, [(0, 0.5), (1.6, 0.5), (2.1, 0.17)])
    return dict(lift=lift, arm=arm, grip=grip, wrist=(0, 1.57, 0), sponge='grip' if u >= 2.1 else 'drawer', drawer=OPEN)


def step3_policy(t, t0, dur=3.5):
    """Clean table: lower the sponge onto the table, wipe back and forth (arm + wrist yaw)."""
    u = t - t0
    lift = keys(u, [(0, LIFT_WIPE + 0.2), (0.7, LIFT_WIPE)])
    phase = max(0.0, u - 0.7)
    arm = 0.38 + 0.08 * math.sin(phase * 2 * math.pi / 1.4) * min(1, phase)
    wyaw = 0.25 * math.sin(phase * 2 * math.pi / 0.7) * min(1, phase)
    return dict(lift=lift, arm=arm, wrist=(wyaw, 1.57, 0), grip=0.17, sponge='grip', drawer=OPEN)


# ---------------------------------------------------------------- camera / overlays


def free_cam(lookat, distance, azimuth, elevation):
    c = mujoco.MjvCamera()
    c.lookat[:] = lookat
    c.distance, c.azimuth, c.elevation = distance, azimuth, elevation
    return c


def cam_from(pos, lookat):
    v = np.array(lookat) - np.array(pos)
    dist = np.linalg.norm(v)
    return free_cam(lookat, dist, math.degrees(math.atan2(v[1], v[0])), math.degrees(math.asin(v[2] / dist)))


def project(cam, p):
    az, el = math.radians(cam.azimuth), math.radians(cam.elevation)
    fwd = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    pos = np.array(cam.lookat) - cam.distance * fwd
    right = np.cross(fwd, [0, 0, 1])
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    v = np.array(p) - pos
    z = v @ fwd
    if z <= 0.05:
        return None
    f = (H / 2) / math.tan(math.radians(m.vis.global_.fovy) / 2)
    return [round(W / 2 + f * (v @ right) / z, 1), round(H / 2 - f * (v @ up) / z, 1)]


class Overlay:
    def __init__(self, scn):
        self.scn = scn

    def _next(self):
        if self.scn.ngeom >= self.scn.maxgeom:
            return None
        g = self.scn.geoms[self.scn.ngeom]
        self.scn.ngeom += 1
        return g

    def geom(self, gtype, size, pos, rgba, mat=None):
        g = self._next()
        if g is not None:
            mujoco.mjv_initGeom(g, gtype, np.array(size, float), np.array(pos, float),
                                (np.eye(3) if mat is None else mat).flatten(), np.array(rgba, np.float32))

    def line(self, p0, p1, width, rgba, gtype=mujoco.mjtGeom.mjGEOM_CAPSULE):
        g = self._next()
        if g is not None:
            mujoco.mjv_initGeom(g, gtype, np.zeros(3), np.zeros(3), np.zeros(9), np.array(rgba, np.float32))
            mujoco.mjv_connector(g, gtype, width, np.array(p0, float), np.array(p1, float))

    def path(self, poly, frac, rgba, z=0.012, width=0.018):
        n = max(1, int(len(poly) * frac))
        for a, b in zip(poly[:n], poly[1:n]):
            self.line([*a, z], [*b, z], width, rgba)

    def goal(self, xy, yaw, rgba, z=0.02, length=0.38):
        self.geom(mujoco.mjtGeom.mjGEOM_CYLINDER, [0.17, 0.004, 0], [xy[0], xy[1], z], [*rgba[:3], 0.35])
        self.line([xy[0], xy[1], z + 0.01], [xy[0] + length * math.cos(yaw), xy[1] + length * math.sin(yaw), z + 0.01],
                  0.03, rgba, mujoco.mjtGeom.mjGEOM_ARROW)


HM = np.load('/out/heightmap.npy')   # 5 cm grid, origin (-2.2, -3.0), from heightmap.py
OCC = [(-2.2 + ix * 0.05 + 0.025, -3.0 + iy * 0.05 + 0.025) for iy, ix in zip(*np.nonzero((HM > 0.05) & (HM < 1.5)))]
OCC += [(x, y) for x in np.arange(-1.92, -1.5, 0.05) for y in np.arange(0.5, 1.3, 0.05)]   # cabinet footprint


def occupancy(ov, alpha):
    if alpha <= 0:
        return
    for x, y in OCC:
        ov.geom(mujoco.mjtGeom.mjGEOM_BOX, [0.024, 0.024, 0.002], [x, y, 0.004], [*PINK, 0.55 * alpha])
    for gx in np.arange(-1.9, 1.95, 0.5):
        ov.line([gx, -2.7, 0.003], [gx, 2.7, 0.003], 0.003, [*CYAN, 0.35 * alpha])
    for gy in np.arange(-2.5, 2.75, 0.5):
        ov.line([-1.95, gy, 0.003], [1.95, gy, 0.003], 0.003, [*CYAN, 0.35 * alpha])


RNG = np.random.default_rng(7)
PARTICLES = RNG.uniform([-1.8, -2.5], [1.8, 2.5], size=(260, 2))
PARTICLE_YAW = RNG.uniform(-math.pi, math.pi, 260)


def particles(ov, k, center, alpha=1.0):
    """AMCL: hypotheses spread over the map converge on the robot (k: 0 spread -> 1 converged)."""
    if alpha <= 0:
        return
    c = np.array(center[:2])
    for p, yaw in zip(PARTICLES, PARTICLE_YAW):
        q = p + (c + (p - c) * 0.06 - p) * k
        a = ang(yaw, center[2], k)
        ov.line([q[0], q[1], 0.03], [q[0] + 0.12 * math.cos(a), q[1] + 0.12 * math.sin(a), 0.03], 0.008,
                [*CYAN, 0.85 * alpha], mujoco.mjtGeom.mjGEOM_ARROW)


# ---------------------------------------------------------------- shots
# Each returns (state, camera, overlay_fn(ov), labels [(text, world_xyz)])


def shot_intro(t):
    s = state(base=(START[0], START[1], -0.6))
    cam = free_cam([0.0, 0.35, 0.55], 3.05, 78 + 14 * ss(t / 12), -27)
    return s, cam, None, [('Stretch 4', [START[0], START[1], 1.05]), ('cabinet', [-1.72, 0.9, 0.95]),
                          ('table', [0.0, -1.0, TABLE_Z + 0.12])]


def shot_navigation(t):
    # 0-1 idle, 1-4.2 AMCL converges, 3.6-5 map, 5-6.4 Nav2 plan, 6.4-13.2 drive, hold
    base = START if t < 6.4 else drive(t, 6.4, 13.2, P_S_G1, START[2], YAW)
    s = state(base=base)
    mid = np.array(base[:2]) * 0.55 + np.array(G1) * 0.45
    cam = free_cam([mid[0], mid[1] - 0.15, 0.3], 3.0 - 0.3 * ss((t - 6) / 7), 72 + 10 * ss(t / 15), -38)

    def ov_fn(ov):
        occupancy(ov, ss((t - 3.6) / 1.4))
        particles(ov, ss((t - 1.2) / 3.0), START, 1 - ss((t - 5.0) / 1.0))
        if t > 5.0:
            ov.path(P_S_G1, ss((t - 5.0) / 1.4), [*PINK, 0.95])
            ov.goal(G1, YAW, [*PINK, 1.0] if t < 13.2 else [0.35, 0.85, 0.55, 1.0])
    labels = [('AMCL', [START[0], START[1] + 0.35, 0.25])] if 1.2 < t < 5.4 else []
    if t > 5.0:
        labels.append(('goal 1', [G1[0], G1[1] - 0.25, 0.08]))
    return s, cam, ov_fn, labels


def shot_pipeline(t):
    # Step 2 -> 3 of the mission, each phase 4.8 s: retract, verify, navigate, stop+lift, policy
    held = dict(sponge='grip', drawer=OPEN, wrist=(0, 1.57, 0), grip=0.17)
    if t < 4.8:      # 1 retract arm, sponge stays in the gripper
        s = state(base=(*G2, YAW), lift=LIFT_SPONGE + 0.25, arm=keys(t, [(0.3, 0.40), (4.0, 0.0)]), **held)
    elif t < 9.6:    # 2 verify pose (overlay compares to goal 3)
        s = state(base=(*G2, YAW), lift=LIFT_SPONGE + 0.25, arm=0.0, **held)
    elif t < 14.4:   # 3 navigate
        s = state(base=drive(t, 9.8, 14.2, P_G2_G3, YAW, YAW), lift=LIFT_SPONGE + 0.25, arm=0.0, **held)
    elif t < 19.2:   # 4 stop, raise lift
        s = state(base=(*G3, YAW), lift=keys(t, [(15.4, LIFT_SPONGE + 0.25), (18.6, 1.2)]), arm=0.0, **held)
    else:            # 5 start pose from the top, then the (conceptual) cleaning policy
        u = t - 19.2
        if u < 1.4:
            s = state(base=(*G3, YAW), lift=keys(u, [(0, 1.2), (1.4, LIFT_WIPE + 0.2)]), arm=0.0, **held)
        else:
            s = state(base=(*G3, YAW), **step3_policy(t, 20.6))
    # camera inside the room: near goal 2 for phases 1-2, moves to the goal-3 view while navigating
    k = ss((t - 9.6) / 4.8)
    pos = np.array([0.45, -0.75, 1.8]) * (1 - k) + np.array([1.55, -2.25, 1.9]) * k
    rb = np.array([*s['base'][:2], 0.75])
    fwd = rb - pos
    right = np.array([fwd[1], -fwd[0], 0]) / np.linalg.norm(fwd[:2])
    cam = cam_from(pos, rb - right * 0.55)         # robot sits right of center; the state panel is on the left

    def ov_fn(ov):
        if 4.8 <= t < 14.4:
            ov.goal(G3, YAW, [*PINK, 1.0])
            if t < 9.6:
                ov.line([*G2, 0.03], [*G3, 0.03], 0.006, [*CYAN, 0.7])
        if 9.6 <= t < 14.4:
            ov.path(P_G2_G3, 1.0, [*PINK, 0.9])
    return s, cam, ov_fn, ([('goal 3', [G3[0], G3[1] - 0.25, 0.1])] if 4.8 <= t < 14.4 else [])


# Full mission, time-compressed (18 s)
T = dict(nav1=(0.0, 2.6), lift1=(2.6, 3.2), pol1=(3.2, 6.1), ret1=(6.1, 6.6), nav2=(6.6, 7.4), lift2=(7.4, 8.0),
         pol2=(8.0, 10.9), ret2=(10.9, 11.6), nav3=(11.6, 14.0), lift3=(14.0, 14.5), pol3=(14.5, 18.0))


def shot_mission(t):
    def inside(k):
        return T[k][0] <= t < T[k][1]
    if t < T['nav1'][1]:
        s = state(base=drive(t, *T['nav1'], P_S_G1, START[2], YAW))
    elif t < T['pol1'][0]:
        s = state(base=(*G1, YAW), lift=keys(t, [(T['lift1'][0], 0.35), (T['lift1'][0] + 0.3, 1.2), (T['lift1'][1], LIFT_HANDLE)]))
    elif t < T['pol1'][1]:
        s = state(base=(*G1, YAW), **step1_policy(t, T['pol1'][0]))
    elif t < T['nav2'][1]:
        s = state(base=drive(t, *T['nav2'], P_G1_G2, YAW, YAW) if t >= T['nav2'][0] else (*G1, YAW),
                  lift=LIFT_HANDLE, arm=0.0, drawer=OPEN, grip=0.0)
    elif t < T['pol2'][0]:
        s = state(base=(*G2, YAW), lift=keys(t, [(T['lift2'][0], LIFT_HANDLE), (T['lift2'][0] + 0.3, 1.2),
                                                  (T['lift2'][1], LIFT_SPONGE + 0.2)]), wrist=(0, 1.57 * ss((t - T['lift2'][0]) / 0.6), 0),
                  drawer=OPEN, grip=0.5)
    elif t < T['pol2'][1]:
        s = state(base=(*G2, YAW), **step2_policy(t, T['pol2'][0]))
    elif t < T['nav3'][1]:
        held = dict(sponge='grip', drawer=OPEN, wrist=(0, 1.57, 0), grip=0.17, lift=LIFT_SPONGE + 0.25)
        arm = keys(t, [(T['ret2'][0], 0.40), (T['ret2'][1], 0.0)])
        s = state(base=drive(t, *T['nav3'], P_G2_G3, YAW, YAW) if t >= T['nav3'][0] else (*G2, YAW), arm=arm, **held)
    elif t < T['pol3'][0]:
        s = state(base=(*G3, YAW), lift=keys(t, [(T['lift3'][0], LIFT_SPONGE + 0.25), (T['lift3'][0] + 0.25, 1.2),
                                                  (T['lift3'][1], LIFT_WIPE + 0.2)]), sponge='grip', drawer=OPEN,
                  wrist=(0, 1.57, 0), grip=0.17)
    else:
        s = state(base=(*G3, YAW), **step3_policy(t, T['pol3'][0]))
    cam = cam_from([1.88, 2.55, 3.0], [-0.35 + 0.15 * ss(t / 18), -0.35, 0.2])

    def ov_fn(ov):
        for k, (key, poly) in enumerate([('nav1', P_S_G1), ('nav2', P_G1_G2), ('nav3', P_G2_G3)]):
            if inside(key):
                ov.path(poly, 1.0, [*PINK, 0.9])
        for i, g in enumerate([G1, G2, G3]):
            ov.goal(g, YAW, [*PINK, 0.9], length=0.25)
    labels = [(str(i + 1), [g[0] + 0.05, g[1] - 0.3, 0.05]) for i, g in enumerate([G1, G2, G3])]
    return s, cam, ov_fn, labels


SHOTS = {'intro': (shot_intro, 12), 'navigation': (shot_navigation, 15), 'pipeline': (shot_pipeline, 24),
         'mission': (shot_mission, 18)}


def main():
    names = [a for a in sys.argv[1:] if not a.startswith('--')] or list(SHOTS)
    scale = 2 if PREVIEW else 1
    r = mujoco.Renderer(m, H // scale, W // scale, max_geom=12000)
    for name in names:
        fn, dur = SHOTS[name]
        out = OUT / name
        out.mkdir(parents=True, exist_ok=True)
        times = [dur * k for k in (0.1, 0.35, 0.6, 0.9)] if PREVIEW else [n / FPS for n in range(int(dur * FPS))]
        labels = []
        for n, t in enumerate(times):
            s, cam, ov_fn, lab = fn(t)
            apply(s)
            r.update_scene(d, cam)
            if ov_fn:
                ov_fn(Overlay(r.scene))
            img = Image.fromarray(r.render())
            img.save(out / (f'{t:05.1f}.jpg' if PREVIEW else f'{n:05d}.jpg'), quality=90)
            labels.append([(text, project(cam, p)) for text, p in lab])
        (out / 'labels.json').write_text(json.dumps(labels))
        print(f'{name}: {len(times)} frames', flush=True)


if __name__ == '__main__':
    main()
