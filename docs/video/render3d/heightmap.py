import mujoco, numpy as np
from PIL import Image
from stretch4_mujoco.stretch4_mujoco_simulator import Stretch4MujocoSimulator as Sim
Sim.get_robot_xml_path()
m = mujoco.MjModel.from_xml_path(Sim.get_scene_xml_path()); d = mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m, d, 0)
robot_root = m.body('base_link').rootid[0]
print('non-robot bodies:', [(m.body(i).name, np.round(m.body_pos[i], 2)) for i in range(1, m.nbody) if m.body_rootid[i] != robot_root])
d.qpos[0] = 100.0  # robot out of the way
mujoco.mj_forward(m, d)
res, x0, x1, y0, y1 = 0.05, -2.2, 2.2, -3.0, 3.0
xs, ys = np.arange(x0, x1, res), np.arange(y0, y1, res)
hm = np.zeros((len(ys), len(xs)))
gid = np.zeros(1, dtype=np.int32)
for iy, y in enumerate(ys):
    for ix, x in enumerate(xs):
        dist = mujoco.mj_ray(m, d, np.array([x, y, 2.5]), np.array([0, 0, -1.0]), None, 1, -1, gid)
        hm[iy, ix] = 2.5 - dist if dist >= 0 else -1
np.save('/out/heightmap.npy', hm)
img = (np.clip(hm, 0, 2.0) / 2.0 * 255).astype(np.uint8)[::-1]
Image.fromarray(img).resize((img.shape[1] * 6, img.shape[0] * 6), Image.NEAREST).save('/out/heightmap.png')
print('grid', hm.shape, 'origin', (x0, y0), 'res', res)
for lo, hi, name in [(0.3, 1.1, 'table/cabinet-height'), (1.1, 2.4, 'tall')]:
    mask = (hm > lo) & (hm < hi)
    if mask.any():
        iy, ix = np.nonzero(mask)
        print(name, 'x', round(xs[ix.min()], 2), '..', round(xs[ix.max()], 2), ' y', round(ys[iy.min()], 2), '..', round(ys[iy.max()], 2))
