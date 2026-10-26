import math

import numpy as np

from stretch4_mission.scan_align import correction, icp, median_ranges, scan_points, transform


def room(rng):
    """Walls of a 6 x 4 m room plus a table and a cabinet, as dense points (reference frame = robot at origin)."""
    pts = []
    for x0, y0, x1, y1 in [(-2, -1.5, 4, -1.5), (4, -1.5, 4, 2.5), (4, 2.5, -2, 2.5), (-2, 2.5, -2, -1.5),
                           (1.0, -0.6, 1.8, -0.6), (1.8, -0.6, 1.8, 0.2),          # table corner
                           (-1.2, 1.2, -0.6, 1.2), (-0.6, 1.2, -0.6, 2.5)]:        # cabinet
        n = int(np.hypot(x1 - x0, y1 - y0) / 0.01)
        t = np.linspace(0, 1, n)
        pts.append(np.column_stack([x0 + (x1 - x0) * t, y0 + (y1 - y0) * t]))
    return np.vstack(pts) + rng.normal(0, 0.005, (sum(len(p) for p in pts), 2))


def test_icp_recovers_offset_and_correction_undoes_it():
    rng = np.random.default_rng(1)
    ref = room(rng)
    true = (0.18, -0.12, math.radians(9))            # robot is here, in the reference frame
    # what the robot sees now: the same world, expressed in its displaced frame
    c, s = math.cos(true[2]), math.sin(true[2])
    cur = (room(rng) - true[:2]) @ np.array([[c, -s], [s, c]])
    cur = cur[rng.random(len(cur)) > 0.3]            # different beams hit, partial view
    cur = np.vstack([cur, rng.uniform(-1, 1, (200, 2)) + [2.5, 0.5]])  # a person / moved chairs
    est, inliers, rms = icp(cur, ref)
    assert abs(est[0] - true[0]) < 0.005 and abs(est[1] - true[1]) < 0.005
    assert abs(est[2] - true[2]) < math.radians(0.3)
    assert inliers > 0.7 and rms < 0.02
    # applying the correction (a move in the robot frame) lands on the reference pose
    dx, dy, dyaw = correction(est)
    x = true[0] + math.cos(true[2]) * dx - math.sin(true[2]) * dy
    y = true[1] + math.sin(true[2]) * dx + math.cos(true[2]) * dy
    assert math.hypot(x, y) < 0.006 and abs(true[2] + dyaw) < math.radians(0.3)


def test_unrelated_scan_has_few_inliers():
    rng = np.random.default_rng(2)
    _, inliers, _ = icp(rng.uniform(-3, 3, (800, 2)), room(rng))
    assert inliers < 0.4


def test_scan_points_and_median():
    ranges = median_ranges([[1.0, float('inf'), 2.0, 0.1], [1.2, 3.0, float('nan'), 0.1], [1.1, 3.0, 2.0, 0.1]])
    assert np.allclose(ranges[:3], [1.1, 3.0, 2.0])
    pts = scan_points(ranges, 0.0, math.pi / 2, 0.25, 30.0, stride=1)   # beam 3 (0.1 m) is below range_min
    assert np.allclose(pts, [[1.1, 0.0], [0.0, 3.0], [-2.0, 0.0]], atol=1e-9)
    assert np.allclose(transform(np.array([[1.0, 0.0]]), 0.5, 0.0, math.pi / 2), [[0.5, 1.0]])
