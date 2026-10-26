"""Match live lidar scans to a mission step's reference scan.

Points are 2D in base_link. ICP returns the robot pose in the reference frame; correction() converts it to the
base motion that restores the recorded pose.
"""

import math
import warnings

import numpy as np
from scipy.spatial import cKDTree


def median_ranges(range_lists):
    """Per-beam median over several scans of the same lidar; invalid readings ignored."""
    r = np.array(range_lists, dtype=float)
    r[~np.isfinite(r)] = np.nan
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)   # beams with no valid reading stay NaN (dropped later)
        return np.nanmedian(r, axis=0)


def scan_points(ranges, angle_min, angle_increment, range_min, range_max, max_range=8.0, stride=2):
    """Valid beams -> (N, 2) points in the scan frame (every `stride`-th beam: 3600 beams are more than ICP needs)."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + np.arange(len(r)) * angle_increment
    ok = np.isfinite(r) & (r > range_min) & (r < min(range_max, max_range))
    return np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])])[::stride]


def base_points(scans, laser_in_base):
    """Median of several LaserScan messages -> (N, 2) points in base_link. laser_in_base = (x, y, yaw) from TF."""
    last = scans[-1]
    pts = scan_points(median_ranges([m.ranges for m in scans]), last.angle_min, last.angle_increment,
                      last.range_min, last.range_max)
    return transform(pts, *laser_in_base)


def transform(points, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return points @ np.array([[c, s], [-s, c]]) + [x, y]


def icp(current, reference, iterations=50, start_dist=0.5, final_dist=0.05):
    """Find (x, y, yaw) such that transform(current, x, y, yaw) overlays `reference`.

    That is the robot's current pose expressed in the reference robot frame. Point-to-point ICP with a
    shrinking match distance (rejects outliers such as people or moved chairs).
    Returns ((x, y, yaw), inlier_fraction, rms): inliers = current points within final_dist of the reference.
    """
    tree = cKDTree(reference)
    x = y = yaw = 0.0
    for i in range(iterations):
        max_dist = max(final_dist, start_dist * (1 - i / (0.7 * iterations)))
        moved = transform(current, x, y, yaw)
        dist, idx = tree.query(moved, distance_upper_bound=max_dist)
        m = np.isfinite(dist)
        if m.sum() < 10:
            break
        a, b = moved[m], reference[idx[m]]
        ca, cb = a.mean(axis=0), b.mean(axis=0)
        h = (a - ca).T @ (b - cb)
        dyaw = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
        c, s = math.cos(dyaw), math.sin(dyaw)
        rot = np.array([[c, -s], [s, c]])
        t = cb - rot @ ca
        x, y = rot @ [x, y] + t          # compose the increment onto the estimate
        yaw += dyaw
        if abs(dyaw) < 1e-6 and np.hypot(*t) < 1e-6 and max_dist == final_dist:
            break
    dist, _ = tree.query(transform(current, x, y, yaw), distance_upper_bound=final_dist)
    inl = np.isfinite(dist)
    rms = float(np.sqrt(np.mean(dist[inl] ** 2))) if inl.any() else float('inf')
    return (float(x), float(y), float(math.atan2(math.sin(yaw), math.cos(yaw)))), float(inl.mean()), rms


def correction(pose_in_reference):
    """Base motion (dx, dy, dyaw) in the robot's current frame that brings it onto the reference pose."""
    x, y, yaw = pose_in_reference
    c, s = math.cos(yaw), math.sin(yaw)
    return (-(c * x + s * y), -(-s * x + c * y), -yaw)
