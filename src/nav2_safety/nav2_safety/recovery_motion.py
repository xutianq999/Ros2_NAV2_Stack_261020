"""Low-speed escape policy, mirrored by recovery_clearance.hpp in the planner."""
import math

import numpy as np


def rectangle_distance(points, x, y, yaw, half_length, half_width):
    c, s = math.cos(yaw), math.sin(yaw)
    px, py = points[:, 0] - x, points[:, 1] - y
    dx = np.abs(c * px + s * py) - half_length
    dy = np.abs(-s * px + c * py) - half_width
    return (np.hypot(np.maximum(dx, 0), np.maximum(dy, 0))
            + np.minimum(np.maximum(dx, dy), 0))


def blocking(points, vx, wz, horizon, half_length, half_width, clearance,
             sample_step=0.005):
    """Count unique points intruded upon by the whole rectangular footprint.

    An already close point may remain equally close or recede. Actual footprint
    intrusion always blocks. Starting close never permits moving closer.
    """
    if points.size == 0:
        return 0
    initial = rectangle_distance(points, 0, 0, 0, half_length, half_width)
    sweep = (abs(vx) + math.hypot(half_length, half_width) * abs(wz)) * horizon
    nearby = initial <= sweep + clearance
    points, initial = points[nearby], initial[nearby]
    if initial.size == 0:
        return 0
    required = np.minimum(initial, clearance)
    hit = initial <= 0.0
    samples = max(1, math.ceil(sweep / sample_step))
    for t in np.linspace(0, horizon, samples + 1)[1:]:
        yaw = wz * t
        if abs(wz) < 1e-8:
            x, y = vx * t, 0.0
        else:
            x, y = vx / wz * math.sin(yaw), vx / wz * (1 - math.cos(yaw))
        distance = rectangle_distance(points, x, y, yaw, half_length, half_width)
        hit |= (distance <= 0.0) | (distance < required - 1e-6)
    return int(np.count_nonzero(hit))


def limit_motion(points, vx, wz, *, horizon=0.5, half_length=0.345,
                 half_width=0.285, clearance=0.025, min_points=2,
                 max_linear=0.15, max_angular=0.8,
                 min_linear=0.02, min_angular=0.04):
    # Preserve curvature even when a requested command exceeds the speed caps.
    cap = min(1.0, max_linear / max(abs(vx), 1e-12),
              max_angular / max(abs(wz), 1e-12))
    for ratio in (1.0, 0.75, 0.5, 0.25, 0.125):
        factor = cap * ratio
        v, w = vx * factor, wz * factor
        if ratio < 1 and ((abs(vx) > 1e-3 and abs(v) < min_linear)
                          or (abs(vx) <= 1e-3 and abs(w) < min_angular)):
            continue
        if blocking(points, v, w, horizon, half_length, half_width, clearance) < min_points:
            return v, w, factor
    return 0.0, 0.0, 0.0
