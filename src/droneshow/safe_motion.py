"""Whole-fleet matching and exact separation for synchronized straight moves."""
from __future__ import annotations

import math
import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist


def match_targets(start: np.ndarray, targets: np.ndarray):
    # Squared *three-dimensional* Euclidean cost is essential: its optimal
    # assignment implies (a_i-a_j).(b_i-b_j) >= 0 for every pair. Do not
    # reorder stacked slots afterwards or change the altitude weight.
    _, cols = linear_sum_assignment(cdist(start, targets, "sqeuclidean"))
    return targets[cols].copy(), cols


def continuous_separation(start: np.ndarray, end: np.ndarray) -> float:
    """Exact minimum over all pairs and all interpolation progress in [0,1].

    Smoothstep is monotone and shared by all drones, so minimizing in spatial
    progress also minimizes over continuous time. Memory remains O(N).
    """
    best = float("inf")
    for i in range(len(start) - 1):
        a = start[i] - start[i + 1:]
        delta = (end[i] - end[i + 1:]) - a
        denom = np.sum(delta * delta, axis=1)
        u = np.divide(-np.sum(a * delta, axis=1), denom,
                      out=np.zeros(len(a)), where=denom > 1e-20)
        u = np.clip(u, 0, 1)
        best = min(best, float(np.min(np.linalg.norm(a + u[:, None] * delta, axis=1))))
    return best


def move_seconds(start: np.ndarray, end: np.ndarray, minimum: float = 1.0):
    travel = float(np.max(np.linalg.norm(end - start, axis=1)))
    return max(minimum, 1.5 * travel / 1.35, math.sqrt(6 * travel / 6.0))


def move_segment(start, end, name, colors, minimum=1.0):
    return {"name": name, "duration": move_seconds(start, end, minimum),
            "start_positions": start.copy(), "end_positions": end.copy(),
            "color": colors[0], "colors": list(colors)}
