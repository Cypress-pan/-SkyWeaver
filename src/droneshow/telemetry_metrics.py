"""Exact recorded-frame distances without Python loops over drone pairs."""
from __future__ import annotations

import numpy as np
from scipy.spatial.distance import pdist


def min_trajectory_distance(frames: np.ndarray) -> float | None:
    """Scan every pair in every recorded frame; never subsample telemetry.

    pdist computes the same Euclidean distances as the original per-drone
    loop, in compiled code. Only one frame's condensed distances is allocated.
    This is a telemetry metric, separate from the frozen nominal validator.
    """
    frames = np.asarray(frames, dtype=float)
    if frames.size == 0 or frames.shape[1] < 2:
        return None
    return float(min(float(pdist(frame).min()) for frame in frames))
