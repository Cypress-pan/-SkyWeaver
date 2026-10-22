"""Fixed, deterministic validator for nominal plans.

The validator samples the planned trajectory at a high fixed rate, converts it
to high-rate positions, velocities and accelerations by central differences,
and checks frozen thresholds.  It never reads the simulator's actual state, so
a validated credential is a statement about the signed plan only.
"""
from __future__ import annotations

import math
from typing import Any

import numpy as np
from scipy.spatial import cKDTree

from .formations import min_pairwise_distance, min_pairwise_distance_fast
from .planner import sample_plan_positions


# Frozen thresholds.  A plan must satisfy all of these to receive a credential.
MIN_SEPARATION_M = 0.25
MAX_SPEED_MPS = 2.0
MAX_ACCEL_MPSS = 8.0
MIN_Z_M = 0.02
MAX_Z_M = 3.0
MAX_XY_M = 5.0
SAMPLE_RATE_HZ = 200.0


def validate_plan(plan: dict[str, Any]) -> dict[str, Any]:
    issues: list[str] = []
    times, positions = sample_plan_positions(plan, SAMPLE_RATE_HZ)
    if len(times) < 3:
        return {
            "valid": False,
            "credential": None,
            "issues": ["E_PLAN_TOO_SHORT"],
            "metrics": {"min_distance": None, "max_speed": None, "max_accel": None,
                        "min_z": None, "max_z": None, "max_xy": None},
        }
    dt = float(times[1] - times[0])
    # Central differences for velocity and acceleration, one-sided at the ends.
    vel = np.empty_like(positions)
    vel[1:-1] = (positions[2:] - positions[:-2]) / (2.0 * dt)
    vel[0] = (positions[1] - positions[0]) / dt
    vel[-1] = (positions[-1] - positions[-2]) / dt
    acc = np.empty_like(positions)
    acc[1:-1] = (vel[2:] - vel[:-2]) / (2.0 * dt)
    acc[0] = (vel[1] - vel[0]) / dt
    acc[-1] = (vel[-1] - vel[-2]) / dt
    speed = np.linalg.norm(vel, axis=2)
    accel = np.linalg.norm(acc, axis=2)
    max_speed = float(np.max(speed))
    max_accel = float(np.max(accel))
    min_z = float(np.min(positions[:, :, 2]))
    max_z = float(np.max(positions[:, :, 2]))
    max_xy = float(np.max(np.linalg.norm(positions[:, :, :2], axis=2)))
    min_distance = None
    for frame in positions:
        d = min_pairwise_distance(frame)
        if d is None:
            continue
        if min_distance is None or d < min_distance:
            min_distance = d

    if min_distance is not None and min_distance < MIN_SEPARATION_M:
        issues.append(f"E_MIN_SEPARATION: {min_distance:.4f} m < {MIN_SEPARATION_M:.2f} m")
    if max_speed > MAX_SPEED_MPS:
        issues.append(f"E_MAX_SPEED: {max_speed:.4f} m/s > {MAX_SPEED_MPS:.2f} m/s")
    if max_accel > MAX_ACCEL_MPSS:
        issues.append(f"E_MAX_ACCEL: {max_accel:.4f} m/s^2 > {MAX_ACCEL_MPSS:.2f} m/s^2")
    if min_z < MIN_Z_M:
        issues.append(f"E_Z_BOUNDS: min z {min_z:.4f} m < {MIN_Z_M:.2f} m")
    if max_z > MAX_Z_M:
        issues.append(f"E_Z_BOUNDS: max z {max_z:.4f} m > {MAX_Z_M:.2f} m")
    if max_xy > MAX_XY_M:
        issues.append(f"E_XY_BOUNDS: max xy radius {max_xy:.4f} m > {MAX_XY_M:.2f} m")

    metrics = {
        "min_distance": None if min_distance is None else round(float(min_distance), 6),
        "max_speed": round(max_speed, 6),
        "max_accel": round(max_accel, 6),
        "min_z": round(min_z, 6),
        "max_z": round(max_z, 6),
        "max_xy": round(max_xy, 6),
        "samples": int(len(times)),
        "sample_rate_hz": SAMPLE_RATE_HZ,
    }
    valid = not issues
    return {
        "valid": valid,
        "credential": plan.get("credential") if valid else None,
        "issues": issues,
        "metrics": metrics,
    }


def _sample_times(total: float) -> np.ndarray:
    """The exact sample grid used by :func:`validate_plan`."""
    step = 1.0 / SAMPLE_RATE_HZ
    times = np.arange(0.0, total + step * 0.5, step)
    if times.size == 0 or times[-1] < total - 1e-12:
        times = np.append(times, total)
    times[-1] = total
    return times


def _segment_index_for_times(segments: list[dict[str, Any]],
                             times: np.ndarray) -> np.ndarray:
    """Vectorised equivalent of the segment lookup in `positions_at`.

    `positions_at` returns from the first segment whose `end_time` is
    strictly greater than `t`, falling back to the last segment.  A
    right-sided search over the cumulative end times reproduces that choice
    exactly, including for zero-duration segments.
    """
    end_times = np.array([float(seg["end_time"]) for seg in segments])
    idx = np.searchsorted(end_times, times, side="right")
    return np.minimum(idx, len(segments) - 1)


def _batch_positions(plan: dict[str, Any], first: int, last: int,
                     times: np.ndarray, seg_of: np.ndarray) -> np.ndarray:
    """Positions for the inclusive frame index range [first, last].

    Frames are grouped by segment (they are contiguous in time) and evaluated
    with the same `start + (end - start) * smoothstep(u)` blend that
    `positions_at` performs per frame, so results are bit-identical.
    """
    segments = plan["_segments"]
    n = int(plan["_num_drones"])
    out = np.empty((last - first + 1, n, 3), dtype=float)
    idx = first
    while idx <= last:
        seg = segments[int(seg_of[idx])]
        run = idx
        while run + 1 <= last and int(seg_of[run + 1]) == int(seg_of[idx]):
            run += 1
        t_local = times[idx:run + 1] - float(seg["start_time"])
        if float(seg["duration"]) <= 0.0:
            block = np.repeat(np.asarray(seg["start_positions"],
                                         dtype=float)[None, :, :], run - idx + 1, axis=0)
        else:
            u = t_local / float(seg["duration"])
            u = np.clip(u, 0.0, 1.0)
            s = u * u * (3.0 - 2.0 * u)
            start = np.asarray(seg["start_positions"], dtype=float)
            end = np.asarray(seg["end_positions"], dtype=float)
            block = start[None, :, :] + (end - start)[None, :, :] * s[:, None, None]
        out[idx - first:run - first + 1] = block
        idx = run + 1
    return out


def validate_plan_chunked(plan: dict[str, Any],
                          chunk_frames: int = 512) -> dict[str, Any]:
    """Validate a plan with bounded memory, identical to `validate_plan`.

    Large fleets (1000+ drones over 60 s at 200 Hz) would need ~300 MB per
    positions array in the frozen validator.  This variant streams the same
    200 Hz samples in time chunks, keeps a small rolling window for the
    central-difference velocity/acceleration (one-sided only at the global
    ends, exactly like the frozen code), and uses a KD-tree nearest-neighbour
    query for the per-frame minimum pairwise distance.  Frames identical to
    their predecessor (static holds) are checked once.  Thresholds, issue
    strings and metric rounding match :func:`validate_plan` exactly.
    """
    total = float(plan["_total_duration"])
    if chunk_frames < 8:
        chunk_frames = 8
    times = _sample_times(total) if total > 0.0 else np.array([0.0])
    count = int(len(times))
    short_metrics = {"min_distance": None, "max_speed": None, "max_accel": None,
                     "min_z": None, "max_z": None, "max_xy": None}
    if count < 3:
        return {
            "valid": False,
            "credential": None,
            "issues": ["E_PLAN_TOO_SHORT"],
            "metrics": short_metrics,
        }
    segments = plan["_segments"]
    seg_of = _segment_index_for_times(segments, times)
    dt = float(times[1] - times[0])

    issues: list[str] = []
    min_distance = None
    max_speed = -math.inf
    max_accel = -math.inf
    min_z = math.inf
    max_z = -math.inf
    max_xy = -math.inf
    prev_frame: np.ndarray | None = None

    first = 0
    while first < count:
        last = min(first + chunk_frames, count) - 1
        ext_first = max(0, first - 2)
        ext_last = min(count - 1, last + 2)
        pos_ext = _batch_positions(plan, ext_first, ext_last, times, seg_of)
        vel_ext = np.empty_like(pos_ext)
        if len(pos_ext) > 1:
            vel_ext[1:-1] = (pos_ext[2:] - pos_ext[:-2]) / (2.0 * dt)
        vel_ext[0] = (pos_ext[1] - pos_ext[0]) / dt
        vel_ext[-1] = (pos_ext[-1] - pos_ext[-2]) / dt
        acc_ext = np.empty_like(vel_ext)
        if len(vel_ext) > 1:
            acc_ext[1:-1] = (vel_ext[2:] - vel_ext[:-2]) / (2.0 * dt)
        acc_ext[0] = (vel_ext[1] - vel_ext[0]) / dt
        acc_ext[-1] = (vel_ext[-1] - vel_ext[-2]) / dt

        lo = first - ext_first
        hi = last - ext_first + 1
        pos = pos_ext[lo:hi]
        vel = vel_ext[lo:hi]
        acc = acc_ext[lo:hi]
        max_speed = max(max_speed, float(np.max(np.linalg.norm(vel, axis=2))))
        max_accel = max(max_accel, float(np.max(np.linalg.norm(acc, axis=2))))
        min_z = min(min_z, float(np.min(pos[:, :, 2])))
        max_z = max(max_z, float(np.max(pos[:, :, 2])))
        max_xy = max(max_xy, float(np.max(np.linalg.norm(pos[:, :, :2], axis=2))))
        for frame in pos:
            if prev_frame is not None and np.array_equal(frame, prev_frame):
                continue
            if min_distance is None:
                d = min_pairwise_distance_fast(frame)
                if d is not None:
                    min_distance = d
            else:
                # Only pairs closer than the running minimum can change the
                # metric, so a radius query is exact and much cheaper than a
                # full nearest-neighbour scan on every frame.
                for i, j in cKDTree(frame).query_pairs(r=min_distance):
                    d = float(np.linalg.norm(frame[int(i)] - frame[int(j)]))
                    if d < min_distance:
                        min_distance = d
            prev_frame = frame
        first = last + 1

    if min_distance is not None and min_distance < MIN_SEPARATION_M:
        issues.append(f"E_MIN_SEPARATION: {min_distance:.4f} m < {MIN_SEPARATION_M:.2f} m")
    if max_speed > MAX_SPEED_MPS:
        issues.append(f"E_MAX_SPEED: {max_speed:.4f} m/s > {MAX_SPEED_MPS:.2f} m/s")
    if max_accel > MAX_ACCEL_MPSS:
        issues.append(f"E_MAX_ACCEL: {max_accel:.4f} m/s^2 > {MAX_ACCEL_MPSS:.2f} m/s^2")
    if min_z < MIN_Z_M:
        issues.append(f"E_Z_BOUNDS: min z {min_z:.4f} m < {MIN_Z_M:.2f} m")
    if max_z > MAX_Z_M:
        issues.append(f"E_Z_BOUNDS: max z {max_z:.4f} m > {MAX_Z_M:.2f} m")
    if max_xy > MAX_XY_M:
        issues.append(f"E_XY_BOUNDS: max xy radius {max_xy:.4f} m > {MAX_XY_M:.2f} m")

    metrics = {
        "min_distance": None if min_distance is None else round(float(min_distance), 6),
        "max_speed": round(max_speed, 6),
        "max_accel": round(max_accel, 6),
        "min_z": round(min_z, 6),
        "max_z": round(max_z, 6),
        "max_xy": round(max_xy, 6),
        "samples": count,
        "sample_rate_hz": SAMPLE_RATE_HZ,
    }
    valid = not issues
    return {
        "valid": valid,
        "credential": plan.get("credential") if valid else None,
        "issues": issues,
        "metrics": metrics,
    }
