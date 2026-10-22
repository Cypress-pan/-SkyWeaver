"""Deterministic transition planner.

A plan is a sequence of takeoff -> transition -> hold -> (optional) landing
segments.  Each segment interpolates every drone's target position with a
smoothstep profile.  Plans are immutable once built; the runtime may apply a
short start-continuity offset when a queued plan begins later, but the nominal
trajectory stored here is what the validator signs.
"""
from __future__ import annotations

import hashlib
import hmac
import math
import secrets
from typing import Any

import numpy as np
from scipy.optimize import linear_sum_assignment

from .colors import ColorError, DEFAULT_COLOR, parse_color_spec
from .formations import canonical_json


# Guidance only: used to make the vertical phases physically reasonable.
# The validator's frozen thresholds are separate and slightly looser.
PLANNER_GUIDE_MAX_ACCEL = 8.0
TAKEOFF_Z = 0.10
LANDING_Z = 0.10
START_CONTINUITY_SECONDS = 0.5


class PlanError(ValueError):
    """Raised when a plan cannot be built from the supplied inputs."""

    def __init__(self, code: str, detail: str):
        super().__init__(detail)
        self.code = code
        self.detail = detail


def smoothstep(u: np.ndarray | float) -> np.ndarray | float:
    u = np.clip(u, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def _vertical_duration(dz: float, minimum: float) -> float:
    if dz <= 1e-9:
        return float(minimum)
    return max(float(minimum), math.sqrt(6.0 * dz / PLANNER_GUIDE_MAX_ACCEL))


def _assign_targets(start: np.ndarray, target_points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Match drones to target points, keeping idle drones at their current spot.

    Returns the matched positions and, for each drone, the target index or -1
    when the drone is idle.  ``target_points`` may be shorter than ``start``
    (grid resources not using every drone); unused drones hold position.
    """
    n = len(start)
    m = len(target_points)
    matched = start.copy()
    indices = np.full(n, -1, dtype=int)
    if m == 0:
        return matched, indices
    used = min(n, m)
    cost = np.linalg.norm(start[:used, None, :] - target_points[None, :used, :], axis=2)
    rows, cols = linear_sum_assignment(cost)
    matched[rows] = target_points[cols]
    indices[rows] = cols
    return matched, indices


def _segment(plan_id: str, index: int, name: str, duration: float,
             start_positions: np.ndarray, end_positions: np.ndarray,
             color: str | None = None,
             colors: list[str] | None = None) -> dict[str, Any]:
    return {
        "id": f"seg_{plan_id}_{index}",
        "name": name,
        "duration": float(duration),
        "start_positions": start_positions.astype(float).copy(),
        "end_positions": end_positions.astype(float).copy(),
        "color": color,
        "colors": None if colors is None else list(colors),
    }


def build_plan(state_positions: Any, num_drones: int, formation: dict[str, Any],
               duration: float, hold_seconds: float, include_landing: bool,
               source_state_revision: int, secret: bytes,
               plan_id: str | None = None,
               color_override: str | None = None,
               colors_override: list[str] | None = None,
               gradient_override: list[str] | None = None) -> dict[str, Any]:
    """Build a nominal plan from the current actual state to a target formation."""
    try:
        n = int(num_drones)
    except (TypeError, ValueError) as exc:
        raise PlanError("E_BAD_STATE", "num_drones must be an integer") from exc
    start = np.asarray(state_positions, dtype=float)
    if start.shape != (n, 3):
        raise PlanError("E_BAD_STATE", f"state positions must be {n}x3")
    try:
        duration = float(duration)
        hold_seconds = float(hold_seconds)
    except (TypeError, ValueError) as exc:
        raise PlanError("E_BAD_PLAN", "duration and hold_seconds must be numbers") from exc
    if duration <= 0.0:
        raise PlanError("E_BAD_PLAN", "duration must be positive")
    if hold_seconds < 0.0:
        raise PlanError("E_BAD_PLAN", "hold_seconds must be non-negative")

    points = np.asarray(formation.get("points", []), dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
        raise PlanError("E_BAD_FORMATION", "formation has no usable 3D points")
    if len(points) > n:
        points = points[:n]

    matched, target_indices = _assign_targets(start, points)
    try:
        if color_override is not None or colors_override is not None or gradient_override is not None:
            color_spec = parse_color_spec(color_override, colors_override,
                                          gradient_override, n,
                                          default=formation.get("color") or DEFAULT_COLOR)
            plan_color = color_spec["color"]
            per_drone_colors = color_spec["colors"]
            if per_drone_colors is None and plan_color is not None:
                per_drone_colors = [plan_color] * n
        else:
            color_spec = parse_color_spec(formation.get("color"), formation.get("colors"),
                                          formation.get("gradient"), len(points),
                                          default=formation.get("color") or DEFAULT_COLOR)
            plan_color = color_spec["color"]
            base_colors = color_spec["colors"]
            if base_colors is None:
                per_drone_colors = [plan_color] * n
            else:
                mapped = [None] * n
                for index, target_index in enumerate(target_indices):
                    if 0 <= target_index < len(base_colors):
                        mapped[index] = base_colors[target_index]
                per_drone_colors = mapped
    except ColorError as exc:
        raise PlanError(exc.code, exc.detail) from exc

    target_z = float(np.max(matched[:, 2]))
    takeoff_z = max(target_z, TAKEOFF_Z)
    takeoff_end = start.copy()
    takeoff_end[:, 2] = takeoff_z
    transition_end = matched.copy()
    transition_end[:, 2] = target_z
    dz_take = float(np.max(np.abs(takeoff_end[:, 2] - start[:, 2])))
    takeoff_duration = _vertical_duration(dz_take, 1.0)
    transition_duration = duration
    hold_duration = hold_seconds
    if include_landing:
        dz_land = max(0.0, takeoff_z - LANDING_Z)
        landing_duration = _vertical_duration(dz_land, 1.5)
    else:
        landing_duration = 0.0

    if plan_id is None:
        plan_id = "p_" + secrets.token_hex(6)
    segments = [
        _segment(plan_id, 0, "takeoff", takeoff_duration, start, takeoff_end,
                 color=plan_color, colors=per_drone_colors),
        _segment(plan_id, 1, "transition", transition_duration, takeoff_end,
                 transition_end, color=plan_color, colors=per_drone_colors),
        _segment(plan_id, 2, "hold", hold_duration, transition_end,
                 transition_end.copy(), color=plan_color, colors=per_drone_colors),
    ]
    if include_landing:
        landing_end = transition_end.copy()
        landing_end[:, 2] = LANDING_Z
        segments.append(_segment(plan_id, 3, "landing", landing_duration,
                                 transition_end, landing_end,
                                 color=plan_color, colors=per_drone_colors))

    cursor = 0.0
    for seg in segments:
        seg["start_time"] = float(cursor)
        cursor += seg["duration"]
        seg["end_time"] = float(cursor)
    total_duration = float(cursor)

    plan_hash = hashlib.sha256(canonical_json({
        "plan_id": plan_id,
        "source_state_revision": int(source_state_revision),
        "num_drones": n,
        "duration": round(duration, 6),
        "hold_seconds": round(hold_seconds, 6),
        "include_landing": bool(include_landing),
        "start_positions": np.round(start, 6).tolist(),
        "segments": [
            {
                "id": s["id"],
                "name": s["name"],
                "duration": round(s["duration"], 6),
                "start": np.round(s["start_positions"], 6).tolist(),
                "end": np.round(s["end_positions"], 6).tolist(),
                "color": s.get("color"),
                "colors": s.get("colors"),
            }
            for s in segments
        ],
    }).encode("utf-8")).hexdigest()
    credential = "vc_" + hmac.new(secret, (plan_id + "|" + plan_hash).encode("utf-8"),
                                  hashlib.sha256).hexdigest()[:32]
    return {
        "plan_id": plan_id,
        "plan_hash": plan_hash,
        "credential": credential,
        "source_state_revision": int(source_state_revision),
        "target_formation_id": formation.get("formation_id"),
        "duration": total_duration,
        "color": plan_color,
        "colors": per_drone_colors,
        "segments": [
            {"id": s["id"], "name": s["name"], "duration": round(s["duration"], 6),
             "color": s.get("color"), "colors": s.get("colors")}
            for s in segments
        ],
        "_num_drones": n,
        "_start_positions": start,
        "_target_positions": matched,
        "_segments": segments,
        "_total_duration": total_duration,
    }


def public_plan(plan: dict[str, Any]) -> dict[str, Any]:
    public = {
        "plan_id": plan["plan_id"],
        "plan_hash": plan["plan_hash"],
        "duration": round(plan["duration"], 6),
        "source_state_revision": plan["source_state_revision"],
        "target_formation_id": plan["target_formation_id"],
        "color": plan.get("color"),
        "colors": plan.get("colors"),
        "segments": plan["segments"],
        "repaired": bool(plan.get("_repaired", False)),
    }
    repair_info = plan.get("_repair_info") or plan.get("repair")
    if repair_info:
        public["repair"] = dict(repair_info)
    return public


def positions_at(plan: dict[str, Any], t_local: float) -> np.ndarray:
    """Nominal target positions at plan-local time ``t_local``."""
    total = float(plan["_total_duration"])
    segments = plan["_segments"]
    if t_local >= total:
        return segments[-1]["end_positions"].copy()
    for seg in segments:
        if t_local < seg["end_time"] or seg is segments[-1]:
            u = 0.0 if seg["duration"] <= 0 else (t_local - seg["start_time"]) / seg["duration"]
            s = smoothstep(u)
            return seg["start_positions"] + (seg["end_positions"] - seg["start_positions"]) * s
    return segments[-1]["end_positions"].copy()


def segment_at(plan: dict[str, Any], t_local: float) -> dict[str, Any] | None:
    total = float(plan["_total_duration"])
    if t_local < 0.0 or t_local > total:
        return None
    for seg in plan["_segments"]:
        if t_local <= seg["end_time"] + 1e-12:
            u = 0.0 if seg["duration"] <= 0 else (t_local - seg["start_time"]) / seg["duration"]
            return {"segment": seg, "progress": float(np.clip(u, 0.0, 1.0))}
    return {"segment": plan["_segments"][-1], "progress": 1.0}


def sample_plan_positions(plan: dict[str, Any], rate: float = 200.0) -> tuple[np.ndarray, np.ndarray]:
    """High-rate samples of the nominal trajectory.  Returns (times, positions)."""
    total = float(plan["_total_duration"])
    if rate <= 0:
        raise PlanError("E_BAD_PLAN", "sample rate must be positive")
    step = 1.0 / float(rate)
    times = np.arange(0.0, total + step * 0.5, step)
    if times.size == 0 or times[-1] < total - 1e-12:
        times = np.append(times, total)
    times[-1] = total
    positions = np.stack([positions_at(plan, float(t)) for t in times], axis=0)
    return times, positions


def plan_min_distance(plan: dict[str, Any], rate: float = 200.0) -> float | None:
    _, positions = sample_plan_positions(plan, rate)
    best = None
    for frame in positions:
        d = min_pairwise_distance(frame)
        if d is None:
            continue
        if best is None or d < best:
            best = d
    return best


def velocity_at(plan: dict[str, Any], t_local: float) -> np.ndarray:
    """Nominal target velocity at plan-local time (for PID feed-forward)."""
    total = float(plan["_total_duration"])
    segments = plan["_segments"]
    if t_local < 0.0 or t_local >= total:
        return np.zeros((int(plan["_num_drones"]), 3), dtype=float)
    for seg in segments:
        if t_local < seg["end_time"] or seg is segments[-1]:
            if seg["duration"] <= 0.0:
                return np.zeros_like(seg["start_positions"], dtype=float)
            u = float(np.clip((t_local - seg["start_time"]) / seg["duration"], 0.0, 1.0))
            factor = 6.0 * u * (1.0 - u) / seg["duration"]
            return (seg["end_positions"] - seg["start_positions"]) * factor
    return np.zeros_like(segments[-1]["start_positions"], dtype=float)
