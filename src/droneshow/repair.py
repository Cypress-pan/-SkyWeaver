"""Collision-repair via-route planning for text transitions.

When a direct plan is rejected by the fixed validator solely because of
E_MIN_SEPARATION, the planner tries deterministic wide via-formations
(start -> via -> target).  A repaired plan is returned only if the full fixed
validator passes; otherwise the original direct plan is returned so callers
observe the real failure.
"""
from __future__ import annotations

import hashlib
import hmac
import math
import secrets
from typing import Any

import numpy as np

from .colors import ColorError, DEFAULT_COLOR, parse_color_spec
from .formations import build_formation, canonical_json
from .planner import (
    LANDING_Z,
    PlanError,
    _assign_targets,
    _segment,
    _vertical_duration,
    build_plan,
)
from .validator import validate_plan


def _via_grid_candidate(num_drones: int) -> dict[str, Any]:
    rows = max(1, int(math.ceil(math.sqrt(num_drones))))
    cols = max(1, int(math.ceil(num_drones / rows)))
    return build_formation(num_drones, {
        "type": "grid", "rows": rows, "cols": cols,
        "spacing": 0.8, "z": 0.1,
    })


def _via_ring_candidate(num_drones: int) -> dict[str, Any]:
    if num_drones > 1:
        default_radius = 0.45 / (2.0 * math.sin(math.pi / num_drones))
        radius = min(4.0, max(0.6, default_radius))
    else:
        radius = 0.6
    return build_formation(num_drones, {
        "type": "ring", "radius": radius, "z": 0.1,
    })


def _via_candidates(num_drones: int) -> list[dict[str, Any]]:
    return [_via_grid_candidate(num_drones), _via_ring_candidate(num_drones)]


def _resolve_colors(formation: dict[str, Any], num_drones: int,
                    target_count: int, target_indices: np.ndarray,
                    color_override: str | None,
                    colors_override: list[str] | None,
                    gradient_override: list[str] | None) -> tuple[str, list[str]]:
    try:
        if color_override is not None or colors_override is not None or gradient_override is not None:
            spec = parse_color_spec(color_override, colors_override, gradient_override,
                                    num_drones,
                                    default=formation.get("color") or DEFAULT_COLOR)
            plan_color = spec["color"]
            per_drone = spec["colors"]
            if per_drone is None and plan_color is not None:
                per_drone = [plan_color] * num_drones
            return plan_color, per_drone
        spec = parse_color_spec(formation.get("color"), formation.get("colors"),
                                formation.get("gradient"), target_count,
                                default=formation.get("color") or DEFAULT_COLOR)
        plan_color = spec["color"]
        base_colors = spec["colors"]
        if base_colors is None:
            base_colors = [plan_color] * target_count
        mapped: list[str] = []
        for target_index in target_indices:
            if 0 <= target_index < len(base_colors):
                mapped.append(base_colors[target_index])
            else:
                mapped.append(plan_color)
        return plan_color, mapped
    except ColorError as exc:
        raise PlanError(exc.code, exc.detail) from exc


def _build_via_plan(start: np.ndarray, num_drones: int, formation: dict[str, Any],
                    via_formation: dict[str, Any], via_points: np.ndarray,
                    duration: float, hold_seconds: float, include_landing: bool,
                    source_state_revision: int, secret: bytes,
                    color_override: str | None,
                    colors_override: list[str] | None,
                    gradient_override: list[str] | None,
                    plan_id: str | None = None) -> dict[str, Any]:
    n = int(num_drones)
    target_points = np.asarray(formation.get("points", []), dtype=float)
    if via_points.ndim != 2 or via_points.shape[1] != 3:
        raise PlanError("E_BAD_FORMATION", "via formation has no usable 3D points")
    if target_points.ndim != 2 or target_points.shape[1] != 3 or len(target_points) == 0:
        raise PlanError("E_BAD_FORMATION", "formation has no usable 3D points")
    via_points = via_points[:n]
    target_points = target_points[:n]

    matched_via, _ = _assign_targets(start, via_points)
    matched_target, target_indices = _assign_targets(matched_via, target_points)
    target_z = float(np.max(matched_target[:, 2]))
    plan_color, per_drone_colors = _resolve_colors(
        formation, n, len(target_points), target_indices,
        color_override, colors_override, gradient_override)

    if plan_id is None:
        plan_id = "p_" + secrets.token_hex(6)

    transition_duration = float(duration)
    hold_duration = float(hold_seconds)
    segments = [
        _segment(plan_id, 0, "transition", transition_duration, start, matched_via,
                 color=plan_color, colors=per_drone_colors),
        _segment(plan_id, 1, "transition", transition_duration, matched_via,
                 matched_target, color=plan_color, colors=per_drone_colors),
        _segment(plan_id, 2, "hold", hold_duration, matched_target,
                 matched_target.copy(), color=plan_color, colors=per_drone_colors),
    ]
    if include_landing:
        landing_duration = _vertical_duration(max(0.0, target_z - LANDING_Z), 1.5)
        landing_end = matched_target.copy()
        landing_end[:, 2] = LANDING_Z
        segments.append(_segment(plan_id, 3, "landing", landing_duration,
                                 matched_target, landing_end,
                                 color=plan_color, colors=per_drone_colors))

    cursor = 0.0
    for segment in segments:
        segment["start_time"] = float(cursor)
        cursor += segment["duration"]
        segment["end_time"] = float(cursor)
    total_duration = float(cursor)

    plan_hash = hashlib.sha256(canonical_json({
        "plan_id": plan_id,
        "source_state_revision": int(source_state_revision),
        "num_drones": n,
        "duration": round(transition_duration, 6),
        "hold_seconds": round(hold_duration, 6),
        "include_landing": bool(include_landing),
        "start_positions": np.round(start, 6).tolist(),
        "via_formation_id": via_formation.get("formation_id"),
        "via_positions": np.round(matched_via, 6).tolist(),
        "target_positions": np.round(matched_target, 6).tolist(),
        "segments": [
            {
                "id": segment["id"],
                "name": segment["name"],
                "duration": round(segment["duration"], 6),
                "start": np.round(segment["start_positions"], 6).tolist(),
                "end": np.round(segment["end_positions"], 6).tolist(),
                "color": segment.get("color"),
                "colors": segment.get("colors"),
            }
            for segment in segments
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
            {"id": segment["id"], "name": segment["name"],
             "duration": round(segment["duration"], 6),
             "color": segment.get("color"), "colors": segment.get("colors")}
            for segment in segments
        ],
        "repaired": True,
        "repair": {
            "reason": "E_MIN_SEPARATION",
            "via_formation_id": via_formation.get("formation_id"),
            "via_type": via_formation.get("type"),
            "via_points": len(via_points),
            "transition_duration_sec": round(transition_duration, 6),
        },
        "_num_drones": n,
        "_start_positions": start,
        "_target_positions": matched_target,
        "_via_positions": matched_via,
        "_segments": segments,
        "_total_duration": total_duration,
        "_repaired": True,
    }


def build_plan_with_repair(state_positions: Any, num_drones: int,
                           formation: dict[str, Any], duration: float,
                           hold_seconds: float, include_landing: bool,
                           source_state_revision: int, secret: bytes,
                           plan_id: str | None = None,
                           color_override: str | None = None,
                           colors_override: list[str] | None = None,
                           gradient_override: list[str] | None = None) -> dict[str, Any]:
    direct = build_plan(state_positions, num_drones, formation, duration,
                        hold_seconds, include_landing, source_state_revision,
                        secret, plan_id=plan_id,
                        color_override=color_override,
                        colors_override=colors_override,
                        gradient_override=gradient_override)
    direct_result = validate_plan(direct)
    if direct_result.get("valid"):
        return direct
    issues = direct_result.get("issues") or []
    if not issues or any(not str(issue).startswith("E_MIN_SEPARATION") for issue in issues):
        return direct

    n = int(num_drones)
    for via_formation in _via_candidates(n):
        try:
            via_points = np.asarray(via_formation.get("points", []), dtype=float)
            repaired = _build_via_plan(
                np.asarray(state_positions, dtype=float), n, formation,
                via_formation, via_points, duration, hold_seconds,
                include_landing, source_state_revision, secret,
                color_override, colors_override, gradient_override,
                plan_id=plan_id,
            )
        except Exception:
            continue
        repaired_result = validate_plan(repaired)
        if repaired_result.get("valid"):
            info = dict(repaired.get("repair") or {})
            info["direct_issues"] = list(issues)
            info["direct_min_distance"] = direct_result.get("metrics", {}).get("min_distance")
            info["repaired_min_distance"] = repaired_result.get("metrics", {}).get("min_distance")
            repaired["repair"] = info
            repaired["_repair_info"] = info
            return repaired
    return direct
