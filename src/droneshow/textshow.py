"""High-level CJK text-show analysis, paging and multi-segment planning."""
from __future__ import annotations

import hashlib
import hmac
import secrets
from typing import Any

import numpy as np

from .cjk import CJKError, build_page_points, render_glyph_coords, resolve_font_path
from .colors import ColorError, DEFAULT_COLOR, parse_color_spec
from .formations import canonical_json, short_hash
from .planner import (
    LANDING_Z,
    PlanError,
    _assign_targets,
    _segment,
    _vertical_duration,
)
from .repair import _via_candidates
from .validator import validate_plan


DEFAULT_POINTS_PER_CHAR = 32
DEFAULT_TRANSITION_SECONDS = 5.0
DEFAULT_HOLD_SECONDS = 1.5


class TextShowError(ValueError):
    def __init__(self, code: str, detail: str, **extra: Any):
        super().__init__(detail)
        self.code = code
        self.detail = detail
        self.extra = extra


def _distribute(total: int, count: int) -> list[int]:
    if count <= 0:
        return []
    base = int(total) // int(count)
    remainder = int(total) % int(count)
    return [base + (1 if index < remainder else 0) for index in range(count)]


def analyze_text(text: str) -> dict[str, Any]:
    if not isinstance(text, str):
        raise TextShowError("E_BAD_SPEC", "text must be a string")
    chars = [char for char in text if not char.isspace()]
    if not chars:
        raise TextShowError("E_BAD_SPEC", "text must contain at least one non-space character")
    return {
        "text": text,
        "characters": chars,
        "character_count": len(chars),
        "unique_characters": sorted(set(chars)),
    }


def _allocate_counts(budget: int, complexities: list[int],
                     min_points_per_char: int) -> list[int]:
    count = len(complexities)
    if count == 0:
        return []
    floor = int(min_points_per_char)
    if budget < floor * count:
        raise TextShowError(
            "E_POINT_BUDGET",
            "not enough drones for " + str(count) + " characters at "
            + str(floor) + " points each",
            required_points=floor * count,
            available_points=budget,
        )
    base = [floor] * count
    remaining = int(budget - floor * count)
    if count == 1 or remaining == 0:
        return [base[0] + remaining] if count == 1 else base
    total_complexity = float(sum(max(1, int(value)) for value in complexities))
    raw = [remaining * max(1, int(value)) / total_complexity for value in complexities]
    extras = [int(value) for value in raw]
    leftover = remaining - sum(extras)
    order = sorted(range(count),
                   key=lambda index: (-(raw[index] - extras[index]), index))
    for index in order[:leftover]:
        extras[index] += 1
    return [base[index] + extras[index] for index in range(count)]


def paginate_text(chars: list[str], num_drones: int,
                  auto_allocate: bool = True,
                  min_points_per_char: int = 16,
                  points_per_char: int = DEFAULT_POINTS_PER_CHAR,
                  complexities: list[int] | None = None) -> dict[str, Any]:
    if num_drones < 1:
        raise TextShowError("E_BAD_SPEC", "num_drones must be positive")
    if len(chars) == 0:
        raise TextShowError("E_BAD_SPEC", "at least one character is required")
    complexities = list(complexities) if complexities else [1] * len(chars)
    if len(complexities) != len(chars):
        raise TextShowError("E_BAD_SPEC", "complexities must match characters")
    if auto_allocate:
        floor = max(1, int(min_points_per_char))
        if num_drones < floor * len(chars):
            chars_per_page = max(1, num_drones // floor)
        else:
            chars_per_page = len(chars)
        pages = []
        start = 0
        while start < len(chars):
            page_chars = list(chars[start:start + chars_per_page])
            page_complexities = list(complexities[start:start + chars_per_page])
            allocations = _allocate_counts(num_drones, page_complexities, floor)
            pages.append({"page_index": len(pages), "characters": page_chars,
                          "allocations": allocations})
            start += len(page_chars)
        strategy = "single_page" if len(pages) == 1 else "auto_paginate"
        requested_total = floor * len(chars)
    else:
        requested = max(1, int(points_per_char))
        if num_drones >= len(chars) * requested:
            chars_per_page = len(chars)
        else:
            chars_per_page = max(1, num_drones // requested)
        pages = []
        start = 0
        while start < len(chars):
            page_chars = list(chars[start:start + chars_per_page])
            allocations = [min(requested, num_drones) for _ in page_chars]
            pages.append({"page_index": len(pages), "characters": page_chars,
                          "allocations": allocations})
            start += len(page_chars)
        strategy = "single_page" if len(pages) == 1 else "auto_paginate"
        requested_total = requested * len(chars)
    for page in pages:
        page["points"] = int(sum(page["allocations"]))
        page["idle"] = int(num_drones - page["points"])
    return {
        "strategy": strategy,
        "num_drones": int(num_drones),
        "auto_allocate": bool(auto_allocate),
        "min_points_per_char": int(min_points_per_char),
        "points_per_char": int(points_per_char),
        "requested_total_points": int(requested_total),
        "pages": pages,
    }


def _resolve_colors(color: Any, colors: Any, gradient: Any,
                    num_drones: int) -> tuple[str, list[str]]:
    try:
        spec = parse_color_spec(color, colors, gradient, num_drones,
                                default=DEFAULT_COLOR)
    except ColorError as exc:
        raise TextShowError(exc.code, exc.detail, **exc.extra) from exc
    plan_color = spec["color"]
    per_drone = spec["colors"]
    if per_drone is None and plan_color is not None:
        per_drone = [plan_color] * num_drones
    return plan_color, per_drone


def _build_segments(plan_id: str, routes: list[list[np.ndarray]],
                    transition_duration: float, hold_duration: float,
                    include_landing: bool, plan_color: str,
                    per_drone_colors: list[str]) -> list[dict[str, Any]]:
    segments: list[dict[str, Any]] = []
    index = 0
    final_point = routes[-1][-1] if routes else None
    for route in routes:
        for route_index in range(len(route) - 1):
            segments.append(_segment(plan_id, index, "transition", transition_duration,
                                     route[route_index], route[route_index + 1],
                                     color=plan_color, colors=per_drone_colors))
            index += 1
        last = route[-1]
        segments.append(_segment(plan_id, index, "hold", hold_duration,
                                 last, last.copy(),
                                 color=plan_color, colors=per_drone_colors))
        index += 1
    if include_landing and final_point is not None:
        landing_end = final_point.copy()
        landing_end[:, 2] = LANDING_Z
        landing_duration = _vertical_duration(max(0.0, float(np.max(final_point[:, 2]) - LANDING_Z)), 1.5)
        segments.append(_segment(plan_id, index, "landing", landing_duration,
                                 final_point, landing_end,
                                 color=plan_color, colors=per_drone_colors))
    cursor = 0.0
    for segment in segments:
        segment["start_time"] = float(cursor)
        cursor += segment["duration"]
        segment["end_time"] = float(cursor)
    return segments


def _assemble_plan(plan_id: str, num_drones: int, start: np.ndarray,
                   routes: list[list[np.ndarray]], transition_duration: float,
                   hold_duration: float, include_landing: bool,
                   source_state_revision: int, secret: bytes,
                   plan_color: str, per_drone_colors: list[str],
                   text_show_meta: dict[str, Any]) -> dict[str, Any]:
    segments = _build_segments(plan_id, routes, transition_duration,
                               hold_duration, include_landing,
                               plan_color, per_drone_colors)
    total_duration = float(segments[-1]["end_time"]) if segments else 0.0
    final_point = routes[-1][-1] if routes else start
    plan_hash = hashlib.sha256(canonical_json({
        "plan_id": plan_id,
        "source_state_revision": int(source_state_revision),
        "num_drones": int(num_drones),
        "transition_duration": round(float(transition_duration), 6),
        "hold_duration": round(float(hold_duration), 6),
        "include_landing": bool(include_landing),
        "start_positions": np.round(start, 6).tolist(),
        "routes": [[np.round(point, 6).tolist() for point in route] for route in routes],
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
        "target_formation_id": None,
        "duration": total_duration,
        "color": plan_color,
        "colors": per_drone_colors,
        "segments": [
            {"id": segment["id"], "name": segment["name"],
             "duration": round(segment["duration"], 6),
             "color": segment.get("color"), "colors": segment.get("colors")}
            for segment in segments
        ],
        "repaired": False,
        "_num_drones": int(num_drones),
        "_start_positions": start,
        "_target_positions": final_point,
        "_segments": segments,
        "_total_duration": total_duration,
        "_text_show": text_show_meta,
    }


def _page_points_3d(page: dict[str, Any], font_path: str, z: float,
                    base_height: float, letter_spacing: float,
                    min_separation: float) -> tuple[np.ndarray, dict[str, Any]]:
    points_2d, meta = build_page_points(
        list(page["characters"]), list(page["allocations"]), font_path,
        base_height=base_height, letter_spacing=letter_spacing,
        min_separation=min_separation)
    points = np.column_stack([points_2d, np.full(len(points_2d), float(z))])
    meta = dict(meta)
    meta["z"] = float(z)
    return points, meta


def _match_pages(start: np.ndarray, page_points: list[np.ndarray]) -> list[np.ndarray]:
    matched_pages: list[np.ndarray] = []
    previous = start
    for points in page_points:
        matched, _ = _assign_targets(previous, points)
        matched_pages.append(matched)
        previous = matched
    return matched_pages


def _direct_routes(start: np.ndarray, matched_pages: list[np.ndarray]) -> list[list[np.ndarray]]:
    routes: list[list[np.ndarray]] = []
    previous = start
    for page in matched_pages:
        routes.append([previous, page])
        previous = page
    return routes


def build_text_show_plan(state_positions: Any, num_drones: int, text: str,
                         duration: float, hold_seconds: float,
                         include_landing: bool, source_state_revision: int,
                         secret: bytes,
                         points_per_char: int = DEFAULT_POINTS_PER_CHAR,
                         font_path: str | None = None,
                         z: float = 1.4,
                         color: Any = None, colors: Any = None,
                         gradient: Any = None,
                         base_height: float = 2.4,
                         letter_spacing: float = 0.16,
                         min_separation: float = 0.30,
                         auto_allocate: bool = True,
                         min_points_per_char: int = 16,
                         plan_id: str | None = None) -> dict[str, Any]:
    start = np.asarray(state_positions, dtype=float)
    n = int(num_drones)
    if start.shape != (n, 3):
        raise TextShowError("E_BAD_STATE", "state positions must be " + str(n) + "x3")
    analysis = analyze_text(text)
    resolved_font = resolve_font_path(font_path)
    complexities = []
    for char in analysis["characters"]:
        coords, _ = render_glyph_coords(char, resolved_font)
        complexities.append(int(len(coords)))
    pagination = paginate_text(
        analysis["characters"], n,
        auto_allocate=auto_allocate,
        min_points_per_char=min_points_per_char,
        points_per_char=points_per_char,
        complexities=complexities)
    plan_color, per_drone_colors = _resolve_colors(color, colors, gradient, n)

    page_points: list[np.ndarray] = []
    page_meta: list[dict[str, Any]] = []
    page_formations: list[dict[str, Any]] = []
    for page in pagination["pages"]:
        points, meta = _page_points_3d(page, resolved_font, float(z),
                                       float(base_height), float(letter_spacing),
                                       float(min_separation))
        formation_id = "f_" + short_hash(canonical_json({
            "type": "cjk_text",
            "text": "".join(page["characters"]),
            "points": np.round(points, 9).tolist(),
        }))
        meta["formation_id"] = formation_id
        page_points.append(points)
        page_meta.append(meta)
        page_formations.append({
            "page": page["page_index"],
            "text": "".join(page["characters"]),
            "formation_id": formation_id,
            "points": int(len(points)),
            "used": int(len(points)),
            "idle": int(n - len(points)),
            "min_distance": meta.get("min_distance"),
            "characters": list(page["characters"]),
            "allocations": list(page["allocations"]),
        })
    allocation_report = []
    original_index = 0
    for page in pagination["pages"]:
        for position, char in enumerate(page["characters"]):
            allocation_report.append({
                "char": char,
                "complexity": int(complexities[original_index]),
                "points": int(page["allocations"][position]),
                "page": int(page["page_index"]),
            })
            original_index += 1
    matched_pages = _match_pages(start, page_points)
    direct_routes = _direct_routes(start, matched_pages)

    if plan_id is None:
        plan_id = "p_" + secrets.token_hex(6)
    meta = {
        "text": analysis["text"],
        "characters": analysis["characters"],
        "font": resolved_font,
        "points_per_char": int(points_per_char),
        "pagination": pagination,
        "page_meta": page_meta,
        "page_formations": page_formations,
        "allocation_report": allocation_report,
        "num_drones": n,
        "z": float(z),
    }
    direct_plan = _assemble_plan(plan_id, n, start, direct_routes,
                                 float(duration), float(hold_seconds),
                                 bool(include_landing), int(source_state_revision),
                                 secret, plan_color, per_drone_colors, meta)
    direct_validation = validate_plan(direct_plan)
    if direct_validation["valid"]:
        direct_plan["_text_show"]["validation"] = direct_validation
        return direct_plan

    direct_plan["_text_show"]["validation"] = direct_validation
    issues = direct_validation.get("issues") or []
    if not issues or any(not str(issue).startswith("E_MIN_SEPARATION") for issue in issues):
        return direct_plan

    for via_formation in _via_candidates(n):
        via_points = np.asarray(via_formation.get("points", []), dtype=float)
        if via_points.ndim != 2 or via_points.shape[0] != n:
            continue
        for leg_index in range(len(direct_routes)):
            matched_via, _ = _assign_targets(direct_routes[leg_index][0], via_points)
            trial_routes = [list(route) for route in direct_routes]
            trial_routes[leg_index] = [direct_routes[leg_index][0].copy(),
                                       matched_via,
                                       direct_routes[leg_index][-1].copy()]
            trial_plan = _assemble_plan(plan_id, n, start, trial_routes,
                                        float(duration), float(hold_seconds),
                                        bool(include_landing), int(source_state_revision),
                                        secret, plan_color, per_drone_colors, meta)
            trial_validation = validate_plan(trial_plan)
            if trial_validation["valid"]:
                trial_plan["repaired"] = True
                trial_plan["_repaired"] = True
                trial_plan["repair"] = {
                    "reason": "E_MIN_SEPARATION",
                    "via_formation_id": via_formation.get("formation_id"),
                    "via_type": via_formation.get("type"),
                    "leg_index": leg_index,
                }
                trial_plan["_text_show"]["validation"] = trial_validation
                trial_plan["_text_show"]["repaired"] = True
                return trial_plan
        all_routes = []
        for route in direct_routes:
            matched_via, _ = _assign_targets(route[0], via_points)
            all_routes.append([route[0].copy(), matched_via, route[-1].copy()])
        all_plan = _assemble_plan(plan_id, n, start, all_routes,
                                  float(duration), float(hold_seconds),
                                  bool(include_landing), int(source_state_revision),
                                  secret, plan_color, per_drone_colors, meta)
        all_validation = validate_plan(all_plan)
        if all_validation["valid"]:
            all_plan["repaired"] = True
            all_plan["_repaired"] = True
            all_plan["repair"] = {
                "reason": "E_MIN_SEPARATION",
                "via_formation_id": via_formation.get("formation_id"),
                "via_type": via_formation.get("type"),
                "leg_index": "all",
            }
            all_plan["_text_show"]["validation"] = all_validation
            all_plan["_text_show"]["repaired"] = True
            return all_plan
    return direct_plan
