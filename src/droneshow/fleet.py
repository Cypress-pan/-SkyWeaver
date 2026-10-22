"""Large-scale fleet timeline planner (task-16).

Builds a signed multi-scene timeline for fleets far larger than the physical
simulator (1000+ drones) with:

* per-scene active-drone budgets (`drone_budget`: "auto" or an integer)
  chosen from text complexity / readability or shape capacity -- never a hard
  cap of 4;
* layered (z-stacked) formation synthesis so a scene can host more points
  than a single plane allows;
* deterministic, collision-free parking shells for idle drones above (or
  below) the active formation;
* optimal squared-distance matching and synchronized transitions for every
  drone, with exact continuous separation checks on all path chords;
* validation of the whole plan against the frozen thresholds using the
  memory-bounded chunked validator.

The resulting plan uses the same segment/credential format as the classic
timeline planner, so replay, evaluate and submit work unchanged.
"""
from __future__ import annotations

import math
import secrets
from typing import Any

import numpy as np
from scipy.optimize import linear_sum_assignment

from .cjk import CJKError, build_page_points, render_glyph_coords, resolve_font_path
from .colors import DEFAULT_COLOR, lerp_hex
from .formations import (
    _heart_curve,
    _resample_closed_curve,
    _resample_closed_polyline,
    _star_anchor_vertices,
    canonical_json,
    min_pairwise_distance_fast,
    short_hash,
)
from .planner import LANDING_Z, smoothstep
from .textshow import TextShowError, _allocate_counts, _distribute
from .timeline import (
    _blend_color_lists,
    _effect_segments,
    _finalize_plan,
    _resolve_scene_colors,
    _rotate_xy,
    _scale_xy,
)
from .validator import validate_plan_chunked
from .safe_motion import match_targets, continuous_separation, move_seconds


# Design geometry.  The frozen validator allows 0.25 m separation, 2.0 m/s,
# 8.0 m/s^2, z in [0.02, 3.0] and |xy| <= 5.0; the fleet planner keeps a
# safety margin on every axis.
FLEET_MIN_SEP = 0.38            # design pairwise separation
FLEET_SLOT_STEP = 0.38          # ring/grid slot pitch (chord and radial)
FLEET_LAYER_GAP = 0.38          # z distance between layers
FLEET_Z_MIN = 0.15
FLEET_Z_MAX = 2.85
FLEET_XY_RADIUS = 4.75
FLEET_PARKING_GAP = 0.40        # formation top -> lowest parking layer
FLEET_MAX_LAYERS = 8
FLEET_TRANSIT_SPEED = 1.45      # design cruise speed (validator 2.0)
FLEET_ROTATE_SPEED = 1.70       # design tangential speed for rotate effects
FLEET_ACCEL_DESIGN = 7.50       # design accel ceiling (validator 8.0)
FLEET_SPEED_FACTOR = 1.5        # smoothstep peak factor (single segment)
FLEET_SUBSTEP_SPEED_FACTOR = 2.25  # smoothstep peak factor when a move is
                                   # split into bumped sub-segments (each
                                   # sub-segment is smoothstepped again)
FLEET_DEFAULT_TRANSITION = 5.0
FLEET_MAX_TOTAL = 4096          # geometric fleet capacity at design pitch

# Readability model for automatic text budgets: points per character scale
# with rendered glyph complexity (pixel count at the 128 px render size).
AUTO_CJK_DIVISOR = 64.0
AUTO_CJK_MAX = 96
AUTO_LATIN_DIVISOR = 42.0
AUTO_LATIN_MAX = 64

_BASE_HEIGHT_LADDER = (2.4, 2.2, 2.0, 1.8, 1.6, 1.5, 1.4, 1.3, 1.2, 1.1,
                       1.0, 0.9, 0.8, 0.7, 0.6)

# Fine altitude-band ladder: more levels give the conflict solver the freedom
# to satisfy many simultaneous pair constraints instead of trading one
# violation for another.  Steps stay well below the required gaps so a
# satisfying assignment can always be expressed.
_BUMP_LEVELS = (0.0, 0.45, -0.45, 0.90, -0.90, 1.35, -1.35)
FLEET_LAYOUT_MIN_DIFF = 0.40    # z gap required between conflicting routes
FLEET_CONFLICT_MIN_GAP = 0.40   # altitude band separation for 3D conflicts
FLEET_XY_CONFLICT = 0.55        # x/y proximity that counts as "stacked"
FLEET_TRANSIT_SEP = 0.30        # design 3D separation during a move
FLEET_STACK_RADIUS = 0.25       # entry x/y distance of a vertical column
FLEET_DEBUG_STATS: dict[str, Any] | None = None  # optional solver diagnostics
FLEET_STACK_STEP = 0.45         # band step between stacked target groups
FLEET_MAX_COLUMN = 4            # largest vertical stack a band ladder covers
FLEET_TARGET_GROUP_TOL = 0.10   # target x/y distance that counts as one group
FLEET_ASSIGN_Z_WEIGHT = 0.5     # assignment cost weight on altitude changes
                                # (kept small: a large weight distorts the
                                #  distance-optimal matching and creates
                                #  crossing routes, which the cruise bands
                                #  cannot separate)

_TRANSIT_ATTEMPTS = (
    {"threshold": 0.33, "samples": 64, "substeps": 8},
    {"threshold": 0.38, "samples": 128, "substeps": 12},
)


class FleetError(ValueError):
    def __init__(self, code: str, detail: str, **extra: Any):
        super().__init__(detail)
        self.code = code
        self.detail = detail
        self.extra = extra


def _warn(code: str, detail: str, **extra: Any) -> dict[str, Any]:
    out: dict[str, Any] = {"code": code, "detail": detail}
    out.update(extra)
    return out


# ---------------------------------------------------------------------------
# slot geometry
# ---------------------------------------------------------------------------

def _ring_radii(outer_radius: float) -> list[float]:
    radii: list[float] = []
    r = float(outer_radius)
    while r >= FLEET_SLOT_STEP - 1e-9:
        radii.append(r)
        r -= FLEET_SLOT_STEP
    return radii


def _ring_layer_xy(count: int, outer_radius: float, phase: float) -> np.ndarray:
    """`count` points on concentric rings, outermost ring first.

    Both the chord spacing inside a ring and the radial spacing between rings
    are FLEET_SLOT_STEP, and concentric circles stay at least one radial gap
    apart, so any two slots of the layer are separated by >= FLEET_SLOT_STEP.
    """
    if count <= 0:
        return np.zeros((0, 2), dtype=float)
    out: list[np.ndarray] = []
    remaining = int(count)
    for radius in _ring_radii(outer_radius):
        if remaining <= 0:
            break
        capacity = max(1, int(2.0 * math.pi * radius / FLEET_SLOT_STEP))
        take = min(remaining, capacity)
        angles = 2.0 * math.pi * (phase + np.arange(take, dtype=float) / take)
        out.append(np.column_stack([radius * np.cos(angles),
                                    radius * np.sin(angles)]))
        remaining -= take
    if remaining > 0:
        out.append(np.zeros((1, 2), dtype=float))
        remaining -= 1
    if remaining > 0:
        raise FleetError("E_FLEET_CAPACITY",
                         "ring stack cannot host " + str(count)
                         + " slots at radius " + str(round(outer_radius, 3)))
    return np.vstack(out)


def _ring_stack_capacity(outer_radius: float) -> int:
    total = 1
    for radius in _ring_radii(outer_radius):
        total += max(1, int(2.0 * math.pi * radius / FLEET_SLOT_STEP))
    return total


def _ring_stack_slots(count: int, outer_radius: float,
                      z_levels: list[float]) -> np.ndarray:
    if count <= 0:
        return np.zeros((0, 3), dtype=float)
    if not z_levels:
        raise FleetError("E_FLEET_CAPACITY", "no z levels available for slots")
    per_layer = _distribute(int(count), len(z_levels))
    blocks: list[np.ndarray] = []
    for layer, k in enumerate(per_layer):
        if k <= 0:
            continue
        phase = (layer * 0.37) % 1.0
        xy = _ring_layer_xy(k, outer_radius, phase)
        blocks.append(np.column_stack(
            [xy, np.full(len(xy), float(z_levels[layer]), dtype=float)]))
    return np.vstack(blocks)


def _grid_lattice(spacing: float, offset: tuple[float, float],
                  limit: float) -> list[tuple[float, float]]:
    """Square lattice inside a disc of radius `limit`, ordered centre-out."""
    s = max(FLEET_MIN_SEP, float(spacing))
    bound = float(limit) - 1e-9
    cells = int(math.ceil(bound / s)) + 1
    sites: list[tuple[float, float]] = []
    for i in range(-cells, cells + 1):
        for j in range(-cells, cells + 1):
            x = i * s + offset[0]
            y = j * s + offset[1]
            if x * x + y * y <= bound * bound:
                sites.append((x, y))
    sites.sort(key=lambda p: (p[0] * p[0] + p[1] * p[1], p[0], p[1]))
    return sites


def _grid_slots(count: int, spacing: float, z_levels: list[float],
                limit: float) -> np.ndarray:
    if count <= 0:
        return np.zeros((0, 3), dtype=float)
    per_layer = _distribute(int(count), len(z_levels))
    blocks: list[np.ndarray] = []
    for layer, k in enumerate(per_layer):
        if k <= 0:
            continue
        offset = (0.0, 0.0) if layer % 2 == 0 else (spacing / 2.0, spacing / 2.0)
        sites = _grid_lattice(spacing, offset, limit)
        if k > len(sites):
            raise FleetError(
                "E_FLEET_CAPACITY",
                "grid layer cannot host " + str(k) + " slots at spacing "
                + str(round(spacing, 3)) + " m inside radius "
                + str(round(limit, 3)) + " m",
                capacity=len(sites))
        xy = np.asarray(sites[:k], dtype=float)
        blocks.append(np.column_stack(
            [xy, np.full(len(xy), float(z_levels[layer]), dtype=float)]))
    return np.vstack(blocks)


def _grid_capacity(spacing: float, limit: float = FLEET_XY_RADIUS) -> int:
    return len(_grid_lattice(spacing, (0.0, 0.0), limit))


def _ground_slots(count: int) -> tuple[np.ndarray, dict[str, Any]]:
    """All landing slots on the ground terraces (z = 0.10 + k * layer gap).

    Every slot of every needed terrace is returned so the landing assignment
    can pick the nearest `count` of them (a rectangular Hungarian problem),
    which keeps the final reposition displacement small.
    """
    if count <= 0:
        return np.zeros((0, 3), dtype=float), {"terraces": 0, "per_level": 0}
    spacing = FLEET_SLOT_STEP
    per_level = _grid_capacity(spacing)
    levels = max(1, int(math.ceil(count / float(per_level))))
    z_levels = [LANDING_Z + k * FLEET_LAYER_GAP for k in range(levels)]
    terrace = np.asarray(_grid_lattice(spacing, (0.0, 0.0), FLEET_XY_RADIUS),
                         dtype=float)
    blocks = [np.column_stack([terrace, np.full(len(terrace), z, dtype=float)])
              for z in z_levels]
    return np.vstack(blocks), {"terraces": levels, "per_level": per_level,
                               "slots": per_level * levels}


def fleet_start_positions(total: int) -> np.ndarray:
    """Deterministic, in-bounds start layout for kinematic (virtual) fleets."""
    n = int(total)
    if n < 1:
        raise FleetError("E_BAD_REQUEST", "total_drones must be positive")
    if n > FLEET_MAX_TOTAL:
        raise FleetError(
            "E_FLEET_CAPACITY",
            "fleet of " + str(n) + " drones cannot be placed within validator "
            "bounds at " + str(FLEET_SLOT_STEP) + " m design separation; the "
            "geometric capacity is about " + str(FLEET_MAX_TOTAL),
            total_drones=n, fleet_capacity=FLEET_MAX_TOTAL)
    outer = 4.55
    per_layer = _ring_stack_capacity(outer)
    z_levels: list[float] = [1.50]
    while (len(z_levels) * per_layer < n
           and z_levels[-1] + FLEET_LAYER_GAP <= FLEET_Z_MAX):
        z_levels.append(round(z_levels[-1] + FLEET_LAYER_GAP, 6))
    z = 1.50 - FLEET_LAYER_GAP
    while len(z_levels) * per_layer < n and z >= FLEET_Z_MIN:
        z_levels.append(round(z, 6))
        z -= FLEET_LAYER_GAP
    if len(z_levels) * per_layer < n:
        raise FleetError(
            "E_FLEET_CAPACITY",
            "fleet of " + str(n) + " drones cannot be parked within the "
            "validator z band; capacity ~" + str(len(z_levels) * per_layer),
            total_drones=n)
    return _ring_stack_slots(n, outer, z_levels)


# ---------------------------------------------------------------------------
# scene geometry helpers
# ---------------------------------------------------------------------------

def _scene_z(scene: dict[str, Any]) -> float:
    try:
        z = float(scene.get("z", 1.4))
    except (TypeError, ValueError) as exc:
        raise FleetError("E_BAD_SCENE", "scene z must be a number") from exc
    if not math.isfinite(z):
        raise FleetError("E_BAD_SCENE", "scene z must be finite")
    return min(FLEET_Z_MAX, max(FLEET_Z_MIN, z))


def _layer_z_levels(z: float, layers: int) -> list[float]:
    """`layers` z levels centred on z and clamped into the design band."""
    levels = [z + (i - (layers - 1) / 2.0) * FLEET_LAYER_GAP
              for i in range(layers)]
    low = min(levels)
    if low < FLEET_Z_MIN:
        levels = [v + (FLEET_Z_MIN - low) for v in levels]
    high = max(levels)
    if high > FLEET_Z_MAX:
        levels = [v - (high - FLEET_Z_MAX) for v in levels]
    if min(levels) < FLEET_Z_MIN - 1e-9 or max(levels) > FLEET_Z_MAX + 1e-9:
        raise FleetError("E_BAD_SCENE",
                         "too many layers to fit inside the z band")
    return [round(float(v), 6) for v in levels]


def _rotate_plan(turns: float, anim: float) -> tuple[int, float] | None:
    """Steps and per-unit-radius chord for a rotate effect of `anim` seconds.

    Steps are chosen so every sub-arc is at most 22.5 degrees (>= 16 steps per
    turn), which keeps the chord interpolation between waypoints from pulling
    neighbouring drones closer together (the shrink factor is cos(dtheta/2),
    i.e. 0.98 at 22.5 degrees).
    """
    if anim <= 1e-9 or turns <= 1e-12:
        return None
    steps = int(max(16.0, min(96.0, math.ceil(16.0 * turns))))
    angle = 2.0 * math.pi * turns / steps
    if angle >= math.pi:
        chord_factor = 2.0
    else:
        chord_factor = 2.0 * math.sin(angle / 2.0)
    return steps, chord_factor


def _rotate_radius_cap(scene: dict[str, Any], anim: float,
                       effect: dict[str, Any] | None = None
                       ) -> float | None:
    """Max formation radius that keeps a rotate effect inside design speeds."""
    if effect is None:
        effect = scene.get("effect")
    if not isinstance(effect, dict):
        return None
    if str(effect.get("type", "none")).lower() != "rotate":
        return None
    try:
        turns = abs(float(effect.get("turns", 1.0)))
    except (TypeError, ValueError):
        turns = 1.0
    plan = _rotate_plan(turns, anim)
    if plan is None:
        return None
    steps, chord_factor = plan
    r_speed = FLEET_ROTATE_SPEED * anim / (FLEET_SPEED_FACTOR * steps * chord_factor)
    r_accel = FLEET_ACCEL_DESIGN * anim * anim / (6.0 * chord_factor * steps * steps)
    return float(min(r_speed, r_accel))


def _fleet_rotate_segments(base: np.ndarray, active_mask: np.ndarray,
                           turns: float, duration: float, name_prefix: str,
                           base_colors: list[str], scene_color: str,
                           scene_gradient: Any) -> list[dict[str, Any]]:
    """Deterministic rotate effect (the shared helper freezes on whole turns)."""
    plan = _rotate_plan(abs(float(turns)), float(duration))
    if plan is None:
        return []
    steps, _ = plan
    sign = 1.0 if float(turns) >= 0 else -1.0
    angle = sign * 2.0 * math.pi * float(turns) / steps
    active = np.where(np.asarray(active_mask, dtype=bool))[0]
    active_base = base[active].copy() if len(active) else base[:0].copy()
    centre = active_base[:, :2].mean(axis=0) if len(active_base) else np.zeros(2)
    positions = [base.copy()]
    for index in range(1, steps + 1):
        frame = base.copy()
        if len(active):
            frame[active] = _rotate_xy(active_base, angle * index, centre)
        positions.append(frame)
    end_color = DEFAULT_COLOR
    if isinstance(scene_gradient, (list, tuple)) and len(scene_gradient) == 2:
        end_color = str(scene_gradient[1])
    end_colors = [lerp_hex(color, end_color, 0.55) for color in base_colors]
    base_duration = float(duration) / float(steps)
    segments: list[dict[str, Any]] = []
    for index in range(steps):
        segment_duration = (base_duration if index < steps - 1 else
                            float(duration) - base_duration * (steps - 1))
        u = (index + 1) / float(steps)
        colors = _blend_color_lists(base_colors, end_colors, u)
        segments.append({
            "name": name_prefix + "_rotate",
            "duration": max(1e-6, segment_duration),
            "start_positions": positions[index].copy(),
            "end_positions": positions[index + 1].copy(),
            "color": colors[0] if colors else scene_color,
            "colors": colors,
        })
    return segments


# ---------------------------------------------------------------------------
# text scenes
# ---------------------------------------------------------------------------

def _text_complexities(chars: list[str], font_path: str) -> list[int]:
    values: list[int] = []
    for char in chars:
        coords, _ = render_glyph_coords(char, font_path)
        values.append(int(len(coords)))
    return values


def _auto_text_budget(complexities: list[int], min_points: int,
                      is_cjk: bool) -> int:
    avg = sum(max(1, c) for c in complexities) / float(len(complexities))
    if is_cjk:
        target = min(int(round(avg / AUTO_CJK_DIVISOR)), AUTO_CJK_MAX)
    else:
        target = min(int(round(avg / AUTO_LATIN_DIVISOR)), AUTO_LATIN_MAX)
    target = max(target, min_points)
    return len(complexities) * target


def _build_text_layers(chars: list[str], allocations: list[int], z: float,
                       font_path: str, base_height_requested: float | None,
                       letter_spacing_requested: float | None
                       ) -> tuple[np.ndarray, dict[str, Any]] | None:
    """Depth-stacked text pages, or None when the geometry is infeasible.

    Each z layer is an independent page built by `build_page_points`, so the
    in-layer separation is >= 0.30 m by construction while the cross-layer
    distance is >= FLEET_LAYER_GAP regardless of xy overlap.
    """
    ladder: list[float] = []
    if base_height_requested is not None:
        ladder.append(float(base_height_requested))
    ladder.extend(b for b in _BASE_HEIGHT_LADDER
                  if base_height_requested is None or b < float(base_height_requested))
    for base_height in ladder:
        if letter_spacing_requested is not None:
            letter_spacing = float(letter_spacing_requested)
        else:
            letter_spacing = base_height / 2.4 * 0.16
        for layers in range(1, FLEET_MAX_LAYERS + 1):
            splits = [_distribute(a, layers) for a in allocations]
            pages: list[np.ndarray] = []
            try:
                for layer in range(layers):
                    alloc_l = [splits[c][layer] for c in range(len(chars))]
                    if sum(alloc_l) <= 0:
                        continue
                    pts, _ = build_page_points(
                        list(chars), alloc_l, font_path,
                        base_height=base_height,
                        letter_spacing=letter_spacing,
                        min_separation=FLEET_MIN_SEP,
                        max_radius=FLEET_XY_RADIUS - 0.05)
                    pages.append(pts)
            except CJKError:
                continue
            if not pages:
                continue
            z_levels = _layer_z_levels(z, len(pages))
            points = np.vstack([
                np.column_stack([page, np.full(len(page), zl, dtype=float)])
                for page, zl in zip(pages, z_levels)
            ])
            radius = (float(np.max(np.linalg.norm(points[:, :2], axis=1)))
                      if len(points) else 0.0)
            meta = {"base_height": float(base_height),
                    "letter_spacing": float(letter_spacing),
                    "layers": int(len(pages)),
                    "z_levels": list(z_levels),
                    "radius": radius}
            return points, meta
    return None


def _text_scene(scene: dict[str, Any], index: int, budget: int | None,
                font_path: str, total_drones: int
                ) -> tuple[np.ndarray, dict[str, Any], list[dict[str, Any]]]:
    warnings: list[dict[str, Any]] = []
    text = str(scene.get("text", "")).strip()
    if not text:
        raise FleetError("E_BAD_SCENE", "text scene requires a non-empty text")
    chars = [ch for ch in text if not ch.isspace()]
    if not chars:
        raise FleetError("E_BAD_SCENE",
                         "text scene requires a non-space character")
    if budget is None and all(ord(ch) < 128 for ch in chars) and not any(
            key in scene for key in ("min_points_per_char", "base_height", "letter_spacing")):
        from .cinematic import _glyph
        points = _glyph(text, .35)
        points[:, 2] += _scene_z(scene)-1.5
        if (len(points) <= total_drones and np.linalg.norm(points[:, :2], axis=1).max() <= 4.97
                and points[:, 2].min() >= FLEET_Z_MIN and points[:, 2].max() <= FLEET_Z_MAX):
            return points, {"type": "text", "upright": True, "points": len(points),
                    "points_per_char": len(points)/len(chars), "layers": 1,
                    "z_levels": [_scene_z(scene)], "radius": float(np.linalg.norm(points[:, :2], axis=1).max()),
                    "min_distance": min_pairwise_distance_fast(points)}, []
    try:
        min_points = max(1, int(scene.get("min_points_per_char", 16)))
    except (TypeError, ValueError) as exc:
        raise FleetError("E_BAD_SCENE",
                         "min_points_per_char must be an integer") from exc
    is_cjk = any(ord(ch) > 127 for ch in chars)
    complexities = _text_complexities(chars, font_path)
    z = _scene_z(scene)

    if budget is None:
        total = min(int(total_drones),
                    _auto_text_budget(complexities, min_points, is_cjk))
        warnings.append(_warn(
            "W_AUTO_BUDGET",
            "auto text budget chose " + str(total) + " active drones from "
            "glyph complexity and readability",
            active=total))
    else:
        total = int(budget)
        if total > int(total_drones):
            warnings.append(_warn(
                "W_BUDGET_REDUCED",
                "text drone_budget reduced from " + str(total) + " to the "
                "fleet size " + str(int(total_drones)),
                requested=total, active=int(total_drones)))
            total = int(total_drones)
    min_total = min_points * len(chars)
    if total < min_total:
        raise FleetError(
            "E_POINT_BUDGET",
            "scene " + str(index) + " text needs at least " + str(min_total)
            + " points for " + str(len(chars)) + " characters at "
            + str(min_points) + " points per character",
            required_points=min_total, available_points=total)

    base_height_requested = (float(scene["base_height"])
                             if scene.get("base_height") is not None else None)
    letter_spacing_requested = (float(scene["letter_spacing"])
                                if scene.get("letter_spacing") is not None else None)
    while True:
        try:
            allocations = _allocate_counts(total, complexities, min_points)
        except TextShowError as exc:
            raise FleetError(exc.code, exc.detail, **exc.extra) from exc
        built = _build_text_layers(chars, allocations, z, font_path,
                                   base_height_requested,
                                   letter_spacing_requested)
        if built is not None:
            points, meta = built
            break
        if total <= min_total:
            raise FleetError(
                "E_POINT_BUDGET",
                "text " + repr(text) + " cannot host " + str(total)
                + " points inside validator bounds even at minimum size; "
                "split the text across scenes or lower min_points_per_char",
                required_points=total)
        total = max(min_total, int(total * 0.8))
        warnings.append(_warn(
            "W_BUDGET_REDUCED",
            "text budget reduced to " + str(total) + " points for bounds "
            "feasibility", active=total))

    meta.update({
        "type": "text",
        "points": int(len(points)),
        "points_per_char": round(float(total) / len(chars), 3),
        "allocations": [int(a) for a in allocations],
        "min_distance": (None if len(points) < 2 else
                         round(float(min_pairwise_distance_fast(points)), 6)),
    })
    return points, meta, warnings


# ---------------------------------------------------------------------------
# shape scenes
# ---------------------------------------------------------------------------

def _shape_scene(scene: dict[str, Any], budget: int | None, total_drones: int,
                 radius_cap: float | None
                 ) -> tuple[np.ndarray, dict[str, Any], list[dict[str, Any]]]:
    warnings: list[dict[str, Any]] = []
    scene_type = str(scene.get("type", "")).lower()
    z = _scene_z(scene)
    grid_limit = FLEET_XY_RADIUS
    make_layer = None

    if scene_type == "ring":
        try:
            radius = float(scene.get("radius", 3.5))
        except (TypeError, ValueError) as exc:
            raise FleetError("E_BAD_SCENE", "ring radius must be a number") from exc
        radius = min(max(radius, FLEET_SLOT_STEP), FLEET_XY_RADIUS)
        if radius_cap is not None and radius > radius_cap:
            warnings.append(_warn(
                "W_ROTATE_RADIUS_CLAMPED",
                "ring radius reduced from " + str(round(radius, 3)) + " m to "
                + str(round(radius_cap, 3)) + " m to keep the rotate effect "
                "inside validator speed limits",
                requested_radius=round(radius, 3),
                used_radius=round(radius_cap, 3)))
            radius = max(FLEET_SLOT_STEP, radius_cap)
        per_layer = _ring_stack_capacity(radius)
        auto_budget = min(total_drones, per_layer * 4)

    elif scene_type == "grid":
        try:
            spacing = float(scene.get("spacing", 0.5))
        except (TypeError, ValueError) as exc:
            raise FleetError("E_BAD_SCENE", "grid spacing must be a number") from exc
        spacing = min(max(spacing, FLEET_MIN_SEP), 2.0)
        if radius_cap is not None and grid_limit > radius_cap:
            warnings.append(_warn(
                "W_ROTATE_RADIUS_CLAMPED",
                "grid radius reduced from " + str(round(grid_limit, 3))
                + " m to " + str(round(radius_cap, 3))
                + " m to keep the rotate effect inside validator speed limits",
                used_radius=round(radius_cap, 3)))
            grid_limit = max(FLEET_SLOT_STEP, radius_cap)
        per_layer = _grid_capacity(spacing, grid_limit)
        auto_budget = min(total_drones, per_layer * 4)

    elif scene_type == "heart":
        try:
            size = float(scene.get("size", 7.0))
        except (TypeError, ValueError) as exc:
            raise FleetError("E_BAD_SCENE", "heart size must be a number") from exc
        size = min(max(size, 1.0), 9.0)
        base_curve = _resample_closed_curve(_heart_curve, 512)
        centre = np.array([(base_curve[:, 0].min() + base_curve[:, 0].max()) / 2.0,
                           (base_curve[:, 1].min() + base_curve[:, 1].max()) / 2.0])
        width = float(np.ptp(base_curve[:, 0]))
        scale = size / width if width > 0 else 1.0
        if radius_cap is not None:
            span = float(np.max(np.linalg.norm((base_curve - centre) * scale,
                                               axis=1)))
            if span > radius_cap:
                scale *= max(0.0, radius_cap / span)
                warnings.append(_warn(
                    "W_ROTATE_RADIUS_CLAMPED",
                    "heart size reduced to stay inside the rotate speed limit",
                    used_radius=round(radius_cap, 3)))

        def make_layer(k: int, phase: float) -> np.ndarray:
            xy = _resample_closed_curve(_heart_curve, k, phase_fraction=phase)
            return (xy - centre) * scale
        layer_curve = (base_curve - centre) * scale
        arc = _closed_arc(layer_curve)
        per_layer = max(1, int(arc / FLEET_SLOT_STEP))
        auto_budget = min(total_drones, per_layer * 4)

    elif scene_type == "star":
        try:
            radius = float(scene.get("radius", 3.5))
        except (TypeError, ValueError) as exc:
            raise FleetError("E_BAD_SCENE", "star radius must be a number") from exc
        radius = min(max(radius, 0.5), FLEET_XY_RADIUS)
        try:
            inner_ratio = float(scene.get("inner_ratio", 0.45))
        except (TypeError, ValueError) as exc:
            raise FleetError("E_BAD_SCENE",
                             "star inner_ratio must be a number") from exc
        inner_ratio = min(max(inner_ratio, 0.05), 0.95)
        if radius_cap is not None and radius > radius_cap:
            warnings.append(_warn(
                "W_ROTATE_RADIUS_CLAMPED",
                "star radius reduced to " + str(round(radius_cap, 3))
                + " m to keep the rotate effect inside validator speed limits",
                used_radius=round(radius_cap, 3)))
            radius = max(0.5, radius_cap)
        verts = _star_anchor_vertices(radius, inner_ratio)

        def make_layer(k: int, phase: float) -> np.ndarray:
            return _resample_closed_polyline(verts, k, phase_fraction=phase)
        arc = _closed_arc(_resample_closed_polyline(verts, 512))
        per_layer = max(1, int(arc / FLEET_SLOT_STEP))
        auto_budget = min(total_drones, per_layer * 4)
    else:
        raise FleetError("E_BAD_SCENE",
                         "unsupported scene type " + repr(scene_type))

    if make_layer is not None:
        # Arc length alone overestimates capacity at cusps and star corners.
        # Certify every layer phase before choosing an automatic budget.
        while per_layer > 1 and any(min_pairwise_distance_fast(make_layer(per_layer, (i*.11) % 1.))
                                    < FLEET_MIN_SEP for i in range(FLEET_MAX_LAYERS)):
            per_layer -= 1
        auto_budget = min(total_drones, per_layer * 4)

    target = auto_budget if budget is None else int(budget)
    target = min(target, total_drones)
    if target < 1:
        raise FleetError("E_BAD_SCENE", "scene budget must be positive")
    capacity = per_layer * FLEET_MAX_LAYERS
    if target > capacity:
        warnings.append(_warn(
            "W_BUDGET_REDUCED",
            "scene budget reduced from " + str(target) + " to " + str(capacity)
            + " (shape capacity at design separation)",
            requested=target, active=capacity))
        target = capacity

    layers = min(FLEET_MAX_LAYERS, max(1, int(math.ceil(target / per_layer))))
    z_levels = _layer_z_levels(z, layers)
    if scene_type == "ring":
        points = _ring_stack_slots(target, radius, z_levels)
    elif scene_type == "grid":
        points = _grid_slots(target, spacing, z_levels, grid_limit)
    else:
        per = _distribute(target, layers)
        blocks: list[np.ndarray] = []
        for layer, k in enumerate(per):
            if k <= 0:
                continue
            phase = (layer * 0.11) % 1.0
            xy = make_layer(k, phase)  # type: ignore[misc]
            blocks.append(np.column_stack(
                [xy, np.full(len(xy), float(z_levels[layer]), dtype=float)]))
        points = np.vstack(blocks)

    radius_used = (float(np.max(np.linalg.norm(points[:, :2], axis=1)))
                   if len(points) else 0.0)
    meta = {"type": scene_type, "points": int(len(points)), "layers": layers,
            "z_levels": [round(float(v), 6) for v in z_levels],
            "per_layer_capacity": int(per_layer),
            "radius": radius_used,
            "points_per_char": 0,
            "min_distance": (None if len(points) < 2 else
                             round(float(min_pairwise_distance_fast(points)), 6))}
    return points, meta, warnings


def centre_of(points: np.ndarray) -> np.ndarray:
    return (points.min(axis=0) + points.max(axis=0)) / 2.0


def _closed_arc(curve: np.ndarray) -> float:
    closed = np.vstack([curve, curve[:1]])
    return float(np.sum(np.linalg.norm(np.diff(closed, axis=0), axis=1)))


# ---------------------------------------------------------------------------
# parking
# ---------------------------------------------------------------------------

def _parking_slots(count: int, form_z_min: float, form_z_max: float,
                   extra_gap: float) -> tuple[np.ndarray, dict[str, Any]]:
    if count <= 0:
        return np.zeros((0, 3), dtype=float), {"slots": 0, "z_levels": [],
                                               "band": "none"}
    outer = FLEET_XY_RADIUS
    per_layer = _ring_stack_capacity(outer)
    levels_above: list[float] = []
    z = form_z_max + FLEET_PARKING_GAP + extra_gap
    while len(levels_above) * per_layer < count and z <= FLEET_Z_MAX + 1e-9:
        levels_above.append(round(float(z), 6))
        z += FLEET_LAYER_GAP
    if len(levels_above) * per_layer >= count:
        return _ring_stack_slots(count, outer, levels_above), {
            "slots": count, "z_levels": levels_above, "band": "above"}
    z = form_z_min - FLEET_PARKING_GAP - extra_gap
    levels_below: list[float] = []
    while ((len(levels_above) + len(levels_below)) * per_layer < count
           and z >= FLEET_Z_MIN - 1e-9):
        levels_below.append(round(float(z), 6))
        z -= FLEET_LAYER_GAP
    levels = levels_below[::-1] + levels_above
    if len(levels) * per_layer < count:
        raise FleetError(
            "E_BAD_SCENE",
            "no collision-free parking band for " + str(count)
            + " idle drones; the active formation occupies too much of the z "
            "volume of this scene",
            idle=count, parking_capacity=len(levels) * per_layer)
    return _ring_stack_slots(count, outer, levels), {
        "slots": count, "z_levels": levels, "band": "above+below"}


# ---------------------------------------------------------------------------
# transitions
# ---------------------------------------------------------------------------

def _stacked_target_groups(targets: np.ndarray,
                           tol: float = 0.05) -> list[list[int]]:
    """Index groups of target slots that share (nearly) the same x/y."""
    n = len(targets)
    if n < 2:
        return []
    from scipy.spatial import cKDTree

    parent = list(range(n))

    def find(a: int) -> int:
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for i, j in cKDTree(targets[:, :2]).query_pairs(r=float(tol)):
        ri, rj = find(int(i)), find(int(j))
        if ri != rj:
            parent[max(ri, rj)] = min(ri, rj)
    buckets: dict[int, list[int]] = {}
    for i in range(n):
        buckets.setdefault(find(i), []).append(i)
    return [group for group in buckets.values() if len(group) > 1]


def _cruise_bands(entry: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Altitude offset per drone for a collision-safe lift/horizontal move.

    The lift takes every drone to `target altitude + band`, the horizontal
    phase then runs at that altitude, and the settle removes the band.

    * Two drones whose targets are stacked (same x/y, different layer) keep
      their full target altitude gap while their x/y paths converge, because
      their cruise altitudes are their target altitudes (the dominant
      collision class).
    * Two drones that share their entry x/y are the ones the lift can push
      through each other, so every entry "column" gets a centred band ladder
      in entry-altitude order: within a column the altitude order (and gap)
      is preserved through the whole move.
    """
    n = len(entry)
    bands = np.zeros(n, dtype=float)
    from scipy.spatial import cKDTree

    parent = list(range(n))

    def find(a: int) -> int:
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for i, j in cKDTree(entry[:, :2]).query_pairs(r=float(FLEET_STACK_RADIUS)):
        ri, rj = find(int(i)), find(int(j))
        if ri != rj:
            parent[max(ri, rj)] = min(ri, rj)
    columns: dict[int, list[int]] = {}
    for index in range(n):
        columns.setdefault(find(index), []).append(index)
    window = FLEET_Z_MAX - FLEET_Z_MIN

    def column_step(size: int) -> float:
        """Ladder step that keeps a whole column inside the altitude band."""
        if size < 2:
            return FLEET_STACK_STEP
        return min(FLEET_STACK_STEP, window / float(size - 1))
    for members in columns.values():
        if len(members) < 2:
            continue
        ordered = sorted(members, key=lambda i: (float(entry[i, 2]), i))
        # Centred ladder in entry-altitude order: within a column the altitude
        # order (and gap) is preserved through the whole move.
        step = column_step(len(ordered))
        span = (len(ordered) - 1) / 2.0
        for rank, index in enumerate(ordered):
            bands[index] = (rank - span) * step
    # Absolute cruise altitude: the target altitude plus the column ladder,
    # shifted per column so every drone stays inside the altitude band.
    absolute = target[:, 2] + bands
    for members in columns.values():
        if len(members) < 2:
            continue
        targets_z = np.asarray([target[index, 2] for index in members],
                               dtype=float)
        band_values = np.asarray([bands[index] for index in members],
                                 dtype=float)
        spread = float(band_values.max() - band_values.min())
        room = window - float(targets_z.max() - targets_z.min())
        if spread > 1e-9 and spread > max(0.0, room):
            # Shrink the ladder (never the target altitudes) so the column
            # still fits the altitude band; the repair pass below recovers any
            # separation this costs.
            scale = max(0.0, room) / spread
            for index in members:
                bands[index] *= scale
        values = np.asarray([target[index, 2] + bands[index]
                             for index in members], dtype=float)
        low, high = float(values.min()), float(values.max())
        shift = 0.0
        if low < FLEET_Z_MIN:
            shift = FLEET_Z_MIN - low
        if high + shift > FLEET_Z_MAX:
            shift -= (high + shift) - FLEET_Z_MAX
        for index in members:
            absolute[index] = target[index, 2] + bands[index] + shift
    absolute = np.clip(absolute, FLEET_Z_MIN, FLEET_Z_MAX)
    cruise = np.clip(_repair_cruise_bands(entry, target,
                                          absolute - entry[:, 2]),
                     FLEET_Z_MIN - FLEET_Z_MAX, FLEET_Z_MAX - FLEET_Z_MIN)
    # Members of a stacked target group (same target x/y, different layer)
    # cruise at exactly their target altitude: the group's target altitude
    # gaps are then held for the whole horizontal move, and the assignment
    # already ordered those targets to match the entry altitude order, so the
    # lift cannot invert them either.  This is applied last because it is the
    # property the horizontal phase depends on most.
    zeroed: set[int] = set()
    for group in _stacked_target_groups(target, tol=FLEET_TARGET_GROUP_TOL):
        for index in group:
            cruise[index] = float(target[index, 2] - entry[index, 2])
            zeroed.add(index)

    # A shared entry column must stay ordered through the lift, so if any of
    # its members was pinned to its target altitude the rest of the column is
    # re-spaced around it in entry-altitude order.
    if zeroed:
        from scipy.spatial import cKDTree

        parent = list(range(len(entry)))

        def find(a: int) -> int:
            while parent[a] != a:
                parent[a] = parent[parent[a]]
                a = parent[a]
            return a

        for i, j in cKDTree(entry[:, :2]).query_pairs(
                r=float(FLEET_STACK_RADIUS)):
            ri, rj = find(int(i)), find(int(j))
            if ri != rj:
                parent[max(ri, rj)] = min(ri, rj)
        members: dict[int, list[int]] = {}
        for index in range(len(entry)):
            members.setdefault(find(index), []).append(index)
        for column in members.values():
            if len(column) < 2 or not (set(column) & zeroed):
                continue
            ordered = sorted(column, key=lambda i: (float(entry[i, 2]), i))
            anchor = next(position for position, index in enumerate(ordered)
                          if index in zeroed)
            anchor_cruise = entry[ordered[anchor], 2] + cruise[ordered[anchor]]
            # Only a few ladder steps around the anchor are meaningful; the
            # union-find can chain unrelated drones into one oversized group,
            # and a ladder that long would leave the altitude band entirely.
            ladder_step = column_step(len(ordered))
            steps = [position - anchor for position in range(len(ordered))]
            levels = [anchor_cruise + step * ladder_step for step in steps]
            # The capped ladder spans at most a couple of metres, so shifting
            # it into the band always works and keeps every gap intact.
            shift = 0.0
            low, high = min(levels), max(levels)
            if low < FLEET_Z_MIN:
                shift = FLEET_Z_MIN - low
            if high + shift > FLEET_Z_MAX:
                shift -= (high + shift) - FLEET_Z_MAX
            for position, index in enumerate(ordered):
                cruise[index] = (levels[position] + shift
                                 - float(entry[index, 2]))
        # Re-pin last: the column ladder above can touch a drone that is also
        # part of a stacked target group, and for those the target altitude
        # itself is what keeps the converging pair apart.
        for group in _stacked_target_groups(target, tol=FLEET_TARGET_GROUP_TOL):
            for index in group:
                cruise[index] = float(target[index, 2] - entry[index, 2])
    return cruise


def _pair_constraints(entry: np.ndarray, target: np.ndarray
                      ) -> list[list[tuple[int, float, float, float, float, float]]]:
    """Per-drone pair constraints used by the band solvers.

    Only pairs whose x/y paths come close (entry, target or in between) can
    collide; each entry carries the x/y distances at the three critical
    instants plus the entry and target altitude differences.
    """
    n = len(entry)
    design = FLEET_TRANSIT_SEP
    from scipy.spatial import cKDTree

    frame_xy = entry[:, :2]
    closest: dict[tuple[int, int], float] = {}
    for sample in range(33):
        s = sample / 32.0
        xy = frame_xy + (target[:, :2] - frame_xy) * s
        for i, j in cKDTree(xy).query_pairs(r=float(FLEET_XY_CONFLICT)):
            i, j = int(i), int(j)
            distance = float(np.linalg.norm(xy[i] - xy[j]))
            key = (i, j) if i < j else (j, i)
            known = closest.get(key)
            if known is None or distance < known:
                closest[key] = distance
    constraints: list[list[tuple[int, float, float, float, float, float]]] = [
        [] for _ in range(n)]
    for (i, j), d_min in closest.items():
        d_entry = float(np.linalg.norm(entry[i, :2] - entry[j, :2]))
        d_target = float(np.linalg.norm(target[i, :2] - target[j, :2]))
        if d_entry >= design and d_target >= design and d_min >= design:
            continue
        info = (d_entry, d_target, d_min,
                float(entry[i, 2] - entry[j, 2]),
                float(target[i, 2] - target[j, 2]))
        constraints[i].append((j, *info))
        constraints[j].append((i, info[0], info[1], info[2], -info[3], -info[4]))
    return constraints


def _repair_cruise_bands(entry: np.ndarray, target: np.ndarray,
                         cruise: np.ndarray) -> np.ndarray:
    """Directed repair of the constructive cruise bands.

    The construction is safe for stacked targets and for entry columns, but a
    column ladder can perturb a stacked-target pair that lives across two
    columns.  This pass walks the residual conflicts and solves one drone's
    cruise altitude at a time (the altitude gap at the end of the lift must
    carry the sign of the target gap and reach the design separation), keeping
    a change only when it reduces that drone's own conflicts.
    """
    design = FLEET_TRANSIT_SEP
    constraints = _pair_constraints(entry, target)
    if not any(constraints):
        return cruise

    def margin_for(index: int, target_index: int, value: float) -> float:
        other = cruise[target_index]
        for constraint in constraints[index]:
            if constraint[0] != target_index:
                continue
            return _pair_band_margin(value - other, constraint[4],
                                     constraint[5], constraint[1],
                                     constraint[2], constraint[3])
        return design

    def violations(index: int, value: float) -> float:
        total = 0.0
        for constraint in constraints[index]:
            other = constraint[0]
            margin = _pair_band_margin(value - cruise[other], constraint[4],
                                       constraint[5], constraint[1],
                                       constraint[2], constraint[3])
            if margin < design - 1e-9:
                total += 1.0
        return total

    for _ in range(12):
        conflicts: list[tuple[int, int]] = []
        for index in range(len(entry)):
            for constraint in constraints[index]:
                other = constraint[0]
                if other < index:
                    continue
                margin = _pair_band_margin(cruise[index] - cruise[other],
                                           constraint[4], constraint[5],
                                           constraint[1], constraint[2],
                                           constraint[3])
                if margin < design - 1e-9:
                    conflicts.append((index, other))
                    if len(conflicts) >= 512:
                        break
            if len(conflicts) >= 512:
                break
        if not conflicts:
            break
        improved = False
        for i, j in conflicts:
            for index, other in ((j, i), (i, j)):
                sign = 1.0 if target[index, 2] >= target[other, 2] else -1.0
                needed = sign * max(design,
                                    abs(float(target[index, 2]
                                              - target[other, 2])))
                value = (float(entry[other, 2]) + cruise[other] + needed
                         - float(entry[index, 2]))
                absolute = float(entry[index, 2]) + value
                if absolute < FLEET_Z_MIN or absolute > FLEET_Z_MAX:
                    continue
                before = violations(index, cruise[index])
                after = violations(index, value)
                if after < before - 1e-9:
                    cruise[index] = value
                    improved = True
                    break
        if not improved:
            break
    return cruise


def _assign_fleet_targets(entry: np.ndarray,
                          targets: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Minimum-total-distance assignment with an altitude-preserving bias.

    Layered formations often stack two or more target slots at the same x/y
    with only the z layer apart.  A plain distance assignment may swap those
    slots between drones that approach from opposite sides, which makes the
    two routes pass straight through each other near the target.  Weighting
    altitude changes keeps every drone in its own altitude band, so stacked
    slots are approached from the correct side.
    """
    delta = entry[:, None, :] - targets[None, :, :]
    cost = (np.sum(delta[:, :, :2] ** 2, axis=2)
            + FLEET_ASSIGN_Z_WEIGHT * delta[:, :, 2] ** 2)
    _, cols = linear_sum_assignment(cost)
    cols = np.asarray(cols, dtype=int)

    # Repair stacked slots: within a group of targets that share the same x/y
    # the drones are re-matched in altitude order, so a drone never swaps its
    # altitude band with the layer stacked directly above/below it.
    groups = _stacked_target_groups(targets)
    if groups:
        group_of = np.full(len(targets), -1, dtype=int)
        for index, group in enumerate(groups):
            group_of[np.asarray(group, dtype=int)] = index
        assigned_group = group_of[cols]
        for index, group in enumerate(groups):
            members = np.asarray(group, dtype=int)
            drones = np.where(assigned_group == index)[0]
            if len(drones) != len(members):
                continue
            drone_order = drones[np.argsort(entry[drones, 2], kind="stable")]
            target_order = members[np.argsort(targets[members, 2], kind="stable")]
            cols[drone_order] = target_order

    # Drones that share their entry x/y are only told apart by altitude.  Their
    # assigned targets are re-ordered in altitude order too, so the lift phase
    # (which moves them vertically while the x/y layout is untouched) can never
    # push one through another.
    for group in _stacked_target_groups(entry, tol=FLEET_STACK_RADIUS):
        members = np.asarray(group, dtype=int)
        drone_order = members[np.argsort(entry[members, 2], kind="stable")]
        assigned = cols[drone_order]
        cols[drone_order] = assigned[np.argsort(targets[assigned, 2],
                                                kind="stable")]
    return _uncross_assignments(entry, targets, cols)


def _uncross_assignments(entry: np.ndarray, targets: np.ndarray,
                         cols: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Remove crossing routes whose swap costs the same (ties).

    A distance-optimal matching has no crossings unless the two alternatives
    tie, which happens for near-parallel routes; the tie is then broken
    arbitrarily and the two drones fly through each other.  This pass looks at
    pairs that are close in both the entry and the target layout and swaps
    their targets when that does not increase the total distance but does
    remove a proper route crossing.
    """
    from scipy.spatial import cKDTree

    n = len(entry)
    if n < 2:
        return targets[cols], cols
    matched = targets[cols].copy()
    radius = 1.0
    seen: set[tuple[int, int]] = set()
    for i, j in cKDTree(entry[:, :2]).query_pairs(r=radius):
        i, j = int(i), int(j)
        if i > j:
            i, j = j, i
        if float(np.linalg.norm(matched[i, :2] - matched[j, :2])) < radius:
            seen.add((i, j))
    for i, j in sorted(seen):
        start_i, start_j = entry[i, :2], entry[j, :2]
        end_i, end_j = matched[i, :2], matched[j, :2]
        d_i = end_i - start_i
        d_j = end_j - start_j
        offset = start_j - start_i

        def cross2(a: np.ndarray, b: np.ndarray) -> float:
            return float(a[0] * b[1] - a[1] * b[0])

        denominator = cross2(d_i, d_j)
        if abs(denominator) < 1e-12:
            continue
        t = cross2(offset, d_j) / denominator
        u = cross2(offset, d_i) / denominator
        if not (1e-6 < t < 1.0 - 1e-6 and 1e-6 < u < 1.0 - 1e-6):
            continue
        before = (float(np.linalg.norm(entry[i] - matched[i]))
                  + float(np.linalg.norm(entry[j] - matched[j])))
        after = (float(np.linalg.norm(entry[i] - matched[j]))
                 + float(np.linalg.norm(entry[j] - matched[i])))
        if after <= before + 1e-9:
            cols[i], cols[j] = cols[j], cols[i]
            matched[i], matched[j] = matched[j].copy(), matched[i].copy()
    return targets[cols], cols


def _pair_band_margin(delta_amp: float, e_delta: float, t_delta: float,
                      d_entry: float, d_target: float, d_min: float) -> float:
    """Worst 3D separation this altitude band choice leaves a pair.

    The move has three phases and the pair's x/y distance is fixed at
    `d_entry` during the lift, travels down to `d_min` during the x/y move
    (with the altitude offset `b = e_delta + delta_amp` held constant) and is
    fixed at `d_target` during the settle, where the altitude gap returns to
    `c = t_delta`.  Each phase is safe when either the x/y distance alone is
    enough or the altitude gap (affine inside a phase) never crosses and
    stays wide enough; the returned value is the smallest 3D distance over
    those critical instants.
    """
    a = float(e_delta)
    b = float(e_delta) + float(delta_amp)
    c = float(t_delta)
    if abs(a) <= 1e-12 or ((a > 0.0) == (b > 0.0)):
        lift = math.hypot(d_entry, min(abs(a), abs(b)))
    else:
        lift = d_entry
    horizontal = math.hypot(d_min, abs(b))
    if (b > 0.0) == (c > 0.0) or abs(b) <= 1e-12:
        settle = math.hypot(d_target, min(abs(b), abs(c)))
    else:
        settle = d_target
    return min(lift, horizontal, settle)


def _conflict_bands(entry: np.ndarray, target: np.ndarray, threshold: float,
                    samples: int,
                    xy_threshold: float = FLEET_XY_CONFLICT) -> np.ndarray:
    """Altitude band amplitude per drone that keeps every close pair apart.

    Only pairs whose x/y paths actually come close (at the entry, at the
    target, or somewhere in between) can collide; for each of those the
    required altitude offset follows from the geometry
    (`hypot(xy, dz) >= design separation`).  Amplitudes come from a greedy
    assignment over `_BUMP_LEVELS` (most constrained drone first) followed by
    coordinate-descent refinement.
    """
    n = len(entry)
    if n < 2:
        return np.zeros(n, dtype=float)
    from scipy.spatial import cKDTree

    design = FLEET_TRANSIT_SEP
    radius = float(max(xy_threshold, design))
    frame_xy = entry[:, :2]
    closest: dict[tuple[int, int], tuple[float, float]] = {}
    for sample in range(samples + 1):
        s = sample / float(samples)
        xy = frame_xy + (target[:, :2] - frame_xy) * s
        for i, j in cKDTree(xy).query_pairs(r=radius):
            i, j = int(i), int(j)
            distance = float(np.linalg.norm(xy[i] - xy[j]))
            key = (i, j) if i < j else (j, i)
            known = closest.get(key)
            if known is None or distance < known[0]:
                closest[key] = (distance, s)

    neighbours: list[dict[int, tuple[float, float, float, float]]] = [
        dict() for _ in range(n)]
    for (i, j), (d_min, s_min) in closest.items():
        d_entry = float(np.linalg.norm(entry[i, :2] - entry[j, :2]))
        d_target = float(np.linalg.norm(target[i, :2] - target[j, :2]))
        if d_entry >= design and d_target >= design and d_min >= design:
            continue
        info = (d_entry, d_target, d_min)
        neighbours[i][j] = info
        neighbours[j][i] = info

    z_low = np.minimum(entry[:, 2], target[:, 2])
    z_high = np.maximum(entry[:, 2], target[:, 2])
    up = (FLEET_Z_MAX + 0.03) - z_high
    down = z_low - (FLEET_Z_MIN - 0.10)

    def clamp(index: int, level: float) -> float:
        return max(-float(down[index]), min(float(level), float(up[index])))

    constraints: list[tuple[tuple[int, float, float, float, float, float], ...]] = [
        tuple((other, info[0], info[1], info[2],
               float(entry[index, 2] - entry[other, 2]),
               float(target[index, 2] - target[other, 2]))
              for other, info in neighbours[index].items())
        for index in range(n)
    ]
    candidates = _BUMP_LEVELS

    # Drones that share (nearly) the same entry x/y form a vertical column.
    # A column moves with a single band: that keeps its internal altitude
    # order intact through the lift and turns the problem into one variable
    # per column instead of one per drone.
    parent = list(range(n))

    def find(a: int) -> int:
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for i, j in cKDTree(entry[:, :2]).query_pairs(r=float(FLEET_STACK_RADIUS)):
        ri, rj = find(int(i)), find(int(j))
        if ri != rj:
            parent[max(ri, rj)] = min(ri, rj)
    column_members: dict[int, list[int]] = {}
    for i in range(n):
        column_members.setdefault(find(i), []).append(i)
    columns = sorted(column_members)
    column_of = {index: column for column, members in column_members.items()
                 for index in members}
    column_constraints: dict[int, list[tuple[int, tuple[float, ...]]]] = {}
    for column, members in column_members.items():
        items: list[tuple[int, tuple[float, ...]]] = []
        for index in members:
            for constraint in constraints[index]:
                items.append((index, constraint))
        column_constraints[column] = items

    effective = np.zeros(n, dtype=float)

    def column_level_score(column: int, level: float
                           ) -> tuple[float, ...]:
        violations = 0.0
        excess = 0.0
        shortfall = 0.0
        clamped = 0.0
        for index, (other, d_entry, d_target, d_min, e_delta,
                    t_delta) in column_constraints[column]:
            eff = clamp(index, level)
            if abs(level) > 1e-9 and abs(eff) < abs(level) - 1e-9:
                clamped = 1.0
            margin = _pair_band_margin(eff - effective[other], e_delta,
                                       t_delta, d_entry, d_target, d_min)
            if margin < design - 1e-9:
                violations += 1.0
                shortfall += (design - margin)
            else:
                # Prefer the smallest band that still satisfies the pair:
                # oversized bands push drones into other conflicts.
                excess += margin - design
        return (violations, clamped, shortfall, excess, abs(level))

    def column_level_score_settled(column: int, level: float
                                   ) -> tuple[float, ...]:
        violations = 0.0
        excess = 0.0
        shortfall = 0.0
        clamped = 0.0
        for index, (other, d_entry, d_target, d_min, e_delta,
                    t_delta) in column_constraints[column]:
            if not settled[column_of[other]]:
                continue
            eff = clamp(index, level)
            if abs(level) > 1e-9 and abs(eff) < abs(level) - 1e-9:
                clamped = 1.0
            margin = _pair_band_margin(eff - effective[other], e_delta,
                                       t_delta, d_entry, d_target, d_min)
            if margin < design - 1e-9:
                violations += 1.0
                shortfall += (design - margin)
            else:
                excess += margin - design
        return (violations, clamped, shortfall, excess, abs(level))

    settled = {column: False for column in columns}
    level_of = {column: 0.0 for column in columns}
    order = sorted(columns,
                   key=lambda c: (-len(column_constraints[c]), c))
    for column in order:
        best_level = 0.0
        best_score: tuple[float, ...] | None = None
        for level in candidates:
            candidate_score = column_level_score_settled(column, level)
            if best_score is None or candidate_score < best_score:
                best_score = candidate_score
                best_level = level
        level_of[column] = best_level
        for index in column_members[column]:
            effective[index] = clamp(index, best_level)
        settled[column] = True

    def apply_levels() -> None:
        for column, level in level_of.items():
            for index in column_members[column]:
                effective[index] = clamp(index, level)

    def total_violations() -> tuple[float, list[float]]:
        per_column = {column: 0.0 for column in columns}
        total = 0.0
        for column in columns:
            for index, (other, d_entry, d_target, d_min, e_delta,
                        t_delta) in column_constraints[column]:
                if other < index:
                    continue
                margin = _pair_band_margin(effective[index] - effective[other],
                                           e_delta, t_delta, d_entry,
                                           d_target, d_min)
                if margin < design - 1e-9:
                    total += 1.0
                    per_column[column] += 1.0
        return total, [per_column[column] for column in columns]

    def descend(passes: int, reverse: bool,
                frozen: int | None = None) -> None:
        for sweep in range(passes):
            changed = False
            sequence = list(reversed(order)) if (reverse != (sweep % 2 == 1)) else order
            if sweep == 0 and frozen is not None:
                sequence = [column for column in sequence if column != frozen]
            for column in sequence:
                best_level = level_of[column]
                best_score = column_level_score(column, best_level)
                for level in candidates:
                    candidate_score = column_level_score(column, level)
                    if candidate_score < best_score:
                        best_score = candidate_score
                        best_level = level
                if abs(best_level - level_of[column]) > 1e-12:
                    level_of[column] = best_level
                    for index in column_members[column]:
                        effective[index] = clamp(index, best_level)
                    changed = True
            if not changed:
                break

    # Coordinate descent plus a deterministic basin-hopping kick loop: when
    # the descent stalls with conflicts left, the most conflicted column is
    # forced onto its next candidate band and the descent runs again.  The
    # best assignment seen is what gets returned.
    apply_levels()
    best_levels = dict(level_of)
    best_total, per_column = total_violations()
    descend(8, False)
    apply_levels()
    total, per_column = total_violations()
    if total < best_total:
        best_total, best_levels = total, dict(level_of)
    kicks = 0
    while best_total > 0.0 and kicks < 3:
        kicks += 1
        worst_position = max(range(len(order)),
                             key=lambda position: (per_column[position],
                                                   -position))
        worst_column = order[worst_position]
        current = level_of[worst_column]
        index = candidates.index(current) if current in candidates else 0
        level_of[worst_column] = candidates[(index + kicks) % len(candidates)]
        for member in column_members[worst_column]:
            effective[member] = clamp(member, level_of[worst_column])
        # The first sweep must not undo the kick, otherwise the search can
        # never leave the local optimum it is stuck in.
        descend(4, kicks % 2 == 1, frozen=worst_column)
        apply_levels()
        total, per_column = total_violations()
        if total < best_total:
            best_total = total
            best_levels = dict(level_of)
        elif kicks % 4 == 0:
            level_of = dict(best_levels)
            apply_levels()
    for column, level in best_levels.items():
        for index in column_members[column]:
            effective[index] = clamp(index, level)


    # Pairwise (2-opt) repair: coordinate descent stalls on plateaus, so for
    # every pair that still conflicts the two columns involved are re-leveled
    # jointly, which escapes those plateaus cheaply.
    for round_index in range(2):
        conflicts: list[tuple[int, int]] = []
        for column in columns:
            for index, (other, d_entry, d_target, d_min, e_delta,
                        t_delta) in column_constraints[column]:
                if other < index:
                    continue
                margin = _pair_band_margin(effective[index] - effective[other],
                                           e_delta, t_delta, d_entry,
                                           d_target, d_min)
                if margin < design - 1e-9:
                    conflicts.append((index, other))
                    if len(conflicts) >= 48:
                        break
            if len(conflicts) >= 48:
                break
        if not conflicts:
            break

        def pair_score(first: int, first_level: float, second: int,
                       second_level: float) -> float:
            total = 0.0
            for column, level in ((first, first_level), (second, second_level)):
                for index, (other, d_entry, d_target, d_min, e_delta,
                            t_delta) in column_constraints[column]:
                    other_column = column_of[other]
                    if other_column == first:
                        other_eff = clamp(other, first_level)
                    elif other_column == second:
                        other_eff = clamp(other, second_level)
                    else:
                        other_eff = effective[other]
                    margin = _pair_band_margin(clamp(index, level) - other_eff,
                                               e_delta, t_delta, d_entry,
                                               d_target, d_min)
                    if margin < design - 1e-9:
                        total += 1.0
            return total

        improved = False
        for i, j in conflicts:
            first, second = column_of[i], column_of[j]
            if first == second:
                continue
            current = pair_score(first, level_of[first], second,
                                 level_of[second])
            best_score = current
            best_pair = (level_of[first], level_of[second])
            for first_level in candidates:
                for second_level in candidates:
                    candidate = pair_score(first, first_level, second,
                                           second_level)
                    if candidate < best_score - 1e-9:
                        best_score = candidate
                        best_pair = (first_level, second_level)
            if best_score < current - 1e-9:
                level_of[first], level_of[second] = best_pair
                for member in column_members[first]:
                    effective[member] = clamp(member, best_pair[0])
                for member in column_members[second]:
                    effective[member] = clamp(member, best_pair[1])
                improved = True
        if not improved:
            break
    # Directed repair.  For every pair that still conflicts the band of one of
    # the two columns is solved directly: the pair needs its altitude gap at
    # the end of the lift (`b`) to carry the sign of the target gap (`c`)
    # and to be at least the design separation.  Candidate repairs are scored
    # on the two columns' own constraints, so a repair is only kept when it
    # strictly reduces local conflicts.
    def local_score(first: int, first_level: float, second: int,
                    second_level: float) -> float:
        total = 0.0
        for column, level in ((first, first_level), (second, second_level)):
            for index, (other, d_entry, d_target, d_min, e_delta,
                        t_delta) in column_constraints[column]:
                other_column = column_of[other]
                if other_column == first:
                    other_eff = clamp(other, first_level)
                elif other_column == second:
                    other_eff = clamp(other, second_level)
                else:
                    other_eff = effective[other]
                margin = _pair_band_margin(clamp(index, level) - other_eff,
                                           e_delta, t_delta, d_entry,
                                           d_target, d_min)
                if margin < design - 1e-9:
                    total += 1.0
        return total

    def set_column(column: int, level: float) -> None:
        level_of[column] = level
        for member in column_members[column]:
            effective[member] = clamp(member, level)

    for _ in range(6):
        conflicts: list[tuple[int, int, float, float, float, float]] = []
        for column in columns:
            for index, (other, d_entry, d_target, d_min, e_delta,
                        t_delta) in column_constraints[column]:
                if other < index:
                    continue
                margin = _pair_band_margin(effective[index] - effective[other],
                                           e_delta, t_delta, d_entry,
                                           d_target, d_min)
                if margin < design - 1e-9:
                    conflicts.append((index, other, float(e_delta),
                                      float(t_delta), d_min, d_target))
                    if len(conflicts) >= 64:
                        break
            if len(conflicts) >= 64:
                break
        if not conflicts:
            break
        improved = False
        for i, j, e_delta, t_delta, d_min, d_target in conflicts:
            first, second = column_of[i], column_of[j]
            if first == second:
                continue
            current = local_score(first, level_of[first], second,
                                  level_of[second])
            if current <= 0.0:
                continue
            sign = 1.0 if t_delta >= 0.0 else -1.0
            needed = sign * max(design, abs(t_delta))
            repair_options = (
                (second, float(entry[i, 2]) + level_of[first]
                 - float(entry[j, 2]) - needed),
                (first, float(entry[j, 2]) + level_of[second]
                 + needed - float(entry[i, 2])),
            )
            best_score = current
            best_option: tuple[int, float] | None = None
            for column, level in repair_options:
                other = first if column == second else second
                other_level = level_of[other]
                candidate = local_score(column, level, other, other_level)
                if candidate < best_score - 1e-9:
                    best_score = candidate
                    best_option = (column, level)
            if best_option is not None:
                set_column(best_option[0], best_option[1])
                improved = True
        if not improved:
            break
    for column in order:
        for index in column_members[column]:
            effective[index] = clamp(index, level_of[column])

    if FLEET_DEBUG_STATS is not None:
        pairs = 0
        violations = 0
        worst = 1.0
        for index in range(n):
            for (other, d_entry, d_target, d_min, e_delta,
                 t_delta) in constraints[index]:
                if other < index:
                    continue
                pairs += 1
                margin = _pair_band_margin(effective[index] - effective[other],
                                           e_delta, t_delta, d_entry,
                                           d_target, d_min)
                worst = min(worst, margin)
                if margin < design - 1e-9:
                    violations += 1
        details = []
        if violations and len(FLEET_DEBUG_STATS.get("calls", [])) < 2:
            for index in range(n):
                for (other, d_entry, d_target, d_min, e_delta,
                     t_delta) in constraints[index]:
                    if other < index:
                        continue
                    margin = _pair_band_margin(effective[index] - effective[other],
                                               e_delta, t_delta, d_entry,
                                               d_target, d_min)
                    if margin < design - 1e-9:
                        details.append({
                            "pair": [index, other],
                            "amps": [round(float(effective[index]), 3),
                                     round(float(effective[other]), 3)],
                            "d_entry": round(d_entry, 3),
                            "d_target": round(d_target, 3),
                            "d_min": round(d_min, 3),
                            "e_delta": round(e_delta, 3),
                            "t_delta": round(t_delta, 3),
                            "margin": round(margin, 3),
                        })
                        if len(details) >= 4:
                            break
                if len(details) >= 4:
                    break
        FLEET_DEBUG_STATS.setdefault("calls", []).append(
            {"n": n, "columns": len(columns), "pairs": pairs,
             "violations": violations, "worst": round(worst, 4),
             "details": details})
    return effective


def _min_move_seconds(travel: float) -> float:
    """Shortest smoothstep segment that respects the design speed and accel."""
    travel = abs(float(travel))
    if travel <= 1e-9:
        return 0.0
    return max(FLEET_SPEED_FACTOR * travel / FLEET_TRANSIT_SPEED,
               math.sqrt(6.0 * travel / FLEET_ACCEL_DESIGN))


def _required_transition_seconds(entry: np.ndarray, target: np.ndarray,
                                 amplitudes: np.ndarray
                                 ) -> tuple[float, float, float, float]:
    """(lift, horizontal, settle, total) seconds for a collision-safe move.

    The move runs lift (vertical, by the altitude band) -> horizontal (to the
    target x/y with the band held, so the pair's altitude gap is *constant*
    while the x/y paths cross) -> settle (vertical, onto the target altitude).
    Every phase is a single smoothstep segment, so the peak factors match
    `_min_move_seconds` exactly.
    """
    lifted = entry.copy()
    if len(amplitudes):
        lifted[:, 2] += amplitudes
    horizontal_target = target.copy()
    horizontal_target[:, 2] = lifted[:, 2]
    lift_travel = float(np.max(np.abs(amplitudes))) if len(amplitudes) else 0.0
    horizontal_travel = (float(np.max(np.linalg.norm(
        horizontal_target[:, :2] - lifted[:, :2], axis=1))) if len(target) else 0.0)
    settle_travel = (float(np.max(np.abs(target[:, 2] - lifted[:, 2])))
                     if len(target) else 0.0)
    lift = _min_move_seconds(lift_travel)
    horizontal = _min_move_seconds(horizontal_travel)
    settle = _min_move_seconds(settle_travel)
    return lift, horizontal, settle, lift + horizontal + settle


def _transition_segments(entry: np.ndarray, target: np.ndarray,
                         lift_duration: float, horizontal_duration: float,
                         settle_duration: float, amplitudes: np.ndarray,
                         previous_colors: list[str], base_colors: list[str],
                         scene_color: str) -> list[dict[str, Any]]:
    """Three-phase (lift, horizontal, settle) move with constant bands."""
    segments: list[dict[str, Any]] = []
    lifted = entry.copy()
    if len(amplitudes) and np.any(np.abs(amplitudes) > 1e-12):
        lifted[:, 2] += amplitudes
    horizontal_end = target.copy()
    horizontal_end[:, 2] = lifted[:, 2]
    phases = [
        (lift_duration, entry, lifted, 0.2),
        (horizontal_duration, lifted, horizontal_end, 0.5),
        (settle_duration, horizontal_end, target, 0.8),
    ]
    for duration, start, end, blend in phases:
        if duration <= 1e-9:
            continue
        colors = _blend_color_lists(previous_colors, base_colors, blend)
        segments.append({
            "name": "transition",
            "duration": float(duration),
            "start_positions": start.copy(),
            "end_positions": end.copy(),
            "color": colors[0] if colors else scene_color,
            "colors": colors,
        })
    return segments


def _clamp_effect(scene: dict[str, Any], max_radius: float
                  ) -> tuple[dict[str, Any], float, list[dict[str, Any]]]:
    """Clamp effect parameters that would otherwise leave validator bounds."""
    effect = scene.get("effect")
    if not isinstance(effect, dict):
        return {"type": "none"}, 0.0, []
    eff = dict(effect)
    warnings: list[dict[str, Any]] = []
    wave = 0.0
    kind = str(eff.get("type", "none")).lower()
    if kind == "scale" and max_radius > 1e-9:
        limit = FLEET_XY_RADIUS / max(max_radius, 1e-9)
        try:
            to = float(eff.get("to", 1.3))
        except (TypeError, ValueError):
            to = 1.3
        safe_to = max(0.01, min(to, limit))
        if safe_to < to - 1e-9:
            warnings.append(_warn(
                "W_SCALE_CLAMPED",
                "scale target reduced from " + str(round(to, 3)) + " to "
                + str(round(safe_to, 3)) + " to stay inside xy bounds",
                requested=round(to, 3), used=round(safe_to, 3)))
            eff["to"] = safe_to
        try:
            frm = float(eff.get("from", 1.0))
        except (TypeError, ValueError):
            frm = 1.0
        if frm > limit:
            eff["from"] = max(0.01, limit)
    if kind == "wave":
        try:
            wave = abs(float(eff.get("amplitude", 0.3)))
        except (TypeError, ValueError):
            wave = 0.3
    return eff, wave, warnings


def _effect_end_positions(base: np.ndarray, active_mask: np.ndarray,
                          effect: dict[str, Any]) -> np.ndarray:
    """Analytic end frame of an effect animation (used for time budgeting)."""
    frame = base.copy()
    active = np.where(np.asarray(active_mask, dtype=bool))[0]
    if not len(active):
        return frame
    active_base = base[active].copy()
    kind = str(effect.get("type", "none")).lower()
    if kind == "rotate":
        try:
            turns = float(effect.get("turns", 1.0))
        except (TypeError, ValueError):
            turns = 1.0
        centre = active_base[:, :2].mean(axis=0)
        frame[active] = _rotate_xy(active_base, 2.0 * math.pi * turns, centre)
    elif kind == "scale":
        try:
            to = float(effect.get("to", 1.3))
        except (TypeError, ValueError):
            to = 1.3
        centre = active_base[:, :2].mean(axis=0)
        frame[active] = _scale_xy(active_base, to, centre)
    elif kind == "wave":
        try:
            amplitude = float(effect.get("amplitude", 0.3))
        except (TypeError, ValueError):
            amplitude = 0.3
        try:
            cycles = float(effect.get("cycles", 2.0))
        except (TypeError, ValueError):
            cycles = 2.0
        frame[active, 2] = (active_base[:, 2]
                            + amplitude * math.sin(2.0 * math.pi * cycles))
    return frame


# ---------------------------------------------------------------------------
# per-scene assembly
# ---------------------------------------------------------------------------

def _scene_common_checks(scene: dict[str, Any],
                         index: int) -> tuple[float, float]:
    if not isinstance(scene, dict):
        raise FleetError("E_BAD_SCENE",
                         "scene " + str(index) + " must be an object")
    try:
        scene_duration = float(scene.get("duration_sec", 0.0))
    except (TypeError, ValueError) as exc:
        raise FleetError("E_BAD_SCENE", "scene " + str(index)
                         + " duration_sec must be a number") from exc
    if scene_duration <= 0.0:
        raise FleetError("E_BAD_SCENE", "scene " + str(index)
                         + " duration_sec must be positive")
    try:
        transition = float(scene.get("transition_seconds",
                                     FLEET_DEFAULT_TRANSITION))
    except (TypeError, ValueError) as exc:
        raise FleetError("E_BAD_SCENE", "scene " + str(index)
                         + " transition_seconds must be a number") from exc
    if transition < 0.0:
        raise FleetError("E_BAD_SCENE", "scene " + str(index)
                         + " transition_seconds must be non-negative")
    return scene_duration, min(transition, scene_duration)


def _resolve_budget(scene: dict[str, Any]) -> int | None:
    raw = scene.get("drone_budget", "auto")
    if raw is None or raw == "auto":
        return None
    try:
        budget = int(raw)
    except (TypeError, ValueError) as exc:
        raise FleetError("E_BAD_SCENE",
                         "drone_budget must be 'auto' or a positive integer") from exc
    if budget < 1:
        raise FleetError("E_BAD_SCENE", "drone_budget must be positive")
    return budget


def _build_scene_points(scene: dict[str, Any], index: int, budget: int | None,
                        total: int, font_path: str | None,
                        radius_cap: float | None
                        ) -> tuple[np.ndarray, dict[str, Any], list[dict[str, Any]]]:
    scene_type = str(scene.get("type", "")).lower()
    if scene_type == "text":
        if font_path is None:
            try:
                font_path = resolve_font_path(None)
            except CJKError as exc:
                raise FleetError(exc.code, exc.detail, **exc.extra) from exc
        return _text_scene(scene, index, budget, font_path, total)
    if scene_type in ("grid", "ring", "heart", "star"):
        return _shape_scene(scene, budget, total, radius_cap)
    raise FleetError("E_BAD_SCENE", "unsupported scene type " + repr(scene_type))


def _shift_wave_band(points: np.ndarray, wave: float,
                     warnings: list[dict[str, Any]],
                     index: int) -> np.ndarray:
    """Shift a layered formation so a rigid z-wave stays inside the z band."""
    if wave <= 1e-9 or len(points) == 0:
        return points
    low = float(np.min(points[:, 2]))
    high = float(np.max(points[:, 2]))
    span = high - low
    window = (FLEET_Z_MAX - 0.02 - wave) - (FLEET_Z_MIN + wave - 0.02)
    if span > window + 1e-9:
        raise FleetError(
            "E_BAD_SCENE",
            "scene " + str(index) + ": formation spans " + str(round(span, 3))
            + " m of altitude but a wave of amplitude " + str(round(wave, 3))
            + " m leaves only " + str(round(window, 3)) + " m",
            wave_amplitude=wave)
    target_low = max(FLEET_Z_MIN + wave - 0.02, min(low, FLEET_Z_MAX - 0.02 - wave - span))
    if abs(target_low - low) > 1e-9:
        warnings.append(_warn(
            "W_WAVE_BAND_SHIFT",
            "scene " + str(index) + ": formation shifted "
            + str(round(target_low - low, 3)) + " m in z to keep the wave "
            "inside the altitude band"))
        points = points.copy()
        points[:, 2] += (target_low - low)
    return points


def _prepare_fleet_scenes(scenes: list[dict[str, Any]], total: int,
                          start: np.ndarray, font_path: str | None,
                          attempt: int
                          ) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Squared 3-D matching plus continuously certified synchronous moves.

    Endpoint design spacing .38 > sqrt(2)*.25 gives a separation margin for
    an optimal squared-distance assignment. Every actual segment is checked
    analytically as well, including starts supplied by the physical runtime.
    Requested scene budgets, radii and effects are never silently reduced.
    """
    infos, warnings = [], []
    entry = start.copy()
    previous_colors = [DEFAULT_COLOR] * total
    if any(str(s.get("type", "")).lower() == "text" for s in scenes):
        font_path = resolve_font_path(font_path)

    def segment(a, b, seconds, name, colors):
        spacing = continuous_separation(a, b)
        if spacing < .25 - 1e-9:
            raise FleetError("E_CONTINUOUS_SEPARATION",
                             f"{name}: continuous separation {spacing:.6f} m < 0.25 m",
                             min_distance=spacing)
        return {"name": name, "duration": seconds,
                "start_positions": a.copy(), "end_positions": b.copy(),
                "colors": list(colors), "color": colors[0]}

    for index, scene in enumerate(scenes):
        scene_duration, base_transition = _scene_common_checks(scene, index)
        budget = _resolve_budget(scene)
        points, meta, extra = _build_scene_points(scene, index, budget, total,
                                                 font_path, None)
        warnings.extend(extra)
        if budget is not None and len(points) != budget:
            raise FleetError("E_POINT_BUDGET", "requested scene budget cannot fit",
                             requested=budget, available=len(points))
        effect, wave, extra = _clamp_effect(scene, float(meta.get("radius", 0)))
        warnings.extend(extra)
        points = _shift_wave_band(points, wave, warnings, index)
        active = len(points)
        if meta.get("upright"):
            from .cinematic import _backdrop, _volume
            parking = _backdrop(points, total, _volume(), True)[active:]
            parking_meta = {"kind": "background_layers", "layers": 6,
                            "min_distance": min_pairwise_distance_fast(parking)}
        else:
            parking, parking_meta = _parking_slots(total-active,
                             float(points[:, 2].min()), float(points[:, 2].max()), wave)
        targets = np.vstack([points, parking]) if len(parking) else points
        matched, cols = match_targets(entry, targets)
        mask = cols < active
        scene_color, palette = _resolve_scene_colors(scene, total)
        palette = [palette[i] if mask[i] else "#26334b" for i in range(total)]
        transition = move_seconds(entry, matched, base_transition)
        end = _effect_end_positions(matched, mask, effect)
        land = bool(scene.get("include_landing", False)) and index == len(scenes)-1
        landing_seconds, landed, ground_meta = 0., None, None
        if land:
            ground, ground_meta = _ground_slots(total)
            landed, _ = match_targets(end, ground)
            landing_seconds = move_seconds(end, landed, 3.)
        anim = scene_duration-transition-landing_seconds
        if anim < 0:
            raise FleetError("E_DURATION_TOO_SHORT", "scene has insufficient time for safe movement",
                             scene_index=index, required_seconds=transition+landing_seconds+2)
        effects = _effect_segments(matched, palette, scene_color, scene.get("gradient"),
                                   effect, anim, "scene"+str(index)+":"+str(scene.get("text", scene.get("type", ""))), active_mask=mask)
        # The effect's exact chords also obey conservative speed/accel limits.
        required_effect = sum(move_seconds(s["start_positions"], s["end_positions"], 0.)
                              for s in effects)
        if required_effect > anim+1e-6:
            raise FleetError("E_DURATION_TOO_SHORT", "scene has insufficient time for the requested effect",
                             scene_index=index,
                             required_seconds=transition+required_effect+landing_seconds+1)
        for s in effects:
            segment(s["start_positions"], s["end_positions"], s["duration"], s["name"], s["colors"])
        transitions = [segment(entry, matched, transition, "assemble:"+str(scene.get("text", scene.get("type", ""))), palette)]
        land_segments = []
        if land:
            land_segments = [segment(end, landed, landing_seconds, "landing", palette)]
            if ground_meta["terraces"] > 1:
                warnings.append(_warn("W_LANDING_TERRACED",
                        "Bounded kinematic stage uses virtual landing terraces",
                        terraces=ground_meta["terraces"]))
        summary = {"scene_index": index, "type": meta.get("type", scene.get("type")),
                   "drone_budget": scene.get("drone_budget", "auto"),
                   "active": active, "idle": total-active,
                   "points_per_char": meta.get("points_per_char", 0),
                   "min_distance": meta.get("min_distance"), "layers": meta.get("layers", 1),
                   "z_levels": meta.get("z_levels", []),
                   "formation_radius": meta.get("radius", 0),
                   "transition_seconds": round(transition, 6), "effect": effect,
                   "landing": land, "formation_id": "f_"+short_hash(canonical_json(points.tolist())),
                   "parking": parking_meta}
        infos.append({"base_colors": palette, "summary": summary,
                      "transition_segments": transitions, "effect_segments": effects,
                      "landing_segments": land_segments})
        entry = landed.copy() if land else end.copy()
        previous_colors = list(palette)
    return infos, warnings


def _raw_fleet_segments(infos: list[dict[str, Any]]) -> list[dict[str, Any]]:
    raw: list[dict[str, Any]] = []
    for info in infos:
        raw.extend(info["transition_segments"])
        raw.extend(info["effect_segments"])
        raw.extend(info["landing_segments"])
    return raw


def build_fleet_timeline_plan(state_positions: Any, total_drones: int,
                              duration_sec: float,
                              scenes: list[dict[str, Any]],
                              motion_mode: str,
                              source_state_revision: int, secret: bytes,
                              font_path: str | None = None,
                              plan_id: str | None = None) -> dict[str, Any]:
    """Build and validate a whole-fleet timeline plan."""
    total = int(total_drones)
    start = np.asarray(state_positions, dtype=float)
    if start.shape != (total, 3):
        raise FleetError("E_BAD_STATE",
                         "state positions must be " + str(total) + "x3")
    if not isinstance(scenes, list) or len(scenes) == 0:
        raise FleetError("E_BAD_SCENE", "scenes must be a non-empty array")
    try:
        scene_total = float(sum(float(scene.get("duration_sec", 0.0))
                                for scene in scenes
                                if isinstance(scene, dict)))
    except (TypeError, ValueError) as exc:
        raise FleetError("E_BAD_SCENE",
                         "scene duration_sec values must be numbers") from exc
    if abs(scene_total - float(duration_sec)) > 1e-6:
        raise FleetError("E_DURATION_MISMATCH",
                         "scene duration sum does not match duration_sec",
                         duration_sec=float(duration_sec),
                         scene_total_sec=scene_total)
    if total > FLEET_MAX_TOTAL:
        raise FleetError(
            "E_FLEET_CAPACITY",
            "fleet of " + str(total) + " drones cannot be placed within "
            "validator bounds; geometric capacity ~" + str(FLEET_MAX_TOTAL),
            total_drones=total, fleet_capacity=FLEET_MAX_TOTAL)
    if plan_id is None:
        plan_id = "p_" + secrets.token_hex(6)

    font: str | None = None
    if font_path is not None:
        try:
            font = resolve_font_path(font_path)
        except CJKError as exc:
            raise FleetError(exc.code, exc.detail, **exc.extra) from exc

    best_plan: dict[str, Any] | None = None
    best_valid = False
    for attempt in range(len(_TRANSIT_ATTEMPTS)):
        infos, warnings = _prepare_fleet_scenes(scenes, total, start, font,
                                                attempt)
        first_colors = infos[0]["base_colors"]
        meta: dict[str, Any] = {
            "scenes": [info["summary"] for info in infos],
            "warnings": list(warnings),
            "duration_sec": float(duration_sec),
            "num_drones": total,
            "color": first_colors[0] if first_colors else DEFAULT_COLOR,
            "colors": first_colors,
            "target_formation_id": infos[-1]["summary"]["formation_id"],
            "repaired": attempt > 0,
            "repair": None if attempt == 0 else {
                "reason": "E_MIN_SEPARATION",
                "via_type": "fleet-transit-tighten",
                "attempt": attempt,
            },
            "motion_mode": str(motion_mode),
            "kinematic_preview": str(motion_mode) == "kinematic",
            "fleet": True,
        }
        raw = _raw_fleet_segments(infos)
        plan = _finalize_plan(plan_id, total, start, raw,
                              int(source_state_revision), secret, meta)
        validation = validate_plan_chunked(plan)
        meta["validation"] = validation
        plan["_timeline"] = meta
        if best_plan is None or (validation["valid"] and not best_valid):
            best_plan, best_valid = plan, bool(validation["valid"])
        if validation["valid"]:
            return plan
        issues = validation.get("issues") or []
        if not any(str(issue).startswith("E_MIN_SEPARATION") for issue in issues):
            break
    assert best_plan is not None
    return best_plan
