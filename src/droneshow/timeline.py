"""Deterministic multi-scene 60-second timeline animation planner."""
from __future__ import annotations

import hashlib
import hmac
import math
import secrets
from typing import Any

import numpy as np

from .cjk import CJKError, build_page_points, render_glyph_coords, resolve_font_path
from .colors import DEFAULT_COLOR, ColorError, lerp_hex, parse_color_spec
from .formations import build_formation, canonical_json, short_hash
from .planner import LANDING_Z, PlanError, _assign_targets, _segment, _vertical_duration
from .repair import _via_candidates
from .textshow import analyze_text, paginate_text
from .validator import validate_plan


class TimelineError(ValueError):
    def __init__(self, code: str, detail: str, **extra: Any):
        super().__init__(detail)
        self.code = code
        self.detail = detail
        self.extra = extra


def _resolve_scene_colors(scene: dict[str, Any], num_drones: int) -> tuple[str, list[str]]:
    try:
        spec = parse_color_spec(scene.get("color"), scene.get("colors"),
                                scene.get("gradient"), num_drones,
                                default=DEFAULT_COLOR)
    except ColorError as exc:
        raise TimelineError(exc.code, exc.detail, **exc.extra) from exc
    color = spec["color"]
    colors = spec["colors"]
    if colors is None and color is not None:
        colors = [color] * num_drones
    return color, colors


def _shape_spec(scene: dict[str, Any], num_drones: int) -> dict[str, Any]:
    scene_type = str(scene.get("type", "")).lower()
    spec: dict[str, Any] = {"type": scene_type, "z": float(scene.get("z", 1.4))}
    if scene_type == "grid":
        spec.update({
            "rows": int(scene.get("rows", max(1, int(math.ceil(math.sqrt(num_drones)))))),
            "cols": int(scene.get("cols", max(1, int(math.ceil(num_drones / max(1, int(math.ceil(math.sqrt(num_drones))))))))),
            "spacing": float(scene.get("spacing", 0.6)),
        })
    elif scene_type == "ring":
        if "radius" in scene:
            spec["radius"] = float(scene["radius"])
        spec["phase"] = float(scene.get("phase", 0.0))
    elif scene_type == "heart":
        if "size" in scene:
            spec["size"] = float(scene["size"])
        spec["phase"] = float(scene.get("phase", 0.0))
    elif scene_type == "star":
        if "radius" in scene:
            spec["radius"] = float(scene["radius"])
        spec["inner_ratio"] = float(scene.get("inner_ratio", 0.45))
        spec["phase"] = float(scene.get("phase", 0.0))
    else:
        raise TimelineError("E_BAD_SCENE", "unsupported scene type " + repr(scene_type))
    return spec


def _cjk_scene(scene: dict[str, Any], num_drones: int, font_path: str,
               z: float) -> dict[str, Any]:
    text = str(scene.get("text", "")).strip()
    if not text:
        raise TimelineError("E_BAD_SCENE", "text scene requires a non-empty text")
    analysis = analyze_text(text)
    complexities = []
    for char in analysis["characters"]:
        coords, _ = render_glyph_coords(char, font_path)
        complexities.append(int(len(coords)))
    min_points = max(1, int(scene.get("min_points_per_char", 16)))
    points_per_char = max(1, int(scene.get("points_per_char", 32)))
    glyph_count = len(analysis["characters"])
    budget = min(int(num_drones), glyph_count * points_per_char)
    if budget < glyph_count * min_points:
        raise TimelineError("E_POINT_BUDGET",
                            "scene text needs at least "
                            + str(glyph_count * min_points) + " points",
                            required_points=glyph_count * min_points,
                            available_points=num_drones)
    pagination = paginate_text(analysis["characters"], budget,
                               auto_allocate=True,
                               min_points_per_char=min_points,
                               complexities=complexities)
    if len(pagination["pages"]) > 1:
        required = int(pagination["requested_total_points"])
        raise TimelineError("E_POINT_BUDGET",
                            "scene text needs multiple pages; increase num_drones",
                            required_points=required,
                            available_points=num_drones)
    page = pagination["pages"][0]
    points_2d, meta = build_page_points(
        list(page["characters"]), list(page["allocations"]), font_path,
        base_height=float(scene.get("base_height", 2.4)),
        letter_spacing=float(scene.get("letter_spacing", 0.16)),
        min_separation=0.30)
    points = np.column_stack([points_2d, np.full(len(points_2d), float(z))])
    formation_id = "f_" + short_hash(canonical_json({
        "type": "cjk_text",
        "text": "".join(page["characters"]),
        "points": np.round(points, 9).tolist(),
    }))
    return {
        "points": points,
        "formation_id": formation_id,
        "used": int(len(points)),
        "idle": int(num_drones - len(points)),
        "meta": meta,
        "scene_type": "cjk_text",
    }


def _scene_target(scene: dict[str, Any], num_drones: int,
                  font_path: str) -> dict[str, Any]:
    scene_type = str(scene.get("type", "")).lower()
    z = float(scene.get("z", 1.4))
    if scene_type == "text":
        text = str(scene.get("text", "")).strip()
        if not text:
            raise TimelineError("E_BAD_SCENE", "text scene requires a non-empty text")
        if any(ord(char) > 127 for char in text):
            return _cjk_scene(scene, num_drones, font_path, z)
        try:
            formation = build_formation(num_drones, {
                "type": "text", "text": text, "z": z,
                "min_points_per_char": int(scene.get("min_points_per_char", 8)),
                "allow_sparse": bool(scene.get("allow_sparse", False)),
            })
        except Exception as exc:
            raise TimelineError("E_BAD_SCENE", str(exc)) from exc
        points = np.asarray(formation["points"], dtype=float)
        return {
            "points": points,
            "formation_id": formation["formation_id"],
            "used": int(formation["resource_report"]["used"]),
            "idle": int(formation["resource_report"]["idle"]),
            "meta": formation,
            "scene_type": "text",
        }
    try:
        formation = build_formation(num_drones, _shape_spec(scene, num_drones))
    except Exception as exc:
        raise TimelineError("E_BAD_SCENE", str(exc)) from exc
    points = np.asarray(formation["points"], dtype=float)
    return {
        "points": points,
        "formation_id": formation["formation_id"],
        "used": int(formation["resource_report"]["used"]),
        "idle": int(formation["resource_report"]["idle"]),
        "meta": formation,
        "scene_type": scene_type,
    }


def _rotate_xy(points: np.ndarray, angle: float, centre: VectorLike) -> np.ndarray:
    out = points.copy()
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    x = points[:, 0] - centre[0]
    y = points[:, 1] - centre[1]
    out[:, 0] = centre[0] + x * cos_a - y * sin_a
    out[:, 1] = centre[1] + x * sin_a + y * cos_a
    return out


VectorLike = np.ndarray | tuple[float, float]


def _scale_xy(points: np.ndarray, scale: float, centre: VectorLike) -> np.ndarray:
    out = points.copy()
    out[:, 0] = centre[0] + (points[:, 0] - centre[0]) * scale
    out[:, 1] = centre[1] + (points[:, 1] - centre[1]) * scale
    return out


def _blend_color_lists(first: list[str], second: list[str], t: float) -> list[str]:
    return [lerp_hex(a, b, t) for a, b in zip(first, second)]


def _safe_scale_steps(radius: float, start_scale: float, end_scale: float,
                      duration: float) -> int:
    if duration <= 1e-9 or radius <= 1e-9 or abs(end_scale - start_scale) < 1e-9:
        return 1
    best = 1
    for steps in range(2, 17):
        delta = radius * abs(end_scale - start_scale) / steps
        dt = duration / steps
        speed = 1.5 * delta / dt
        accel = 6.0 * delta / (dt * dt)
        if speed <= 1.90 and accel <= 7.50:
            best = steps
    return best


def _safe_wave_steps(amplitude: float, cycles: float, duration: float) -> int:
    if duration <= 1e-9 or abs(amplitude) < 1e-9 or abs(cycles) < 1e-9:
        return 1
    best = 1
    for steps in range(2, 33):
        dt = duration / steps
        max_speed = 0.0
        max_accel = 0.0
        for index in range(steps):
            first = amplitude * math.sin(2.0 * math.pi * cycles * index / steps)
            second = amplitude * math.sin(2.0 * math.pi * cycles * (index + 1) / steps)
            chord = abs(second - first)
            max_speed = max(max_speed, 1.5 * chord / dt)
            max_accel = max(max_accel, 6.0 * chord / (dt * dt))
        if max_speed <= 1.90 and max_accel <= 7.50:
            best = steps
    return best


def _effect_segments(base: np.ndarray, base_colors: list[str],
                     scene_color: str, scene_gradient: Any,
                     effect: dict[str, Any], duration: float,
                     name_prefix: str,
                     active_mask: np.ndarray | None = None) -> list[dict[str, Any]]:
    if duration <= 0.0:
        return []
    if active_mask is None:
        active_indices = np.arange(len(base), dtype=int)
    else:
        active_indices = np.where(np.asarray(active_mask, dtype=bool))[0]
    active_base = base[active_indices].copy() if len(active_indices) else base[:0].copy()
    effect_type = str(effect.get("type", "none")).lower()
    positions: list[np.ndarray]
    if effect_type == "none":
        positions = [base.copy(), base.copy()]
    elif effect_type == "rotate":
        turns = float(effect.get("turns", 1.0))
        centre = active_base[:, :2].mean(axis=0) if len(active_base) else np.zeros(2)
        max_radius = (float(np.max(np.linalg.norm(active_base[:, :2] - centre, axis=1)))
                      if len(active_base) else 0.0)
        steps = _safe_rotation_steps(max_radius, turns, duration)
        positions = [base.copy()]
        for index in range(1, steps + 1):
            frame = base.copy()
            if len(active_indices):
                frame[active_indices] = _rotate_xy(
                    active_base, 2.0 * math.pi * turns * index / steps, centre)
            positions.append(frame)
    elif effect_type == "scale":
        start_scale = float(effect.get("from", 1.0))
        end_scale = float(effect.get("to", 1.3))
        centre = active_base[:, :2].mean(axis=0) if len(active_base) else np.zeros(2)
        radius = (float(np.max(np.linalg.norm(active_base[:, :2] - centre, axis=1)))
                  if len(active_base) else 0.0)
        steps = _safe_scale_steps(radius, start_scale, end_scale, duration)
        positions = [base.copy()]
        for index in range(1, steps + 1):
            frame = base.copy()
            if len(active_indices):
                scale = start_scale + (end_scale - start_scale) * index / steps
                frame[active_indices] = _scale_xy(active_base, scale, centre)
            positions.append(frame)
    elif effect_type == "wave":
        amplitude = float(effect.get("amplitude", 0.3))
        cycles = float(effect.get("cycles", 2.0))
        steps = _safe_wave_steps(amplitude, cycles, duration)
        positions = [base.copy()]
        for index in range(1, steps + 1):
            frame = base.copy()
            if len(active_indices):
                phase = 2.0 * math.pi * cycles * index / steps
                frame[active_indices, 2] = active_base[:, 2] + amplitude * math.sin(phase)
            positions.append(frame)
    else:
        raise TimelineError("E_BAD_EFFECT", "unsupported effect type " + repr(effect_type))

    steps = len(positions) - 1
    end_color = None
    if isinstance(scene_gradient, (list, tuple)) and len(scene_gradient) == 2:
        end_color = str(scene_gradient[1])
    end_colors = [color if end_color is None or color == "#26334b"
                  else lerp_hex(color, end_color, 0.55) for color in base_colors]
    base_duration = float(duration) / float(steps)
    segments = []
    for index in range(steps):
        segment_duration = base_duration if index < steps - 1 else float(duration) - base_duration * (steps - 1)
        u = (index + 1) / steps
        colors = _blend_color_lists(base_colors, end_colors, u)
        segments.append({
            "name": name_prefix + "_" + effect_type,
            "duration": max(0.0, segment_duration),
            "start_positions": positions[index].copy(),
            "end_positions": positions[index + 1].copy(),
            "color": colors[0] if colors else scene_color,
            "colors": colors,
        })
    return segments


def _smoothstep_scalar(u: float) -> float:
    u = max(0.0, min(1.0, float(u)))
    return u * u * (3.0 - 2.0 * u)


def _transition_conflict_amplitudes(route: list[np.ndarray],
                                    threshold: float = 0.0) -> np.ndarray:
    n = len(route[0]) if route else 0
    amplitudes = np.zeros(n, dtype=float)
    if n <= 1 or len(route) < 2:
        return amplitudes
    hop_count = len(route) - 1
    frames: list[np.ndarray] = []
    for hop in range(hop_count):
        start = np.asarray(route[hop], dtype=float)
        end = np.asarray(route[hop + 1], dtype=float)
        for sample in range(7):
            u = sample / 6.0
            frames.append(start + (end - start) * _smoothstep_scalar(u))
    adjacency = [set() for _ in range(n)]
    for frame in frames:
        for i in range(n - 1):
            distances = np.linalg.norm(frame[i + 1:] - frame[i], axis=1)
            for offset in np.where(distances < threshold)[0]:
                j = i + 1 + int(offset)
                adjacency[i].add(j)
                adjacency[j].add(i)
    order = sorted(range(n), key=lambda index: -len(adjacency[index]))
    colors = [-1] * n
    for index in order:
        if not adjacency[index]:
            colors[index] = 0
            continue
        used = {colors[other] for other in adjacency[index] if colors[other] >= 0}
        color = 0
        while color in used:
            color += 1
        colors[index] = min(color, 2)
    levels = [-0.15, 0.0, 0.15]
    z_min = np.min(np.stack(frames, axis=0), axis=0)[:, 2]
    z_max = np.max(np.stack(frames, axis=0), axis=0)[:, 2]
    for index in range(n):
        if not adjacency[index]:
            continue
        amplitude = levels[colors[index]]
        upper = 2.90 - float(z_max[index])
        lower = float(z_min[index]) - 0.12
        amplitude = max(-lower, min(amplitude, upper))
        amplitudes[index] = amplitude
    return amplitudes


def _bumped_transition_segments(route: list[np.ndarray], duration: float,
                                previous_colors: list[str],
                                base_colors: list[str],
                                scene_color: str,
                                substeps: int = 4) -> list[dict[str, Any]]:
    if duration <= 0.0 or len(route) < 2:
        return []
    n = len(route[0])
    amplitudes = _transition_conflict_amplitudes(route)
    hop_count = len(route) - 1
    base_hop = float(duration) / hop_count
    segments: list[dict[str, Any]] = []
    for hop in range(hop_count):
        start = np.asarray(route[hop], dtype=float)
        end = np.asarray(route[hop + 1], dtype=float)
        hop_duration = base_hop if hop < hop_count - 1 else float(duration) - base_hop * (hop_count - 1)
        substep_duration = max(1e-6, hop_duration / substeps)
        for step in range(substeps):
            u0 = step / substeps
            u1 = (step + 1) / substeps
            first = start + (end - start) * _smoothstep_scalar(u0)
            second = start + (end - start) * _smoothstep_scalar(u1)
            if np.any(amplitudes):
                first = first.copy()
                second = second.copy()
                first[:, 2] += amplitudes * math.sin(math.pi * u0)
                second[:, 2] += amplitudes * math.sin(math.pi * u1)
            global_mid = (hop + (step + 0.5) / substeps) / hop_count
            colors = _blend_color_lists(previous_colors, base_colors, global_mid)
            segments.append({
                "name": "transition",
                "duration": substep_duration if step < substeps - 1 else max(
                    1e-6, hop_duration - substep_duration * (substeps - 1)),
                "start_positions": first,
                "end_positions": second,
                "color": colors[0] if colors else scene_color,
                "colors": colors,
            })
    return segments


def _transition_segment(start: np.ndarray, end: np.ndarray,
                        previous_colors: list[str], base_colors: list[str],
                        duration: float, name: str) -> dict[str, Any]:
    colors = _blend_color_lists(previous_colors, base_colors, 0.5)
    return {
        "name": name,
        "duration": float(duration),
        "start_positions": start.copy(),
        "end_positions": end.copy(),
        "color": colors[0] if colors else DEFAULT_COLOR,
        "colors": colors,
    }


def _finalize_plan(plan_id: str, num_drones: int, start: np.ndarray,
                   raw_segments: list[dict[str, Any]],
                   source_state_revision: int, secret: bytes,
                   meta: dict[str, Any]) -> dict[str, Any]:
    segments: list[dict[str, Any]] = []
    cursor = 0.0
    for index, raw in enumerate(raw_segments):
        segment = {
            "id": "seg_" + plan_id + "_" + str(index),
            "name": raw["name"],
            "duration": float(raw["duration"]),
            "start_positions": np.asarray(raw["start_positions"], dtype=float).copy(),
            "end_positions": np.asarray(raw["end_positions"], dtype=float).copy(),
            "color": raw.get("color"),
            "colors": None if raw.get("colors") is None else list(raw.get("colors")),
            "start_time": float(cursor),
        }
        cursor += segment["duration"]
        segment["end_time"] = float(cursor)
        segments.append(segment)
    total_duration = float(cursor)
    final_point = segments[-1]["end_positions"] if segments else start
    plan_hash = hashlib.sha256(canonical_json({
        "plan_id": plan_id,
        "source_state_revision": int(source_state_revision),
        "num_drones": int(num_drones),
        "start_positions": np.round(start, 6).tolist(),
        "scene_count": len(meta.get("scenes", [])),
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
    plan_color = meta.get("color") or DEFAULT_COLOR
    plan_colors = meta.get("colors") or [plan_color] * num_drones
    return {
        "plan_id": plan_id,
        "plan_hash": plan_hash,
        "credential": credential,
        "source_state_revision": int(source_state_revision),
        "target_formation_id": meta.get("target_formation_id"),
        "duration": total_duration,
        "color": plan_color,
        "colors": plan_colors,
        "segments": [
            {"id": segment["id"], "name": segment["name"],
             "duration": round(segment["duration"], 6),
             "color": segment.get("color"), "colors": segment.get("colors")}
            for segment in segments
        ],
        "repaired": bool(meta.get("repaired", False)),
        "repair": meta.get("repair"),
        "_num_drones": int(num_drones),
        "_start_positions": start,
        "_target_positions": final_point,
        "_segments": segments,
        "_total_duration": total_duration,
        "_timeline": meta,
    }


def _fps_order(points: np.ndarray) -> list[int]:
    values = np.asarray(points, dtype=float)
    count = len(values)
    if count <= 1:
        return list(range(count))
    centre = values.mean(axis=0)
    first = int(np.argmin(np.linalg.norm(values - centre, axis=1)))
    order = [first]
    best = np.linalg.norm(values - values[first], axis=1)
    while len(order) < count:
        index = int(np.argmax(best))
        order.append(index)
        best = np.minimum(best, np.linalg.norm(values - values[index], axis=1))
    return order


def _limit_scene_points(points: np.ndarray, min_sep: float = 0.30,
                       max_radius: float = 4.80) -> tuple[np.ndarray, int, str | None]:
    values = np.asarray(points, dtype=float)
    if len(values) <= 1:
        return values, len(values), None
    order = _fps_order(values)
    selected: list[np.ndarray] = []
    for index in order:
        candidate = values[index]
        if selected:
            nearest = min(float(np.linalg.norm(candidate - other)) for other in selected)
            if nearest < float(min_sep):
                break
        selected.append(candidate)
    if not selected:
        selected = [values[order[0]]]
    result = np.asarray(selected, dtype=float)
    radius = float(np.max(np.linalg.norm(result[:, :2], axis=1))) if len(result) else 0.0
    warning = None
    if len(result) < len(values):
        warning = ("scene target reduced to " + str(len(result))
                   + " active points for validator separation safety")
    if radius > float(max_radius):
        raise TimelineError("E_XY_BOUNDS",
                            "scene target radius exceeds validator bounds",
                            max_radius=radius, limit=max_radius)
    return result, len(result), warning


def _limit_rotate_target(points: np.ndarray, effect: dict[str, Any],
                         duration: float) -> tuple[np.ndarray, int, str | None]:
    turns = abs(float(effect.get("turns", 1.0)))
    if duration <= 1e-9 or turns <= 1e-12 or len(points) < 2:
        return points, len(points), None
    steps = max(8, int(math.ceil(turns * 16)))
    chord_factor = 2 * abs(math.sin(math.pi * turns / steps))
    radius_speed_limit = min(1.9 * duration / (1.5 * steps * chord_factor),
                             7.5 * (duration / steps) ** 2 / (6 * chord_factor))
    xy = points[:, :2]
    centre = xy.mean(axis=0)
    radial = np.linalg.norm(xy - centre, axis=1)
    max_radius = float(np.max(radial)) if len(radial) else 0.0
    scale = 1.0 if max_radius <= 1e-12 else min(1.0, radius_speed_limit / max_radius)
    scaled = points.copy()
    scaled[:, :2] = centre + (xy - centre) * scale
    order = _fps_order(scaled)
    selected: list[np.ndarray] = []
    for index in order:
        candidate = scaled[index]
        if selected:
            nearest = min(float(np.linalg.norm(candidate - other)) for other in selected)
            if nearest < 0.30:
                break
        selected.append(candidate)
    if not selected:
        selected = [scaled[order[0]]]
    warning = None
    if scale < 1.0:
        warning = "rotate radius scaled for validator speed/acceleration safety"
    if len(selected) < len(points):
        warning = ("rotate scene reduced to " + str(len(selected))
                   + " active points for validator speed/separation safety")
    return np.asarray(selected, dtype=float), len(selected), warning


def _safe_idle_points(active_points: np.ndarray, count: int, z: float,
                     min_sep: float = 0.30, max_radius: float = 4.8) -> np.ndarray:
    if count <= 0:
        return np.zeros((0, 3), dtype=float)
    values = np.asarray(active_points, dtype=float)
    if len(values):
        centre = values[:, :2].mean(axis=0)
        active_radius = float(np.max(np.linalg.norm(values[:, :2] - centre, axis=1)))
    else:
        centre = np.zeros(2)
        active_radius = 0.0
    if count > 1:
        min_radius = min_sep / (2.0 * math.sin(math.pi / count))
    else:
        min_radius = 0.0
    centre_norm = float(np.linalg.norm(centre))
    radius_limit = max(0.5, max_radius - centre_norm)
    radius = min(radius_limit, max(active_radius + 1.0, min_radius + 0.1, 1.0))
    angles = 2.0 * math.pi * np.arange(count, dtype=float) / count
    halo_z = float(z) + 0.65
    if halo_z > 2.90:
        halo_z = max(0.10, float(z) - 0.65)
    return np.column_stack([
        centre[0] + radius * np.cos(angles),
        centre[1] + radius * np.sin(angles),
        np.full(count, halo_z),
    ])


def _assign_targets_angular(start: np.ndarray, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    n = len(start)
    values = np.asarray(points, dtype=float)
    matched = start.copy()
    indices = np.arange(len(values), dtype=int)
    used = min(n, len(values))
    if used <= 1:
        base_matched, base_indices = _assign_targets(start, values)
        return base_matched, base_indices
    start_sub = start[:used]
    target_sub = values[:used]
    centre_start = start_sub[:, :2].mean(axis=0)
    centre_target = target_sub[:, :2].mean(axis=0)
    angle_start = np.arctan2(start_sub[:, 1] - centre_start[1],
                             start_sub[:, 0] - centre_start[0])
    angle_target = np.arctan2(target_sub[:, 1] - centre_target[1],
                              target_sub[:, 0] - centre_target[0])
    order_start = np.argsort(angle_start)
    order_target = np.argsort(angle_target)
    for index in range(used):
        matched[order_start[index]] = target_sub[order_target[index]]
    indices[order_start] = order_target
    return matched, indices


def _assign_scene_target(entry: np.ndarray, points: np.ndarray,
                         mode: str = "hungarian") -> tuple[np.ndarray, np.ndarray]:
    n = len(entry)
    values = np.asarray(points, dtype=float)
    if len(values) > n:
        values = values[:n]
    active_count = len(values)
    matched = entry.copy()
    active_mask = np.zeros(n, dtype=bool)
    if active_count == 0:
        return matched, active_mask
    if active_count > 1:
        if mode == "angular":
            active_matched, _ = _assign_targets_angular(entry[:active_count], values)
        else:
            active_matched, _ = _assign_targets(entry[:active_count], values)
    else:
        active_matched = values.copy()
    matched[:active_count] = active_matched
    active_mask[:active_count] = True
    z_levels = np.array([0.0, 0.20, -0.20, 0.38, -0.38], dtype=float)
    for row in range(active_count):
        offset = float(z_levels[row % len(z_levels)])
        candidate = matched[row, 2] + offset
        if candidate < 0.15 or candidate > 2.90:
            candidate = matched[row, 2] - offset
        matched[row, 2] = max(0.15, min(2.90, candidate))
    return matched, active_mask


def _safe_rotation_steps(radius: float, turns: float, duration: float) -> int:
    turns = abs(float(turns))
    if duration <= 1e-9 or turns <= 1e-12 or radius <= 1e-9:
        return 1
    minimum_steps = max(8, int(math.ceil(turns * 16)))
    best_steps = minimum_steps
    best_score = None
    for steps in range(minimum_steps, max(minimum_steps, 96) + 1):
        angle = 2.0 * math.pi * turns / steps
        chord = abs(2.0 * radius * math.sin(angle / 2.0))
        segment_time = float(duration) / steps
        max_speed = 1.5 * chord / segment_time
        max_accel = 6.0 * chord / (segment_time * segment_time)
        speed_ok = max_speed <= 1.90
        accel_ok = max_accel <= 7.50
        score = (0 if speed_ok and accel_ok else 1, max_speed - 1.90, max_accel - 7.50)
        if best_score is None or score < best_score:
            best_score = score
            best_steps = steps
    return best_steps


def _prepare_scenes(scenes: list[dict[str, Any]], num_drones: int,
                    start: np.ndarray, font_path: str,
                    assignment_mode: str = "hungarian") -> tuple[list[dict[str, Any]], list[str]]:
    infos: list[dict[str, Any]] = []
    warnings: list[str] = []
    entry = start.copy()
    previous_colors = [DEFAULT_COLOR] * num_drones
    n = int(num_drones)
    for index, scene in enumerate(scenes):
        if not isinstance(scene, dict):
            raise TimelineError("E_BAD_SCENE", "scene " + str(index) + " must be an object")
        scene_duration = float(scene.get("duration_sec", 0.0))
        if scene_duration <= 0.0:
            raise TimelineError("E_BAD_SCENE", "scene " + str(index) + " duration_sec must be positive")
        base_transition = float(scene.get("transition_seconds", 0.0))
        if base_transition < 0.0:
            raise TimelineError("E_BAD_SCENE", "scene " + str(index) + " transition_seconds must be non-negative")
        if base_transition > scene_duration:
            warnings.append("scene " + str(index) + ": transition_seconds clamped to duration_sec")
            base_transition = scene_duration

        target = _scene_target(scene, n, font_path)
        points = np.asarray(target["points"], dtype=float)
        if len(points) > n:
            points = points[:n]
        points, limited_count, limit_warning = _limit_scene_points(points)
        # Default to the full fleet. Users may still set a smaller max_active explicitly.
        max_active = int(scene.get("max_active", n))
        if max_active > 0 and len(points) > max_active:
            points = points[:max_active]
            limited_count = len(points)
            limit_warning = (limit_warning + " and capped to " + str(max_active)
                             if limit_warning else
                             "scene target capped to " + str(max_active) + " active points")
        if limit_warning:
            warnings.append("scene " + str(index) + ": " + limit_warning)
            target = dict(target)
            target["points"] = points
            target["used"] = int(limited_count)
            target["idle"] = int(n - limited_count)
            target["formation_id"] = "f_" + short_hash(canonical_json({
                "type": target.get("scene_type", "shape"),
                "text": str(scene.get("text", "")),
                "points": np.round(points, 9).tolist(),
            }))
        effect = scene.get("effect") or {"type": "none"}
        effect = effect if isinstance(effect, dict) else {"type": "none"}
        scene_color, scene_colors = _resolve_scene_colors(scene, n)
        transition_duration = base_transition
        active_count = len(points)
        matched: np.ndarray | None = None
        provisional_animation = max(0.0, scene_duration - transition_duration)

        for _ in range(4):
            if str(effect.get("type", "none")).lower() == "rotate":
                adjusted, new_active, rotate_warning = _limit_rotate_target(
                    points, effect, provisional_animation)
                if new_active < len(points) or not np.allclose(adjusted, points):
                    points = adjusted
                    active_count = new_active
                    if rotate_warning:
                        warnings.append("scene " + str(index) + ": " + rotate_warning)
                    target = dict(target)
                    target["points"] = points
                    target["used"] = int(len(points))
                    target["idle"] = int(n - len(points))
                    target["formation_id"] = "f_" + short_hash(canonical_json({
                        "type": target.get("scene_type", "shape"),
                        "text": str(scene.get("text", "")),
                        "points": np.round(points, 9).tolist(),
                    }))
            matched, active_mask = _assign_scene_target(entry, points, assignment_mode)
            displacement = np.linalg.norm(matched - entry, axis=1)
            max_displacement = float(np.max(displacement)) if len(displacement) else 0.0
            required_transition = 1.5 * max_displacement / 1.45
            new_transition = min(scene_duration, max(base_transition, required_transition))
            if new_transition > transition_duration + 1e-9:
                transition_duration = new_transition
                warnings.append("scene " + str(index)
                                + ": transition extended to "
                                + str(round(transition_duration, 3))
                                + " s for validator speed safety")
            new_animation = max(0.0, scene_duration - transition_duration)
            if abs(new_animation - provisional_animation) < 1e-9:
                provisional_animation = new_animation
                break
            provisional_animation = new_animation

        assert matched is not None
        remaining = max(0.0, scene_duration - transition_duration)
        is_final = index == len(scenes) - 1
        include_landing = bool(scene.get("include_landing", False)) and is_final
        landing_total = min(12.0, remaining) if include_landing else 0.0
        if include_landing and landing_total <= 0.0:
            warnings.append("scene " + str(index) + ": no time available for landing")
        animation_duration = max(0.0, remaining - landing_total)
        if str(effect.get("type", "none")).lower() == "rotate":
            final_adjusted, final_active, rotate_warning = _limit_rotate_target(
                points, effect, animation_duration)
            if final_active < len(points) or not np.allclose(final_adjusted, points):
                points = final_adjusted
                active_count = final_active
                if rotate_warning:
                    warnings.append("scene " + str(index) + ": " + rotate_warning)
                matched, active_mask = _assign_scene_target(entry, points, assignment_mode)
                target = dict(target)
                target["points"] = points
                target["used"] = int(len(points))
                target["idle"] = int(n - len(points))
                target["formation_id"] = "f_" + short_hash(canonical_json({
                    "type": target.get("scene_type", "shape"),
                    "text": str(scene.get("text", "")),
                    "points": np.round(points, 9).tolist(),
                }))
        effect_segments = _effect_segments(
            matched, scene_colors, scene_color, scene.get("gradient"),
            effect, animation_duration, "scene" + str(index),
            active_mask=active_mask)
        if effect_segments:
            final_positions = effect_segments[-1]["end_positions"].copy()
            final_colors = list(effect_segments[-1]["colors"] or scene_colors)
        else:
            final_positions = matched.copy()
            final_colors = list(scene_colors)
        landing_segment = None
        reposition_segment = None
        preland_segment = None
        if include_landing and landing_total > 0.0:
            active_rows = np.where(active_mask)[0]
            descent_duration = min(1.5, max(0.5, landing_total * 0.15))
            reposition_duration = max(0.0, landing_total - descent_duration)
            reposition_end = final_positions.copy()
            if len(active_rows):
                reposition_end[active_rows, :2] = np.asarray(start, dtype=float)[active_rows, :2]
            if reposition_duration > 0.0 and len(active_rows):
                reposition_segment = {
                    "name": "transition",
                    "duration": float(reposition_duration),
                    "start_positions": final_positions.copy(),
                    "end_positions": reposition_end.copy(),
                    "color": final_colors[0] if final_colors else scene_color,
                    "colors": list(final_colors),
                }
            landing_end = reposition_end.copy()
            if len(active_rows):
                landing_end[active_rows, 2] = LANDING_Z
            landing_segment = {
                "name": "landing",
                "duration": float(descent_duration),
                "start_positions": reposition_end.copy(),
                "end_positions": landing_end,
                "color": final_colors[0] if final_colors else scene_color,
                "colors": list(final_colors),
            }
            final_positions = landing_end.copy()
            final_colors = list(final_colors)
        effect_names = [segment["name"] for segment in effect_segments]
        summary = {
            "index": index,
            "type": target["scene_type"],
            "text": str(scene.get("text", "")),
            "duration_sec": round(scene_duration, 6),
            "transition_seconds": round(transition_duration, 6),
            "effect": effect,
            "formation_id": target["formation_id"],
            "points": int(len(points)),
            "used": int(len(points)),
            "idle": int(max(0, n - len(points))),
            "min_distance": None if len(points) < 2 else round(float(
                min_pairwise_distance_2d(points)), 6),
            "segments": effect_names,
            "landing": bool(landing_segment is not None),
        }
        infos.append({
            "index": index,
            "entry_positions": entry.copy(),
            "base": matched,
            "active_mask": active_mask,
            "base_points": points,
            "base_colors": scene_colors,
            "scene_color": scene_color,
            "transition_duration": transition_duration,
            "effect_segments": effect_segments,
            "reposition_segment": reposition_segment,
            "preland_segment": preland_segment,
            "landing_segment": landing_segment,
            "final_positions": final_positions,
            "final_colors": final_colors,
            "previous_colors": list(previous_colors),
            "summary": summary,
        })
        entry = final_positions.copy()
        previous_colors = list(final_colors)
    return infos, warnings


def min_pairwise_distance_2d(points: np.ndarray) -> float:
    values = np.asarray(points, dtype=float)
    if len(values) < 2:
        return float("inf")
    best = float("inf")
    for i in range(len(values) - 1):
        d = np.linalg.norm(values[i + 1:] - values[i], axis=1)
        if len(d):
            best = min(best, float(np.min(d)))
    return best


def _raw_from_scenes(scene_infos: list[dict[str, Any]],
                     overrides: dict[int, np.ndarray]) -> list[dict[str, Any]]:
    raw: list[dict[str, Any]] = []
    for info in scene_infos:
        entry = info["entry_positions"]
        base = info["base"]
        previous_colors = info["previous_colors"]
        base_colors = info["base_colors"]
        active_mask = info.get("active_mask")
        if info["index"] in overrides:
            route = [entry, overrides[info["index"]], base]
        elif active_mask is not None and np.any(active_mask):
            active_rows = np.where(active_mask)[0]
            lift = entry.copy()
            cruise = max(float(entry[active_rows, 2].max()),
                         float(base[active_rows, 2].max())) + 0.50
            lift[active_rows, 2] = min(2.85, cruise)
            route = [entry, lift, base]
        else:
            route = [entry, base]
        transition_duration = float(info["transition_duration"])
        hop_weights = [max(1e-6, float(np.max(np.linalg.norm(
            route[hop + 1] - route[hop], axis=1)))) for hop in range(len(route) - 1)]
        total_weight = sum(hop_weights)
        if hop_weights and transition_duration > 0.0:
            hop_durations = [transition_duration * weight / total_weight for weight in hop_weights]
            hop_durations[-1] = transition_duration - sum(hop_durations[:-1])
            for hop, hop_duration in enumerate(hop_durations):
                global_mid = (sum(hop_weights[:hop]) + 0.5 * hop_weights[hop]) / total_weight
                colors = _blend_color_lists(previous_colors, base_colors, global_mid)
                raw.append({
                    "name": "transition",
                    "duration": max(0.0, hop_duration),
                    "start_positions": route[hop].copy(),
                    "end_positions": route[hop + 1].copy(),
                    "color": colors[0] if colors else info["scene_color"],
                    "colors": colors,
                })
        raw.extend(info["effect_segments"])
        if info.get("preland_segment") is not None:
            raw.append(info["preland_segment"])
        if info.get("reposition_segment") is not None:
            raw.append(info["reposition_segment"])
        if info["landing_segment"] is not None:
            raw.append(info["landing_segment"])
    return raw


def _timeline_frames(raw_segments: list[dict[str, Any]],
                     samples_per_segment: int = 40) -> list[np.ndarray]:
    frames: list[np.ndarray] = []
    for segment in raw_segments:
        start = np.asarray(segment["start_positions"], dtype=float)
        end = np.asarray(segment["end_positions"], dtype=float)
        for sample in range(samples_per_segment):
            u = (sample + 0.5) / samples_per_segment
            s = _smoothstep_scalar(u)
            frames.append(start + (end - start) * s)
    return frames


def _color_conflict_offsets(raw_segments: list[dict[str, Any]], num_drones: int,
                            threshold: float = 0.30) -> np.ndarray:
    n = int(num_drones)
    frames = _timeline_frames(raw_segments)
    if not frames or n <= 1:
        return np.zeros(n, dtype=float)
    adjacency = [set() for _ in range(n)]
    for frame in frames:
        for i in range(n - 1):
            distances = np.linalg.norm(frame[i + 1:] - frame[i], axis=1)
            for offset in np.where(distances < float(threshold))[0]:
                j = i + 1 + int(offset)
                adjacency[i].add(j)
                adjacency[j].add(i)
    order = sorted(range(n), key=lambda index: -len(adjacency[index]))
    colors = [-1] * n
    for index in order:
        if not adjacency[index]:
            colors[index] = 0
            continue
        used = {colors[other] for other in adjacency[index] if colors[other] >= 0}
        color = 0
        while color in used:
            color += 1
        colors[index] = min(color, 4)
    levels = [0.0, 0.24, 0.48, 0.72, 0.96]
    all_positions = np.stack(frames, axis=0)
    z_min = np.min(all_positions[:, :, 2], axis=0)
    z_max = np.max(all_positions[:, :, 2], axis=0)
    offsets = np.zeros(n, dtype=float)
    for index in range(n):
        desired = levels[colors[index]]
        upper = 2.90 - float(z_max[index])
        offsets[index] = max(0.0, min(desired, upper))
    return offsets


def _apply_z_offsets(raw_segments: list[dict[str, Any]],
                     offsets: np.ndarray) -> list[dict[str, Any]]:
    adjusted: list[dict[str, Any]] = []
    for segment in raw_segments:
        start = np.asarray(segment["start_positions"], dtype=float).copy()
        end = np.asarray(segment["end_positions"], dtype=float).copy()
        start[:, 2] += offsets
        end[:, 2] += offsets
        if segment.get("name") == "landing":
            end[:, 2] = LANDING_Z
        adjusted.append({
            "name": segment["name"],
            "duration": float(segment["duration"]),
            "start_positions": start,
            "end_positions": end,
            "color": segment.get("color"),
            "colors": None if segment.get("colors") is None else list(segment.get("colors")),
        })
    return adjusted


def build_timeline_plan(state_positions: Any, num_drones: int, duration_sec: float,
                        scenes: list[dict[str, Any]],
                        source_state_revision: int, secret: bytes,
                        font_path: str | None = None,
                        plan_id: str | None = None,
                        assignment_mode: str = "hungarian") -> dict[str, Any]:
    start = np.asarray(state_positions, dtype=float)
    n = int(num_drones)
    if start.shape != (n, 3):
        raise TimelineError("E_BAD_STATE", "state positions must be " + str(n) + "x3")
    if not isinstance(scenes, list) or len(scenes) == 0:
        raise TimelineError("E_BAD_SCENE", "scenes must be a non-empty array")
    try:
        total_scene_time = float(sum(float(scene.get("duration_sec", 0.0))
                                     for scene in scenes if isinstance(scene, dict)))
    except (TypeError, ValueError) as exc:
        raise TimelineError("E_BAD_SCENE", "scene duration_sec values must be numbers") from exc
    if abs(total_scene_time - float(duration_sec)) > 1e-6:
        raise TimelineError("E_DURATION_MISMATCH",
                            "scene duration sum does not match duration_sec",
                            duration_sec=float(duration_sec),
                            scene_total_sec=total_scene_time)
    resolved_font = resolve_font_path(font_path)
    infos, warnings = _prepare_scenes(scenes, n, start, resolved_font, assignment_mode)
    if plan_id is None:
        plan_id = "p_" + secrets.token_hex(6)
    first_colors = infos[0]["base_colors"] if infos else [DEFAULT_COLOR] * n
    meta = {
        "scenes": [info["summary"] for info in infos],
        "warnings": list(warnings),
        "duration_sec": float(duration_sec),
        "num_drones": n,
        "color": first_colors[0] if first_colors else DEFAULT_COLOR,
        "colors": first_colors,
        "target_formation_id": infos[-1]["summary"]["formation_id"] if infos else None,
        "repaired": False,
        "repair": None,
    }
    raw_direct = _raw_from_scenes(infos, {})
    direct = _finalize_plan(plan_id, n, start, raw_direct,
                            int(source_state_revision), secret, meta)
    direct_validation = validate_plan(direct)
    meta["validation"] = direct_validation
    if direct_validation["valid"]:
        direct["_timeline"] = meta
        return direct

    offsets = _color_conflict_offsets(raw_direct, n)
    if np.any(np.abs(offsets) > 1e-9):
        adjusted_meta = dict(meta)
        adjusted_meta["warnings"] = list(meta["warnings"]) + [
            "collision repair applied via deterministic per-drone altitude separation"
        ]
        adjusted_meta["repaired"] = True
        adjusted_meta["repair"] = {
            "reason": "E_MIN_SEPARATION",
            "via_type": "altitude-offset",
        }
        adjusted_plan = _finalize_plan(plan_id, n, start,
                                       _apply_z_offsets(raw_direct, offsets),
                                       int(source_state_revision), secret,
                                       adjusted_meta)
        adjusted_validation = validate_plan(adjusted_plan)
        if adjusted_validation["valid"]:
            adjusted_meta["validation"] = adjusted_validation
            adjusted_plan["_timeline"] = adjusted_meta
            return adjusted_plan

    issues = direct_validation.get("issues") or []
    if not issues or any(not str(issue).startswith("E_MIN_SEPARATION") for issue in issues):
        direct["_timeline"] = meta
        return direct

    for via_formation in _via_candidates(n):
        via_points = np.asarray(via_formation.get("points", []), dtype=float)
        if via_points.ndim != 2 or via_points.shape[0] != n:
            continue
        for leg_index in range(len(infos)):
            matched_via, _ = _assign_targets(infos[leg_index]["entry_positions"], via_points)
            trial_meta = dict(meta)
            trial_meta["warnings"] = list(meta["warnings"]) + [
                "collision repair applied on transition " + str(leg_index)
                + " via " + str(via_formation.get("type"))
            ]
            trial_meta["repaired"] = True
            trial_meta["repair"] = {
                "reason": "E_MIN_SEPARATION",
                "via_formation_id": via_formation.get("formation_id"),
                "via_type": via_formation.get("type"),
                "leg_index": leg_index,
            }
            trial = _finalize_plan(plan_id, n, start,
                                  _raw_from_scenes(infos, {leg_index: matched_via}),
                                  int(source_state_revision), secret, trial_meta)
            trial_validation = validate_plan(trial)
            if trial_validation["valid"]:
                trial_meta["validation"] = trial_validation
                trial["_timeline"] = trial_meta
                return trial
        overrides = {}
        for info in infos:
            matched_via, _ = _assign_targets(info["entry_positions"], via_points)
            overrides[info["index"]] = matched_via
        trial_meta = dict(meta)
        trial_meta["warnings"] = list(meta["warnings"]) + [
            "collision repair applied on all transitions via " + str(via_formation.get("type"))
        ]
        trial_meta["repaired"] = True
        trial_meta["repair"] = {
            "reason": "E_MIN_SEPARATION",
            "via_formation_id": via_formation.get("formation_id"),
            "via_type": via_formation.get("type"),
            "leg_index": "all",
        }
        trial = _finalize_plan(plan_id, n, start,
                              _raw_from_scenes(infos, overrides),
                              int(source_state_revision), secret, trial_meta)
        trial_validation = validate_plan(trial)
        if trial_validation["valid"]:
            trial_meta["validation"] = trial_validation
            trial["_timeline"] = trial_meta
            return trial
    direct["_timeline"] = meta
    return direct
