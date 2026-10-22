"""High-level show defaults; the model describes scenes, never coordinates."""
from __future__ import annotations

import math
import secrets
import numpy as np

from .cjk import CJKError, build_page_points, resolve_font_path
from .formations import build_formation, _text_lit_cells
from .safe_motion import match_targets, continuous_separation, move_segment
from .textshow import paginate_text
from .timeline import _effect_segments, _finalize_plan, _resolve_scene_colors
from .validator import validate_plan_chunked
from . import choreography


class ShowError(ValueError):
    pass


def ground_layout(n):
    cols = math.ceil(math.sqrt(n))
    rows = math.ceil(n / cols)
    return np.array([[(i % cols - (cols - 1) / 2) * .50,
                      (i // cols - (rows - 1) / 2) * .50, .1]
                     for i in range(n)], dtype=float)


def scene_targets(scene, n):
    budget = scene.get("drone_budget", "auto")
    if budget != "auto":
        if not isinstance(budget, int) or not 1 <= budget <= n:
            raise ShowError("场景点数必须在机队数量范围内")
    kind = str(scene.get("type", "text")).lower()
    if kind == "text":
        text = str(scene.get("text", "")).strip()
        if not text:
            raise ShowError("文字场景需要文字内容")
        if any(ord(c) > 127 for c in text):
            # Centreline lights describe strokes more clearly than a sparse
            # filled mask. Keep room between strokes in the frozen arena.
            pages = paginate_text(list(text.replace(" ", "")), min(n, 96) if budget == "auto" else budget,
                                  points_per_char=48, auto_allocate=True,
                                  min_points_per_char=min(48, n))["pages"]
            out = []
            font = resolve_font_path()
            def render(chars, allocations):
                try:
                    xy, _ = build_page_points(chars, allocations, font,
                                  base_height=4., letter_spacing=.16,
                                  min_separation=.32, stroke=True)
                except CJKError as exc:
                    if budget != "auto" or exc.code != "E_GLYPH_ALLOCATION":
                        raise
                    if len(chars) > 1:
                        middle = len(chars)//2
                        render(chars[:middle], allocations[:middle])
                        render(chars[middle:], allocations[middle:])
                        return
                    # Simple strokes cannot use 48 lights inside a 10 m
                    # arena. Find a fitting automatic budget without ever
                    # changing an explicitly requested budget or threshold.
                    if allocations[0] > 16:
                        render(chars, [allocations[0]-4])
                        return
                    raise
                out.append((choreography.audience_plane(
                    np.column_stack([xy, np.full(len(xy), 1.5)]),
                    scene.get("tilt_degrees")), "".join(chars)))
            for page in pages:
                render(page["characters"], page["allocations"])
            return out
        # Preserve every lit cell of the 5x7 font rather than dropping half
        # the strokes to an arbitrary eight lights per letter. Long words
        # are paginated automatically inside the arena and fleet budget.
        pages, page = [], ""
        for char in text.upper():
            candidate = page + char
            cells, width = _text_lit_cells(candidate, 1)
            if page and (len(cells) > n or width > 29):
                pages.append(page)
                page = char
            else:
                page = candidate
        if page:
            pages.append(page)
        out = []
        for page in pages:
            cells, width = _text_lit_cells(page, 1)
            count = len(cells) if budget == "auto" else budget
            if count > n:
                raise ShowError("当前机队数量不足以完整显示这一字符")
            if budget == "auto":
                # One light for every occupied font cell. The legacy formation
                # builder expands each cell to corners and farthest-samples it,
                # which changes the strokes even when count equals cell count.
                # Font rows go downward; ENU y and the audience plane go upward.
                points = np.array([[(x - (width - 1) / 2) * .34,
                                    (3 - row) * .34, 1.5]
                                   for x, row in sorted(cells)], dtype=float)
            else:
                points = np.asarray(build_formation(count, {"type": "text", "text": page,
                                    "z": 1.5, "point_spacing": .34})["points"], dtype=float)
                points[:, 1] *= -1
            out.append((choreography.audience_plane(points,
                        scene.get("tilt_degrees")), page))
        return out
    if kind in choreography.KINDS:
        count = choreography.capacity(kind, n, budget)
        return [(choreography.points(kind, count), kind)]
    if kind not in ("heart", "ring", "star", "grid"):
        raise ShowError("支持文字、爱心、圆环、星星、方阵、双螺旋、原子轨道和飞鸟")
    # Outline capacity, selected for readable spacing rather than cramming all
    # available drones onto one short curve. Idle drones return to the apron.
    count = min(n, {"heart": 32, "ring": 48, "star": 40, "grid": 64}[kind]) if budget == "auto" else budget
    spec = {"type": kind, "z": 1.5, "radius": 3.2, "size": 6.0,
            "spacing": .55}
    return [(np.asarray(build_formation(count, spec)["points"], dtype=float), kind)]


def build_show(start, scenes, revision, secret, include_landing=True):
    n = len(start)
    if not isinstance(scenes, list) or not scenes or len(scenes) > 12:
        raise ShowError("请提供 1 到 12 个表演场景")
    ground = ground_layout(n)
    raw, summaries = [], []
    entry = start.copy()
    colors = ["#5ac8fa"] * n

    def move(target, name, new_colors, minimum=2.5):
        nonlocal entry, colors
        permutation = np.arange(n)
        minimum_distance = continuous_separation(entry, target)
        if minimum_distance < .27 and (name.startswith("assemble:") or name == "return"):
            # Two dense glyphs can have a safe endpoint but unsafe straight
            # morph. A spacious intermediate apron restores a provable gap;
            # both legs still receive the same exact continuous check.
            clearance = ground.copy()
            clearance[:, :2] *= 1.1
            clearance[:, 2] = 1.2
            clearance, _ = match_targets(entry, clearance)
            if continuous_separation(entry, clearance) < .27:
                raise ShowError("当前机队无法安全进入过渡编队")
            raw.append(move_segment(entry, clearance, "clearance", colors, 3.))
            entry = clearance
            target, permutation = match_targets(entry, target)
            new_colors = [new_colors[i] for i in permutation]
            minimum_distance = continuous_separation(entry, target)
        if minimum_distance < .27:
            raise ShowError(f"{name} 连续最小间距 {minimum_distance:.3f} m，无法安全执行")
        raw.append(move_segment(entry, target, name, new_colors, minimum))
        entry, colors = target.copy(), list(new_colors)
        return permutation

    if np.max(entry[:, 2]) < .3:
        lift = entry.copy()
        lift[:, 2] = 1.2
        move(lift, "takeoff", colors, 3.0)
    for index, scene in enumerate(scenes):
        if not isinstance(scene, dict):
            raise ShowError("场景必须是对象")
        for points, label in scene_targets(scene, n):
            if len(points) > n:
                raise ShowError("场景点数超过机队数量")
            idle = n - len(points)
            # Keep idle drones on a separate altitude plane. Squared matching
            # covers them too; no hidden cap and no unplanned parked drones.
            targets = np.vstack([points, ground[:idle]]) if idle else points
            matched, cols = match_targets(entry, targets)
            color, palette = _resolve_scene_colors(scene, n)
            active = cols < len(points)
            palette = [palette[i] if active[i] else "#26334b" for i in range(n)]
            permutation = move(matched, "assemble:" + label, palette, 4.0)
            assignment = cols[permutation]
            palette = [palette[i] for i in permutation]
            active = active[permutation]
            hold = float(scene.get("hold_seconds", 4.0))
            if not math.isfinite(hold) or hold < 2 or hold > 120:
                raise ShowError("每幕停留时间应在 2 到 120 秒之间")
            effect = scene.get("effect") or {"type": "none"}
            if not isinstance(effect, dict):
                raise ShowError("动画效果必须是对象")
            if label in choreography.KINDS:
                cycles = float(effect.get("cycles", effect.get("turns", .5)))
                if not math.isfinite(cycles) or not .25 <= cycles <= 3:
                    raise ShowError("动态编队圈数应在 .25 到 3 之间")
                steps = max(16, math.ceil(cycles * 24))
                segments = []
                previous = entry.copy()
                for step in range(1, steps + 1):
                    animated = choreography.points(label, len(points),
                                                    math.tau * cycles * step / steps)
                    full = np.vstack([animated, ground[:idle]]) if idle else animated
                    target = full[assignment]
                    # Motor response benefits from a slower orbit than the nominal limits
                    # alone require. These are trajectory choices, not validator thresholds.
                    beat = {"helix": .8, "atom": 1.2, "bird": .35}[label]
                    segments.append(move_segment(previous, target,
                         "scene" + str(index) + ":" + label + "_motion", palette, max(beat, hold / steps)))
                    previous = target
                hold = sum(s["duration"] for s in segments)
            else:
                segments = _effect_segments(entry, palette, color, scene.get("gradient"),
                                        effect, hold, "scene" + str(index) + ":" + label,
                                        active_mask=active)
            required = sum(move_segment(s["start_positions"], s["end_positions"],
                                        s["name"], palette, 0.)["duration"] for s in segments)
            if label not in choreography.KINDS and required > hold:
                hold = required + .5
                segments = _effect_segments(entry, palette, color, scene.get("gradient"),
                                            effect, hold, "scene" + str(index) + ":" + label,
                                            active_mask=active)
            for segment in segments:
                # Keep exact continuous separation checks for effect chords.
                if continuous_separation(segment["start_positions"], segment["end_positions"]) < .27:
                    raise ShowError("动画使无人机靠得太近")
            raw.extend(segments)
            entry = segments[-1]["end_positions"].copy()
            # A zero-velocity, validated checkpoint enables live amendments.
            raw.append(move_segment(entry, entry, "dwell:" + label, palette, 2.0))
            summaries.append({"type": scene.get("type"), "text": label,
                              "active": len(points), "idle": idle,
                              "effect": effect, "hold_seconds": hold})
    if include_landing:
        apron = ground.copy()
        apron[:, 2] = 1.2
        matched, _ = match_targets(entry, apron)
        move(matched, "return", colors, 4.0)
        landed = entry.copy()
        landed[:, 2] = .1
        move(landed, "landing", colors, 3.0)
    meta = {"scenes": summaries, "warnings": [], "color": colors[0],
            "colors": colors, "director": True, "camera": "audience"}
    plan = _finalize_plan("p_" + secrets.token_hex(6), n, start, raw, revision, secret, meta)
    validation = validate_plan_chunked(plan)
    if not validation["valid"]:
        raise ShowError("表演未通过安全检查：" + "; ".join(validation["issues"]))
    meta["validation"] = validation
    plan["_timeline"] = meta
    return plan
