"""AGNES six-act score. All visible drones have certified controller targets.

The ground is a single real plane. Three-colour hexagonal batches lift into
separate altitudes before spatial assignment; landing reverses that procedure.
"""
from __future__ import annotations

import colorsys
import math
import secrets

import numpy as np
from scipy.spatial import cKDTree

from .formations import _text_lit_cells, min_pairwise_distance_fast
from .safe_motion import continuous_separation, match_targets, move_seconds
from .timeline import _finalize_plan
from .validator import validate_plan_chunked

THEME = "agnes-light"
TITLE = "AGNES · 一念成光"


def ground_layout(total=1000):
    if not 16 <= int(total) <= 1000:
        raise ValueError("离线主题秀支持 16 到 1000 架")
    slots = [(i*.28+j*.14, j*.28*math.sqrt(3)/2, .10, (i-j) % 3)
             for j in range(-23, 24) for i in range(-26, 27)
             if (i*.28+j*.14)**2+(j*.28*math.sqrt(3)/2)**2 < 4.8**2]
    slots = np.asarray(slots)
    slots = slots[np.argsort(np.linalg.norm(slots[:, :2], axis=1), kind="stable")][:total]
    return slots[:, :3].copy(), slots[:, 3].astype(int)


def _volume():
    values = []
    for layer, z in enumerate(np.arange(.38, 2.8, .46)):
        for radius in np.arange(.46, 4.65, .46):
            count = int(math.floor(math.tau*radius/.46))
            for angle in np.arange(count)*math.tau/count + layer*.035:
                values.append([radius*math.cos(angle), radius*math.sin(angle), z])
    return np.asarray(values)


def _glyph(text, pitch, upright=True):
    cells, width = _text_lit_cells(text, 1)
    local = np.array([[(x-(width-1)/2)*pitch, (3-y)*pitch] for x, y in cells])
    if upright:
        return np.column_stack([local[:, 0], np.full(len(local), -.8), 1.5+local[:, 1]])
    angle = math.radians(25)
    return np.column_stack([local[:, 0], local[:, 1]*math.cos(angle), 1.5+local[:, 1]*math.sin(angle)])


def _backdrop(front, total, volume, word=False):
    candidates = volume[volume[:, 1] > .15] if word else volume[np.linalg.norm(volume[:, :2], axis=1) > 2.9]
    distance, _ = cKDTree(front).query(candidates)
    candidates = candidates[distance >= .42]
    # Spread stars through the depth, rather than filling one layer first.
    order = np.argsort((np.arctan2(candidates[:, 1], candidates[:, 0])*.618
                        + candidates[:, 2]*2.31) % math.tau, kind="stable")
    if len(candidates) < total-len(front):
        raise ValueError("主题机群没有足够的安全背景槽位")
    return np.vstack([front, candidates[order[:total-len(front)]]])


def _palette(points, foreground=0, gold=False, phase=0):
    colors = []
    for i, point in enumerate(points):
        if i < foreground:
            color = (1., .68, .20) if gold else (.55, .87, 1.)
        else:
            hue = (.57 + .19*math.sin(math.atan2(point[1], point[0])*2+phase+point[2])) % 1
            pulse = .42+.58*(.5+.5*math.sin(point[0]*1.7+point[1]*.7+phase))
            color = colorsys.hsv_to_rgb(hue, .72, pulse*(.28 if foreground else 1.))
        colors.append("#"+"".join(f"{round(c*255):02x}" for c in color))
    return colors


def build_cinematic_plan(start, revision=1, secret=b"drone-show-local-dev-secret", total=1000):
    start = np.asarray(start, dtype=float)
    ground, batches = ground_layout(total)
    if start.shape != ground.shape or np.max(np.linalg.norm(start-ground, axis=1)) > .04:
        raise ValueError("主题秀须从专用地面机位开始")
    volume = _volume()
    raw, scenes = [], []
    entry = start.copy()

    def add(target, name, seconds, palette=None, certify=True):
        nonlocal entry
        if certify:
            separation = continuous_separation(entry, target)
            if separation < .25:
                raise ValueError(f"{name}: 转场间距 {separation:.6f} m 不通过")
        palette = palette or _palette(target)
        raw.append({"name": name, "duration": move_seconds(entry, target, seconds),
                    "start_positions": entry.copy(), "end_positions": target.copy(),
                    "color": palette[0], "colors": palette})
        entry = target.copy()

    def assemble(points, title, front=0, gold=False):
        nonlocal entry
        matched, order = match_targets(entry, points)
        colors = _palette(points, front, gold)
        add(matched, "assemble:"+title, 7., [colors[i] for i in order])
        scenes.append({"type": title, "active": total, "idle": 0,
                       "foreground": front, "background": total-front,
                       "min_distance": min_pairwise_distance_fast(points)})
        return order

    for batch in range(3):
        target = entry.copy()
        target[batches == batch, 2] = .7+batch*.7
        add(target, "takeoff:苏醒", 3.)

    logo = _glyph("A", .72, False)
    logo = np.vstack([logo + [dx, dy*math.cos(math.radians(25)), dy*math.sin(math.radians(25))]
                      for dx, dy in ((-.18,-.18),(-.18,.18),(.18,-.18),(.18,.18))])
    word = _glyph("AGNES", .35)
    if total < len(word):
        raise ValueError("完整 AGNES 字标需要至少 85 架")
    for points, title, front in ((_backdrop(logo, total, volume), "A诞生", len(logo)),
                                 (_backdrop(word, total, volume, True), "AGNES", len(word))):
        order = assemble(points, title, front)
        for beat in range(8):
            colors = _palette(points, front, phase=beat*.5)
            for i in range(front):
                if points[i, 0] > -4.7+beat*1.4:
                    colors[i] = "#102034"
            add(entry, "scene:"+title, .6, [colors[i] for i in order], False)

    angle = np.arctan2(volume[:, 1], volume[:, 0])
    helix_score = np.abs(np.sin(angle-volume[:, 2]*math.tau/2.3))
    helix = volume[np.argsort(helix_score, kind="stable")[:total]]
    assemble(helix, "双螺旋")
    for beat in range(10):
        a = math.pi/24
        target = entry.copy()
        target[:, :2] = entry[:, :2] @ np.array([[math.cos(a), math.sin(a)],[-math.sin(a), math.cos(a)]])
        add(target, "scene:双螺旋", 1.25, _palette(target, phase=beat*.55))

    # Full disk layers are a connected spatial star field, all thousand move.
    order = np.argsort((angle*1.618+volume[:, 2]*3.7) % math.tau, kind="stable")
    galaxy = volume[order[:total]]
    assemble(galaxy, "星河绽放")
    for beat in range(12):
        a = math.pi/24
        target = entry.copy()
        target[:, :2] = entry[:, :2] @ np.array([[math.cos(a), math.sin(a)],[-math.sin(a), math.cos(a)]])
        add(target, "scene:星河绽放", 1.25, _palette(target, phase=beat*.6))
    base = entry.copy()
    for beat in range(13):
        target = base.copy()
        target[:, 2] += .16*math.sin(beat*math.pi/6)*np.sin(base[:, 0]*.7+beat*math.pi/6)
        add(target, "scene:千机光浪", 1., _palette(target, phase=beat*.8))

    for points, title, front in ((_backdrop(logo, total, volume), "金色A", len(logo)),
                                 (_backdrop(word, total, volume, True), "AGNES归来", len(word))):
        order = assemble(points, title, front, True)
        for beat in range(5):
            colors = _palette(points, front, True, beat*.5)
            add(entry, "scene:"+title, .7, [colors[i] for i in order], False)

    lift = ground.copy()
    lift[:, 2] = .7+batches*.7
    matched, order = match_targets(entry, lift)
    add(matched, "return:归位", 8.)
    for batch in reversed(range(3)):
        target = entry.copy()
        target[batches[order] == batch, 2] = .10
        add(target, "landing:落幕", 3.)
    add(entry, "scene:落幕", 3., ["#080f19"]*total, False)
    plan = _finalize_plan("p_"+secrets.token_hex(6), total, start, raw, revision, secret,
                          {"title": TITLE, "theme": THEME, "offline_execution": True,
                           "scenes": scenes, "color": "#8cddff"})
    validation = validate_plan_chunked(plan)
    plan["_timeline"]["validation"] = validation
    if not validation["valid"]:
        raise ValueError("; ".join(validation["issues"]))
    return plan
