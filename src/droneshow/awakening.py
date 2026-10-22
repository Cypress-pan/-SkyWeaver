"""Offline AGNES film score, with distinct silhouettes and a quiet star fleet.

This is a deterministic film composition. It does not change the live preset or
claim model-generated choreography. Every point belongs to one physical drone.
"""
from __future__ import annotations

import colorsys
import math
import secrets

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial import cKDTree

from .cinematic import _volume, ground_layout
from .formations import min_pairwise_distance_fast
from .safe_motion import continuous_separation, match_targets, move_seconds
from .timeline import _finalize_plan
from .validator import validate_plan_chunked

TITLE = "AGNES · 光的觉醒"


def _inside_polygon(points, polygon):
    x, y = points.T
    inside = np.zeros(len(points), dtype=bool)
    for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
        if abs(b[1] - a[1]) < 1e-12:
            continue
        crossing = ((a[1] > y) != (b[1] > y)) & (
            x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0])
        inside ^= crossing
    return inside


def phoenix():
    # Feathered wings, narrow neck, beak and a three-pronged tail. Its plane is
    # tilted 18 degrees from horizontal, allowing a tall readable silhouette
    # within the unchanged 3 m flight volume.
    right = np.array([[0, 2.65], [.75, 2.25], [.32, 2.05], [.40, 1.50],
                      [1.1, 1.8], [2.45, 2.65], [4.05, 2.0], [3.02, 1.58],
                      [3.72, 1.30], [2.58, 1.06], [3.12, .69], [1.97, .55],
                      [2.24, .17], [.78, .30], [.38, -.52], [1.10, -2.13],
                      [.27, -1.70], [0, -2.55]])
    outline = np.vstack([right, right[-2:0:-1] * [-1, 1]])
    grid = np.array([[x, y] for y in np.arange(-2.5, 2.7, .40)
                     for x in np.arange(-4.0, 4.1, .40)])
    local = grid[_inside_polygon(grid, outline)]
    tilt = math.radians(18)
    return np.column_stack([local[:, 0], local[:, 1] * math.cos(tilt),
                            1.48 + local[:, 1] * math.sin(tilt)])


def wordmark():
    font = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial Bold.ttf', 200)
    box = font.getbbox('AGNES')
    image = Image.new('L', (box[2]-box[0], box[3]-box[1]))
    ImageDraw.Draw(image).text((-box[0], -box[1]), 'AGNES', font=font, fill=255)
    mask = np.asarray(image)
    points = []
    for v in np.arange(-1.4, 1.5, .35):
        for x in np.arange(-4.375, 4.4, .35):
            px = int(np.clip((x+4.55)/9.1*mask.shape[1], 0, mask.shape[1]-1))
            py = int(np.clip((1.575-v)/3.15*mask.shape[0], 0, mask.shape[0]-1))
            if mask[py, px] > 100:
                points.append([x, v*math.cos(math.pi/6), 1.5+v*.5])
    return np.array(points)


def helix_pose(points, phase):
    out = points.copy()
    y, z = points[:, 1], points[:, 2]-1.5
    out[:, 1] = y*math.cos(phase)-z*math.sin(phase)
    out[:, 2] = 1.5+y*math.sin(phase)+z*math.cos(phase)
    return out


def helix():
    points = []
    for strand in range(2):
        for radius in (.72, 1.16):
            for x in np.linspace(-4, 4, 28):
                angle = (x + 4) / 8 * math.tau * 1.6 + strand * math.pi
                points.append([x, radius * math.cos(angle), 1.5 + radius * math.sin(angle)])
    return np.array(points)


def star_core():
    # A sparse shell with two concentric orbital halos. Orbital planes do not
    # intersect the shell or each other; negative space separates all layers.
    sphere = []
    for latitude in np.linspace(-math.pi / 2, math.pi / 2, 7):
        r = 1.35 * math.cos(latitude)
        count = max(1, int(math.tau * r / .46))
        for a in np.arange(count) * math.tau / count:
            sphere.append([r * math.cos(a), r * math.sin(a), 1.5 + 1.1 * math.sin(latitude)])
    rings = []
    for radius, tilt in ((2.6, .32), (3.7, -.22)):
        count = int(math.tau * radius / .46)
        for a in np.arange(count) * math.tau / count:
            rings.append([radius * math.cos(a), radius * math.sin(a) * math.cos(tilt),
                          1.5 + radius * math.sin(a) * math.sin(tilt)])
    return np.vstack([sphere, rings]), len(sphere)


def with_stars(front, reserve=()):
    candidates = _volume()
    avoid = np.vstack([front, *reserve]) if len(reserve) else front
    distances, _ = cKDTree(avoid).query(candidates)
    candidates = candidates[distances >= .50]
    order = np.argsort((np.arctan2(candidates[:, 1], candidates[:, 0]) * .618
                        + candidates[:, 2] * 2.31) % math.tau, kind="stable")
    if len(candidates) < 1000 - len(front):
        raise ValueError("Not enough safe supporting star slots")
    return np.vstack([front, candidates[order[:1000-len(front)]]])


def palette(points, front=0, act="word", phase=0):
    colors = []
    for i, p in enumerate(points):
        if i < front:
            pulse = .72 + .28 * (.5 + .5 * math.sin(p[0]*1.2 + phase))
            hue = (.54 if act == "word" else .60 if act == "helix" else
                   .11 if act == "phoenix" else .58)
            hue += .09 * math.sin(p[0]*.6 + phase) if act == "helix" else 0
            if act == "core":
                radius = math.hypot(p[0], p[1])
                hue = .74 if radius < 1.6 else .51 if radius < 3.0 else .62
            rgb = colorsys.hsv_to_rgb(hue % 1, .35 if act == "word" else .72, pulse)
        else:
            # Keep all thousand physical aircraft, while their lights support
            # the foreground instead of erasing its outline.
            hue = .58 + .08*math.sin(p[0]+phase)
            rgb = colorsys.hsv_to_rgb(hue, .6, .10)
        colors.append("#" + "".join(f"{round(v*255):02x}" for v in rgb))
    return colors


def formations():
    word = wordmark()
    bird = phoenix()
    sphere, _ = star_core()
    h = helix()
    return {"word": with_stars(word), "helix": with_stars(h, [helix_pose(h, a) for a in np.linspace(0,.8,9)]),
            "core": with_stars(sphere), "phoenix": with_stars(bird)}, {
                "word": len(word), "helix": len(helix()), "core": len(sphere), "phoenix": len(bird)}


def build_awakening_plan(start=None, validate=True):
    ground, batches = ground_layout()
    start = ground if start is None else np.asarray(start, dtype=float)
    if start.shape != ground.shape or np.max(np.linalg.norm(start-ground, axis=1)) > .04:
        raise ValueError("The film must begin at the physical ground layout")
    raw, scenes = [], []
    entry = start.copy()

    def add(target, name, seconds, colors, certify=True):
        nonlocal entry
        gap = continuous_separation(entry, target) if certify else min_pairwise_distance_fast(target)
        if gap < .265:
            raise ValueError(f"{name}: continuous separation {gap:.6f} m")
        raw.append({"name": name, "duration": move_seconds(entry, target, seconds),
                    "start_positions": entry.copy(), "end_positions": target.copy(),
                    "color": colors[0], "colors": colors})
        entry = target.copy()

    def assemble(points, name, foreground, act):
        target, order = match_targets(entry, points)
        if continuous_separation(entry, target) < .27:
            # A wider star layout gives the denser letter strokes a safer
            # approach. Both legs receive their own exact continuous check.
            slots = _volume()
            slots = slots[np.argsort((np.arctan2(slots[:,1],slots[:,0])*1.618 + slots[:,2]*3.7) % math.tau)[:1000]]
            bridge, _ = match_targets(entry, slots)
            add(bridge, "assemble:"+name, 3., palette(bridge, 1000, act))
            target, order = match_targets(entry, points)
        colors = palette(points, foreground, act)
        add(target, "assemble:"+name, 6., [colors[i] for i in order])
        scenes.append({"type": name, "active": 1000, "idle": 0,
                       "foreground": foreground, "background": 1000-foreground,
                       "min_distance": min_pairwise_distance_fast(points)})
        return order

    add(entry, "scene:AGNES苏醒", 2., ["#16415a"]*1000, False)
    for batch in range(3):
        target = entry.copy()
        target[batches == batch, 2] = .7 + batch*.7
        add(target, "takeoff:AGNES升空", 2.6, palette(target, 1000))

    shapes, front = formations()
    order = assemble(shapes["word"], "AGNES", front["word"], "word")
    for beat in range(6):
        colors = palette(shapes["word"], front["word"], "word", beat*.7)
        for i in range(front["word"]):
            if shapes["word"][i, 0] > -4.6 + beat*1.8:
                colors[i] = "#142534"
        add(entry, "scene:AGNES", .8, [colors[i] for i in order], False)
    add(entry, "scene:AGNES", 2., [palette(shapes["word"], front["word"])[i] for i in order], False)

    order = assemble(shapes["helix"], "双螺旋", front["helix"], "helix")
    base = entry.copy()
    for beat in range(8):
        target = base.copy()
        active = order < front['helix']
        target[active] = helix_pose(base[active], (beat+1)*.1)
        colors = palette(shapes["helix"], front["helix"], "helix", beat*.8)
        add(target, "scene:双螺旋", 1., [colors[i] for i in order])

    order = assemble(shapes["core"], "AGNES星核轨道", front["core"], "core")
    for beat in range(8):
        angle = .11
        target = entry.copy()
        target[:, :2] = entry[:, :2] @ np.array([[math.cos(angle), math.sin(angle)],
                                               [-math.sin(angle), math.cos(angle)]])
        add(target, "scene:AGNES星核轨道", 1., [palette(shapes["core"], front["core"], "core", beat*.7)[i] for i in order])

    # The full thousand form a spatial wave; stacked hexagonal decks retain
    # visible structure and avoid collisions when the wave bends vertically.
    wave = _volume()
    wave = wave[np.argsort((np.arctan2(wave[:, 1], wave[:, 0])*1.618 + wave[:, 2]*3.7) % math.tau)[:1000]]
    order = assemble(wave, "千机光浪", 1000, "helix")
    base = entry.copy()
    for beat in range(10):
        phase = beat*math.pi/4
        target = base.copy()
        target[:, 2] += .13*math.sin(phase)*np.sin(base[:, 0]*.8+phase)
        colors = palette(wave, 1000, "helix" if beat < 6 else "phoenix", phase)
        add(target, "scene:千机光浪", 1., [colors[i] for i in order])

    bird = phoenix()
    poses = []
    for phase in (-1, 0, 1):
        p = bird.copy()
        wing = np.abs(p[:, 0]) > .65
        p[wing, 2] += phase*.42*np.abs(p[wing, 0])/4
        poses.append(p)
    bird_full = with_stars(bird, poses)
    order = assemble(bird_full, "AGNES金色凤凰", len(bird), "phoenix")
    base = entry.copy()
    for beat in range(12):
        target = base.copy()
        wing = order < len(bird)
        target[wing, 2] += .42*math.sin(beat*math.pi/3)*np.abs(base[wing, 0])/4
        add(target, "scene:AGNES金色凤凰", .8,
            [palette(bird_full, len(bird), "phoenix", beat*.55)[i] for i in order])

    order = assemble(shapes["word"], "AGNES归来", front["word"], "phoenix")
    add(entry, "scene:AGNES归来", 4., [palette(shapes["word"], front["word"], "phoenix")[i] for i in order], False)
    lift = ground.copy()
    lift[:, 2] = .7 + batches*.7
    target, order = match_targets(entry, lift)
    add(target, "return:AGNES归位", 6., palette(target, 1000, "phoenix"))
    for batch in reversed(range(3)):
        target = entry.copy()
        target[batches[order] == batch, 2] = .1
        add(target, "landing:AGNES落幕", 2.6, palette(target, 1000, "phoenix"))
    add(entry, "scene:AGNES落幕", 2., ["#010306"]*1000, False)

    plan = _finalize_plan("p_"+secrets.token_hex(6), 1000, start, raw, 1, b"awakening-offline-film",
                          {"title": TITLE, "theme": "agnes-awakening", "offline_execution": True,
                           "scenes": scenes, "color": "#8cddff"})
    if validate:
        result = validate_plan_chunked(plan)
        plan["_timeline"]["validation"] = result
        if not result["valid"]:
            raise ValueError("; ".join(result["issues"]))
    return plan
