"""Procedural 3D formations. Every keyframe is checked by the normal planner.

No display-only displacement: these points become real controller targets.
"""
from __future__ import annotations

import math
import numpy as np

KINDS = ("helix", "atom", "bird")


def capacity(kind: str, fleet: int, budget="auto") -> int:
    if budget != "auto":
        count = int(budget)
    elif kind == "helix":
        count = min(64, fleet) // 2 * 2
    elif kind == "atom":
        count = min(48, fleet) // 3 * 3
    else:
        count = 8 + (min(40, fleet) - 8) // 4 * 4
    if count > fleet or count < 12:
        raise ValueError("动态编队需要至少 12 架且不能超过机队数量")
    if ((kind == "helix" and count % 2) or
            (kind == "atom" and count % 3) or
            (kind == "bird" and (count < 16 or (count - 8) % 4))):
        raise ValueError("双螺旋点数应为 2 的倍数；原子为 3 的倍数；飞鸟为 8+4k")
    return count


def points(kind: str, count: int, phase: float = 0.0) -> np.ndarray:
    if kind == "helix":
        t = np.linspace(0, 1, count // 2)
        angle = t * math.tau * 2 + phase
        strands = [np.column_stack([1.25 * np.cos(angle + offset),
                    1.25 * np.sin(angle + offset), .7 + 1.6 * t])
                   for offset in (0, math.pi)]
        return np.vstack(strands)
    if kind == "atom":
        angle = np.arange(count // 3) * math.tau / (count // 3)
        rings = []
        for i, (radius, tilt) in enumerate(((1., 15), (1.45, -22), (1.9, 30))):
            a = angle + phase * (1 if i % 2 == 0 else -1) + i * .23
            inclination = math.radians(tilt)
            ring = np.column_stack([radius * np.cos(a),
                   radius * np.sin(a) * math.cos(inclination),
                   radius * np.sin(a) * math.sin(inclination) + 1.6])
            # Different orbital planes, separate radial shells.
            azimuth = i * math.tau / 3
            x, y = ring[:, 0].copy(), ring[:, 1].copy()
            ring[:, 0] = x * math.cos(azimuth) - y * math.sin(azimuth)
            ring[:, 1] = x * math.sin(azimuth) + y * math.cos(azimuth)
            rings.append(ring)
        return np.vstack(rings)
    if kind == "bird":
        x = np.linspace(.6, 3.2, (count - 8) // 4)
        wing_z = 1.5 + .55 * np.sin(phase) * x / 3.2
        wings = [np.column_stack([sign * x, y, wing_z])
                 for sign in (-1, 1) for y in (.2 + .35 * x, -.3 + .12 * x)]
        body = np.column_stack([np.zeros(8), np.linspace(-1.6, 1.35, 8), np.full(8, 1.5)])
        return np.vstack([*wings, body])
    raise ValueError("未知动态编队")


def audience_plane(values: np.ndarray, tilt_degrees: float | None = None) -> np.ndarray:
    extent = max(float(np.max(np.abs(values[:, 1]))), 1e-9)
    safe_angle = math.degrees(math.asin(min(1., 1.05 / extent)))
    if tilt_degrees is None:
        tilt_degrees = min(30., safe_angle)
    if not math.isfinite(tilt_degrees) or not 0 <= tilt_degrees <= min(35., safe_angle + 1e-8):
        raise ValueError("文字倾角应在 0 到 35 度之间")
    result = values.copy()
    angle = math.radians(tilt_degrees)
    result[:, 1] = values[:, 1] * math.cos(angle)
    result[:, 2] = 1.5 + values[:, 1] * math.sin(angle)
    return result
