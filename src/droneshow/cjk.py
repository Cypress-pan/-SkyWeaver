"""CJK glyph rendering and deterministic point sampling using Pillow."""
from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .formations import _farthest_point_select, min_pairwise_distance


DEFAULT_FONT_CANDIDATES = [
    "C:/Windows/Fonts/msyh.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/System/Library/Fonts/STHeiti Medium.ttc",
    "/System/Library/Fonts/Hiragino Sans GB.ttc",
    "/System/Library/Fonts/Supplemental/Songti.ttc",
    "/System/Library/Fonts/Supplemental/Arial Unicode.ttf",
    "/System/Library/Fonts/PingFang.ttc",
    "/Library/Fonts/Arial Unicode.ttf",
]
DEFAULT_RENDER_SIZE = 128
DEFAULT_BASE_HEIGHT = 2.4
DEFAULT_MIN_SEPARATION = 0.30
DEFAULT_MAX_RADIUS = 4.80


class CJKError(ValueError):
    def __init__(self, code: str, detail: str, **extra: Any):
        super().__init__(detail)
        self.code = code
        self.detail = detail
        self.extra = extra


def resolve_font_path(font_path: str | None = None) -> str:
    candidates: list[str] = []
    if font_path:
        candidates.append(str(font_path))
    env_path = os.environ.get("DRONE_SHOW_CJK_FONT")
    if env_path:
        candidates.append(env_path)
    candidates.extend(DEFAULT_FONT_CANDIDATES)
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            return str(candidate)
    raise CJKError(
        "E_FONT_UNAVAILABLE",
        "no CJK font found; set DRONE_SHOW_CJK_FONT to a .ttf/.ttc/.otf path",
    )


def font_index() -> int:
    raw = os.environ.get("DRONE_SHOW_CJK_FONT_INDEX", "0")
    try:
        return max(0, int(raw))
    except (TypeError, ValueError):
        return 0


def load_font(font_path: str, size: int = DEFAULT_RENDER_SIZE,
              index: int | None = None) -> ImageFont.FreeTypeFont:
    try:
        return ImageFont.truetype(font_path, size=size,
                                  index=font_index() if index is None else int(index))
    except Exception as exc:
        raise CJKError("E_FONT_UNAVAILABLE",
                       "could not load font " + repr(font_path) + ": " + str(exc)) from exc


def render_glyph_coords(char: str, font_path: str,
                        size: int = DEFAULT_RENDER_SIZE,
                        index: int | None = None) -> tuple[np.ndarray, tuple[int, int, int, int]]:
    if len(char) != 1:
        raise CJKError("E_BAD_SPEC", "glyph rendering expects one character")
    font = load_font(font_path, size, index)
    bbox = font.getbbox(char)
    if bbox is None or bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
        raise CJKError("E_GLYPH_EMPTY", "font produced an empty glyph for " + repr(char))
    pad = 4
    width = int(bbox[2] - bbox[0] + 2 * pad)
    height = int(bbox[3] - bbox[1] + 2 * pad)
    image = Image.new("L", (max(1, width), max(1, height)), 0)
    draw = ImageDraw.Draw(image)
    draw.text((pad - bbox[0], pad - bbox[1]), char, fill=255, font=font)
    array = np.asarray(image)
    ys, xs = np.where(array > 0)
    if len(xs) == 0:
        raise CJKError("E_GLYPH_EMPTY", "font produced an empty glyph for " + repr(char))
    return np.column_stack([xs, ys]).astype(float), bbox


def sample_glyph_points(char: str, count: int, font_path: str,
                        size: int = DEFAULT_RENDER_SIZE,
                        index: int | None = None, stroke: bool = False) -> np.ndarray:
    if count <= 0:
        return np.zeros((0, 2), dtype=float)
    coords, _ = render_glyph_coords(char, font_path, size, index)
    if stroke:
        coords = _stroke_coords(coords)
    if len(coords) < count:
        raise CJKError("E_GLYPH_ALLOCATION",
                       "not enough rendered pixels for " + repr(char)
                       + ": needed " + str(count) + ", got " + str(len(coords)))
    selected = _farthest_point_select(coords, count)
    x = selected[:, 0]
    y = selected[:, 1]
    x_span = float(x.max() - x.min())
    y_span = float(y.max() - y.min())
    scale = 1.0 / max(x_span, y_span, 1.0)
    centre_x = (x.max() + x.min()) / 2.0
    centre_y = (y.max() + y.min()) / 2.0
    return np.column_stack([(x - centre_x) * scale, -(y - centre_y) * scale])


def _stroke_coords(coords: np.ndarray) -> np.ndarray:
    """Zhang-Suen thinning keeps strokes readable with a small light budget.

    Filled-glyph sampling spends lights on both edges of thick strokes. A
    one-pixel centreline preserves the same glyph topology using fewer lights.
    """
    mask = np.zeros((int(coords[:, 1].max()) + 3,
                     int(coords[:, 0].max()) + 3), dtype=bool)
    mask[coords[:, 1].astype(int), coords[:, 0].astype(int)] = True
    while True:
        changed = False
        for second in (False, True):
            p = [mask[:-2, 1:-1], mask[:-2, 2:], mask[1:-1, 2:],
                 mask[2:, 2:], mask[2:, 1:-1], mask[2:, :-2],
                 mask[1:-1, :-2], mask[:-2, :-2]]
            neighbours = sum(v.astype(np.uint8) for v in p)
            crossings = sum((~p[i] & p[(i+1) % 8]).astype(np.uint8) for i in range(8))
            if second:
                corners = ~(p[0] & p[2] & p[6]) & ~(p[0] & p[4] & p[6])
            else:
                corners = ~(p[0] & p[2] & p[4]) & ~(p[2] & p[4] & p[6])
            remove = mask[1:-1, 1:-1] & (neighbours >= 2) & (neighbours <= 6) & (crossings == 1) & corners
            if remove.any():
                mask[1:-1, 1:-1][remove] = False
                changed = True
        if not changed:
            break
    ys, xs = np.where(mask)
    return np.column_stack([xs, ys]).astype(float)


def build_page_points(chars: list[str], allocations: list[int], font_path: str,
                      base_height: float = DEFAULT_BASE_HEIGHT,
                      letter_spacing: float = 0.16,
                      min_separation: float = DEFAULT_MIN_SEPARATION,
                      max_radius: float = DEFAULT_MAX_RADIUS,
                      size: int = DEFAULT_RENDER_SIZE,
                      index: int | None = None,
                      stroke: bool = False) -> tuple[np.ndarray, dict[str, Any]]:
    if len(chars) != len(allocations):
        raise CJKError("E_BAD_ALLOCATION", "chars and allocations must have equal length")
    if not chars:
        raise CJKError("E_BAD_SPEC", "page must contain at least one character")
    glyph_points: list[np.ndarray] = []
    widths: list[float] = []
    for char, count in zip(chars, allocations):
        local = sample_glyph_points(char, int(count), font_path, size, index, stroke)
        width = float(local[:, 0].max() - local[:, 0].min()) if len(local) else 0.0
        glyph_points.append(local)
        widths.append(max(width, 1e-6))
    total_width = sum(widths) + letter_spacing * (len(chars) - 1)
    cursor = -total_width / 2.0
    page = []
    for local, width in zip(glyph_points, widths):
        centre = cursor + width / 2.0
        if len(local):
            shifted = local.copy()
            shifted[:, 0] += centre
            page.append(shifted)
        cursor += width + letter_spacing
    points = np.vstack(page) if page else np.zeros((0, 2), dtype=float)
    points = points * float(base_height)

    for _ in range(6):
        distance = min_pairwise_distance(points)
        if distance is None or distance >= min_separation:
            break
        factor = min_separation / distance * (1 + 1e-9)
        points = points * factor
    distance = min_pairwise_distance(points)
    radius = float(np.max(np.linalg.norm(points, axis=1))) if len(points) else 0.0
    if distance is None or distance < min_separation:
        raise CJKError("E_GLYPH_ALLOCATION",
                       "could not reach minimum separation for page")
    if radius > max_radius:
        raise CJKError("E_GLYPH_ALLOCATION",
                       "page exceeds maximum radius " + str(max_radius)
                       + " m after separation scaling")
    meta = {
        "glyphs": len(chars),
        "points": int(len(points)),
        "min_distance": round(float(distance), 6),
        "max_radius": round(radius, 6),
        "base_height": float(base_height),
    }
    return points, meta
