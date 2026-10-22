"""Deterministic formation generation for the drone-show service.

All formations are pure functions of the requested spec and drone count so that
the same request always produces the same formation id and points.  Positions
are metres in an ENU-like frame (x east, y north, z up).
"""
from __future__ import annotations

import hashlib
import json
import math
from typing import Any

import numpy as np

from .colors import ColorError, parse_color_spec


class FormationError(ValueError):
    """Raised when a formation specification cannot be honoured."""

    def __init__(self, code: str, detail: str, **extra: Any):
        super().__init__(detail)
        self.code = code
        self.detail = detail
        self.extra = extra


def canonical_json(obj: Any) -> str:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), allow_nan=False)


def short_hash(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()[:12]


def min_pairwise_distance(points: Any) -> float | None:
    """Minimum Euclidean distance between any two distinct points."""
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or len(pts) < 2:
        return None
    best = math.inf
    for i in range(len(pts) - 1):
        d = np.linalg.norm(pts[i + 1:] - pts[i], axis=1)
        m = float(np.min(d))
        if m < best:
            best = m
    return best


def min_pairwise_distance_fast(points: Any) -> float | None:
    """Minimum pairwise distance via a KD-tree nearest-neighbour query.

    Returns the same value as :func:`min_pairwise_distance` but runs in
    O(n log n) per call, which matters when validating large fleets frame by
    frame.  Requires scipy (already a planner dependency).
    """
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or len(pts) < 2:
        return None
    if len(pts) < 64:
        return min_pairwise_distance(pts)
    from scipy.spatial import cKDTree

    tree = cKDTree(pts)
    nearest = tree.query(pts, k=2)[0][:, 1]
    return float(np.min(nearest))


def _ensure_separation(points: np.ndarray, min_sep: float) -> np.ndarray:
    """Scale a planar set about its centroid until it is sufficiently spread."""
    pts = np.array(points, dtype=float, copy=True)
    current = min_pairwise_distance(pts)
    if current is None or current >= min_sep or current <= 0.0:
        return pts
    centre = pts[:, :2].mean(axis=0)
    factor = min_sep / current
    pts[:, :2] = centre + (pts[:, :2] - centre) * factor
    return pts


def _grid_xy(rows: int, cols: int, spacing: float) -> np.ndarray:
    ys = (np.arange(rows, dtype=float) - (rows - 1) / 2.0) * spacing
    xs = (np.arange(cols, dtype=float) - (cols - 1) / 2.0) * spacing
    return np.array([[x, y] for y in ys for x in xs], dtype=float)


def _ring_default_radius(num_drones: int, min_chord: float = 0.45) -> float:
    if num_drones < 2:
        return 0.35
    return max(0.60, min_chord / (2.0 * math.sin(math.pi / num_drones)))


def _ring_xy(num_drones: int, radius: float, phase: float) -> np.ndarray:
    if num_drones == 1:
        return np.zeros((1, 2), dtype=float)
    angles = phase + 2.0 * math.pi * np.arange(num_drones, dtype=float) / num_drones
    return np.column_stack([radius * np.cos(angles), radius * np.sin(angles)])


def _heart_curve(t: np.ndarray) -> np.ndarray:
    x = 16.0 * np.sin(t) ** 3
    y = (13.0 * np.cos(t) - 5.0 * np.cos(2.0 * t)
         - 2.0 * np.cos(3.0 * t) - np.cos(4.0 * t))
    return np.column_stack([x, y])


def _resample_closed_curve(curve_fn, num_points: int, phase_fraction: float = 0.0,
                           samples: int = 4096) -> np.ndarray:
    t = np.linspace(0.0, 2.0 * math.pi, samples, endpoint=False)
    pts = curve_fn(t)
    pts = np.vstack([pts, pts[:1]])
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    total = cum[-1]
    goals = ((phase_fraction + np.arange(num_points, dtype=float) / num_points) % 1.0) * total
    x = np.interp(goals, cum, pts[:, 0])
    y = np.interp(goals, cum, pts[:, 1])
    return np.column_stack([x, y])


def _star_anchor_vertices(outer_radius: float, inner_ratio: float = 0.45,
                          points_count: int = 5) -> np.ndarray:
    inner_radius = outer_radius * inner_ratio
    verts: list[list[float]] = []
    for k in range(points_count):
        a_out = math.pi / 2.0 + 2.0 * math.pi * k / points_count
        a_in = a_out + math.pi / points_count
        verts.append([outer_radius * math.cos(a_out), outer_radius * math.sin(a_out)])
        verts.append([inner_radius * math.cos(a_in), inner_radius * math.sin(a_in)])
    return np.asarray(verts, dtype=float)


def _resample_closed_polyline(vertices: np.ndarray, num_points: int,
                              phase_fraction: float = 0.0) -> np.ndarray:
    pts = np.vstack([vertices, vertices[:1]])
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    total = cum[-1]
    goals = ((phase_fraction + np.arange(num_points, dtype=float) / num_points) % 1.0) * total
    out = np.empty((num_points, 2), dtype=float)
    for axis in range(2):
        out[:, axis] = np.interp(goals, cum, pts[:, axis])
    return out


def build_formation(num_drones: int, spec: dict[str, Any]) -> dict[str, Any]:
    """Build a formation object suitable for the HTTP response."""
    if not isinstance(spec, dict):
        raise FormationError("E_BAD_SPEC", "spec must be an object")
    try:
        n = int(num_drones)
    except (TypeError, ValueError) as exc:
        raise FormationError("E_BAD_SPEC", "num_drones must be an integer") from exc
    if n < 1:
        raise FormationError("E_BAD_SPEC", "num_drones must be positive")
    if n > 256:
        raise FormationError("E_BAD_SPEC", "num_drones must be <= 256")

    prebuilt_points = None
    warnings: list[dict[str, Any]] = []
    required_points: int | None = None

    ftype = str(spec.get("type", "")).strip().lower()
    if ftype == "grid":
        rows = int(spec.get("rows", max(1, math.ceil(math.sqrt(n)))))
        cols = int(spec.get("cols", max(1, math.ceil(n / rows))))
        spacing = float(spec.get("spacing", 0.6))
        z = float(spec.get("z", 1.2))
        if rows < 1 or cols < 1:
            raise FormationError("E_BAD_SPEC", "grid rows and cols must be positive")
        if spacing <= 0.0:
            raise FormationError("E_BAD_SPEC", "grid spacing must be positive")
        capacity = rows * cols
        used = min(capacity, n)
        xy = _grid_xy(rows, cols, spacing)[:used]
        num_targets = used
        idle = n - used

    elif ftype == "ring":
        radius_given = spec.get("radius")
        radius = float(radius_given) if radius_given is not None else _ring_default_radius(n)
        phase = float(spec.get("phase", 0.0))
        z = float(spec.get("z", 1.2))
        if radius < 0.0:
            raise FormationError("E_BAD_SPEC", "ring radius must be non-negative")
        xy = _ring_xy(n, radius, phase)
        num_targets = n
        idle = 0

    elif ftype == "heart":
        z = float(spec.get("z", 1.2))
        phase = float(spec.get("phase", 0.0))
        if "size" in spec:
            size = float(spec["size"])
            if size <= 0:
                raise FormationError("E_BAD_SPEC", "heart size must be positive")
            auto = False
        else:
            size = max(2.4, 0.16 * n)
            auto = True
        xy = _resample_closed_curve(_heart_curve, n, phase_fraction=phase)
        width = float(np.ptp(xy[:, 0]))
        xy = xy * (size / width if width > 0 else 1.0)
        xy -= xy.min(axis=0) + 0.5 * (xy.max(axis=0) - xy.min(axis=0))
        if auto:
            xy = _ensure_separation(xy, 0.35)
        num_targets = n
        idle = 0

    elif ftype == "star":
        z = float(spec.get("z", 1.2))
        phase = float(spec.get("phase", 0.0))
        if "radius" in spec:
            outer = float(spec["radius"])
            auto = False
            if outer <= 0:
                raise FormationError("E_BAD_SPEC", "star radius must be positive")
        else:
            outer = max(1.3, 0.5 / (2.0 * math.sin(math.pi / max(n, 5))))
            auto = True
        inner_ratio = float(spec.get("inner_ratio", 0.45))
        if not 0.0 < inner_ratio < 1.0:
            raise FormationError("E_BAD_SPEC", "star inner_ratio must be in (0,1)")
        verts = _star_anchor_vertices(outer, inner_ratio)
        xy = _resample_closed_polyline(verts, n, phase_fraction=phase)
        if auto:
            xy = _ensure_separation(xy, 0.35)
        num_targets = n
        idle = 0

    elif ftype == "text":
        text = str(spec.get("text", "")).strip().upper()
        if not text:
            raise FormationError("E_BAD_SPEC", "text must be a non-empty string")
        unsupported = sorted(set(text) - _TEXT_CHARS)
        if unsupported:
            raise FormationError("E_BAD_SPEC",
                                 "unsupported text character(s): "
                                 + ", ".join(repr(char) for char in unsupported))
        prebuilt_points, required_points, warnings = _text_formation_points(text, n, spec)
        num_targets = n
        idle = 0
        z = float(spec.get("z", 1.2))

    else:
        raise FormationError("E_BAD_SPEC", "unsupported formation type " + repr(ftype))

    if prebuilt_points is not None:
        points = np.asarray(prebuilt_points, dtype=float)
    else:
        points = np.column_stack([xy, np.full(num_targets, z, dtype=float)])
    min_dist = min_pairwise_distance(points)
    try:
        color_spec = parse_color_spec(spec.get("color"), spec.get("colors"),
                                      spec.get("gradient"), num_targets)
    except ColorError as exc:
        raise FormationError(exc.code, exc.detail, **exc.extra) from exc
    if required_points is None:
        required_points = int(num_targets)
    canonical = {
        "type": ftype,
        "num_drones": n,
        "points": np.round(points, 9).tolist(),
        "color": color_spec["color"],
        "colors": color_spec["colors"],
        "gradient": color_spec["gradient"],
    }
    formation_id = "f_" + short_hash(canonical_json(canonical))
    report = {
        "used": int(num_targets),
        "idle": int(idle),
        "min_distance": None if min_dist is None else round(float(min_dist), 6),
    }
    return {
        "formation_id": formation_id,
        "type": ftype,
        "num_drones": n,
        "points": np.round(points, 6).tolist(),
        "color": color_spec["color"],
        "colors": color_spec["colors"],
        "gradient": color_spec["gradient"],
        "required_points": int(required_points),
        "warnings": warnings,
        "resource_report": report,
    }


FONT_HEIGHT = 7
FONT_WIDTH = 5

_FONT_ROWS = {
    'A': ('.###.', '#...#', '#...#', '#####', '#...#', '#...#', '#...#'),
    'B': ('####.', '#...#', '#...#', '####.', '#...#', '#...#', '####.'),
    'C': ('.###.', '#...#', '#....', '#....', '#....', '#...#', '.###.'),
    'D': ('####.', '#...#', '#...#', '#...#', '#...#', '#...#', '####.'),
    'E': ('#####', '#....', '#....', '####.', '#....', '#....', '#####'),
    'F': ('#####', '#....', '#....', '####.', '#....', '#....', '#....'),
    'G': ('.###.', '#...#', '#....', '#.###', '#...#', '#...#', '.###.'),
    'H': ('#...#', '#...#', '#...#', '#####', '#...#', '#...#', '#...#'),
    'I': ('#####', '..#..', '..#..', '..#..', '..#..', '..#..', '#####'),
    'J': ('..###', '...#.', '...#.', '...#.', '...#.', '#..#.', '.##..'),
    'K': ('#...#', '#..#.', '#.#..', '##...', '#.#..', '#..#.', '#...#'),
    'L': ('#....', '#....', '#....', '#....', '#....', '#....', '#####'),
    'M': ('#...#', '##.##', '#.#.#', '#...#', '#...#', '#...#', '#...#'),
    'N': ('#...#', '##..#', '#.#.#', '#..##', '#...#', '#...#', '#...#'),
    'O': ('.###.', '#...#', '#...#', '#...#', '#...#', '#...#', '.###.'),
    'P': ('####.', '#...#', '#...#', '####.', '#....', '#....', '#....'),
    'Q': ('.###.', '#...#', '#...#', '#...#', '#.#.#', '#..#.', '.##.#'),
    'R': ('####.', '#...#', '#...#', '####.', '#.#..', '#..#.', '#...#'),
    'S': ('.####', '#....', '#....', '.###.', '....#', '....#', '####.'),
    'T': ('#####', '..#..', '..#..', '..#..', '..#..', '..#..', '..#..'),
    'U': ('#...#', '#...#', '#...#', '#...#', '#...#', '#...#', '.###.'),
    'V': ('#...#', '#...#', '#...#', '#...#', '#...#', '.#.#.', '..#..'),
    'W': ('#...#', '#...#', '#...#', '#.#.#', '#.#.#', '##.##', '#...#'),
    'X': ('#...#', '#...#', '.#.#.', '..#..', '.#.#.', '#...#', '#...#'),
    'Y': ('#...#', '#...#', '.#.#.', '..#..', '..#..', '..#..', '..#..'),
    'Z': ('#####', '....#', '...#.', '..#..', '.#...', '#....', '#####'),
    ' ': ('.....', '.....', '.....', '.....', '.....', '.....', '.....'),
    '0': ('.###.', '#...#', '#..##', '#.#.#', '##..#', '#...#', '.###.'),
    '1': ('..#..', '.##..', '..#..', '..#..', '..#..', '..#..', '#####'),
    '2': ('.###.', '#...#', '....#', '...#.', '..#..', '.#...', '#####'),
    '3': ('#####', '...#.', '..#..', '...#.', '....#', '#...#', '.###.'),
    '4': ('...#.', '..##.', '.#.#.', '#..#.', '#####', '...#.', '...#.'),
    '5': ('#####', '#....', '####.', '....#', '....#', '#...#', '.###.'),
    '6': ('.###.', '#...#', '#....', '####.', '#...#', '#...#', '.###.'),
    '7': ('#####', '....#', '...#.', '..#..', '.#...', '.#...', '.#...'),
    '8': ('.###.', '#...#', '#...#', '.###.', '#...#', '#...#', '.###.'),
    '9': ('.###.', '#...#', '#...#', '.####', '....#', '#...#', '.###.'),
    '!': ('..#..', '..#..', '..#..', '..#..', '..#..', '.....', '..#..'),
    '?': ('.###.', '#...#', '....#', '...#.', '..#..', '.....', '..#..'),
    '-': ('.....', '.....', '.....', '#####', '.....', '.....', '.....'),
    '.': ('.....', '.....', '.....', '.....', '.....', '.##..', '.##..'),
    ':': ('.....', '.##..', '.##..', '.....', '.##..', '.##..', '.....'),
    ';': ('.....', '.##..', '.##..', '.....', '.##..', '.#...', '#....'),
}

def _font_bits(pattern_rows):
    pattern = "".join(pattern_rows)
    return tuple(int(pattern[index:index + 5].replace('#', '1').replace('.', '0'), 2)
                 for index in range(0, 35, 5))

_FONT_5X7 = {key: _font_bits(value) for key, value in _FONT_ROWS.items()}
_TEXT_CHARS = set(_FONT_5X7)


def _text_lit_cells(text, letter_spacing):
    lit = set()
    x_offset = 0
    for char in text:
        rows = _FONT_5X7.get(char)
        if rows is None:
            raise FormationError("E_BAD_SPEC", "unsupported text character " + repr(char))
        for row_index, bits in enumerate(rows):
            for column in range(FONT_WIDTH):
                if bits & (1 << (FONT_WIDTH - 1 - column)):
                    lit.add((x_offset + column, row_index))
        x_offset += FONT_WIDTH + letter_spacing
    width_cells = len(text) * FONT_WIDTH + max(0, len(text) - 1) * letter_spacing
    return lit, width_cells


def _text_candidates(text, letter_spacing, cell_scale, spacing):
    lit, width_cells = _text_lit_cells(text, letter_spacing)
    if not lit:
        raise FormationError("E_BAD_SPEC", "text has no visible glyphs")
    coordinates = set()
    for cell_x, cell_y in lit:
        for offset_x in range(cell_scale + 1):
            for offset_y in range(cell_scale + 1):
                coordinates.add((cell_x * cell_scale + offset_x,
                                 cell_y * cell_scale + offset_y))
    height_cells = FONT_HEIGHT
    points = []
    for lattice_x, lattice_y in sorted(coordinates):
        x = (lattice_x - width_cells * cell_scale / 2.0) * spacing
        y = (lattice_y - height_cells * cell_scale / 2.0) * spacing
        points.append((x, y))
    return np.asarray(points, dtype=float)


def _farthest_point_select(points, count):
    if count <= 0:
        return points[:0]
    if count >= len(points):
        return points.copy()
    centre = points.mean(axis=0)
    first = int(np.argmin(np.linalg.norm(points - centre, axis=1)))
    selected = [first]
    best = np.linalg.norm(points - points[first], axis=1)
    for _ in range(count - 1):
        index = int(np.argmax(best))
        selected.append(index)
        distance = np.linalg.norm(points - points[index], axis=1)
        best = np.minimum(best, distance)
    return points[selected]


def _text_formation_points(text, num_drones, spec):
    try:
        min_points_per_char = int(spec.get("min_points_per_char", 8))
    except (TypeError, ValueError) as exc:
        raise FormationError("E_BAD_SPEC", "min_points_per_char must be an integer") from exc
    if min_points_per_char < 1:
        raise FormationError("E_BAD_SPEC", "min_points_per_char must be positive")
    allow_sparse = bool(spec.get("allow_sparse", False))
    try:
        letter_spacing = int(spec.get("letter_spacing", 1))
    except (TypeError, ValueError) as exc:
        raise FormationError("E_BAD_SPEC", "letter_spacing must be an integer") from exc
    if letter_spacing < 0:
        raise FormationError("E_BAD_SPEC", "letter_spacing must be non-negative")

    _, width_cells = _text_lit_cells(text, letter_spacing)
    max_safe_spacing = max(0.30, min(2.0, 9.6 / max(1, width_cells)))
    spacing_warnings: list[dict[str, Any]] = []
    if "point_spacing" in spec:
        try:
            requested_spacing = float(spec["point_spacing"])
        except (TypeError, ValueError) as exc:
            raise FormationError("E_BAD_SPEC", "point_spacing must be a number") from exc
        spacing = requested_spacing
        if spacing < 0.30:
            spacing = 0.30
            spacing_warnings.append({
                "code": "E_POINT_SPACING_CLAMPED",
                "requested_point_spacing": requested_spacing,
                "used_point_spacing": spacing,
                "detail": "point_spacing was raised to preserve validator spacing during transitions",
            })
        if spacing > max_safe_spacing:
            spacing = max_safe_spacing
            spacing_warnings.append({
                "code": "E_POINT_SPACING_CLAMPED",
                "requested_point_spacing": requested_spacing,
                "used_point_spacing": spacing,
                "detail": "point_spacing was lowered to keep x/y inside validator bounds",
            })
    elif "size" in spec:
        try:
            requested_size = float(spec["size"])
        except (TypeError, ValueError) as exc:
            raise FormationError("E_BAD_SPEC", "size must be a number") from exc
        if requested_size <= 0.0:
            raise FormationError("E_BAD_SPEC", "size must be positive")
        desired_spacing = requested_size / max(1, width_cells)
        spacing = min(max(desired_spacing, 0.30), max_safe_spacing)
        if abs(spacing - desired_spacing) > 1e-12:
            spacing_warnings.append({
                "code": "E_SIZE_CLAMPED",
                "requested_size": requested_size,
                "used_size": round(spacing * width_cells, 6),
                "used_point_spacing": round(spacing, 6),
                "detail": "size was clamped to keep the text inside validator bounds",
            })
    else:
        spacing = 0.30
        if spacing > max_safe_spacing:
            spacing = max_safe_spacing

    glyph_count = sum(1 for char in text if char != " ")
    required_points = max(1, glyph_count) * min_points_per_char
    if num_drones < required_points and not allow_sparse:
        raise FormationError(
            "E_POINT_BUDGET",
            "text needs at least " + str(required_points) + " points",
            required_points=required_points,
            available_points=num_drones,
            min_points_per_char=min_points_per_char,
            text=text,
        )

    cell_scale = 1
    candidates = _text_candidates(text, letter_spacing, cell_scale, spacing)
    while len(candidates) < num_drones and cell_scale < 8:
        cell_scale += 1
        candidates = _text_candidates(text, letter_spacing, cell_scale, spacing)
    if len(candidates) < num_drones:
        raise FormationError(
            "E_POINT_BUDGET",
            "text cannot host " + str(num_drones) + " points without exceeding the text bounds",
            required_points=required_points,
            available_points=len(candidates),
            min_points_per_char=min_points_per_char,
            text=text,
        )
    selected = _farthest_point_select(candidates, num_drones)
    z = float(spec.get("z", 1.2))
    points = np.column_stack([selected, np.full(num_drones, z, dtype=float)])
    warnings = list(spacing_warnings)
    if num_drones < required_points:
        warnings.append({
            "code": "E_POINT_BUDGET_SPARSE",
            "required_points": required_points,
            "used_points": num_drones,
            "min_points_per_char": min_points_per_char,
            "detail": "allow_sparse=true accepted a sparse text formation",
        })
    return points, required_points, warnings
