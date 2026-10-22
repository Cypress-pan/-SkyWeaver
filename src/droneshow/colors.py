"""Deterministic color parsing and interpolation for formations and plans."""
from __future__ import annotations

from typing import Any, Iterable

DEFAULT_COLOR = "#ffffff"


class ColorError(ValueError):
    def __init__(self, code: str, detail: str, **extra: Any):
        super().__init__(detail)
        self.code = code
        self.detail = detail
        self.extra = extra


def normalize_hex(value: Any) -> str:
    if not isinstance(value, str):
        raise ColorError("E_BAD_COLOR", "color must be a hex string")
    text = value.strip()
    if text.startswith("#"):
        text = text[1:]
    if len(text) == 3:
        text = "".join(ch * 2 for ch in text)
    if len(text) != 6:
        raise ColorError("E_BAD_COLOR", "invalid hex color: " + repr(value))
    try:
        int(text, 16)
    except ValueError as exc:
        raise ColorError("E_BAD_COLOR", "invalid hex color: " + repr(value)) from exc
    return "#" + text.lower()


def hex_to_rgb(value: str) -> tuple[int, int, int]:
    text = normalize_hex(value)[1:]
    return int(text[0:2], 16), int(text[2:4], 16), int(text[4:6], 16)


def rgb_to_hex(rgb: Iterable[int]) -> str:
    return "#" + "".join(f"{max(0, min(255, int(round(c)))):02x}" for c in rgb)


def interpolate_gradient(pair: Any, num_points: int) -> list[str]:
    if not isinstance(pair, (list, tuple)) or len(pair) != 2:
        raise ColorError("E_BAD_COLOR", "gradient must be a two-element color pair")
    first = normalize_hex(pair[0])
    second = normalize_hex(pair[1])
    if num_points <= 0:
        return []
    if num_points == 1:
        return [first]
    a = hex_to_rgb(first)
    b = hex_to_rgb(second)
    out: list[str] = []
    for index in range(num_points):
        t = index / (num_points - 1)
        out.append(rgb_to_hex((
            a[0] + (b[0] - a[0]) * t,
            a[1] + (b[1] - a[1]) * t,
            a[2] + (b[2] - a[2]) * t,
        )))
    return out


def lerp_hex(first: str, second: str, t: float) -> str:
    a = hex_to_rgb(normalize_hex(first))
    b = hex_to_rgb(normalize_hex(second))
    clamped = max(0.0, min(1.0, float(t)))
    return rgb_to_hex((
        a[0] + (b[0] - a[0]) * clamped,
        a[1] + (b[1] - a[1]) * clamped,
        a[2] + (b[2] - a[2]) * clamped,
    ))


def expand_colors(colors: Any, num_points: int) -> list[str]:
    if not isinstance(colors, (list, tuple)) or len(colors) == 0:
        raise ColorError("E_BAD_COLOR", "colors must be a non-empty array")
    normalized = [normalize_hex(color) for color in colors]
    if num_points <= 0:
        return []
    return [normalized[index % len(normalized)] for index in range(num_points)]


def parse_color_spec(color: Any = None, colors: Any = None, gradient: Any = None,
                     num_points: int = 0, default: str = DEFAULT_COLOR) -> dict[str, Any]:
    """Return a normalized color description.

    Precedence: an explicit ``colors`` array wins over ``gradient``; a solid
    ``color`` is kept alongside per-point colors as a fallback.
    """
    out: dict[str, Any] = {"color": None, "colors": None, "gradient": None}
    if color is not None:
        out["color"] = normalize_hex(color)
    if gradient is not None:
        out["gradient"] = [normalize_hex(gradient[0]), normalize_hex(gradient[1])]
        out["colors"] = interpolate_gradient(out["gradient"], num_points)
    if colors is not None:
        out["colors"] = expand_colors(colors, num_points)
    if out["color"] is None and out["colors"]:
        out["color"] = out["colors"][0]
    if out["color"] is None:
        out["color"] = normalize_hex(default)
    return out
