"""Compare old bright-glow screenshot with the softened task-10 rendering."""
from __future__ import annotations

import json
import sys
from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
CANVAS_FRACTION = 0.55
WHITE_LEVEL = 235
BRIGHT_SUM = 690


def stats(path):
    image = Image.open(path).convert("RGB")
    width, height = image.size
    limit = int(width * CANVAS_FRACTION)
    pixels = image.load()
    total = 0
    white = 0
    bright = 0
    for x in range(limit):
        for y in range(height):
            r, g, b = pixels[x, y]
            total += 1
            if r >= WHITE_LEVEL and g >= WHITE_LEVEL and b >= WHITE_LEVEL:
                white += 1
            if r + g + b >= BRIGHT_SUM:
                bright += 1
    return {
        "file": str(path),
        "canvas_pixels": total,
        "near_white": white,
        "bright": bright,
        "near_white_pct": round(100.0 * white / total, 4),
        "bright_pct": round(100.0 * bright / total, 4),
    }


def main():
    before = stats(ROOT / "evidence/web-color-mock-active.png")
    after = stats(ROOT / "evidence/web-live-glow-soft.png")
    print(json.dumps({"old_bright_glow": before, "task10_soft_glow": after}, sort_keys=True))
    checks = {
        "no_near_white_core": after["near_white"] == 0,
        "fewer_bright_pixels": before["bright"] > after["bright"],
        "bright_reduction_pct": round(100.0 * (before["bright"] - after["bright"]) / before["bright"], 2),
    }
    print(json.dumps(checks, sort_keys=True))
    if checks["no_near_white_core"] and checks["fewer_bright_pixels"]:
        print("LIVE_PIXEL_CHECK_OK")
        return 0
    print("LIVE_PIXEL_CHECK_FAILED")
    return 1


if __name__ == "__main__":
    sys.exit(main())
