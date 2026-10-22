import json
import sys
from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parents[2]


def near_count(image, target, tolerance=34, x_limit=None):
    pixels = image.load()
    width, height = image.size
    limit = width if x_limit is None else min(width, x_limit)
    count = 0
    for x in range(limit):
        for y in range(height):
            r, g, b = pixels[x, y][:3]
            if tolerance >= abs(r - target[0]) and tolerance >= abs(g - target[1]) and tolerance >= abs(b - target[2]):
                count += 1
    return count


def check(path, expected, label):
    image = Image.open(path).convert('RGB')
    canvas_limit = int(image.size[0] * 0.55)
    results = {}
    total = 0
    for name, target in expected.items():
        count = near_count(image, target, x_limit=canvas_limit)
        results[name] = count
        total += count
    ok = total > 0
    print(json.dumps({
        'file': str(path),
        'label': label,
        'canvas_pixel_limit_x': canvas_limit,
        'counts': results,
        'pass': ok,
    }, sort_keys=True))
    return ok


def main():
    active = check(
        ROOT / 'evidence/web-color-mock-active.png',
        {
            'actual_cyan_55d6ff': (85, 214, 255),
            'actual_violet_8b7cff': (139, 124, 255),
            'reference_amber_ffb038': (255, 176, 56),
        },
        'per-drone active_segment.colors',
    )
    solid = check(
        ROOT / 'evidence/web-color-mock-solid.png',
        {
            'actual_orange_ff8844': (255, 136, 68),
            'reference_amber_ffb038': (255, 176, 56),
        },
        'solid active_segment.color',
    )
    if active and solid:
        print('COLOR_PIXEL_CHECK_OK')
        return 0
    print('COLOR_PIXEL_CHECK_FAILED')
    return 1


if __name__ == '__main__':
    sys.exit(main())
