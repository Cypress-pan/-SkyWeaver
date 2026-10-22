"""Tests for CJK text_show analysis, pagination and multi-segment plans."""
from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from droneshow import cjk  # noqa: E402
from droneshow.cjk import CJKError  # noqa: E402
from droneshow.formations import build_formation  # noqa: E402
from droneshow.textshow import (  # noqa: E402
    analyze_text,
    build_text_show_plan,
    paginate_text,
)
from droneshow.validator import validate_plan  # noqa: E402


def grid_start(num_drones: int) -> np.ndarray:
    rows = 8 if num_drones >= 64 else 4
    cols = 8 if num_drones >= 64 else 4
    points = np.array(build_formation(num_drones, {
        "type": "grid", "rows": rows, "cols": cols,
        "spacing": 0.6, "z": 0.1})["points"], dtype=float)
    return points


class TextShowUnitTests(unittest.TestCase):
    def test_analyze_and_paginate_64(self) -> None:
        analysis = analyze_text("卷卷")
        self.assertEqual(analysis["characters"], ["卷", "卷"])
        page_info = paginate_text(analysis["characters"], 64,
                                  auto_allocate=True, min_points_per_char=16,
                                  complexities=[100, 100])
        self.assertEqual(page_info["strategy"], "single_page")
        self.assertEqual(len(page_info["pages"]), 1)
        self.assertEqual(page_info["pages"][0]["allocations"], [32, 32])

    def test_paginate_16(self) -> None:
        page_info = paginate_text(["卷", "卷"], 16,
                                  auto_allocate=True, min_points_per_char=16,
                                  complexities=[100, 100])
        self.assertEqual(page_info["strategy"], "auto_paginate")
        self.assertEqual(len(page_info["pages"]), 2)
        self.assertEqual([page["allocations"] for page in page_info["pages"]],
                         [[16], [16]])

    def test_text_show_plan_64(self) -> None:
        plan = build_text_show_plan(
            grid_start(64), 64, "卷卷", 4.0, 1.5, True, 1, b"secret",
            points_per_char=32, gradient=["#55d6ff", "#8b7cff"],
            plan_id="p_text64")
        result = validate_plan(plan)
        self.assertTrue(result["valid"], result["issues"])
        self.assertEqual([segment["name"] for segment in plan["segments"]],
                         ["transition", "hold", "landing"])
        self.assertEqual(plan["_text_show"]["pagination"]["strategy"], "single_page")
        self.assertEqual(len(plan["colors"]), 64)
        self.assertEqual(plan["colors"][0], "#55d6ff")
        self.assertEqual(plan["colors"][-1], "#8b7cff")

    def test_text_show_plan_16_paginates(self) -> None:
        plan = build_text_show_plan(
            grid_start(16), 16, "卷卷", 3.0, 1.0, False, 1, b"secret",
            points_per_char=32, color="#00ffaa", plan_id="p_text16")
        result = validate_plan(plan)
        self.assertTrue(result["valid"], result["issues"])
        self.assertEqual([segment["name"] for segment in plan["segments"]],
                         ["transition", "hold", "transition", "hold"])
        pagination = plan["_text_show"]["pagination"]
        self.assertEqual(pagination["strategy"], "auto_paginate")
        self.assertEqual(len(pagination["pages"]), 2)

    def test_font_unavailable(self) -> None:
        with mock.patch.dict(os.environ, {"DRONE_SHOW_CJK_FONT": "/no/such/font.ttf"}):
            with mock.patch.object(cjk, "DEFAULT_FONT_CANDIDATES", []):
                with self.assertRaises(CJKError) as context:
                    cjk.resolve_font_path()
        self.assertEqual(context.exception.code, "E_FONT_UNAVAILABLE")


if __name__ == "__main__":
    unittest.main()
