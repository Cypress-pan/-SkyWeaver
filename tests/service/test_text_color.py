"""Tests for the text formation and color pipeline."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from droneshow.formations import FormationError, build_formation, min_pairwise_distance  # noqa: E402
from droneshow.planner import build_plan, public_plan  # noqa: E402
from droneshow.validator import validate_plan  # noqa: E402


class TextFormationTests(unittest.TestCase):
    def test_hello_64(self) -> None:
        formation = build_formation(64, {"type": "text", "text": "HELLO",
                                         "z": 1.2, "color": "#00ffaa"})
        self.assertEqual(formation["type"], "text")
        self.assertEqual(len(formation["points"]), 64)
        self.assertEqual(formation["required_points"], 40)
        self.assertEqual(formation["resource_report"]["used"], 64)
        self.assertEqual(formation["resource_report"]["idle"], 0)
        self.assertGreaterEqual(formation["resource_report"]["min_distance"], 0.25)
        self.assertEqual(formation["warnings"], [])
        self.assertEqual(formation["color"], "#00ffaa")

    def test_hello_16_point_budget(self) -> None:
        with self.assertRaises(FormationError) as context:
            build_formation(16, {"type": "text", "text": "HELLO"})
        error = context.exception
        self.assertEqual(error.code, "E_POINT_BUDGET")
        self.assertEqual(error.extra.get("required_points"), 40)
        self.assertEqual(error.extra.get("available_points"), 16)

    def test_hello_16_allow_sparse_warns(self) -> None:
        formation = build_formation(16, {"type": "text", "text": "HELLO",
                                         "allow_sparse": True})
        self.assertEqual(len(formation["points"]), 16)
        self.assertTrue(formation["warnings"])
        self.assertEqual(formation["warnings"][0]["code"], "E_POINT_BUDGET_SPARSE")
        self.assertGreaterEqual(min_pairwise_distance(formation["points"]), 0.25)

    def test_size_is_clamped_with_warning(self) -> None:
        formation = build_formation(64, {"type": "text", "text": "HELLO",
                                         "size": 1.2, "z": 1.4})
        self.assertEqual(len(formation["points"]), 64)
        self.assertTrue(any(w["code"] == "E_SIZE_CLAMPED" for w in formation["warnings"]))
        self.assertGreaterEqual(formation["resource_report"]["min_distance"], 0.25)

    def test_unsupported_character_rejected(self) -> None:
        with self.assertRaises(FormationError) as context:
            build_formation(64, {"type": "text", "text": "HI@"})
        self.assertEqual(context.exception.code, "E_BAD_SPEC")

    def test_bad_color_rejected(self) -> None:
        with self.assertRaises(FormationError) as context:
            build_formation(16, {"type": "ring", "color": "not-a-color"})
        self.assertEqual(context.exception.code, "E_BAD_COLOR")


class ColorPipelineTests(unittest.TestCase):
    def test_gradient_expands_per_point(self) -> None:
        formation = build_formation(16, {"type": "ring",
                                         "gradient": ["#ff0000", "#0000ff"]})
        self.assertEqual(len(formation["colors"]), 16)
        self.assertEqual(formation["colors"][0], "#ff0000")
        self.assertEqual(formation["colors"][-1], "#0000ff")
        self.assertEqual(formation["color"], "#ff0000")

    def test_solid_color_and_plan_propagation(self) -> None:
        start = np.array(build_formation(16, {"type": "grid"})["points"], dtype=float)
        start[:, 2] = 0.1
        target = build_formation(16, {"type": "ring", "color": "#00ff00"})
        plan = build_plan(start, 16, target, 4.0, 1.0, False, 1,
                          b"secret", plan_id="p_color")
        for segment in plan["segments"]:
            self.assertEqual(segment["color"], "#00ff00")
        self.assertTrue(all(color == "#00ff00" for color in plan["segments"][0]["colors"]))
        public = public_plan(plan)
        self.assertEqual(public["color"], "#00ff00")
        self.assertEqual(public["segments"][0]["color"], "#00ff00")

    def test_plan_color_overrides(self) -> None:
        start = np.array(build_formation(16, {"type": "grid"})["points"], dtype=float)
        start[:, 2] = 0.1
        target = build_formation(16, {"type": "ring",
                                      "gradient": ["#ff0000", "#0000ff"]})
        plan = build_plan(start, 16, target, 4.0, 1.0, False, 1, b"secret",
                          plan_id="p_override", color_override="#123456")
        self.assertEqual(plan["segments"][0]["color"], "#123456")
        plan2 = build_plan(start, 16, target, 4.0, 1.0, False, 1, b"secret",
                           plan_id="p_override2",
                           colors_override=["#111111", "#eeeeee"])
        self.assertEqual(len(plan2["segments"][0]["colors"]), 16)
        self.assertEqual(plan2["segments"][0]["colors"][0], "#111111")
        self.assertEqual(plan2["segments"][0]["colors"][1], "#eeeeee")

    def test_hello_64_plan_validates(self) -> None:
        start = np.array(build_formation(64, {"type": "grid"})["points"], dtype=float)
        start[:, 2] = 0.1
        target = build_formation(64, {"type": "text", "text": "HELLO",
                                      "z": 1.4, "color": "#00ffaa"})
        plan = build_plan(start, 64, target, 6.0, 1.0, False, 1, b"secret",
                          plan_id="p_hello64")
        result = validate_plan(plan)
        self.assertTrue(result["valid"], result["issues"])
        self.assertGreaterEqual(result["metrics"]["min_distance"], 0.25)
        self.assertEqual(len(plan["colors"]), 64)


if __name__ == "__main__":
    unittest.main()
