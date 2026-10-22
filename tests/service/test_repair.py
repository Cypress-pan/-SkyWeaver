"""Tests for HELLO -> JJ collision-repair via-route planning."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from droneshow.formations import build_formation  # noqa: E402
from droneshow.planner import build_plan  # noqa: E402
from droneshow.repair import _via_candidates, build_plan_with_repair  # noqa: E402
from droneshow.validator import validate_plan  # noqa: E402


class RepairPlannerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.hello = build_formation(64, {"type": "text", "text": "HELLO", "z": 1.4})
        self.jj = build_formation(64, {"type": "text", "text": "JJ",
                                       "z": 1.4, "color": "#00ffaa",
                                       "gradient": ["#ff0000", "#0000ff"]})
        self.start = np.array(self.hello["points"], dtype=float)

    def test_direct_hello_jj_fails_min_separation(self) -> None:
        direct = build_plan(self.start, 64, self.jj, 4.0, 2.0, True, 1,
                            b"secret", plan_id="p_direct")
        result = validate_plan(direct)
        self.assertFalse(result["valid"])
        self.assertTrue(result["issues"])
        self.assertTrue(all(issue.startswith("E_MIN_SEPARATION")
                            for issue in result["issues"]))
        self.assertLess(result["metrics"]["min_distance"], 0.25)

    def test_via_repair_validates(self) -> None:
        repaired = build_plan_with_repair(self.start, 64, self.jj, 4.0, 2.0,
                                          True, 1, b"secret", plan_id="p_repair")
        result = validate_plan(repaired)
        self.assertTrue(result["valid"], result["issues"])
        self.assertGreaterEqual(result["metrics"]["min_distance"], 0.25)
        self.assertTrue(repaired.get("_repaired"))
        self.assertEqual([segment["name"] for segment in repaired["segments"]],
                         ["transition", "transition", "hold", "landing"])
        repair = repaired.get("repair") or {}
        self.assertEqual(repair.get("via_type"), "grid")
        self.assertGreater(repaired["duration"], 8.5)

    def test_repair_preserves_gradient_colors(self) -> None:
        repaired = build_plan_with_repair(self.start, 64, self.jj, 4.0, 1.0,
                                          False, 1, b"secret", plan_id="p_repair_color")
        self.assertTrue(validate_plan(repaired)["valid"])
        self.assertEqual(repaired["color"], "#00ffaa")
        for segment in repaired["segments"]:
            self.assertEqual(len(segment["colors"]), 64)
        self.assertIn("#ff0000", repaired["segments"][0]["colors"])
        self.assertIn("#0000ff", repaired["segments"][0]["colors"])

    def test_direct_valid_plan_is_not_repaired(self) -> None:
        wide = build_formation(64, {"type": "grid", "rows": 8, "cols": 8,
                                    "spacing": 0.8, "z": 0.1})
        plan = build_plan_with_repair(self.start, 64, wide, 4.0, 2.0, False,
                                      1, b"secret", plan_id="p_valid")
        self.assertFalse(plan.get("_repaired", False))
        self.assertTrue(validate_plan(plan)["valid"])
        self.assertEqual(plan["plan_id"], "p_valid")

    def test_via_candidates_are_deterministic_and_safe(self) -> None:
        candidates = _via_candidates(64)
        self.assertEqual(len(candidates), 2)
        self.assertEqual(candidates[0]["type"], "grid")
        self.assertEqual(candidates[0]["resource_report"]["used"], 64)
        self.assertEqual(candidates[1]["type"], "ring")
        self.assertGreaterEqual(candidates[0]["resource_report"]["min_distance"], 0.25)
        self.assertGreaterEqual(candidates[1]["resource_report"]["min_distance"], 0.25)


if __name__ == "__main__":
    unittest.main()
