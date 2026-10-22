"""Unit tests for deterministic formations, planning and validation."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from droneshow.formations import build_formation, min_pairwise_distance  # noqa: E402
from droneshow.planner import build_plan, public_plan, sample_plan_positions  # noqa: E402
from droneshow.validator import validate_plan  # noqa: E402


class FormationTests(unittest.TestCase):
    def test_grid_16(self) -> None:
        f = build_formation(16, {"type": "grid", "rows": 4, "cols": 4,
                                 "spacing": 0.6, "z": 1.2})
        self.assertEqual(len(f["points"]), 16)
        self.assertEqual(f["resource_report"]["used"], 16)
        self.assertEqual(f["resource_report"]["idle"], 0)
        self.assertAlmostEqual(f["resource_report"]["min_distance"], 0.6, places=6)

    def test_all_shapes_have_positive_separation(self) -> None:
        for shape in ("ring", "heart", "star"):
            f = build_formation(16, {"type": shape, "z": 1.2})
            self.assertEqual(len(f["points"]), 16)
            dist = min_pairwise_distance(f["points"])
            self.assertIsNotNone(dist)
            self.assertGreaterEqual(dist, 0.25)

    def test_formation_is_deterministic(self) -> None:
        spec = {"type": "ring", "radius": 1.1, "z": 1.2, "phase": 0.3}
        a = build_formation(16, spec)
        b = build_formation(16, spec)
        self.assertEqual(a["formation_id"], b["formation_id"])
        self.assertEqual(a["points"], b["points"])

    def test_grid_more_cells_than_drones_reports_idle(self) -> None:
        f = build_formation(5, {"type": "grid", "rows": 4, "cols": 4, "spacing": 0.6})
        self.assertEqual(f["resource_report"]["used"], 5)
        self.assertEqual(f["resource_report"]["idle"], 0)


class PlannerValidatorTests(unittest.TestCase):
    def _plan(self, plan_id: str = "p_test"):
        start = np.array(build_formation(16, {"type": "grid", "rows": 4, "cols": 4,
                                              "spacing": 0.6, "z": 1.2})["points"],
                         dtype=float)
        start[:, 2] = 0.1
        target = build_formation(16, {"type": "ring", "radius": 1.2, "z": 1.2})
        return build_plan(start, 16, target, 4.0, 2.0, True, 1,
                          b"secret", plan_id=plan_id)

    def test_plan_segments_and_hash(self) -> None:
        plan = self._plan()
        self.assertAlmostEqual(plan["duration"], 8.5, places=6)
        self.assertEqual([s["name"] for s in plan["segments"]],
                         ["takeoff", "transition", "hold", "landing"])
        same = self._plan(plan_id=plan["plan_id"])
        self.assertEqual(plan["plan_hash"], same["plan_hash"])
        self.assertEqual(plan["credential"], same["credential"])

    def test_plan_validates(self) -> None:
        result = validate_plan(self._plan())
        self.assertTrue(result["valid"], result["issues"])
        self.assertIsNotNone(result["credential"])
        self.assertGreaterEqual(result["metrics"]["min_distance"], 0.25)
        self.assertLessEqual(result["metrics"]["max_speed"], 2.0)
        self.assertLessEqual(result["metrics"]["max_accel"], 8.0)

    def test_fast_plan_is_rejected(self) -> None:
        start = np.array(build_formation(16, {"type": "grid"})["points"], dtype=float)
        start[:, 2] = 0.1
        target = build_formation(16, {"type": "ring", "radius": 1.2, "z": 1.2})
        plan = build_plan(start, 16, target, 0.2, 0.0, True, 1,
                          b"secret", plan_id="p_fast")
        result = validate_plan(plan)
        self.assertFalse(result["valid"])
        self.assertTrue(any(i.startswith("E_MAX_") for i in result["issues"]))

    def test_sample_rate_is_high_rate(self) -> None:
        plan = self._plan()
        times, positions = sample_plan_positions(plan, 200.0)
        self.assertGreater(len(times), 1500)
        self.assertEqual(positions.shape[1:], (16, 3))
        self.assertAlmostEqual(float(times[-1]), plan["duration"], places=6)


if __name__ == "__main__":
    unittest.main()
