"""Full-fleet safety regression, real ground layout and six-act score."""
import sys
import unittest
from pathlib import Path
import numpy as np
from scipy.spatial.distance import pdist

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from droneshow.cinematic import ground_layout, build_cinematic_plan
from droneshow.safe_motion import continuous_separation
from droneshow.fleet import fleet_start_positions, build_fleet_timeline_plan


class CinematicTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ground, cls.batches = ground_layout()
        cls.plan = build_cinematic_plan(cls.ground)

    def test_ground_and_lift_fit_without_virtual_terraces(self):
        self.assertEqual(self.ground.shape, (1000, 3))
        np.testing.assert_array_equal(self.ground[:, 2], .1)
        self.assertLess(np.linalg.norm(self.ground[:, :2], axis=1).max(), 4.8)
        self.assertGreater(pdist(self.ground).min(), .279)
        lift = self.ground.copy()
        lift[:, 2] = .7+self.batches*.7
        self.assertGreater(pdist(lift).min(), .48)

    def test_complete_score_has_certified_transitions_and_full_fleet(self):
        plan = self.plan
        self.assertTrue(plan["_timeline"]["validation"]["valid"])
        for segment in plan["_segments"]:
            self.assertEqual(segment["start_positions"].shape, (1000, 3))
            self.assertGreaterEqual(continuous_separation(segment["start_positions"], segment["end_positions"]), .25)
        self.assertTrue(all(s["active"] == 1000 and s["idle"] == 0 for s in plan["_timeline"]["scenes"]))
        end = plan["_target_positions"]
        np.testing.assert_array_equal(end[:, 2], .1)
        np.testing.assert_allclose(np.sort_complex(end[:, 0]+1j*end[:, 1]),
                                   np.sort_complex(self.ground[:, 0]+1j*self.ground[:, 1]))
        names = " ".join(s["name"] for s in plan["_segments"])
        for act in ("苏醒", "AGNES", "双螺旋", "星河绽放", "千机光浪", "金色A", "落幕"):
            self.assertIn(act, names)
        self.assertEqual(plan["_timeline"]["scenes"][1]["foreground"], 85)

    def test_auto_word_also_works_in_generic_thousand_fleet_api(self):
        plan = build_fleet_timeline_plan(fleet_start_positions(1000), 1000, 20.,
               [{"type": "text", "text": "AGNES", "duration_sec": 20., "drone_budget": "auto"}],
               "kinematic", 1, b"test")
        self.assertTrue(plan["_timeline"]["validation"]["valid"])
        self.assertEqual(plan["_timeline"]["scenes"][0]["active"], 85)
        self.assertEqual(plan["_timeline"]["scenes"][0]["idle"], 915)


if __name__ == "__main__":
    unittest.main()
