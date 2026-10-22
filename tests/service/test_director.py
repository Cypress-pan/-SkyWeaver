"""User-facing show behavior and continuous motion safety."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from droneshow.director import ShowError, build_show, ground_layout
from droneshow.planner import positions_at
from droneshow.safe_motion import continuous_separation, match_targets
from droneshow import choreography


class DirectorTests(unittest.TestCase):
    def test_complete_chinese_show_without_configuration(self):
        plan = build_show(ground_layout(128), [
            {"type": "text", "text": "卷卷"},
            {"type": "heart", "color": "#4285ff"},
        ], 1, b"secret")
        self.assertTrue(plan["_timeline"]["validation"]["valid"])
        self.assertEqual(plan["_timeline"]["scenes"][0]["active"], 96)
        self.assertEqual(plan["_segments"][0]["name"], "takeoff")
        self.assertEqual(plan["_segments"][-1]["name"], "landing")
        np.testing.assert_allclose(positions_at(plan, plan["duration"])[:, 2], .1)
        display = next(s for s in plan["_segments"] if s["name"] == "scene1:heart_none")
        self.assertEqual(display["colors"].count("#26334b"), 96)
        self.assertEqual(display["colors"].count("#4285ff"), 32)

    def test_full_rotation_moves_and_automatically_gets_safe_time(self):
        plan = build_show(ground_layout(128), [
            {"type": "ring", "effect": {"type": "rotate", "turns": 1}},
        ], 1, b"secret")
        self.assertTrue(plan["_timeline"]["validation"]["valid"])
        rotations = [s for s in plan["_segments"] if "rotate" in s["name"]]
        self.assertGreater(len(rotations), 1)
        self.assertGreater(np.max(np.linalg.norm(rotations[len(rotations)//2]["start_positions"]
                             - rotations[0]["start_positions"], axis=1)), 3)
        np.testing.assert_allclose(rotations[-1]["end_positions"], rotations[0]["start_positions"], atol=1e-8)

    def test_exact_check_detects_a_collision_between_samples(self):
        start = np.array([[-1., 0, 0], [1., 0, 0]])
        end = start[::-1].copy()
        self.assertEqual(continuous_separation(start, end), 0)
        assigned, _ = match_targets(start, end)
        self.assertEqual(continuous_separation(start, assigned), 2)

    def test_next_show_starts_from_current_airborne_formation(self):
        first = build_show(ground_layout(128), [{"type": "text", "text": "卷卷"}],
                           1, b"secret", include_landing=False)
        current = positions_at(first, first["duration"])
        second = build_show(current, [{"type": "heart"}], 2, b"secret")
        np.testing.assert_array_equal(second["_start_positions"], current)
        self.assertTrue(second["_timeline"]["validation"]["valid"])
        self.assertNotEqual(second["_segments"][0]["name"], "takeoff")

    def test_english_keeps_all_font_cells_and_long_text_paginates(self):
        plan = build_show(ground_layout(128), [{"type": "text", "text": "HELLO"}], 1, b"secret")
        self.assertEqual(plan["_timeline"]["scenes"][0]["active"], 73)
        self.assertTrue(plan["_timeline"]["validation"]["valid"])
        long = build_show(ground_layout(128), [{"type": "text", "text": "HELLOWORLD"}], 1, b"secret")
        self.assertEqual([s["text"] for s in long["_timeline"]["scenes"]], ["HELLO", "WORLD"])

    def test_english_preserves_font_topology_and_upright_audience_orientation(self):
        from droneshow.director import scene_targets
        from droneshow.formations import _text_lit_cells
        points, label = scene_targets({"type": "text", "text": "AGNES"}, 128)[0]
        cells, width = _text_lit_cells("AGNES", 1)
        # Recover font coordinates from the physical audience plane. Every
        # stroke must survive, and the top row must have the highest altitude.
        reconstructed = {(round(x / .34 + (width-1)/2),
                          round(3 - (z - 1.5) / (.34 * .5))) for x, _, z in points}
        self.assertEqual(reconstructed, cells)
        self.assertEqual(len(points), len(cells))
        self.assertEqual(label, "AGNES")
        self.assertAlmostEqual(max(points[:, 2]), 2.01)

    def test_other_glyphs_fit_without_dropping_text_or_unsafe_morphs(self):
        for word in ("你好", "生日快乐", "我爱你", "一"):
            with self.subTest(word=word):
                plan = build_show(ground_layout(128), [{"type": "text", "text": word}], 1, b"secret")
                self.assertEqual("".join(s["text"] for s in plan["_timeline"]["scenes"]), word)
                self.assertTrue(plan["_timeline"]["validation"]["valid"])

    def test_insufficient_fleet_cannot_report_more_active_drones_than_exist(self):
        with self.assertRaises(ShowError):
            build_show(ground_layout(16), [{"type": "text", "text": "HELLO"}], 1, b"secret")

    def test_three_dimensional_choreography_and_every_transition_are_safe(self):
        plan = build_show(ground_layout(128), [{"type": kind} for kind in choreography.KINDS],
                          1, b"secret")
        self.assertTrue(plan["_timeline"]["validation"]["valid"])
        for kind in choreography.KINDS:
            motion = [s for s in plan["_segments"] if kind + "_motion" in s["name"]]
            self.assertGreater(len(motion), 15)
            self.assertGreater(np.max(np.linalg.norm(motion[len(motion)//4]["end_positions"]
                               - motion[0]["start_positions"], axis=1)), .4)
            for segment in motion:
                self.assertGreaterEqual(continuous_separation(segment["start_positions"],
                                          segment["end_positions"]), .27)
        self.assertEqual(plan["_segments"][-1]["name"], "landing")

    def test_tilted_letters_remain_above_parked_drones_and_within_arena(self):
        plan = build_show(ground_layout(128), [{"type": "text", "text": "我爱你"}], 1, b"secret")
        for segment in plan["_segments"]:
            if segment["name"].startswith("dwell:"):
                active = np.asarray(segment["colors"]) != "#26334b"
                points = segment["start_positions"][active]
                self.assertGreater(np.ptp(points[:, 2]), .5)
                self.assertGreaterEqual(np.min(points[:, 2]), .45 - 1e-8)
                self.assertLessEqual(np.max(points[:, 2]), 2.55 + 1e-8)

    def test_dynamic_budget_does_not_silently_drop_requested_drones(self):
        with self.assertRaises(ValueError):
            build_show(ground_layout(128), [{"type": "helix", "drone_budget": 63}], 1, b"secret")


if __name__ == "__main__":
    unittest.main()
