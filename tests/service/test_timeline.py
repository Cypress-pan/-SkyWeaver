"""Tests for the 60-second timeline planner."""
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
from droneshow.timeline import TimelineError, build_timeline_plan  # noqa: E402
from droneshow.validator import validate_plan  # noqa: E402


def start_positions(num_drones: int) -> np.ndarray:
    size = int(np.ceil(np.sqrt(num_drones)))
    return np.array(build_formation(num_drones, {
        "type": "grid", "rows": size, "cols": size,
        "spacing": 0.6, "z": 0.1})["points"], dtype=float)


def valid_scenes() -> list[dict]:
    return [
        {"type": "text", "text": "HELLO", "z": 1.0, "max_active": 4,
         "duration_sec": 10, "transition_seconds": 4, "color": "#55d6ff",
         "effect": {"type": "none"}},
        {"type": "ring", "radius": 2.0, "z": 1.6, "max_active": 4,
         "duration_sec": 10, "transition_seconds": 4,
         "gradient": ["#55d6ff", "#8b7cff"],
         "effect": {"type": "rotate", "turns": .125}},
        {"type": "heart", "size": 2.5, "z": 0.6, "max_active": 4,
         "duration_sec": 10, "transition_seconds": 4, "color": "#ff66aa",
         "effect": {"type": "none"}},
        {"type": "text", "text": "卷卷", "z": 1.4, "max_active": 4,
         "duration_sec": 30, "transition_seconds": 4,
         "color": "#55d6ff", "gradient": ["#55d6ff", "#8b7cff"],
         "effect": {"type": "wave", "amplitude": 0.2, "cycles": 1},
         "include_landing": True},
    ]


class TimelineTests(unittest.TestCase):
    def test_duration_mismatch(self) -> None:
        scenes = valid_scenes()
        with self.assertRaises(TimelineError) as context:
            build_timeline_plan(start_positions(16), 16, 59.0, scenes, 1, b"secret")
        self.assertEqual(context.exception.code, "E_DURATION_MISMATCH")

    def test_valid_60s_128_drone_timeline(self) -> None:
        plan = build_timeline_plan(start_positions(128), 128, 60.0,
                                   valid_scenes(), 1, b"secret",
                                   plan_id="p_timeline")
        result = validate_plan(plan)
        self.assertTrue(result["valid"], result["issues"])
        self.assertAlmostEqual(plan["duration"], 60.0, places=3)
        meta = plan["_timeline"]
        self.assertEqual(len(meta["scenes"]), 4)
        self.assertIn("rotate", str(meta["scenes"][1]["effect"]))
        self.assertIn("wave", str(meta["scenes"][3]["effect"]))
        self.assertTrue(meta["scenes"][3]["landing"])

    def test_all_effects_parse(self) -> None:
        scenes = valid_scenes()
        scenes[0]["effect"] = {"type": "scale", "from": 1.0, "to": 1.05}
        plan = build_timeline_plan(start_positions(64), 64, 60.0, scenes,
                                   1, b"secret", plan_id="p_effects")
        self.assertIn("scale", str(plan["_timeline"]["scenes"][0]["effect"]))


if __name__ == "__main__":
    unittest.main()
