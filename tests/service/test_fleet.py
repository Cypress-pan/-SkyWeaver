"""Tests for the large-scale fleet timeline planner (task-16).

Covers the estimator/allocator, the layered formations, the safe parking
shells, the memory-bounded validator equivalence and the HTTP-facing
`ServiceCore.fleet_timeline` contract (including the kinematic preview run
registry and the physical-limit rejection).
"""
from __future__ import annotations

import sys
import time
import unittest
import tempfile
from unittest.mock import patch
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from droneshow.core import ServiceCore, ServiceError  # noqa: E402
from droneshow.planner import positions_at
from droneshow.fleet import (  # noqa: E402
    FleetError,
    build_fleet_timeline_plan,
    fleet_start_positions,
)
from droneshow.planner import build_plan, sample_plan_positions  # noqa: E402
from droneshow.validator import validate_plan, validate_plan_chunked  # noqa: E402


def live_scenes() -> list[dict]:
    return [
        {
            "type": "text",
            "text": "卷卷卷",
            "drone_budget": "auto",
            "min_points_per_char": 80,
            "duration_sec": 20,
            "transition_seconds": 5,
            "gradient": ["#55d6ff", "#8b7cff"],
            "effect": {"type": "none"},
        },
        {
            "type": "ring",
            "drone_budget": 510,
            "radius": 4.0,
            "z": 1.6,
            "duration_sec": 35,
            "transition_seconds": 5,
            "color": "#55d6ff",
            "effect": {"type": "rotate", "turns": 1},
        },
        {
            "type": "ring",
            "drone_budget": 300,
            "radius": 3.0,
            "z": 1.2,
            "duration_sec": 20,
            "transition_seconds": 5,
            "color": "#8b7cff",
            "include_landing": True,
        },
    ]


def small_scenes() -> list[dict]:
    return [
        {
            "type": "ring",
            "drone_budget": 24,
            "radius": 2.0,
            "z": 1.4,
            "duration_sec": 8,
            "transition_seconds": 2,
            "effect": {"type": "none"},
        },
        {
            "type": "grid",
            "drone_budget": 32,
            "spacing": 0.6,
            "z": 1.0,
            "duration_sec": 8,
            "transition_seconds": 2,
            "effect": {"type": "none"},
            "include_landing": True,
        },
    ]


class _StubRuntime:
    """Minimal runtime double for core-level tests (no PyBullet)."""

    def __init__(self, num_drones: int = 128):
        self.num_drones = int(num_drones)
        self.positions = fleet_start_positions(self.num_drones)
        self.queue: list[dict] = []

    def start(self) -> None:
        return None

    def close(self) -> None:
        return None

    def snapshot(self) -> dict:
        return {
            "num_drones": self.num_drones,
            "status": "idle",
            "simulation_time": 0.0,
            "positions": self.positions.tolist(),
            "latest_run_id": None,
            "active_segment": None,
            "pending_segment": None,
            "actual_error": None,
            "last_error": None,
        }

    def begin_planning(self) -> None:
        return None

    def end_planning(self) -> None:
        return None

    def ensure_num_drones(self, num_drones: int, timeout: float = 60.0) -> None:
        self.num_drones = int(num_drones)
        self.positions = fleet_start_positions(self.num_drones)

    def schedule_plan(self, plan: dict, run_id: str,
                      insert_after: str = "immediate") -> str:
        if int(plan.get("_num_drones", -1)) != self.num_drones:
            from droneshow.errors import RuntimeBusy

            raise RuntimeBusy("plan drone count does not match runtime")
        self.queue.append({"plan": plan, "run_id": run_id})
        return "immediate" if len(self.queue) == 1 else "next_dwell"

    def cancel_pending(self) -> list[str]:
        removed = [item["run_id"] for item in self.queue]
        self.queue.clear()
        return removed

    def public_run(self, run_id: str):
        return None

    def run_arrays(self, run_id: str):
        return None

    def run_replay_data(self, run_id: str):
        return None

    def latest_run_info(self) -> dict:
        return {"run_id": None, "status": None, "plan_id": None}

    def drain_events(self) -> list:
        return []


class ChunkedValidatorTests(unittest.TestCase):
    def test_chunked_matches_frozen_on_classic_plan(self) -> None:
        from droneshow.formations import build_formation

        start = np.array(build_formation(64, {
            "type": "grid", "rows": 8, "cols": 8, "spacing": 0.6,
            "z": 0.1})["points"], dtype=float)
        target = build_formation(64, {"type": "ring", "radius": 2.5, "z": 1.4})
        plan = build_plan(start, 64, target, 4.0, 2.0, True, 1, b"secret",
                          plan_id="p_chunk_equiv")
        frozen = validate_plan(plan)
        chunked = validate_plan_chunked(plan)
        self.assertEqual(frozen["valid"], chunked["valid"])
        self.assertEqual(frozen["issues"], chunked["issues"])
        self.assertEqual(frozen["metrics"]["samples"], chunked["metrics"]["samples"])
        for key, value in frozen["metrics"].items():
            if isinstance(value, float):
                self.assertAlmostEqual(value, float(chunked["metrics"][key]),
                                       places=9)
            else:
                self.assertEqual(value, chunked["metrics"][key])

    def test_chunked_matches_frozen_on_fleet_plan(self) -> None:
        plan = build_fleet_timeline_plan(
            fleet_start_positions(64), 64, 16.0, small_scenes(), "kinematic",
            1, b"secret", plan_id="p_fleet_chunk")
        frozen = validate_plan(plan)
        chunked = validate_plan_chunked(plan)
        self.assertEqual(frozen["valid"], chunked["valid"], frozen["issues"])
        self.assertEqual(frozen["credential"], chunked["credential"])
        for key, value in frozen["metrics"].items():
            if isinstance(value, float):
                self.assertAlmostEqual(value, float(chunked["metrics"][key]),
                                       places=9)
            else:
                self.assertEqual(value, chunked["metrics"][key])


class FleetPlannerTests(unittest.TestCase):
    def test_small_fleet_plan_validates_with_allocations(self) -> None:
        total = 64
        plan = build_fleet_timeline_plan(
            fleet_start_positions(total), total, 16.0, small_scenes(),
            "kinematic", 1, b"secret", plan_id="p_fleet_small")
        meta = plan["_timeline"]
        validation = meta["validation"]
        self.assertTrue(validation["valid"], validation["issues"])
        self.assertAlmostEqual(plan["duration"], 16.0, places=6)
        summaries = meta["scenes"]
        self.assertEqual([s["active"] for s in summaries], [24, 32])
        self.assertEqual([s["idle"] for s in summaries], [40, 32])
        self.assertEqual(meta["motion_mode"], "kinematic")
        self.assertTrue(meta["kinematic_preview"])
        metrics = validation["metrics"]
        self.assertGreaterEqual(metrics["min_distance"], 0.25)
        self.assertLessEqual(metrics["max_speed"], 2.0)
        self.assertLessEqual(metrics["max_accel"], 8.0)
        self.assertLessEqual(metrics["max_xy"], 5.0)

    def test_auto_budget_scales_with_text_complexity(self) -> None:
        # Glyph complexity (卷 = 5828 px) drives the readability budget, it is
        # never a fixed cap of 4.
        scenes = [
            {"type": "text", "text": "卷卷卷", "drone_budget": "auto",
             "min_points_per_char": 80, "duration_sec": 20,
             "transition_seconds": 4, "effect": {"type": "none"}},
        ]
        plan = build_fleet_timeline_plan(fleet_start_positions(300), 300,
                                         20.0, scenes, "kinematic", 1,
                                         b"secret", plan_id="p_auto")
        summary = plan["_timeline"]["scenes"][0]
        self.assertGreaterEqual(summary["points_per_char"], 80)
        self.assertLessEqual(summary["points_per_char"], 96)
        self.assertEqual(summary["active"],
                         int(round(summary["points_per_char"] * 3)))
        self.assertEqual(summary["idle"], 300 - summary["active"])

        # ... and it is capped by the fleet size, producing a valid plan.
        small_scene = [dict(scenes[0], min_points_per_char=30)]
        small = build_fleet_timeline_plan(fleet_start_positions(100), 100,
                                          20.0, small_scene, "kinematic", 1,
                                          b"secret", plan_id="p_auto_small")
        summary = small["_timeline"]["scenes"][0]
        self.assertEqual(summary["active"], 100)
        self.assertEqual(summary["idle"], 0)
        self.assertTrue(small["_timeline"]["validation"]["valid"],
                        small["_timeline"]["validation"]["issues"])

    def test_duration_mismatch(self) -> None:
        with self.assertRaises(FleetError) as context:
            build_fleet_timeline_plan(fleet_start_positions(32), 32, 15.0,
                                      small_scenes(), "kinematic", 1, b"secret")
        self.assertEqual(context.exception.code, "E_DURATION_MISMATCH")

    def test_point_budget_rejected(self) -> None:
        scenes = [
            {"type": "text", "text": "卷卷卷", "drone_budget": 30,
             "min_points_per_char": 80, "duration_sec": 10,
             "transition_seconds": 2, "effect": {"type": "none"}},
        ]
        with self.assertRaises(FleetError) as context:
            build_fleet_timeline_plan(fleet_start_positions(100), 100, 10.0,
                                      scenes, "kinematic", 1, b"secret")
        self.assertEqual(context.exception.code, "E_POINT_BUDGET")

    def test_fleet_capacity_rejected(self) -> None:
        scenes = [
            {"type": "ring", "drone_budget": "auto", "radius": 4.0,
             "duration_sec": 4, "transition_seconds": 1,
             "effect": {"type": "none"}},
        ]
        with self.assertRaises(FleetError) as context:
            build_fleet_timeline_plan(fleet_start_positions(4096), 4096, 4.0,
                                      scenes, "kinematic", 1, b"secret")
        self.assertEqual(context.exception.code, "E_FLEET_CAPACITY")

    def test_physical_limit_rejected_by_core(self) -> None:
        core = ServiceCore(runtime=_StubRuntime(num_drones=128), output_root=tempfile.mkdtemp(prefix="fleet-test-"))
        with self.assertRaises(ServiceError) as context:
            core.fleet_timeline({
                "total_drones": 1000,
                "duration_sec": 75.0,
                "motion_mode": "physical",
                "scenes": live_scenes(),
                "submit": False,
            })
        self.assertEqual(context.exception.code, "E_PHYSICAL_LIMIT")
        self.assertEqual(context.exception.status, 400)
        self.assertEqual(context.exception.extra["physical_limit"], 128)
        self.assertEqual(context.exception.extra["total_drones"], 1000)

    def test_core_kinematic_preview_and_submit(self) -> None:
        root = tempfile.mkdtemp(prefix="fleet-test-")
        core = ServiceCore(runtime=_StubRuntime(num_drones=128), output_root=root)
        preview_scenes = [
            {"type": "text", "text": "卷卷卷", "drone_budget": "auto",
             "min_points_per_char": 30, "duration_sec": 12,
             "transition_seconds": 3, "effect": {"type": "none"}},
            {"type": "ring", "drone_budget": 60, "radius": 3.0, "z": 1.6,
             "duration_sec": 35, "transition_seconds": 3,
             "effect": {"type": "rotate", "turns": 1}, "include_landing": True},
        ]
        result = core.fleet_timeline({
            "total_drones": 100,
            "duration_sec": 47.0,
            "motion_mode": "kinematic",
            "scenes": preview_scenes,
            "submit": True,
        })
        self.assertTrue(result["ok"])
        self.assertEqual(result["total_drones"], 100)
        self.assertEqual(result["motion_mode"], "kinematic")
        self.assertTrue(result["kinematic_preview"])
        self.assertAlmostEqual(result["duration"], 47.0, places=3)
        self.assertTrue(result["submitted"])
        self.assertIsInstance(result["run_id"], str)
        self.assertEqual(result["scheduled_for"], "kinematic_preview")
        allocations = result["scene_allocations"]
        self.assertEqual([a["active"] for a in allocations], [100, 60])
        self.assertEqual([a["idle"] for a in allocations], [0, 40])
        self.assertTrue(result["validation"]["valid"], result["validation"]["issues"])
        self.assertTrue(any(w.get("code") == "KINEMATIC_PREVIEW"
                            for w in result["warnings"]))
        # the preview run is addressable like a real run
        run = core.get_run(result["run_id"])
        self.assertEqual(run["status"], "running")
        self.assertFalse(run["completed"])
        self.assertIsNone(run["finished_at"])
        self.assertEqual(core.latest_run()["run_id"], result["run_id"])
        self.assertTrue(core.snapshot()["kinematic_preview"])
        self.assertTrue(run["summary"]["kinematic_preview"])
        replay = core.replay_run(result["run_id"], hz=5)
        self.assertGreater(len(replay["frames"]), 1)
        self.assertIn("kinematic", replay["actual_positions_source"])
        evaluation = core.evaluate_run(result["run_id"])
        self.assertEqual(evaluation["verdict"], "preview")
        preview = core.preview_runs[result["run_id"]]
        started = preview["started_monotonic"]
        with patch("droneshow.core.time.monotonic", return_value=started+10):
            before = np.array(core.snapshot()["positions"])
            next_run = core.fleet_timeline({"total_drones": 100, "duration_sec": 20,
                        "motion_mode": "kinematic", "submit": True,
                        "scenes": [{"type": "ring", "drone_budget": 60,
                                    "radius": 3., "duration_sec": 20}]})
            np.testing.assert_allclose(core.preview_runs[next_run["run_id"]]["plan"]["_start_positions"], before)
        with patch("droneshow.core.time.monotonic", return_value=started+48):
            self.assertTrue(core.get_run(result["run_id"])["completed"])
            self.assertIsNotNone(core.get_run(result["run_id"])["finished_at"])
        restored = ServiceCore(runtime=_StubRuntime(num_drones=128), output_root=root)
        self.assertTrue(restored.get_run(result["run_id"])["completed"])
        restored_replay = restored.replay_run(result["run_id"], hz=5)
        self.assertTrue(restored_replay["kinematic_preview"])
        self.assertIn("nominal", restored_replay["actual_positions_source"])
        physical = core.perform({"num_drones": 128, "scenes": [{"type": "heart"}]})
        self.assertEqual(core.presentation_run_id, physical["run_id"])
        self.assertFalse(core.snapshot().get("kinematic_preview", False))

    def test_director_automatic_large_heart_and_current_count(self) -> None:
        core = ServiceCore(runtime=_StubRuntime(num_drones=64), output_root=tempfile.mkdtemp(prefix="fleet-test-"))
        core.presentation_run_id = "r_previous_physical"
        current = np.array(core.runtime.snapshot()["positions"])
        physical = core.perform({"scenes": [{"type": "heart"}]})
        self.assertEqual(physical["num_drones"], 64)
        np.testing.assert_array_equal(core.plans[physical["plan_id"]]["_start_positions"], current)
        large = core.perform({"num_drones": 1000, "submit": False,
                         "scenes": [{"type": "text", "text": "卷卷"}, {"type": "heart"}]})
        self.assertTrue(large["validation"]["valid"])
        self.assertEqual(large["total_drones"], 1000)
        self.assertEqual(large["scene_allocations"][1]["layers"], 4)

    def test_live_scenario_allocations_1000_drones(self) -> None:
        """All 1000 trajectories are safe and explicit budgets are preserved."""
        total = 1000
        started = time.monotonic()
        plan = build_fleet_timeline_plan(
            fleet_start_positions(total), total, 75.0, live_scenes(),
            "kinematic", 1, b"secret", plan_id="p_fleet_1000")
        elapsed = time.monotonic() - started
        validation = plan["_timeline"]["validation"]
        self.assertAlmostEqual(plan["duration"], 75.0, places=3)
        summaries = plan["_timeline"]["scenes"]
        # the text scene gets the readability budget; the ring scenes are
        # preserve the requested counts
        self.assertEqual(summaries[0]["active"], 273)
        self.assertEqual(summaries[0]["idle"], 727)
        for summary, request in zip(summaries[1:], (510, 300)):
            self.assertGreater(summary["active"], 0)
            self.assertEqual(summary["active"], request)
        for summary in summaries:
            self.assertEqual(summary["idle"] + summary["active"], total)
        self.assertTrue(validation["valid"], validation["issues"])
        self.assertIsNotNone(validation["credential"])
        metrics = validation["metrics"]
        self.assertEqual(metrics["samples"], 15001)
        self.assertLessEqual(metrics["max_xy"], 5.0)
        self.assertLessEqual(metrics["max_z"], 3.0)
        # layers/parking are reported for every scene
        for summary in summaries:
            self.assertGreaterEqual(summary["layers"], 1)
            self.assertEqual(summary["idle"] + summary["active"], total)
        self.assertGreater(summaries[0]["parking"]["slots"], 0)
        self.assertEqual(summaries[0]["points_per_char"], 91.0)
        self.assertLess(elapsed, 60.0)
        short = live_scenes()
        short[1]["duration_sec"] = 20
        with self.assertRaises(FleetError) as rejected:
            build_fleet_timeline_plan(fleet_start_positions(total), total, 60., short,
                                     "kinematic", 1, b"secret")
        self.assertEqual(rejected.exception.code, "E_DURATION_TOO_SHORT")

    def test_validated_fleet_sizes(self) -> None:
        """Fleets up to ~100 drones produce fully validated plans."""
        scenes = [
            {"type": "ring", "drone_budget": 40, "radius": 2.5, "z": 1.4,
             "duration_sec": 8, "transition_seconds": 2,
             "effect": {"type": "none"}},
            {"type": "ring", "drone_budget": 30, "radius": 3.0, "z": 1.2,
             "duration_sec": 8, "transition_seconds": 2, "include_landing": True,
             "effect": {"type": "none"}},
        ]
        for total in (64, 100):
            plan = build_fleet_timeline_plan(
                fleet_start_positions(total), total, 16.0, scenes,
                "kinematic", 1, b"secret", plan_id="p_valid_" + str(total))
            validation = plan["_timeline"]["validation"]
            self.assertTrue(validation["valid"],
                            (total, validation["issues"]))
            self.assertGreaterEqual(validation["metrics"]["min_distance"], 0.25)


if __name__ == "__main__":
    unittest.main()
