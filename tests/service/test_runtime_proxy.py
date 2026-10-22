"""Tests for runtime decoupling and ServiceCore runtime injection."""
from __future__ import annotations

import sys
import tempfile
import time
import unittest
from unittest.mock import patch
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from droneshow.core import ServiceCore  # noqa: E402
from droneshow.runtime_proxy import RuntimeProxy  # noqa: E402
from droneshow.core import ServiceError
from droneshow.director import build_show


class _DummyRuntime:
    def __init__(self) -> None:
        self.started = False
        self.closed = False

    def start(self) -> None:
        self.started = True

    def close(self) -> None:
        self.closed = True

    def snapshot(self):
        return {"num_drones": 0, "positions": [], "status": "idle"}


class RuntimeProxyTests(unittest.TestCase):
    def test_service_core_accepts_injected_runtime(self) -> None:
        dummy = _DummyRuntime()
        core = ServiceCore(runtime=dummy)
        self.assertIs(core.runtime, dummy)
        core.start()
        self.assertTrue(dummy.started)
        core.close()
        self.assertTrue(dummy.closed)

    def test_proxy_roundtrip_and_clean_close(self) -> None:
        proxy = RuntimeProxy(num_drones=1, ctrl_freq=48, pyb_freq=240,
                             realtime_factor=8.0,
                             output_root=tempfile.mkdtemp(prefix="proxy-test-"))
        try:
            proxy.start(timeout=60.0)
            state = proxy.snapshot()
            self.assertEqual(state["num_drones"], 1)
            self.assertIsInstance(proxy.drain_events(), list)
            self.assertIsInstance(proxy.latest_run_info(), dict)
        finally:
            proxy.close()

    def test_live_amendment_preserves_physics_and_archive_survives_restart(self):
        root = tempfile.mkdtemp(prefix="amendment-test-")
        proxy = RuntimeProxy(num_drones=32, realtime_factor=4., output_root=root)
        core = ServiceCore(runtime=proxy, output_root=root)
        try:
            proxy.start()
            first = core.perform({"num_drones": 32, "scenes": [{"type": "ring"}],
                                  "include_landing": False})
            deadline = time.monotonic() + 10
            while not proxy.snapshot()["active_segment"] and time.monotonic() < deadline:
                time.sleep(.03)
            before = proxy.snapshot()
            with self.assertRaises(ServiceError):
                core.perform({"scenes": [{"type": "helix", "drone_budget": 31}]})
            self.assertEqual(proxy.snapshot()["num_drones"], 32)
            amended = core.perform({"scenes": [{"type": "bird"}]})
            self.assertEqual(amended["scheduled_for"], "safe_dwell")
            self.assertEqual(amended["source_run_id"], first["run_id"])
            self.assertGreaterEqual(proxy.snapshot()["simulation_time"], before["simulation_time"])
            deadline = time.monotonic() + 120
            while time.monotonic() < deadline:
                result = core.get_run(amended["run_id"])
                if result["status"] in ("completed", "rejected", "error"):
                    break
                time.sleep(.08)
            self.assertEqual(result["status"], "completed", result)
            self.assertFalse(core.get_run(first["run_id"])["completed"])
            self.assertEqual(core.get_run(first["run_id"])["status"], "superseded")
            self.assertGreaterEqual(result["summary"]["min_actual_pairwise_distance_m"], .25)
            self.assertLess(result["summary"]["max_tracking_error_m"], .4)
            frames = core.replay_run(amended["run_id"])
            self.assertTrue(frames["frames"])
        finally:
            proxy.close()
        restored = RuntimeProxy(num_drones=32, realtime_factor=4., output_root=root)
        try:
            restored.start()
            reread = restored.public_run(amended["run_id"])
            self.assertEqual(reread["status"], "completed")
            self.assertEqual(reread["summary"], result["summary"])
            self.assertIsNotNone(restored.run_replay_data(amended["run_id"]))
            self.assertEqual(restored.snapshot()["status"], "idle")
        finally:
            restored.close()

    def test_late_amendment_is_refused_while_original_show_finishes(self):
        root = tempfile.mkdtemp(prefix="late-amendment-test-")
        proxy = RuntimeProxy(num_drones=16, realtime_factor=4., output_root=root)
        core = ServiceCore(runtime=proxy, output_root=root)
        try:
            proxy.start()
            first = core.perform({"num_drones": 16, "scenes": [{"type": "ring"}]})
            cutoff = next(s["end_time"] for s in core.plans[first["plan_id"]]["_segments"]
                          if s["name"].startswith("dwell:"))
            deadline = time.monotonic() + 60
            while not proxy.snapshot()["active_segment"] and time.monotonic() < deadline:
                time.sleep(.03)
            before = proxy.snapshot()["simulation_time"]

            def slow_planner(*args, **kwargs):
                plan = build_show(*args, **kwargs)
                while time.monotonic() < deadline:
                    state = proxy.snapshot()
                    if not state["active_segment"] or state["show_elapsed_seconds"] > cutoff + .5:
                        return plan
                    time.sleep(.04)
                self.fail("Original physics stopped while building an amendment")

            with patch("droneshow.core.build_show", side_effect=slow_planner):
                with self.assertRaises(ServiceError) as refused:
                    core.perform({"scenes": [{"type": "heart"}]})
            self.assertEqual(refused.exception.code, "E_CHECKPOINT_EXPIRED")
            self.assertGreater(proxy.snapshot()["simulation_time"], before)
            while time.monotonic() < deadline:
                run = core.get_run(first["run_id"])
                if run["status"] == "completed":
                    break
                time.sleep(.05)
            self.assertEqual(run["status"], "completed")
            self.assertGreaterEqual(run["summary"]["min_actual_pairwise_distance_m"], .25)
        finally:
            proxy.close()

    def test_interrupted_physical_run_is_restored_without_execution(self):
        root = tempfile.mkdtemp(prefix="interrupted-run-test-")
        proxy = RuntimeProxy(num_drones=16, realtime_factor=4., output_root=root)
        core = ServiceCore(runtime=proxy, output_root=root)
        try:
            proxy.start()
            first = core.perform({"num_drones": 16, "scenes": [{"type": "ring"}]})
            deadline = time.monotonic() + 10
            while not proxy.snapshot()["active_segment"] and time.monotonic() < deadline:
                time.sleep(.03)
        finally:
            proxy.close()
        restored = RuntimeProxy(num_drones=16, output_root=root)
        try:
            restored.start()
            run = restored.public_run(first["run_id"])
            self.assertEqual(run["status"], "interrupted")
            self.assertFalse(run["completed"])
            self.assertEqual(restored.snapshot()["status"], "idle")
            self.assertIsNotNone(restored.run_replay_data(first["run_id"]))
        finally:
            restored.close()


if __name__ == "__main__":
    unittest.main()
