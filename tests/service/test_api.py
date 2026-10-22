"""End-to-end API test: headless service, 16-drone grid to ring."""
from __future__ import annotations

import json
import base64
import socket
import subprocess
import sys
import tempfile
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def free_port() -> int:
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


class ServiceApiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.tmp = tempfile.TemporaryDirectory(prefix="drone-show-api-")
        cls.port = free_port()
        cls.base = f"http://127.0.0.1:{cls.port}"
        cmd = [
            sys.executable, str(ROOT / "service" / "server.py"),
            "--host", "127.0.0.1", "--port", str(cls.port),
            "--num-drones", "16", "--realtime-factor", "4.0",
            "--output-root", cls.tmp.name,
            "--web-dir", str(ROOT / "web"),
        ]
        cls.proc = subprocess.Popen(cmd, cwd=str(ROOT), stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True)
        deadline = time.monotonic() + 40.0
        while time.monotonic() < deadline:
            if cls.proc.poll() is not None:
                out = cls.proc.stdout.read() if cls.proc.stdout else ""
                raise RuntimeError(f"service exited early: {out}")
            try:
                status, _ = cls.call("GET", "/api/health")
                if status == 200:
                    return
            except Exception:
                time.sleep(0.2)
        raise RuntimeError("service did not become healthy in time")

    @classmethod
    def tearDownClass(cls) -> None:
        if getattr(cls, "proc", None) is not None and cls.proc.poll() is None:
            cls.proc.terminate()
            try:
                cls.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                cls.proc.kill()
        if getattr(cls, "tmp", None) is not None:
            cls.tmp.cleanup()

    @classmethod
    def call(cls, method: str, path: str, payload: dict | None = None):
        data = json.dumps(payload).encode("utf-8") if payload is not None else None
        request = urllib.request.Request(cls.base + path, data=data, method=method,
                                         headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(request, timeout=15) as response:
                return response.status, json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            return exc.code, json.loads(exc.read().decode("utf-8"))

    def test_full_grid_to_ring_flow(self) -> None:
        status, health = self.call("GET", "/api/health")
        self.assertEqual(status, 200)
        self.assertTrue(health["ok"])
        self.assertEqual(health["version"], "0.1.0")

        status, state = self.call("GET", "/api/state")
        self.assertEqual(status, 200)
        self.assertEqual(state["num_drones"], 16)
        self.assertEqual(state["status"], "idle")
        self.assertEqual(len(state["positions"]), 16)
        show_rev = state["show_revision"]
        req_rev = state["request_revision"]

        status, grid = self.call("POST", "/api/formation", {
            "num_drones": 16,
            "spec": {"type": "grid", "rows": 4, "cols": 4, "spacing": 0.6, "z": 1.2},
        })
        self.assertEqual(status, 200)
        self.assertEqual(grid["resource_report"]["used"], 16)
        self.assertEqual(grid["resource_report"]["idle"], 0)

        status, ring = self.call("POST", "/api/formation", {
            "num_drones": 16,
            "spec": {"type": "ring", "radius": 1.2, "z": 1.2, "phase": 0.0,
                     "color": "#00ff00",
                     "gradient": ["#ff0000", "#0000ff"]},
        })
        self.assertEqual(status, 200)
        self.assertGreaterEqual(ring["resource_report"]["min_distance"], 0.25)
        self.assertEqual(ring["color"], "#00ff00")
        self.assertEqual(len(ring["colors"]), 16)
        self.assertEqual(ring["colors"][0], "#ff0000")
        self.assertEqual(ring["colors"][-1], "#0000ff")

        status, plan = self.call("POST", "/api/plan", {
            "state_revision": show_rev,
            "target_formation_id": ring["formation_id"],
            "duration": 4.0,
            "hold_seconds": 2.0,
            "include_landing": True,
        })
        self.assertEqual(status, 200)
        self.assertAlmostEqual(plan["duration"], 8.5, places=6)
        self.assertEqual([s["name"] for s in plan["segments"]],
                         ["takeoff", "transition", "hold", "landing"])
        self.assertEqual(plan["color"], "#00ff00")
        for segment in plan["segments"]:
            self.assertEqual(segment["color"], "#00ff00")
            self.assertEqual(len(segment["colors"]), 16)
        self.assertIn("#ff0000", plan["segments"][0]["colors"])
        self.assertIn("#0000ff", plan["segments"][0]["colors"])

        status, validation = self.call("POST", "/api/validate",
                                       {"plan_id": plan["plan_id"]})
        self.assertEqual(status, 200)
        self.assertTrue(validation["valid"], validation["issues"])
        self.assertIsNotNone(validation["credential"])
        self.assertGreaterEqual(validation["metrics"]["min_distance"], 0.25)
        self.assertLessEqual(validation["metrics"]["max_speed"], 2.0)
        self.assertLessEqual(validation["metrics"]["max_accel"], 8.0)

        status, stale = self.call("POST", "/api/submit", {
            "plan_id": plan["plan_id"],
            "credential": validation["credential"],
            "request_revision": req_rev,
            "source_state_revision": show_rev - 1,
            "insert_after": "immediate",
        })
        self.assertEqual(status, 409)
        self.assertEqual(stale["error"], "E_STALE_STATE")
        self.assertEqual(stale["state_revision"], show_rev)

        status, submit = self.call("POST", "/api/submit", {
            "plan_id": plan["plan_id"],
            "credential": validation["credential"],
            "request_revision": req_rev,
            "source_state_revision": show_rev,
            "insert_after": "immediate",
        })
        self.assertEqual(status, 200)
        self.assertTrue(submit["accepted"])
        run_id = submit["run_id"]

        active_colors = None
        color_deadline = time.monotonic() + 6.0
        while time.monotonic() < color_deadline:
            _, live_state = self.call("GET", "/api/state")
            segment = live_state.get("active_segment")
            if segment and segment.get("colors"):
                active_colors = segment
                break
            time.sleep(0.03)
        self.assertIsNotNone(active_colors, "active_segment colors were not observed")
        self.assertEqual(active_colors["color"], "#00ff00")
        self.assertEqual(len(active_colors["colors"]), 16)
        self.assertIn("#ff0000", active_colors["colors"])
        self.assertIn("#0000ff", active_colors["colors"])

        deadline = time.monotonic() + 40.0
        run = None
        while time.monotonic() < deadline:
            status, run = self.call("GET", f"/api/run/{run_id}")
            if run["completed"]:
                break
            time.sleep(0.15)
        self.assertIsNotNone(run)
        self.assertTrue(run["completed"], run)
        self.assertEqual(run["status"], "completed")
        summary = run["summary"]
        self.assertLessEqual(summary["max_tracking_error_m"], 0.40)
        self.assertLessEqual(summary["rms_tracking_error_m"], 0.20)
        self.assertGreaterEqual(summary["min_actual_pairwise_distance_m"], 0.25)
        self.assertFalse(summary["all_frames_identical"])
        self.assertGreater(summary["tracking_error_nonzero_fraction"], 0.5)
        self.assertEqual(summary["actual_positions_source"],
                         "CtrlAviary.step observation")
        self.assertTrue(summary["target_positions_separate"])

        status, evaluation = self.call("POST", "/api/evaluate", {"run_id": run_id})
        self.assertEqual(status, 200)
        self.assertTrue(evaluation["ok"])
        self.assertEqual(evaluation["verdict"], "pass", evaluation["checks"])
        self.assertGreaterEqual(evaluation["metrics"]["min_distance"], 0.25)

        status, final_state = self.call("GET", "/api/state")
        self.assertEqual(status, 200)
        self.assertGreater(final_state["show_revision"], show_rev)
        self.assertGreater(final_state["request_revision"], req_rev)
        self.assertEqual(final_state["status"], "completed")
        self.assertEqual(final_state["latest_run_id"], run_id)

        status, latest = self.call("GET", "/api/runs/latest")
        self.assertEqual(status, 200)
        self.assertEqual(latest["run_id"], run_id)
        self.assertEqual(latest["status"], "completed")
        self.assertEqual(latest["plan_id"], plan["plan_id"])

        status, replay = self.call("GET", f"/api/run/{run_id}/replay?hz=15")
        self.assertEqual(status, 200)
        self.assertEqual(replay["run_id"], run_id)
        self.assertEqual(replay["actual_positions_source"],
                         "CtrlAviary.step observation")
        self.assertTrue(replay["target_positions_separate"])
        self.assertLessEqual(len(replay["frames"]), 600)
        self.assertGreater(len(replay["frames"]), 5)
        self.assertEqual(replay["frames"][0]["t"], 0.0)
        self.assertEqual(len(replay["frames"][0]["positions"]), 16)
        colored_frame = next(frame for frame in replay["frames"]
                            if frame["active_segment"]
                            and frame["active_segment"].get("colors"))
        self.assertEqual(colored_frame["active_segment"]["color"], "#00ff00")
        self.assertEqual(len(colored_frame["active_segment"]["colors"]), 16)
        self.assertIn("#ff0000", colored_frame["active_segment"]["colors"])
        self.assertIn("#0000ff", colored_frame["active_segment"]["colors"])

        status, replay_low = self.call("GET", f"/api/run/{run_id}/replay?hz=1")
        self.assertEqual(status, 200)
        self.assertEqual(replay_low["hz"], 5)
        status, replay_high = self.call("GET", f"/api/run/{run_id}/replay?hz=100")
        self.assertEqual(status, 200)
        self.assertEqual(replay_high["hz"], 30)

        status, text_show = self.call("POST", "/api/text_show", {
            "text": "卷卷",
            "num_drones": 16,
            "auto_allocate": True,
            "min_points_per_char": 16,
            "transition_seconds": 3.0,
            "hold_seconds": 1.0,
            "include_landing": False,
            "submit": False,
            "gradient": ["#55d6ff", "#8b7cff"],
        })
        self.assertEqual(status, 200)
        self.assertTrue(text_show["ok"])
        self.assertEqual(text_show["num_drones"], 16)
        self.assertIsNone(text_show["run_id"])
        self.assertFalse(text_show["submitted"])
        self.assertIsNone(text_show["scheduled_for"])
        self.assertEqual(len(text_show["pages"]), 2)
        self.assertEqual(text_show["pages"][0]["text"], "卷")
        self.assertEqual(text_show["pages"][0]["points"], 16)
        self.assertEqual(text_show["pages"][0]["idle"], 0)
        self.assertTrue(str(text_show["pages"][0]["formation_id"]).startswith("f_"))
        self.assertEqual(len(text_show["allocation_report"]), 2)
        self.assertTrue(all(item["complexity"] > 0 for item in text_show["allocation_report"]))
        self.assertTrue(text_show["plan_id"].startswith("p_"))
        self.assertTrue(text_show["plan_hash"])
        self.assertTrue(text_show["validation"]["valid"],
                        text_show["validation"]["issues"])
        self.assertEqual([segment["name"] for segment in text_show["plan"]["segments"]],
                         ["transition", "hold", "transition", "hold"])

    def test_frame_export_is_bounded_and_rejects_remote_origin_and_non_png(self):
        png = base64.b64decode("iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+aX1sAAAAASUVORK5CYII=")

        def export(body, origin=self.base):
            request = urllib.request.Request(self.base + "/api/frame-captures", data=body,
                headers={"Content-Type": "image/png", "Origin": origin})
            with urllib.request.urlopen(request, timeout=10) as response:
                return json.load(response)["url"]

        for body, origin, status in ((b"not a PNG", self.base, 400),
                                     (png, "https://untrusted.example", 403),
                                     (b"x" * (4 * 1024 * 1024 + 1), self.base, 413)):
            with self.assertRaises(urllib.error.HTTPError) as rejected:
                export(body, origin)
            self.assertEqual(rejected.exception.code, status)
        first = export(png)
        with urllib.request.urlopen(self.base + first) as response:
            self.assertEqual(response.headers["Content-Type"], "image/png")
            self.assertEqual(response.headers["X-Content-Type-Options"], "nosniff")
            self.assertEqual(response.read(), png)
        for _ in range(4):
            newest = export(png)
        with self.assertRaises(urllib.error.HTTPError) as evicted:
            urllib.request.urlopen(self.base + first)
        self.assertEqual(evicted.exception.code, 404)
        with urllib.request.urlopen(self.base + newest) as response:
            self.assertEqual(response.read(), png)


if __name__ == "__main__":
    unittest.main()
