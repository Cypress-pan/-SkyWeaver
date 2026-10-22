"""Telemetry equivalence, run isolation, and HTTP responsiveness during IPC waits."""
import asyncio
import sys
import threading
import unittest
from pathlib import Path

import numpy as np
from tornado.testing import AsyncHTTPTestCase, gen_test

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src'))
from droneshow.formations import min_pairwise_distance
from droneshow.telemetry_metrics import min_trajectory_distance
from droneshow.core import ServiceCore
from droneshow.http_server import make_app


class TelemetryTests(unittest.TestCase):
    def test_only_finished_checks_are_cached(self):
        core = ServiceCore.__new__(ServiceCore)
        core.lock = threading.RLock()
        core._evaluations = {}
        core.preview_runs = {}
        status = ['running']
        core.get_run = lambda _: {'status': status[0], 'completed': status[0] == 'completed', 'num_drones': 2}
        class Runtime:
            calls = 0
            def run_arrays(self, _):
                self.calls += 1
                actual = np.array([[[0., 0., 1.], [1., 0., 1.]]]*3)
                return np.arange(3), actual, actual.copy()
        core.runtime = Runtime()
        self.assertEqual(core.evaluate_run('r_a')['verdict'], 'inconclusive')
        self.assertFalse(core._evaluations)
        status[0] = 'completed'
        self.assertEqual(core.evaluate_run('r_a')['verdict'], 'pass')
        core.evaluate_run('r_a')
        self.assertEqual(core.runtime.calls, 2)

    def test_exact_all_pairs_and_frames_including_duplicates(self):
        rng = np.random.default_rng(312)
        for n in (1, 16, 64, 128):
            frames = rng.normal(size=(5, n, 3))
            if n > 1:
                frames[3, -1] = frames[3, 0] + [1e-7, 0, 0]
            expected = min((d for f in frames if (d := min_pairwise_distance(f)) is not None), default=None)
            value = min_trajectory_distance(frames)
            if expected is None:
                self.assertIsNone(value)
            else:
                self.assertAlmostEqual(value, expected, places=14)
                frames[2, -1] = frames[2, 0]
                self.assertEqual(min_trajectory_distance(frames), 0.)

    def test_queued_amendment_never_inherits_old_run_phase(self):
        core = ServiceCore.__new__(ServiceCore)
        core.plans = {'p': {'duration': 12}}
        core.get_run = lambda _: {'run_id': 'r_b', 'plan_id': 'p', 'status': 'running', 'completed': False, 'num_drones': 128}
        core.snapshot = lambda: {'active_run_id': 'r_a', 'latest_run_id': 'r_b', 'show_elapsed_seconds': 10, 'active_segment': {'name': 'landing'}}
        result = core.run_progress('r_b')
        self.assertIsNone(result['segment'])
        self.assertIsNone(result['elapsed_seconds'])
        self.assertIsNone(result['evaluation'])


class ResponsiveHttpTests(AsyncHTTPTestCase):
    def get_app(self):
        self.entered = threading.Event()
        self.release = threading.Event()
        outer = self
        class Core:
            def health(self): return {'ok': True}
            def fleet_timeline(self, body):
                outer.entered.set()
                outer.release.wait(3)
                return {'ok': True}
            def snapshot(self):
                outer.entered.set()
                outer.release.wait(3)
                return {'status': 'completed'}
        return make_app(Core())

    @gen_test(timeout=5)
    async def test_health_responds_while_state_waits_for_worker(self):
        state = self.http_client.fetch(self.get_url('/api/state'))
        try:
            while not self.entered.is_set():
                await asyncio.sleep(.005)
            health = await asyncio.wait_for(self.http_client.fetch(self.get_url('/api/health')), .5)
            self.assertEqual(health.code, 200)
            self.assertFalse(state.done())
        finally:
            self.release.set()
        self.assertEqual((await state).code, 200)

    @gen_test(timeout=5)
    async def test_health_responds_during_fleet_planning(self):
        planning = self.http_client.fetch(self.get_url('/api/fleet_timeline'), method='POST', body='{}')
        try:
            while not self.entered.is_set():
                await asyncio.sleep(.005)
            health = await asyncio.wait_for(self.http_client.fetch(self.get_url('/api/health')), .5)
            self.assertEqual(health.code, 200)
            self.assertFalse(planning.done())
        finally:
            self.release.set()
        self.assertEqual((await planning).code, 200)
