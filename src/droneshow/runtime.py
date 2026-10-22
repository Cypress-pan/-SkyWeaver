"""Background CtrlAviary runtime for the drone-show service.

The runtime owns the PyBullet environment in its own thread.  Actual positions
are always read from ``CtrlAviary.step()`` observations; target positions come
from the signed plan and are stored separately.  No target coordinate is ever
written into the actual-position arrays.
"""
from __future__ import annotations

import collections
import json
import math
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from .errors import RuntimeBusy
from .formations import _grid_xy, min_pairwise_distance_fast as min_pairwise_distance
from .archive import RunArchive
from .planner import positions_at, segment_at, velocity_at
from .telemetry_metrics import min_trajectory_distance

from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics


INITIAL_Z = 0.10
INITIAL_SPACING = 0.6
CONTROL_FREQ_HZ = 48
PYB_FREQ_HZ = 240
START_BLEND_SECONDS = 0.5


def initial_positions(num_drones: int) -> np.ndarray:
    """Deterministic parking grid used before a show is submitted."""
    n = int(num_drones)
    cols = max(1, int(math.ceil(math.sqrt(n))))
    rows = max(1, int(math.ceil(n / cols)))
    xy = _grid_xy(rows, cols, INITIAL_SPACING)
    points = np.column_stack([xy[:n], np.full(n, INITIAL_Z, dtype=float)])
    return points


class SimRuntime:
    def __init__(self, num_drones: int = 16, ctrl_freq: int = CONTROL_FREQ_HZ,
                 pyb_freq: int = PYB_FREQ_HZ, realtime_factor: float = 1.0,
                 output_root: str | Path = "results", offline: bool = False):
        self._requested_num_drones = int(num_drones)
        self.ctrl_freq = int(ctrl_freq)
        self.pyb_freq = int(pyb_freq)
        self.realtime_factor = float(realtime_factor)
        self.offline = bool(offline)
        self.output_root = Path(output_root)
        self.lock = threading.RLock()
        self.cond = threading.Condition(self.lock)
        self.events: collections.deque[dict[str, Any]] = collections.deque()
        self.runs: dict[str, dict[str, Any]] = {}
        self._latest_run_id: str | None = None
        self.archive = RunArchive(self.output_root)
        self.runs = self.archive.load()
        self._latest_run_id = next(reversed(self.runs), None)
        self._thread: threading.Thread | None = None
        self._ready = False
        self._stop = False
        self._pause_depth = 0
        self._error: str | None = None
        self._reset_request: int | None = None
        self.env = None
        self.ctrls: list[Any] = []
        self._obs: np.ndarray | None = None
        self.num_drones = 0
        self.step_index = 0
        self.sim_time = 0.0
        self.positions = np.zeros((0, 3), dtype=float)
        self.velocities = np.zeros((0, 3), dtype=float)
        self.target_positions = np.zeros((0, 3), dtype=float)
        self.actual_error: dict[str, float] | None = None
        self.status = "starting"
        self.active: dict[str, Any] | None = None
        self.active_segment_public: dict[str, Any] | None = None
        self.queue: list[dict[str, Any]] = []
        self._hold_target: np.ndarray | None = None

    # -- lifecycle ---------------------------------------------------------
    def start(self, timeout: float = 60.0) -> None:
        self._thread = threading.Thread(target=self._run_forever,
                                        name="drone-show-runtime", daemon=True)
        self._thread.start()
        with self.cond:
            ok = self.cond.wait_for(lambda: self._ready or self._error is not None, timeout=timeout)
            if not ok:
                raise RuntimeError("runtime did not become ready within timeout")
            if self._error is not None:
                raise RuntimeError(self._error)

    def close(self) -> None:
        with self.cond:
            if self.active is not None:
                self._finalize_active_locked(outcome="interrupted")
            self.cancel_pending()
            self._stop = True
            self.cond.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=10.0)
        with self.cond:
            self._close_env_locked()
            self._ready = False

    def ensure_num_drones(self, num_drones: int, timeout: float = 60.0) -> None:
        n = int(num_drones)
        with self.cond:
            if self._error is not None:
                raise RuntimeError(self._error)
            if self.num_drones == n:
                return
            if self.active is not None or self.queue:
                raise RuntimeBusy("cannot change drone count while a plan is active or pending")
            self._reset_request = n
            self.cond.notify_all()
            ok = self.cond.wait_for(lambda: self._reset_request is None or self._error is not None,
                                    timeout=timeout)
            if not ok:
                raise RuntimeBusy("runtime reconfiguration timed out")
            if self._error is not None:
                raise RuntimeError(self._error)
            if self.num_drones != n:
                raise RuntimeBusy("runtime reconfiguration failed")

    def begin_planning(self) -> None:
        with self.cond:
            self._pause_depth += 1
            self.cond.notify_all()

    def end_planning(self) -> None:
        with self.cond:
            self._pause_depth = max(0, self._pause_depth - 1)
            self.cond.notify_all()

    # -- scheduling --------------------------------------------------------
    def schedule_plan(self, plan: dict[str, Any], run_id: str,
                      insert_after: str = "immediate") -> str:
        with self.cond:
            if self._error is not None:
                raise RuntimeBusy(f"runtime is in error state: {self._error}")
            if plan.get("_num_drones") != self.num_drones:
                raise RuntimeBusy("plan drone count does not match runtime")
            item = {"plan": plan, "run_id": run_id, "insert_after": insert_after}
            if self.active is None and not self.queue:
                scheduled_for = "immediate"
                self.queue.append(item)
            else:
                scheduled_for = "next_dwell"
                if insert_after == "immediate":
                    self.queue.insert(0, item)
                else:
                    self.queue.append(item)
            self.runs[run_id] = {
                "run_id": run_id,
                "plan_id": plan["plan_id"],
                "status": "running",
                "completed": False,
                "created_at": datetime.now(timezone.utc).isoformat(),
                "started_at": None,
                "finished_at": None,
                "plan_start_time": None,
                "actual_error": None,
                "summary": None,
                "telemetry_ref": None,
                "error": None,
                "_actual": None,
                "_target": None,
                "_times": None,
                "_plan": plan,
                "_num_drones": int(plan["_num_drones"]),
                "offline_execution": self.offline,
            }
            self._latest_run_id = run_id
            self.archive.save(self.runs[run_id])
            self.cond.notify_all()
        return scheduled_for

    def amendment_checkpoint(self) -> dict[str, Any]:
        """Reserve geometry, not a pause: the existing controller keeps flying."""
        with self.cond:
            if self.active is None:
                raise RuntimeBusy("E_NO_ACTIVE_SHOW")
            active = self.active
            local = self.sim_time - active["start_time"]
            for segment in active["plan"]["_segments"]:
                if (segment["name"].startswith("dwell:") and
                        segment["end_time"] > local + 1.0):
                    return {"run_id": active["run_id"], "segment_id": segment["id"],
                            "at": max(segment["start_time"], local + .5),
                            "deadline": segment["end_time"],
                            "positions": segment["end_positions"].copy()}
            raise RuntimeBusy("E_NO_SAFE_CHECKPOINT")

    def schedule_amendment(self, plan: dict[str, Any], run_id: str,
                           checkpoint: dict[str, Any]) -> str:
        with self.cond:
            if (self.active is None or self.active["run_id"] != checkpoint["run_id"] or
                    self.sim_time - self.active["start_time"] >= checkpoint["deadline"] - .25):
                raise RuntimeBusy("E_CHECKPOINT_EXPIRED")
            if not np.array_equal(plan["_start_positions"], checkpoint["positions"]):
                raise RuntimeBusy("E_CHECKPOINT_GEOMETRY")
            self.cancel_pending()
            self.schedule_plan(plan, run_id, "dwell")
            self.queue[-1]["checkpoint"] = checkpoint
            self.runs[run_id]["status"] = "waiting_checkpoint"
            self.archive.save(self.runs[run_id])
            self._emit_event_locked("amendment_waiting", {"run_id": run_id,
                "source_run_id": checkpoint["run_id"], "segment_id": checkpoint["segment_id"]})
            return "safe_dwell"

    def _try_amendment_locked(self) -> None:
        if not self.queue or "checkpoint" not in self.queue[0]:
            return
        item = self.queue[0]
        checkpoint = item["checkpoint"]
        local = self.sim_time - self.active["start_time"] if self.active else float("inf")
        expired = (self.active is None or self.active["run_id"] != checkpoint["run_id"] or local >= checkpoint["deadline"])
        if expired:
            self.cancel_pending()
            self.runs[item["run_id"]]["error"] = "E_SWITCH_NOT_SETTLED"
            self.archive.save(self.runs[item["run_id"]])
            self._emit_event_locked("plan_rejected", {"run_id": item["run_id"],
                "reason": "E_SWITCH_NOT_SETTLED"})
            return
        if local < checkpoint["at"]:
            return
        offset = self.positions - checkpoint["positions"]
        distance = np.max(np.linalg.norm(offset, axis=1))
        speed = np.max(np.linalg.norm(self.velocities, axis=1))
        nominal_gap = item["plan"]["_timeline"]["validation"]["metrics"]["min_distance"]
        relative_offset = max((float(np.max(np.linalg.norm(offset[i] - offset[i + 1:], axis=1)))
                               for i in range(len(offset) - 1)), default=0.)
        # A common settling offset moves all drones together. Relative offsets bound
        # the possible loss of pair separation during the shared 0.5 s start blend.
        if distance > .03 or relative_offset > nominal_gap - .25 - 1e-6 or speed > .05:
            return  # Wait for actual settling; never move coordinates to satisfy the gate.
        self._finalize_active_locked(outcome="superseded")
        self._start_next_locked()

    def cancel_pending(self) -> list[str]:
        with self.cond:
            removed = list(self.queue)
            self.queue.clear()
            cancelled: list[str] = []
            for item in removed:
                run_id = item["run_id"]
                run = self.runs.get(run_id)
                if run is not None:
                    run["status"] = "rejected"
                    run["completed"] = True
                    run["error"] = "E_CANCELLED"
                    run["finished_at"] = datetime.now(timezone.utc).isoformat()
                    self.archive.save(run)
                cancelled.append(run_id)
                self._emit_event_locked("plan_rejected", {
                    "run_id": run_id,
                    "plan_id": item["plan"].get("plan_id"),
                    "reason": "E_CANCELLED",
                })
            self.cond.notify_all()
            return cancelled

    def emit_event(self, kind: str, detail: dict[str, Any]) -> None:
        with self.cond:
            self._emit_event_locked(kind, detail)

    def drain_events(self) -> list[dict[str, Any]]:
        with self.cond:
            out = list(self.events)
            self.events.clear()
            return out

    # -- read-only views ---------------------------------------------------
    def snapshot(self) -> dict[str, Any]:
        with self.cond:
            if self._error is not None:
                status = "error"
            elif self.active is not None or self.queue:
                status = "running"
            elif self.status == "completed":
                status = "completed"
            else:
                status = "idle"
            pending = None
            if self.queue:
                first = self.queue[0]["plan"]
                seg = first["_segments"][0]
                pending = {"id": seg["id"], "name": seg["name"], "progress": 0.0,
                           "color": seg.get("color"),
                           "colors": None if seg.get("colors") is None else list(seg.get("colors"))}
            return {
                "simulation_time": round(self.sim_time, 6),
                "show_elapsed_seconds": (round(max(0., self.sim_time-self.active["start_time"]), 3)
                                         if self.active else None),
                "num_drones": int(self.num_drones),
                "latest_run_id": self._latest_run_id,
                "active_run_id": self.active["run_id"] if self.active else None,
                "status": status,
                "active_segment": None if self.active_segment_public is None
                                  else dict(self.active_segment_public),
                "pending_segment": pending,
                "positions": np.round(self.positions.copy(), 6).tolist(),
                "velocities": np.round(self.velocities.copy(), 6).tolist(),
                "orientations": ([] if self._obs is None else np.round(self._obs[:, 7:10], 6).tolist()),
                "target_positions": np.round(self.target_positions.copy(), 6).tolist(),
                "actual_error": None if self.actual_error is None else dict(self.actual_error),
                "last_error": self._error,
            }

    def public_run(self, run_id: str) -> dict[str, Any] | None:
        with self.cond:
            run = self.runs.get(run_id)
            if run is None:
                return None
            return {
                "run_id": run["run_id"],
                "plan_id": run["plan_id"],
                "status": run["status"],
                "completed": bool(run["completed"]),
                "actual_error": None if run["actual_error"] is None else dict(run["actual_error"]),
                "telemetry_ref": run["telemetry_ref"],
                "summary": None if run["summary"] is None else dict(run["summary"]),
                "error": run["error"],
                "num_drones": run["_num_drones"],
                "created_at": run["created_at"],
                "started_at": run["started_at"],
                "finished_at": run["finished_at"],
                "offline_execution": bool(run.get("offline_execution")),
            }

    def run_arrays(self, run_id: str) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        with self.cond:
            run = self.runs.get(run_id)
            if run is None or run["_actual"] is None:
                return None
            return (run["_times"].copy(), run["_actual"].copy(), run["_target"].copy())

    def latest_run_info(self) -> dict[str, Any]:
        with self.cond:
            run_id = self._latest_run_id
            if run_id is None:
                return {"run_id": None, "status": None, "plan_id": None}
            run = self.runs.get(run_id)
            if run is None:
                return {"run_id": None, "status": None, "plan_id": None}
            return {
                "run_id": run["run_id"],
                "status": run["status"],
                "plan_id": run["plan_id"],
            }

    def run_replay_data(self, run_id: str) -> dict[str, Any] | None:
        with self.cond:
            run = self.runs.get(run_id)
            if run is None or run["_actual"] is None:
                return None
            return {
                "times": run["_times"].copy(),
                "actual": run["_actual"].copy(),
                "target": run["_target"].copy(),
                "plan": run["_plan"],
                "plan_start_time": run.get("plan_start_time"),
                "control_timestep": 1.0 / float(self.ctrl_freq),
            }

    # -- runtime thread ----------------------------------------------------
    def _run_forever(self) -> None:
        try:
            self._create_env_locked(self._requested_num_drones)
            with self.cond:
                self._init_idle_state_locked(self._requested_num_drones)
                self._ready = True
                self.status = "idle"
                self.cond.notify_all()
            next_wall = time.perf_counter()
            while True:
                with self.cond:
                    if self._stop:
                        break
                    if self._reset_request is not None:
                        n = self._reset_request
                        self._close_env_locked()
                        self._create_env_locked(n)
                        self._init_idle_state_locked(n)
                        self._reset_request = None
                        self.status = "idle"
                        self.cond.notify_all()
                    self._try_amendment_locked()
                    if self.active is None and self.queue:
                        self._start_next_locked()
                    paused = self._pause_depth > 0 or (self.offline and self.active is None)
                    target_tick, target_vel_tick, active_now = self._target_for_tick_locked()
                if paused:
                    time.sleep(0.01)
                    continue
                try:
                    action = self._compute_action(target_tick, target_vel_tick)
                    obs, _, _, _, _ = self.env.step(action)
                except Exception as exc:  # pragma: no cover - hardware/runtime failure
                    with self.cond:
                        self._handle_step_error_locked(exc)
                    break
                with self.cond:
                    self._after_step_locked(obs, target_tick)
                    self.cond.notify_all()
                if self.realtime_factor > 0.0:
                    next_wall += (1.0 / self.ctrl_freq) / self.realtime_factor
                    now = time.perf_counter()
                    if next_wall < now:
                        if now - next_wall > 0.5:
                            next_wall = now
                    else:
                        time.sleep(next_wall - now)
        except Exception as exc:  # pragma: no cover
            with self.cond:
                self._error = f"runtime error: {exc}"
                self.status = "error"
                self.cond.notify_all()
        finally:
            with self.cond:
                self._close_env_locked()
                if self._ready and self._error is None:
                    self.status = "idle"
                self.cond.notify_all()

    def _create_env_locked(self, num_drones: int) -> None:
        n = int(num_drones)
        if self.offline:
            from .cinematic import ground_layout
            initial = ground_layout(n)[0]
        else:
            initial = initial_positions(n)
        self.env = CtrlAviary(drone_model=DroneModel.CF2X, num_drones=n,
                              initial_xyzs=initial, initial_rpys=np.zeros((n, 3)),
                              physics=Physics.PYB, pyb_freq=self.pyb_freq,
                              ctrl_freq=self.ctrl_freq, gui=False, record=False,
                              obstacles=False, user_debug_gui=False)
        self.ctrls = [DSLPIDControl(drone_model=DroneModel.CF2X) for _ in range(n)]
        obs, _ = self.env.reset()
        self._obs = np.asarray(obs, dtype=float).copy()
        self.num_drones = n

    def _close_env_locked(self) -> None:
        if self.env is not None:
            try:
                self.env.close()
            except Exception:
                pass
        self.env = None
        self.ctrls = []
        self._obs = None

    def _init_idle_state_locked(self, num_drones: int) -> None:
        n = int(num_drones)
        initial = initial_positions(n)
        if self._obs is not None:
            actual0 = np.asarray(self._obs, dtype=float)[:, 0:3].copy()
            vel0 = np.asarray(self._obs, dtype=float)[:, 10:13].copy()
        else:
            actual0 = initial.copy()
            vel0 = np.zeros((n, 3), dtype=float)
        self.num_drones = n
        self.step_index = 0
        self.sim_time = 0.0
        self.positions = actual0
        self.velocities = vel0
        self.target_positions = actual0.copy()
        self.actual_error = {"max": 0.0, "rms": 0.0}
        self.active = None
        self.active_segment_public = None
        self.queue = []
        self._hold_target = actual0.copy()
        self._error = None

    def _target_for_tick_locked(self) -> tuple[np.ndarray, np.ndarray, bool]:
        if self.active is None:
            if self._hold_target is None:
                self._hold_target = self.positions.copy()
            zeros = np.zeros((self.num_drones, 3), dtype=float)
            return self._hold_target.copy(), zeros, False
        active = self.active
        plan = active["plan"]
        t_local = self.sim_time - active["start_time"]
        if t_local >= float(plan["_total_duration"]):
            zeros = np.zeros((self.num_drones, 3), dtype=float)
            return plan["_segments"][-1]["end_positions"].copy(), zeros, True
        target = self._target_with_blend_locked(active, t_local)
        target_vel = velocity_at(plan, t_local)
        if t_local < START_BLEND_SECONDS:
            target_vel = target_vel - active["offset"] / START_BLEND_SECONDS
        self._update_segment_public_locked(active, t_local)
        return target, target_vel, True

    def _target_with_blend_locked(self, active: dict[str, Any], t_local: float) -> np.ndarray:
        nominal = positions_at(active["plan"], t_local)
        blend = max(0.0, 1.0 - t_local / START_BLEND_SECONDS)
        return nominal + active["offset"] * blend

    def _update_segment_public_locked(self, active: dict[str, Any], t_local: float) -> None:
        info = segment_at(active["plan"], t_local)
        if info is None:
            return
        seg = info["segment"]
        plan_segments = active["plan"]["_segments"]
        idx = plan_segments.index(seg)
        if idx != active["last_segment_index"]:
            if active["last_segment_index"] >= 0:
                old = plan_segments[active["last_segment_index"]]
                self._emit_event_locked("segment_finished", {
                    "segment_id": old["id"], "name": old["name"],
                    "run_id": active["run_id"], "plan_id": active["plan"]["plan_id"],
                })
            self._emit_event_locked("segment_started", {
                "segment_id": seg["id"], "name": seg["name"],
                "run_id": active["run_id"], "plan_id": active["plan"]["plan_id"],
            })
            active["last_segment_index"] = idx
        active["segment_public"] = {
            "id": seg["id"],
            "name": seg["name"],
            "progress": round(float(info["progress"]), 4),
            "color": seg.get("color"),
            "colors": None if seg.get("colors") is None else list(seg.get("colors")),
        }
        self.active_segment_public = active["segment_public"]

    def _compute_action(self, target: np.ndarray, target_vel: np.ndarray) -> np.ndarray:
        if self._obs is None or self.env is None:
            raise RuntimeError("environment is not ready")
        n = self.num_drones
        action = np.zeros((n, 4), dtype=float)
        for i in range(n):
            action[i, :], _, _ = self.ctrls[i].computeControlFromState(
                control_timestep=self.env.CTRL_TIMESTEP,
                state=self._obs[i],
                target_pos=np.asarray(target[i], dtype=float),
                target_rpy=np.zeros(3),
                target_vel=np.asarray(target_vel[i], dtype=float),
            )
        return action

    def _after_step_locked(self, obs: Any, target_tick: np.ndarray) -> None:
        obs = np.asarray(obs, dtype=float)
        self._obs = obs.copy()
        self.step_index += 1
        self.sim_time = float(self.step_index) / float(self.ctrl_freq)
        self.positions = obs[:, 0:3].astype(float).copy()
        self.velocities = obs[:, 10:13].astype(float).copy()
        self.target_positions = np.asarray(target_tick, dtype=float).copy()
        err = np.linalg.norm(self.positions - self.target_positions, axis=1)
        self.actual_error = {
            "max": round(float(np.max(err)), 6),
            "rms": round(float(np.sqrt(np.mean(err ** 2))), 6),
        }
        if self.active is not None:
            active = self.active
            active["telemetry"]["t"].append(self.sim_time)
            active["telemetry"]["actual"].append(self.positions.copy())
            active["telemetry"]["target"].append(self.target_positions.copy())
            active["latest_error"] = dict(self.actual_error)
            plan = active["plan"]
            t_local = self.sim_time - active["start_time"]
            if self.offline:
                spacing = min_pairwise_distance(self.positions)
                unsafe = (spacing is not None and spacing < .25) or self.actual_error["max"] > .40
                unsafe = unsafe or np.max(self.positions[:, 2]) > 3 or np.max(np.linalg.norm(self.positions[:, :2], axis=1)) > 5
                timed_out = time.perf_counter()-active["wall_start"] > 2100
                if unsafe or timed_out:
                    reason = "E_OFFLINE_SAFETY" if unsafe else "E_OFFLINE_DEADLINE"
                    run_id = active["run_id"]
                    self._finalize_active_locked(outcome="rejected")
                    self.runs[run_id]["error"] = reason
                    self.archive.save(self.runs[run_id])
                    return
            if t_local >= float(plan["_total_duration"]) - 1e-9:
                self._finalize_active_locked()

    def _start_next_locked(self) -> None:
        item = self.queue.pop(0)
        plan = item["plan"]
        run_id = item["run_id"]
        first = plan["_segments"][0]
        self.active = {
            "plan": plan,
            "run_id": run_id,
            "start_time": float(self.sim_time),
            "wall_start": time.perf_counter(),
            "start_actual": self.positions.copy(),
            "offset": self.positions - plan["_start_positions"],
            "last_segment_index": -1,
            "segment_public": None,
            "telemetry": {"t": [], "actual": [], "target": []},
            "latest_error": None,
        }
        self.status = "running"
        run = self.runs.get(run_id)
        if run is not None:
            run["status"] = "running"
            run["started_at"] = datetime.now(timezone.utc).isoformat()
            run["plan_start_time"] = float(self.sim_time)
            self.archive.save(run)
        self._update_segment_public_locked(self.active, 0.0)

    def _finalize_active_locked(self, outcome: str = "completed") -> None:
        active = self.active
        if active is None:
            return
        plan = active["plan"]
        run_id = active["run_id"]
        telemetry = active["telemetry"]
        actual = np.asarray(telemetry["actual"], dtype=float)
        target = np.asarray(telemetry["target"], dtype=float)
        times = np.asarray(telemetry["t"], dtype=float)
        summary = self._compute_summary(actual, target, times, plan)
        wall_clock = max(1e-9, time.perf_counter() - active.get("wall_start", time.perf_counter()))
        summary["wall_clock_sec"] = round(float(wall_clock), 6)
        executed = max(0., self.sim_time - active["start_time"])
        summary["executed_duration_sec"] = round(executed, 6)
        summary["execution_outcome"] = outcome
        summary["realtime_factor"] = round(executed / wall_clock, 3)
        telemetry_ref = self._write_telemetry(run_id, actual, target, times, summary)
        run = self.runs.get(run_id)
        if run is not None:
            run["status"] = outcome
            run["completed"] = outcome == "completed"
            run["finished_at"] = datetime.now(timezone.utc).isoformat()
            run["actual_error"] = dict(self.actual_error) if self.actual_error is not None else None
            run["summary"] = summary
            run["telemetry_ref"] = telemetry_ref
            run["_actual"] = actual
            run["_target"] = target
            run["_times"] = times
            self.archive.save(run)
        last_idx = active["last_segment_index"]
        if last_idx >= 0:
            last = plan["_segments"][last_idx]
            self._emit_event_locked("segment_finished", {
                "segment_id": last["id"], "name": last["name"],
                "run_id": run_id, "plan_id": plan["plan_id"],
            })
        self._emit_event_locked("completed" if outcome == "completed" else "amended", {
            "run_id": run_id, "plan_id": plan["plan_id"],
            "summary": summary,
        })
        self.active = None
        self.active_segment_public = None
        self.status = "completed"
        self._hold_target = plan["_segments"][-1]["end_positions"].copy()

    def _handle_step_error_locked(self, exc: Exception) -> None:
        self._error = f"simulation step failed: {exc}"
        self.status = "error"
        if self.active is not None:
            run = self.runs.get(self.active["run_id"])
            if run is not None:
                run["status"] = "error"
                run["completed"] = True
                run["error"] = self._error
                run["finished_at"] = datetime.now(timezone.utc).isoformat()
                self.archive.save(run)
            self._emit_event_locked("plan_rejected", {
                "run_id": self.active["run_id"],
                "plan_id": self.active["plan"]["plan_id"],
                "reason": "E_RUNTIME",
            })
            self.active = None
            self.active_segment_public = None

    def _emit_event_locked(self, kind: str, detail: dict[str, Any]) -> None:
        self.events.append({
            "kind": kind,
            "at": round(self.sim_time, 6),
            "detail": detail,
        })
        if len(self.events) > 1000:
            while len(self.events) > 1000:
                self.events.popleft()

    def _compute_summary(self, actual: np.ndarray, target: np.ndarray,
                         times: np.ndarray, plan: dict[str, Any]) -> dict[str, Any]:
        if actual.size == 0:
            return {
                "num_drones": self.num_drones,
                "samples": 0,
                "completed": True,
                "max_tracking_error_m": None,
                "rms_tracking_error_m": None,
                "min_actual_pairwise_distance_m": None,
                "actual_positions_source": "CtrlAviary.step observation",
                "target_positions_separate": True,
            }
        err = np.linalg.norm(actual - target, axis=2)
        min_actual = min_trajectory_distance(actual)
        min_target = min_trajectory_distance(target)
        return {
            "num_drones": int(self.num_drones),
            "control_freq_hz": int(self.ctrl_freq),
            "samples": int(len(times)),
            "plan_duration_sec": round(float(plan["_total_duration"]), 6),
            "wall_clock_sec": None,
            "max_tracking_error_m": round(float(np.max(err)), 6),
            "rms_tracking_error_m": round(float(np.sqrt(np.mean(err ** 2))), 6),
            "final_tracking_error_m": round(float(np.max(np.linalg.norm(
                actual[-1] - target[-1], axis=1))), 6),
            "min_actual_pairwise_distance_m": None if min_actual is None
                                              else round(float(min_actual), 6),
            "min_target_pairwise_distance_m": None if min_target is None
                                              else round(float(min_target), 6),
            "all_frames_identical": bool(np.all(np.abs(actual - target) < 1e-12)),
            "tracking_error_nonzero_fraction": round(float(np.mean(err > 1e-9)), 6),
            "actual_positions_source": "CtrlAviary.step observation",
            "target_positions_separate": True,
        }

    def _write_telemetry(self, run_id: str, actual: np.ndarray, target: np.ndarray,
                         times: np.ndarray, summary: dict[str, Any]) -> str:
        run_dir = self.output_root / f"run_{run_id}"
        run_dir.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(run_dir / "telemetry.npz", actual=actual,
                            target=target, time=times)
        try:
            (run_dir / "summary.json").write_text(
                json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
        except Exception:
            pass
        return str((run_dir / "telemetry.npz").resolve())
