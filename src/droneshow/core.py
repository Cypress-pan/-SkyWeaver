"""Service core: state, formations, plans, credentials and the runtime facade.

All mutation endpoints serialise through ``ServiceCore.lock`` so revision
checks and queue insertion are atomic.  The simulation itself runs in the
``SimRuntime`` background thread.
"""
from __future__ import annotations

import hashlib
import hmac
import secrets
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from . import __version__
from .archive import RunArchive
from .cjk import CJKError
from .director import ShowError, build_show
from .fleet import FleetError, build_fleet_timeline_plan, fleet_start_positions
from .formations import FormationError, build_formation, min_pairwise_distance
from .planner import PlanError, positions_at, public_plan, segment_at
from .repair import build_plan_with_repair
from .errors import RuntimeBusy
from .runtime_proxy import RuntimeProxy
from .textshow import TextShowError, build_text_show_plan
from .timeline import TimelineError, build_timeline_plan
from .validator import validate_plan
from .telemetry_metrics import min_trajectory_distance


EVALUATE_MAX_ERROR_M = 0.40
EVALUATE_RMS_ERROR_M = 0.20
EVALUATE_MIN_SEPARATION_M = 0.25
EVALUATE_MAX_Z_M = 3.0
EVALUATE_MAX_XY_M = 5.0
ALLOWED_INSERT_AFTER = {"current_segment", "dwell", "immediate"}


class ServiceError(Exception):
    def __init__(self, code: str, status: int = 400, detail: str | None = None,
                 **extra: Any):
        super().__init__(detail or code)
        self.code = code
        self.status = status
        self.detail = detail
        self.extra = extra

    def to_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {"error": self.code}
        if self.detail is not None:
            out["detail"] = self.detail
        out.update(self.extra)
        return out


class ServiceCore:
    def __init__(self, num_drones: int = 16, ctrl_freq: int = 48,
                 pyb_freq: int = 240, realtime_factor: float = 1.0,
                 output_root: str | Path = "results", secret: bytes | None = None,
                 runtime: object | None = None):
        self.lock = threading.RLock()
        self.show_revision = 1
        self.request_revision = 1
        self.formations: dict[str, dict[str, Any]] = {}
        self.plans: dict[str, dict[str, Any]] = {}
        # Kinematic fleet previews are recorded here instead of the simulator:
        # there is no PyBullet run to replay, but the run stays addressable
        # through /api/run/<id> and /api/run/<id>/replay.
        self.preview_runs: dict[str, dict[str, Any]] = {}
        self.presentation_run_id: str | None = None
        self._evaluations: dict[str, dict[str, Any]] = {}
        self.offline_runtime = None
        self.offline_run_ids: set[str] = set()
        self.secret = secret or b"drone-show-local-dev-secret"
        self.output_root = Path(output_root)
        self.archive = RunArchive(self.output_root)
        for key, recorded in self.archive.load(kinematic=True).items():
            self.preview_runs[key] = {
                "public": {k: v for k, v in recorded.items() if not k.startswith("_") and k != "archive_version"},
                "plan": recorded["_plan"], "arrays": (recorded["_times"], recorded["_actual"], recorded["_target"]),
                "started_monotonic": time.monotonic() - recorded["_plan"]["duration"],
                "restored": True,
            }
        if runtime is not None:
            self.runtime = runtime
        else:
            self.runtime = RuntimeProxy(num_drones=num_drones, ctrl_freq=ctrl_freq,
                                        pyb_freq=pyb_freq, realtime_factor=realtime_factor,
                                        output_root=self.output_root)

    def start(self) -> None:
        self.runtime.start()
        if self.preview_runs:
            newest = max(self.preview_runs, key=lambda k: self.preview_runs[k]["public"]["created_at"])
            physical = self.runtime.latest_run_info()
            previous = self.runtime.public_run(physical["run_id"]) if physical.get("run_id") else None
            if previous is None or self.preview_runs[newest]["public"]["created_at"] > previous["created_at"]:
                self.presentation_run_id = newest

    def close(self) -> None:
        self.runtime.close()
        if self.offline_runtime is not None:
            self.offline_runtime.close()

    def _runtime_for(self, run_id):
        offline = getattr(self, "offline_runtime", None)
        return offline if offline is not None and run_id in getattr(self, "offline_run_ids", ()) else self.runtime

    def drain_events(self):
        events = self.runtime.drain_events()
        if self.offline_runtime is not None:
            events.extend(self.offline_runtime.drain_events())
        return events

    def health(self) -> dict[str, Any]:
        return {"ok": True, "service": "drone-show", "version": __version__}

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            state = self._runtime_for(self.presentation_run_id).snapshot()
            state["offline_execution"] = self.presentation_run_id in self.offline_run_ids
            preview = self.preview_runs.get(self.presentation_run_id)
            if preview is not None:
                public = self._preview_status(preview)
                elapsed = min(float(preview["plan"]["duration"]),
                              max(0.0, time.monotonic() - preview["started_monotonic"]))
                info = segment_at(preview["plan"], elapsed)
                segment = info["segment"] if info and public["status"] == "running" else None
                state.update({
                    "positions": positions_at(preview["plan"], elapsed).tolist(),
                    "target_positions": [], "velocities": [], "orientations": [],
                    "num_drones": preview["plan"]["_num_drones"],
                    "simulation_time": elapsed,
                    "show_elapsed_seconds": elapsed,
                    "status": "running" if segment else "idle",
                    "latest_run_id": self.presentation_run_id,
                    "kinematic_preview": True, "motion_mode": "kinematic",
                    "actual_error": None,
                    "active_segment": None if segment is None else {
                        "id": segment["id"], "name": segment["name"],
                        "progress": info["progress"], "colors": segment.get("colors"),
                        "color": segment.get("color")},
                    "pending_segment": None,
                })
            state["show_revision"] = int(self.show_revision)
            state["request_revision"] = int(self.request_revision)
            return state

    def perform(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Plan, validate and start a complete show with safe defaults."""
        if payload.get("theme") == "agnes-light":
            return self._perform_cinematic(payload)
        if self.offline_runtime is not None and self.offline_runtime.snapshot()["status"] == "running":
            raise ServiceError("E_RUNTIME_BUSY", 409, "请等待离线主题秀完成")
        if self.runtime.snapshot()["status"] == "running":
            return self.amend(payload)
        try:
            preview = self.preview_runs.get(self.presentation_run_id)
            default_count = (preview["plan"]["_num_drones"] if preview else
                             self.runtime.snapshot()["num_drones"] if self.presentation_run_id else 128)
            requested = int(payload.get("num_drones", default_count))
        except (ValueError, TypeError) as exc:
            raise ServiceError("E_BAD_REQUEST", 400, "无人机数量必须是整数") from exc
        if requested > 128:
            scenes = payload.get("scenes")
            if (not isinstance(scenes, list) or not scenes or len(scenes) > 12
                    or any(not isinstance(s, dict) for s in scenes)):
                raise ServiceError("E_BAD_REQUEST", 400, "请提供 1 到 12 个场景")
            scenes = [dict(scene, duration_sec=35.0, transition_seconds=5.0)
                      for scene in scenes]
            scenes[-1]["include_landing"] = bool(payload.get("include_landing", True))
            for _ in range(4):
                try:
                    return self.fleet_timeline({"total_drones": requested,
                            "motion_mode": "kinematic", "scenes": scenes,
                            "duration_sec": sum(s["duration_sec"] for s in scenes),
                            "submit": bool(payload.get("submit", True))})
                except ServiceError as exc:
                    if exc.code != "E_DURATION_TOO_SHORT":
                        raise
                    scenes[exc.extra["scene_index"]]["duration_sec"] = float(exc.extra["required_seconds"]) + 1.0
            raise ServiceError("E_SHOW_PLAN", 400, "无法在安全时长内编排此表演")
        with self.lock:
            current = self.runtime.snapshot()
            if current["status"] not in ("idle", "completed"):
                raise ServiceError("E_RUNTIME_BUSY", 409, "请等待当前表演完成后再开始")
            n = requested
            if not 16 <= n <= 128:
                raise ServiceError("E_PHYSICAL_LIMIT", 400, "物理表演支持 16 到 128 架")
            if n != current["num_drones"]:
                self.runtime.ensure_num_drones(n)
                current = self.runtime.snapshot()
                self.show_revision += 1
            self.runtime.begin_planning()
            try:
                plan = build_show(np.asarray(current["positions"], dtype=float),
                                  payload.get("scenes"), self.show_revision, self.secret,
                                  bool(payload.get("include_landing", True)))
            except (ShowError, CJKError, ValueError) as exc:
                raise ServiceError("E_SHOW_PLAN", 400, str(exc)) from exc
            finally:
                self.runtime.end_planning()
            self.plans[plan["plan_id"]] = plan
            run_id = None
            if payload.get("submit", True):
                run_id = "r_" + secrets.token_hex(6)
                self.runtime.schedule_plan(plan, run_id, "immediate")
                self.presentation_run_id = run_id
                self.show_revision += 1
                self.request_revision += 1
            return {"ok": True, "run_id": run_id, "submitted": run_id is not None,
                    "plan_id": plan["plan_id"], "duration": plan["duration"],
                    "num_drones": n, "motion_mode": "physical",
                    "scenes": plan["_timeline"]["scenes"],
                    "validation": plan["_timeline"]["validation"]}

    def _perform_cinematic(self, payload):
        from .cinematic import build_cinematic_plan, ground_layout
        if int(payload.get("num_drones", 1000)) != 1000:
            raise ServiceError("E_BAD_REQUEST", 400, "本主题编排使用 1000 架")
        if self.runtime.snapshot()["status"] == "running":
            raise ServiceError("E_RUNTIME_BUSY", 409, "请等待当前演出结束")
        if self.offline_runtime is not None and self.offline_runtime.snapshot()["status"] == "running":
            raise ServiceError("E_RUNTIME_BUSY", 409, "离线主题秀正在执行")
        if payload.get("preview"):
            plan = build_cinematic_plan(ground_layout()[0], self.show_revision, self.secret)
            plan["_timeline"]["kinematic_preview"] = True
            self.plans[plan["plan_id"]] = plan
            run_id = "r_"+secrets.token_hex(6)
            self._fleet_preview_run(plan, run_id)
            return {"ok": True, "run_id": run_id, "num_drones": 1000, "duration": plan["duration"],
                    "kinematic_preview": True, "validation": plan["_timeline"]["validation"]}
        with self.lock:
            if self.offline_runtime is None:
                worker = RuntimeProxy(num_drones=1000, ctrl_freq=48, pyb_freq=240,
                                      realtime_factor=0, output_root=self.output_root, offline=True)
                worker.start()
                self.offline_runtime = worker
            worker = self.offline_runtime
            state = worker.snapshot()
            if state["status"] not in ("idle", "completed"):
                raise ServiceError("E_RUNTIME_BUSY", 409, "离线主题秀正在执行")
            # Recreate ground identities after a completed show's assignment.
            if state["status"] == "completed":
                worker.close()
                self.offline_runtime = worker = RuntimeProxy(num_drones=1000, ctrl_freq=48, pyb_freq=240,
                                      realtime_factor=0, output_root=self.output_root, offline=True)
                worker.start()
                state = worker.snapshot()
            try:
                plan = build_cinematic_plan(state["positions"], self.show_revision, self.secret)
            except ValueError as exc:
                raise ServiceError("E_SHOW_PLAN", 400, str(exc)) from exc
            self.plans[plan["plan_id"]] = plan
            run_id = None
            if payload.get("submit", True):
                run_id = "r_"+secrets.token_hex(6)
                worker.schedule_plan(plan, run_id, "immediate")
                self.offline_run_ids.add(run_id)
                self.presentation_run_id = run_id
                self.show_revision += 1
                self.request_revision += 1
            return {"ok": True, "run_id": run_id, "submitted": run_id is not None,
                    "plan_id": plan["plan_id"], "duration": plan["duration"],
                    "num_drones": 1000, "motion_mode": "physical", "offline_execution": True,
                    "title": plan["_timeline"]["title"], "scenes": plan["_timeline"]["scenes"],
                    "validation": plan["_timeline"]["validation"]}

    def amend(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            state = self.runtime.snapshot()
            if self.preview_runs.get(self.presentation_run_id) is not None:
                raise ServiceError("E_PREVIEW_ACTIVE", 409, "请等待规模预演结束")
            if int(payload.get("num_drones", state["num_drones"])) != state["num_drones"]:
                raise ServiceError("E_DRONE_MISMATCH", 409, "表演中改编保留当前机队数量")
            if payload.get("request_revision", self.request_revision) != self.request_revision:
                raise ServiceError("E_STALE_REQUEST", 409, "改编请求版本已过期")
            try:
                checkpoint = self.runtime.amendment_checkpoint()
            except RuntimeBusy as exc:
                raise ServiceError(str(exc), 409, "本场已进入收尾，请等待降落后开始") from exc
            revision = self.show_revision
            self.request_revision += 1
            token = self.request_revision
            self.runtime.emit_event("amendment_planning", {"request_revision": token,
                "source_run_id": checkpoint["run_id"]})
        # No planning pause and no core lock: broadcasts and physics continue.
        try:
            plan = build_show(checkpoint["positions"], payload.get("scenes"), revision,
                              self.secret, bool(payload.get("include_landing", True)))
        except (ShowError, CJKError, ValueError) as exc:
            self.runtime.emit_event("plan_rejected", {"reason": "E_SHOW_PLAN", "detail": str(exc)})
            raise ServiceError("E_SHOW_PLAN", 400, str(exc)) from exc
        with self.lock:
            if token != self.request_revision or revision != self.show_revision:
                raise ServiceError("E_STALE_REQUEST", 409, "已有更新的改编请求")
            self.plans[plan["plan_id"]] = plan
            run_id = None
            if payload.get("submit", True):
                run_id = "r_" + secrets.token_hex(6)
                try:
                    self.runtime.schedule_amendment(plan, run_id, checkpoint)
                except RuntimeBusy as exc:
                    raise ServiceError(str(exc), 409, "稳定阶段已错过，原表演继续执行") from exc
                self.presentation_run_id = run_id
                self.show_revision += 1
            return {"ok": True, "run_id": run_id, "plan_id": plan["plan_id"],
                    "submitted": run_id is not None, "scheduled_for": "safe_dwell",
                    "source_run_id": checkpoint["run_id"], "duration": plan["duration"],
                    "num_drones": state["num_drones"], "motion_mode": "physical",
                    "scenes": plan["_timeline"]["scenes"],
                    "validation": plan["_timeline"]["validation"]}

    # -- formations / plans ------------------------------------------------
    def create_formation(self, num_drones: int, spec: dict[str, Any]) -> dict[str, Any]:
        try:
            formation = build_formation(num_drones, spec)
        except FormationError as exc:
            raise ServiceError(exc.code, 400, exc.detail, **exc.extra) from exc
        with self.lock:
            self.formations[formation["formation_id"]] = formation
        return formation

    def create_plan(self, state_revision: int, target_formation_id: str,
                    duration: float, hold_seconds: float,
                    include_landing: bool,
                    color_override: str | None = None,
                    colors_override: list[str] | None = None,
                    gradient_override: list[str] | None = None) -> dict[str, Any]:
        with self.lock:
            if int(state_revision) != int(self.show_revision):
                raise ServiceError("E_STALE_STATE", 409,
                                   "state_revision does not match current show revision",
                                   state_revision=int(self.show_revision))
            formation = self.formations.get(str(target_formation_id))
            if formation is None:
                raise ServiceError("E_NO_FORMATION", 404,
                                   "unknown target_formation_id")
            current = self.runtime.snapshot()
            target_n = int(formation["num_drones"])
            if target_n != int(current["num_drones"]):
                if current["status"] not in ("idle", "completed"):
                    raise ServiceError("E_DRONE_MISMATCH", 409,
                                       "cannot change drone count while a plan is active",
                                       num_drones=int(current["num_drones"]))
                try:
                    self.runtime.ensure_num_drones(target_n)
                except RuntimeBusy as exc:
                    raise ServiceError("E_DRONE_MISMATCH", 409, str(exc)) from exc
                self.show_revision += 1
                current = self.runtime.snapshot()
            try:
                plan = build_plan_with_repair(
                    current["positions"], int(current["num_drones"]),
                    formation, duration, hold_seconds, bool(include_landing),
                    int(self.show_revision), self.secret,
                    color_override=color_override,
                    colors_override=colors_override,
                    gradient_override=gradient_override)
            except PlanError as exc:
                raise ServiceError(exc.code, 400, exc.detail) from exc
            self.plans[plan["plan_id"]] = plan
            return public_plan(plan)

    def validate(self, plan_id: str) -> dict[str, Any]:
        with self.lock:
            plan = self.plans.get(str(plan_id))
            if plan is None:
                raise ServiceError("E_NO_PLAN", 404, "unknown plan_id")
            result = validate_plan(plan)
            return result

    # -- credentials / submission -----------------------------------------
    def _verify_credential(self, plan: dict[str, Any], credential: str | None) -> bool:
        expected = str(plan.get("credential", ""))
        given = "" if credential is None else str(credential)
        return bool(expected) and hmac.compare_digest(expected, given)

    def _reject_event(self, code: str, plan_id: str, run_id: str | None = None) -> None:
        detail = {"reason": code, "plan_id": plan_id}
        if run_id is not None:
            detail["run_id"] = run_id
        self.runtime.emit_event("plan_rejected", detail)

    def text_show(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            current = self.runtime.snapshot()
            text = payload.get("text")
            if not isinstance(text, str) or not text.strip():
                raise ServiceError("E_BAD_SPEC", 400, "text must be a non-empty string")
            n = int(payload.get("num_drones", current["num_drones"]))
            if n < 1:
                raise ServiceError("E_BAD_SPEC", 400, "num_drones must be positive")
            try:
                auto_allocate = bool(payload.get("auto_allocate", True))
                points_per_char = int(payload.get("points_per_char", 32))
                min_points_per_char = int(payload.get("min_points_per_char", 16))
                duration = float(payload.get("transition_seconds",
                                             payload.get("duration", 5.0)))
                hold_seconds = float(payload.get("hold_seconds",
                                                 payload.get("hold", 1.5)))
                z = float(payload.get("z", 1.4))
                base_height = float(payload.get("base_height", 2.4))
                letter_spacing = float(payload.get("letter_spacing", 0.16))
            except (TypeError, ValueError) as exc:
                raise ServiceError("E_BAD_SPEC", 400, "invalid numeric text_show option") from exc
            if points_per_char < 1:
                raise ServiceError("E_BAD_SPEC", 400, "points_per_char must be positive")
            if min_points_per_char < 1:
                raise ServiceError("E_BAD_SPEC", 400, "min_points_per_char must be positive")
            if duration <= 0.0:
                raise ServiceError("E_BAD_SPEC", 400, "duration must be positive")
            if hold_seconds < 0.0:
                raise ServiceError("E_BAD_SPEC", 400, "hold_seconds must be non-negative")
            include_landing = bool(payload.get("include_landing", True))
            submit = bool(payload.get("submit", True))
            insert_after = str(payload.get("insert_after", "immediate"))
            if insert_after not in ALLOWED_INSERT_AFTER:
                raise ServiceError("E_BAD_REQUEST", 400,
                                   "insert_after must be one of "
                                   + ", ".join(sorted(ALLOWED_INSERT_AFTER)))

            source_revision = int(payload.get("source_state_revision", self.show_revision))
            request_revision = int(payload.get("request_revision", self.request_revision))
            if source_revision != int(self.show_revision):
                raise ServiceError("E_STALE_STATE", 409,
                                   "source_state_revision is stale",
                                   state_revision=int(self.show_revision))
            if request_revision < int(self.request_revision):
                raise ServiceError("E_STALE_STATE", 409,
                                   "request_revision is stale",
                                   request_revision=int(self.request_revision))

            if n != int(current["num_drones"]):
                if current["status"] not in ("idle", "completed"):
                    raise ServiceError("E_DRONE_MISMATCH", 409,
                                       "cannot change drone count while a plan is active",
                                       num_drones=int(current["num_drones"]))
                try:
                    self.runtime.ensure_num_drones(n)
                except RuntimeBusy as exc:
                    raise ServiceError("E_DRONE_MISMATCH", 409, str(exc)) from exc
                self.show_revision += 1
                source_revision = int(self.show_revision)
                current = self.runtime.snapshot()

            self.runtime.begin_planning()
            try:
                try:
                    plan = build_text_show_plan(
                        current["positions"], n, text, duration, hold_seconds,
                        include_landing, source_revision, self.secret,
                        points_per_char=points_per_char,
                        auto_allocate=auto_allocate,
                        min_points_per_char=min_points_per_char,
                        font_path=payload.get("font_path"),
                        z=z,
                        color=payload.get("color"),
                        colors=payload.get("colors"),
                        gradient=payload.get("gradient"),
                        base_height=base_height,
                        letter_spacing=letter_spacing,
                    )
                except CJKError as exc:
                    status = 503 if exc.code == "E_FONT_UNAVAILABLE" else 400
                    raise ServiceError(exc.code, status, exc.detail, **exc.extra) from exc
                except TextShowError as exc:
                    raise ServiceError(exc.code, 400, exc.detail, **exc.extra) from exc
                except PlanError as exc:
                    raise ServiceError(exc.code, 400, exc.detail) from exc

                validation = validate_plan(plan)
            finally:
                self.runtime.end_planning()
            if not validation["valid"]:
                raise ServiceError("E_VALIDATION_FAILED", 400,
                                   "text show plan failed fixed validation",
                                   plan_id=plan["plan_id"],
                                   issues=validation["issues"],
                                   metrics=validation["metrics"])
            self.plans[plan["plan_id"]] = plan

            submission = None
            run_id = None
            if submit:
                run_id = "r_" + secrets.token_hex(6)
                try:
                    scheduled_for = self.runtime.schedule_plan(plan, run_id, insert_after)
                except RuntimeBusy as exc:
                    raise ServiceError("E_RUNTIME_BUSY", 409, str(exc)) from exc
                self.presentation_run_id = run_id
                self.show_revision += 1
                self.request_revision = max(int(self.request_revision), request_revision) + 1
                submission = {"accepted": True, "run_id": run_id,
                              "scheduled_for": scheduled_for}

            meta = plan.get("_text_show", {})
            pagination = meta.get("pagination", {})
            page_formations = meta.get("page_formations", [])
            allocation_report = meta.get("allocation_report", [])
            scheduled_for = submission.get("scheduled_for") if submission else None
            public_plan_dict = public_plan(plan)
            characters = list(meta.get("characters", list(text)))
            allocation = {
                "num_drones": n,
                "auto_allocate": bool(pagination.get("auto_allocate", True)),
                "points_per_char": points_per_char,
                "min_points_per_char": min_points_per_char,
                "pages": [
                    {
                        "page_index": page["page_index"],
                        "characters": page["characters"],
                        "allocations": page["allocations"],
                        "points": page["points"],
                        "idle": page["idle"],
                    }
                    for page in pagination.get("pages", [])
                ],
                "used_points_per_page": [page["points"] for page in pagination.get("pages", [])],
                "total_rendered_points": int(sum(page["points"] for page in pagination.get("pages", []))),
            }
            analysis = {
                "text": meta.get("text", text),
                "characters": characters,
                "character_count": len(characters),
                "unique_characters": sorted(set(characters)),
            }
            return {
                "ok": True,
                "text": meta.get("text", text),
                "num_drones": n,
                "pages": [
                    {
                        "page": page.get("page"),
                        "text": page.get("text"),
                        "formation_id": page.get("formation_id"),
                        "points": page.get("points"),
                        "used": page.get("used"),
                        "idle": page.get("idle"),
                        "min_distance": page.get("min_distance"),
                        "characters": page.get("characters", []),
                        "allocations": page.get("allocations", []),
                    }
                    for page in page_formations
                ],
                "allocation_report": allocation_report,
                "plan_id": plan["plan_id"],
                "plan_hash": plan["plan_hash"],
                "validation": validation,
                "submitted": bool(submission is not None and submission.get("accepted")),
                "run_id": run_id,
                "scheduled_for": scheduled_for,
                "font": meta.get("font"),
                "analysis": analysis,
                "allocation": allocation,
                "pagination": pagination,
                "plan": public_plan_dict,
                "submission": submission,
            }

    def timeline(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            current = self.runtime.snapshot()
            n = int(payload.get("num_drones", current["num_drones"]))
            if n < 1:
                raise ServiceError("E_BAD_SPEC", 400, "num_drones must be positive")
            duration_sec = float(payload.get("duration_sec", 0.0))
            scenes = payload.get("scenes")
            if not isinstance(scenes, list) or len(scenes) == 0:
                raise ServiceError("E_BAD_SPEC", 400, "scenes must be a non-empty array")
            submit = bool(payload.get("submit", True))
            insert_after = str(payload.get("insert_after", "immediate"))
            if insert_after not in ALLOWED_INSERT_AFTER:
                raise ServiceError("E_BAD_REQUEST", 400, "invalid insert_after")
            source_revision = int(payload.get("source_state_revision", self.show_revision))
            request_revision = int(payload.get("request_revision", self.request_revision))
            if source_revision != int(self.show_revision):
                raise ServiceError("E_STALE_STATE", 409,
                                   "source_state_revision is stale",
                                   state_revision=int(self.show_revision))
            if request_revision < int(self.request_revision):
                raise ServiceError("E_STALE_STATE", 409, "request_revision is stale")
            if n != int(current["num_drones"]):
                if current["status"] not in ("idle", "completed"):
                    raise ServiceError("E_DRONE_MISMATCH", 409,
                                       "cannot change drone count while active")
                try:
                    self.runtime.ensure_num_drones(n)
                except RuntimeBusy as exc:
                    raise ServiceError("E_DRONE_MISMATCH", 409, str(exc)) from exc
                self.show_revision += 1
                source_revision = int(self.show_revision)
                current = self.runtime.snapshot()
            self.runtime.begin_planning()
            try:
                try:
                    plan = build_timeline_plan(
                        current["positions"], n, duration_sec, scenes,
                        source_revision, self.secret,
                        font_path=payload.get("font_path"))
                except TimelineError as exc:
                    status = 503 if exc.code == "E_FONT_UNAVAILABLE" else 400
                    raise ServiceError(exc.code, status, exc.detail, **exc.extra) from exc
                except PlanError as exc:
                    raise ServiceError(exc.code, 400, exc.detail) from exc
                validation = plan.get("_timeline", {}).get("validation")
                if validation is None:
                    validation = validate_plan(plan)
            finally:
                self.runtime.end_planning()
            if not validation.get("valid"):
                raise ServiceError("E_VALIDATION_FAILED", 400,
                                   "timeline plan failed fixed validation",
                                   plan_id=plan["plan_id"],
                                   issues=validation.get("issues", []),
                                   metrics=validation.get("metrics", {}))
            self.plans[plan["plan_id"]] = plan
            submission = None
            run_id = None
            scheduled_for = None
            if submit:
                run_id = "r_" + secrets.token_hex(6)
                try:
                    scheduled_for = self.runtime.schedule_plan(plan, run_id, insert_after)
                except RuntimeBusy as exc:
                    raise ServiceError("E_RUNTIME_BUSY", 409, str(exc)) from exc
                self.presentation_run_id = run_id
                self.show_revision += 1
                self.request_revision = max(int(self.request_revision), request_revision) + 1
                submission = {"accepted": True, "run_id": run_id,
                              "scheduled_for": scheduled_for}
            meta = plan.get("_timeline", {})
            return {
                "ok": True,
                "plan_id": plan["plan_id"],
                "plan_hash": plan["plan_hash"],
                "duration": round(float(plan["duration"]), 6),
                "num_drones": n,
                "scenes": meta.get("scenes", []),
                "validation": validation,
                "submitted": bool(submission is not None),
                "run_id": run_id,
                "scheduled_for": scheduled_for,
                "repaired": bool(plan.get("repaired", False)),
                "warnings": meta.get("warnings", []),
                "submission": submission,
            }

    # -- fleet timeline (task-16) ------------------------------------------
    def _fleet_physical_limit(self) -> int:
        """Largest fleet this service is provisioned to simulate physically."""
        try:
            return int(self.runtime.snapshot()["num_drones"])
        except Exception:
            return 0

    def _build_fleet_plan(self, start: np.ndarray, total: int, duration: float,
                          scenes: list[dict[str, Any]], motion_mode: str,
                          source_revision: int,
                          font_path: str | None) -> dict[str, Any]:
        try:
            return build_fleet_timeline_plan(
                start, total, duration, scenes, motion_mode, source_revision,
                self.secret, font_path=font_path)
        except FleetError as exc:
            status = 503 if exc.code == "E_FONT_UNAVAILABLE" else 400
            raise ServiceError(exc.code, status, exc.detail, **exc.extra) from exc
        except PlanError as exc:
            raise ServiceError(exc.code, 400, exc.detail) from exc

    def _fleet_preview_run(self, plan: dict[str, Any], run_id: str) -> None:
        """Record a kinematic preview run (no physical simulation)."""
        total = float(plan["_total_duration"])
        count = int(max(2, min(600, round(total * 15.0) + 1)))
        times = np.linspace(0.0, total, count)
        positions = np.stack([positions_at(plan, float(t)) for t in times], axis=0)
        now = datetime.now(timezone.utc).isoformat()
        self.preview_runs[run_id] = {
            "public": {
                "run_id": run_id,
                "plan_id": plan["plan_id"],
                "status": "running",
                "completed": False,
                "actual_error": None,
                "telemetry_ref": None,
                "summary": {
                    "kinematic_preview": True,
                    "num_drones": int(plan["_num_drones"]),
                    "duration": round(total, 6),
                    "plan_hash": plan.get("plan_hash"),
                },
                "error": None,
                "num_drones": int(plan["_num_drones"]),
                "created_at": now,
                "started_at": now,
                "finished_at": None,
            },
            "arrays": (times, positions, positions.copy()),
            "plan": plan,
            "started_monotonic": time.monotonic(),
        }
        self.presentation_run_id = run_id
        self._save_preview(self.preview_runs[run_id])

    def _save_preview(self, preview):
        public = preview["public"]
        record = dict(public, _plan=preview["plan"],
                      _num_drones=preview["plan"]["_num_drones"], kinematic_preview=True)
        self.archive.save(record)
        times, actual, target = preview["arrays"]
        if actual is not None:
            directory = self.output_root / ("run_" + public["run_id"])
            path = directory / "telemetry.npz"
            if not path.exists():
                np.savez_compressed(path, actual=actual, target=target, time=times)

    def _preview_status(self, preview: dict[str, Any]) -> dict[str, Any]:
        public = preview["public"]
        if preview.get("restored"):
            return dict(public)
        if public["status"] == "running" and time.monotonic() - preview["started_monotonic"] >= preview["plan"]["duration"]:
            public.update(status="completed", completed=True,
                          finished_at=datetime.now(timezone.utc).isoformat())
            self._save_preview(preview)
        return dict(public)

    def fleet_timeline(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Plan (and optionally submit) a whole-fleet layered timeline."""
        with self.lock:
            if not isinstance(payload, dict):
                raise ServiceError("E_BAD_REQUEST", 400,
                                   "payload must be a JSON object")
            try:
                total = int(payload.get("total_drones", 0))
            except (TypeError, ValueError) as exc:
                raise ServiceError("E_BAD_REQUEST", 400,
                                   "total_drones must be an integer") from exc
            if total < 1:
                raise ServiceError("E_BAD_REQUEST", 400,
                                   "total_drones must be positive")
            try:
                duration = float(payload.get("duration_sec", 0.0))
            except (TypeError, ValueError) as exc:
                raise ServiceError("E_BAD_REQUEST", 400,
                                   "duration_sec must be a number") from exc
            if duration <= 0.0:
                raise ServiceError("E_BAD_REQUEST", 400,
                                   "duration_sec must be positive")
            scenes = payload.get("scenes")
            if not isinstance(scenes, list) or len(scenes) == 0:
                raise ServiceError("E_BAD_REQUEST", 400,
                                   "scenes must be a non-empty array")
            scene_total = 0.0
            for index, scene in enumerate(scenes):
                if not isinstance(scene, dict):
                    raise ServiceError("E_BAD_SCENE", 400,
                                       "scene " + str(index) + " must be an object")
                try:
                    scene_total += float(scene.get("duration_sec", 0.0))
                except (TypeError, ValueError) as exc:
                    raise ServiceError("E_BAD_SCENE", 400,
                                       "scene " + str(index)
                                       + " duration_sec must be a number") from exc
            if abs(scene_total - duration) > 1e-6:
                raise ServiceError("E_DURATION_MISMATCH", 400,
                                   "scene duration sum does not match duration_sec",
                                   duration_sec=duration,
                                   scene_total_sec=scene_total)

            physical_limit = self._fleet_physical_limit()
            motion_mode = payload.get("motion_mode")
            if motion_mode is None:
                motion_mode = ("physical" if total <= physical_limit
                               else "kinematic")
            motion_mode = str(motion_mode).lower()
            if motion_mode not in ("kinematic", "physical"):
                raise ServiceError("E_BAD_REQUEST", 400,
                                   "motion_mode must be 'kinematic' or 'physical'")
            if motion_mode == "physical" and total > physical_limit:
                raise ServiceError(
                    "E_PHYSICAL_LIMIT", 400,
                    "physical mode is unavailable above the provisioned fleet "
                    "limit of " + str(physical_limit)
                    + " drones; use motion_mode='kinematic' for a preview",
                    total_drones=total, physical_limit=physical_limit)
            kinematic = motion_mode == "kinematic"
            submit = bool(payload.get("submit", False))
            source_revision = int(payload.get("source_state_revision",
                                              self.show_revision))
            if source_revision != int(self.show_revision):
                raise ServiceError("E_STALE_STATE", 409,
                                   "source_state_revision is stale",
                                   state_revision=int(self.show_revision))
            font_path = payload.get("font_path")

            if kinematic:
                try:
                    prior = self.preview_runs.get(self.presentation_run_id)
                    if prior is not None and prior["plan"]["_num_drones"] == total:
                        elapsed = time.monotonic() - prior["started_monotonic"]
                        start = positions_at(prior["plan"], elapsed)
                    else:
                        start = fleet_start_positions(total)
                except FleetError as exc:
                    raise ServiceError(exc.code, 400, exc.detail,
                                       **exc.extra) from exc
                plan = self._build_fleet_plan(start, total, duration, scenes,
                                              motion_mode, source_revision,
                                              font_path)
            else:
                current = self.runtime.snapshot()
                if total != int(current["num_drones"]):
                    if current["status"] not in ("idle", "completed"):
                        raise ServiceError("E_DRONE_MISMATCH", 409,
                                           "cannot change drone count while active")
                    try:
                        self.runtime.ensure_num_drones(total)
                    except RuntimeBusy as exc:
                        raise ServiceError("E_DRONE_MISMATCH", 409, str(exc)) from exc
                    self.show_revision += 1
                    source_revision = int(self.show_revision)
                    current = self.runtime.snapshot()
                self.runtime.begin_planning()
                try:
                    plan = self._build_fleet_plan(
                        np.asarray(current["positions"], dtype=float), total,
                        duration, scenes, motion_mode, source_revision, font_path)
                finally:
                    self.runtime.end_planning()

            validation = plan.get("_timeline", {}).get("validation")
            if validation is None:
                validation = validate_plan(plan)
            if not validation.get("valid"):
                raise ServiceError("E_VALIDATION_FAILED", 400,
                                   "fleet timeline plan failed fixed validation",
                                   plan_id=plan["plan_id"],
                                   issues=validation.get("issues", []),
                                   metrics=validation.get("metrics", {}))
            self.plans[plan["plan_id"]] = plan
            meta = plan.get("_timeline", {})
            warnings = [dict(warning) for warning in meta.get("warnings", [])]

            scene_allocations = []
            for position, summary in enumerate(meta.get("scenes", [])):
                scene_allocations.append({
                    "scene_index": summary.get("scene_index", position),
                    "type": summary.get("type"),
                    "drone_budget": summary.get("drone_budget", "auto"),
                    "active": summary.get("active"),
                    "idle": summary.get("idle"),
                    "points_per_char": summary.get("points_per_char", 0),
                    "min_distance": summary.get("min_distance"),
                    "layers": summary.get("layers"),
                    "z_levels": summary.get("z_levels", []),
                    "formation_radius": summary.get("formation_radius"),
                    "transition_seconds": summary.get("transition_seconds"),
                    "effect": summary.get("effect"),
                    "landing": bool(summary.get("landing")),
                    "formation_id": summary.get("formation_id"),
                    "parking": summary.get("parking"),
                })

            run_id = None
            scheduled_for = None
            submission = None
            if submit:
                run_id = "r_" + secrets.token_hex(6)
                if kinematic:
                    warnings.append({
                        "code": "KINEMATIC_PREVIEW",
                        "detail": "large fleet preview recorded without starting "
                                  "a physical simulation",
                    })
                    self._fleet_preview_run(plan, run_id)
                    self.show_revision += 1
                    self.request_revision += 1
                    scheduled_for = "kinematic_preview"
                    submission = {"accepted": True, "run_id": run_id,
                                  "scheduled_for": scheduled_for,
                                  "kinematic_preview": True}
                else:
                    try:
                        scheduled_for = self.runtime.schedule_plan(plan, run_id,
                                                                   "immediate")
                    except RuntimeBusy as exc:
                        raise ServiceError("E_RUNTIME_BUSY", 409, str(exc)) from exc
                    request_revision = int(payload.get("request_revision",
                                                       self.request_revision))
                    self.show_revision += 1
                    self.request_revision = max(int(self.request_revision),
                                                request_revision) + 1
                    submission = {"accepted": True, "run_id": run_id,
                                  "scheduled_for": scheduled_for}
                    self.presentation_run_id = run_id

            return {
                "ok": True,
                "total_drones": total,
                "duration": round(float(plan["duration"]), 6),
                "motion_mode": motion_mode,
                "kinematic_preview": bool(kinematic),
                "scene_allocations": scene_allocations,
                "plan_id": plan["plan_id"],
                "plan_hash": plan["plan_hash"],
                "validation": validation,
                "submitted": bool(submission is not None),
                "run_id": run_id,
                "scheduled_for": scheduled_for,
                "warnings": warnings,
                "submission": submission,
            }

    def submit(self, plan_id: str, credential: str | None, request_revision: int,
               source_state_revision: int, insert_after: str) -> dict[str, Any]:
        with self.lock:
            plan = self.plans.get(str(plan_id))
            if plan is None:
                raise ServiceError("E_NO_PLAN", 404, "unknown plan_id")
            if not self._verify_credential(plan, credential):
                self._reject_event("E_BAD_CREDENTIAL", str(plan_id))
                raise ServiceError("E_BAD_CREDENTIAL", 403,
                                   "credential does not match plan")
            if str(insert_after) not in ALLOWED_INSERT_AFTER:
                raise ServiceError("E_BAD_REQUEST", 400,
                                   "insert_after must be one of "
                                   + ", ".join(sorted(ALLOWED_INSERT_AFTER)))
            if int(source_state_revision) != int(self.show_revision):
                self._reject_event("E_STALE_STATE", str(plan_id))
                raise ServiceError("E_STALE_STATE", 409,
                                   "source_state_revision is stale",
                                   state_revision=int(self.show_revision))
            if int(request_revision) < int(self.request_revision):
                self._reject_event("E_STALE_STATE", str(plan_id))
                raise ServiceError("E_STALE_STATE", 409,
                                   "request_revision is stale",
                                   state_revision=int(self.show_revision),
                                   request_revision=int(self.request_revision))
            if int(plan.get("source_state_revision", -1)) != int(source_state_revision):
                self._reject_event("E_STALE_STATE", str(plan_id))
                raise ServiceError("E_STALE_STATE", 409,
                                   "plan was built for a different state revision",
                                   state_revision=int(self.show_revision))
            current = self.runtime.snapshot()
            if int(plan.get("_num_drones", -1)) != int(current["num_drones"]):
                raise ServiceError("E_DRONE_MISMATCH", 409,
                                   "plan drone count does not match runtime",
                                   num_drones=int(current["num_drones"]))
            run_id = "r_" + secrets.token_hex(6)
            try:
                scheduled_for = self.runtime.schedule_plan(plan, run_id, str(insert_after))
            except RuntimeBusy as exc:
                raise ServiceError("E_RUNTIME_BUSY", 409, str(exc)) from exc
            self.show_revision += 1
            self.request_revision = max(int(self.request_revision), int(request_revision)) + 1
            self.presentation_run_id = run_id
            return {"accepted": True, "run_id": run_id, "scheduled_for": scheduled_for}

    def cancel_pending(self, request_revision: int) -> dict[str, Any]:
        with self.lock:
            if int(request_revision) < int(self.request_revision):
                raise ServiceError("E_STALE_STATE", 409, "request_revision is stale",
                                   request_revision=int(self.request_revision))
            removed = self.runtime.cancel_pending()
            if removed:
                self.request_revision += 1
            return {"cancelled": bool(removed), "run_ids": removed}

    def get_run(self, run_id: str) -> dict[str, Any]:
        run = self._runtime_for(run_id).public_run(str(run_id))
        if run is None:
            preview = self.preview_runs.get(str(run_id))
            if preview is not None:
                return self._preview_status(preview)
            raise ServiceError("E_NO_RUN", 404, "unknown run_id")
        return run

    def latest_run(self) -> dict[str, Any]:
        if self.presentation_run_id in self.preview_runs:
            return self._preview_status(self.preview_runs[self.presentation_run_id])
        return self._runtime_for(self.presentation_run_id).latest_run_info()

    def run_progress(self, run_id: str) -> dict[str, Any]:
        """Small, run-specific read view for the AGH live card.

        A queued amendment must never inherit the previous run's phase.
        Recorded completion and independent actual checks remain distinct.
        """
        run = self.get_run(run_id)
        plan = self.plans.get(run["plan_id"])
        duration = (plan["duration"] if plan else
                    (run.get("summary") or {}).get("plan_duration_sec"))
        elapsed, segment = None, None
        if run["status"] == "running":
            state = self.snapshot()
            if state.get("active_run_id") == run_id or (
                state.get("kinematic_preview") and state.get("latest_run_id") == run_id
            ):
                elapsed = state.get("show_elapsed_seconds")
                segment = state.get("active_segment")
        if run["status"] == "completed":
            elapsed = duration
        evaluation = self.evaluate_run(run_id) if run["status"] == "completed" else None
        return {"run_id": run_id, "status": run["status"],
                "completed": run["completed"], "num_drones": run["num_drones"],
                "elapsed_seconds": elapsed, "duration_seconds": duration,
                "segment": None if segment is None else {
                    "name": segment["name"], "progress": segment.get("progress")},
                "kinematic_preview": bool(run.get("kinematic_preview")),
                "evaluation": evaluation, "error": run.get("error")}

    def replay_run(self, run_id: str, hz: int = 15) -> dict[str, Any]:
        run_id = str(run_id)
        runtime = self._runtime_for(run_id)
        run = runtime.public_run(run_id)
        preview = None
        if run is None:
            preview = self.preview_runs.get(run_id)
            if preview is None:
                raise ServiceError("E_NO_RUN", 404, "unknown run_id")
            run = self._preview_status(preview)
        data = runtime.run_replay_data(run_id)
        if data is None and preview is not None:
            times, actual, target = preview["arrays"]
            if any(value is None for value in (times, actual, target)):
                raise ServiceError("E_NO_TELEMETRY", 409, "预演记录没有可回放的轨迹")
            data = {
                "times": times.copy(),
                "actual": actual.copy(),
                "target": target.copy(),
                "plan": preview["plan"],
                "plan_start_time": 0.0,
                "control_timestep": 1.0 / 15.0,
            }
        if data is None:
            raise ServiceError("E_NO_TELEMETRY", 409,
                               "run has no completed telemetry to replay",
                               run_id=run_id, status=run["status"])
        try:
            requested_hz = int(hz)
        except (TypeError, ValueError):
            requested_hz = 15
        requested_hz = max(5, min(30, requested_hz))

        times = np.asarray(data["times"], dtype=float)
        actual = np.asarray(data["actual"], dtype=float)
        target = np.asarray(data["target"], dtype=float)
        plan = data.get("plan")
        if plan is None:
            raise ServiceError("E_NO_PLAN", 404, "run has no stored plan")
        plan_start_time = data.get("plan_start_time")
        total = len(times)
        if total == 0:
            indices = np.array([], dtype=int)
            replay_times = np.array([], dtype=float)
        else:
            duration = max(0.0, float(times[-1] - times[0]))
            target_count = int(round(duration * requested_hz)) + 1
            target_count = max(2, min(2400, target_count))
            if total <= target_count:
                indices = np.arange(total, dtype=int)
            else:
                indices = np.unique(np.linspace(0, total - 1, target_count).astype(int))
            replay_times = times[indices] - float(times[0])

        frames: list[dict[str, Any]] = []
        for position, index in enumerate(indices):
            raw_time = float(times[index])
            if plan_start_time is None:
                local_time = float(replay_times[position])
            else:
                local_time = max(0.0, raw_time - float(plan_start_time))
            info = segment_at(plan, local_time)
            active_segment: dict[str, Any] | None = None
            if info is not None:
                segment = info["segment"]
                active_segment = {
                    "id": segment["id"],
                    "name": segment["name"],
                    "progress": round(float(info["progress"]), 4),
                    "color": segment.get("color"),
                    "colors": None if segment.get("colors") is None
                              else list(segment.get("colors")),
                }
            frames.append({
                "t": round(float(replay_times[position]), 6),
                "positions": np.round(actual[index], 6).tolist(),
                "target_positions": np.round(target[index], 6).tolist(),
                "active_segment": active_segment,
            })
        return {
            "run_id": run_id,
            "status": run["status"],
            "plan_id": run["plan_id"],
            "kinematic_preview": preview is not None,
            "offline_execution": bool(run.get("offline_execution")),
            "hz": requested_hz,
            "actual_positions_source": (
                "kinematic preview of the signed nominal plan"
                if preview is not None else "CtrlAviary.step observation"),
            "target_positions_separate": True,
            "frames": frames,
        }

    # -- experiment evaluation ---------------------------------------------
    def evaluate_run(self, run_id: str) -> dict[str, Any]:
        with self.lock:
            if str(run_id) in self._evaluations:
                return self._evaluations[str(run_id)]
        run = self.get_run(str(run_id))
        if str(run_id) in self.preview_runs:
            plan = self.preview_runs[str(run_id)]["plan"]
            return {"ok": True, "run_id": str(run_id), "verdict": "preview",
                    "kinematic_preview": True,
                    "checks": [{"name": "nominal_safety", "pass": True,
                                "detail": "规划轨迹通过冻结检查；未进行物理仿真"}],
                    "metrics": plan["_timeline"]["validation"]["metrics"]}
        arrays = self._runtime_for(run_id).run_arrays(str(run_id))
        if arrays is None:
            preview = self.preview_runs.get(str(run_id))
            if preview is not None:
                arrays = preview["arrays"]
        if arrays is None:
            checks = [
                {"name": "completed", "pass": False,
                 "detail": f"run status is {run['status']}"},
                {"name": "tracking", "pass": False,
                 "detail": "telemetry not available yet"},
                {"name": "spacing", "pass": False,
                 "detail": "telemetry not available yet"},
                {"name": "bounds", "pass": False,
                 "detail": "telemetry not available yet"},
            ]
            return {"ok": True, "run_id": str(run_id), "verdict": "inconclusive",
                    "checks": checks,
                    "metrics": {"max_error": None, "rms_error": None,
                                "min_distance": None, "samples": 0}}
        _, actual, target = arrays
        err = np.linalg.norm(actual - target, axis=2)
        max_error = float(np.max(err))
        rms_error = float(np.sqrt(np.mean(err ** 2)))
        final_error = float(np.max(np.linalg.norm(actual[-1] - target[-1], axis=1)))
        min_distance = min_trajectory_distance(actual)
        min_z = float(np.min(actual[:, :, 2]))
        max_z = float(np.max(actual[:, :, 2]))
        max_xy = float(np.max(np.linalg.norm(actual[:, :, :2], axis=2)))
        completed = bool(run["completed"]) and run["status"] == "completed"
        checks = [
            {"name": "completed", "pass": completed,
             "detail": f"run status {run['status']} with {len(actual)} samples"},
            {"name": "tracking",
             "pass": max_error <= EVALUATE_MAX_ERROR_M and rms_error <= EVALUATE_RMS_ERROR_M,
             "detail": f"max {max_error:.4f} m <= {EVALUATE_MAX_ERROR_M:.2f} m, "
                       f"rms {rms_error:.4f} m <= {EVALUATE_RMS_ERROR_M:.2f} m"},
            {"name": "spacing",
             "pass": min_distance is None or min_distance >= EVALUATE_MIN_SEPARATION_M,
             "detail": ("single drone" if min_distance is None else
                        f"min actual pairwise {min_distance:.4f} m >= "
                        f"{EVALUATE_MIN_SEPARATION_M:.2f} m")},
            {"name": "bounds",
             "pass": min_z >= -0.05 and max_z <= EVALUATE_MAX_Z_M and max_xy <= EVALUATE_MAX_XY_M,
             "detail": f"z in [{min_z:.4f}, {max_z:.4f}] m, max xy {max_xy:.4f} m"},
        ]
        if not completed:
            verdict = "inconclusive"
        elif all(c["pass"] for c in checks):
            verdict = "pass"
        else:
            verdict = "fail"
        result = {
            "ok": True,
            "run_id": str(run_id),
            "verdict": verdict,
            "checks": checks,
            "metrics": {
                "max_error": round(max_error, 6),
                "rms_error": round(rms_error, 6),
                "min_distance": None if min_distance is None else round(min_distance, 6),
                "final_error": round(final_error, 6),
                "samples": int(len(actual)),
                "num_drones": int(run["num_drones"]),
                "min_z": round(min_z, 6),
                "max_z": round(max_z, 6),
                "max_xy": round(max_xy, 6),
            },
        }
        if completed:
            with self.lock:
                self._evaluations[str(run_id)] = result
                while len(self._evaluations) > 32:
                    del self._evaluations[next(iter(self._evaluations))]
        return result
