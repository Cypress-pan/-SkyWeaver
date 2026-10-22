"""Tornado HTTP + WebSocket surface for the drone-show service."""
from __future__ import annotations

import json
import asyncio
import mimetypes
import secrets
import struct
from pathlib import Path
from typing import Any

import tornado.ioloop
import tornado.web
import tornado.websocket

from . import __version__
from .core import ServiceCore, ServiceError


def _decode_body(handler: tornado.web.RequestHandler) -> dict[str, Any]:
    body = handler.request.body or b"{}"
    try:
        parsed = json.loads(body.decode("utf-8"))
    except Exception as exc:
        raise ServiceError("E_BAD_JSON", 400, f"invalid JSON body: {exc}") from exc
    if not isinstance(parsed, dict):
        raise ServiceError("E_BAD_JSON", 400, "JSON body must be an object")
    return parsed


class BaseHandler(tornado.web.RequestHandler):
    def initialize(self, core: ServiceCore) -> None:
        self.core = core

    def set_default_headers(self) -> None:
        self.set_header("Content-Type", "application/json; charset=UTF-8")
        self.set_header("Access-Control-Allow-Origin", "*")
        self.set_header("Access-Control-Allow-Headers", "Content-Type")
        self.set_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")

    def write_json(self, payload: dict[str, Any], status: int = 200) -> None:
        self.set_status(status)
        self.write(json.dumps(payload, ensure_ascii=False))

    def options(self, *args: Any, **kwargs: Any) -> None:
        self.set_status(204)
        self.finish()

    def write_error(self, status_code: int, **kwargs: Any) -> None:
        if self._finished:
            return
        message = "internal server error"
        if "exc_info" in kwargs:
            exc_type, exc, _ = kwargs["exc_info"]
            message = f"{exc_type.__name__}: {exc}"
        self.set_status(status_code)
        self.write(json.dumps({"error": "E_HTTP", "status": status_code,
                               "detail": message}, ensure_ascii=False))


class HealthHandler(BaseHandler):
    def get(self) -> None:
        self.write_json(self.core.health())


class StateHandler(BaseHandler):
    async def get(self) -> None:
        self.write_json(await asyncio.to_thread(self.core.snapshot))


class FrameCaptureHandler(tornado.web.RequestHandler):
    """Bounded in-memory PNG exports, with no caller-controlled filesystem paths."""
    def post(self, key=None):
        if key is not None:
            raise tornado.web.HTTPError(405)
        origin = self.request.headers.get("Origin")
        if origin and origin != f"{self.request.protocol}://{self.request.host}":
            raise tornado.web.HTTPError(403)
        body = self.request.body
        if len(body) > 4 * 1024 * 1024:
            raise tornado.web.HTTPError(413)
        if len(body) < 33 or body[:16] != b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR":
            raise tornado.web.HTTPError(400)
        width, height = struct.unpack(">II", body[16:24])
        if not (0 < width <= 8192 and 0 < height <= 8192):
            raise tornado.web.HTTPError(400)
        key = secrets.token_hex(12)
        captures = self.application.frame_captures
        captures[key] = body
        while len(captures) > 4:
            del captures[next(iter(captures))]
        self.set_header("Content-Type", "application/json")
        self.write({"url": "/api/frame-captures/" + key + ".png"})

    def get(self, key=None):
        if key is None:
            raise tornado.web.HTTPError(405)
        image = self.application.frame_captures.get(key)
        if image is None:
            raise tornado.web.HTTPError(404)
        self.set_header("Content-Type", "image/png")
        self.set_header("X-Content-Type-Options", "nosniff")
        self.set_header("Cache-Control", "no-store")
        self.write(image)


class FormationHandler(BaseHandler):
    def post(self) -> None:
        try:
            body = _decode_body(self)
            if "num_drones" not in body:
                raise ServiceError("E_BAD_REQUEST", 400, "num_drones is required")
            spec = body.get("spec")
            if not isinstance(spec, dict):
                raise ServiceError("E_BAD_REQUEST", 400, "spec must be an object")
            result = self.core.create_formation(int(body["num_drones"]), spec)
            self.write_json(result)
        except ServiceError as exc:
            self.write_json(exc.to_dict(), exc.status)
        except Exception as exc:
            self.write_json({"error": "E_INTERNAL", "detail": str(exc)}, 500)


class PlanHandler(BaseHandler):
    def post(self) -> None:
        try:
            body = _decode_body(self)
            required = ["state_revision", "target_formation_id"]
            missing = [k for k in required if k not in body]
            if missing:
                raise ServiceError("E_BAD_REQUEST", 400,
                                   "missing fields: " + ", ".join(missing))
            result = self.core.create_plan(
                state_revision=int(body["state_revision"]),
                target_formation_id=str(body["target_formation_id"]),
                duration=float(body.get("duration", 4.0)),
                hold_seconds=float(body.get("hold_seconds", 2.0)),
                include_landing=bool(body.get("include_landing", True)),
                color_override=body.get("color"),
                colors_override=body.get("colors"),
                gradient_override=body.get("gradient"),
            )
            self.write_json(result)
        except ServiceError as exc:
            self.write_json(exc.to_dict(), exc.status)
        except Exception as exc:
            self.write_json({"error": "E_INTERNAL", "detail": str(exc)}, 500)


class ValidateHandler(BaseHandler):
    def post(self) -> None:
        try:
            body = _decode_body(self)
            if "plan_id" not in body:
                raise ServiceError("E_BAD_REQUEST", 400, "plan_id is required")
            self.write_json(self.core.validate(str(body["plan_id"])))
        except ServiceError as exc:
            self.write_json(exc.to_dict(), exc.status)
        except Exception as exc:
            self.write_json({"error": "E_INTERNAL", "detail": str(exc)}, 500)


class SubmitHandler(BaseHandler):
    def post(self) -> None:
        try:
            body = _decode_body(self)
            required = ["plan_id", "credential", "request_revision",
                        "source_state_revision"]
            missing = [k for k in required if k not in body]
            if missing:
                raise ServiceError("E_BAD_REQUEST", 400,
                                   "missing fields: " + ", ".join(missing))
            result = self.core.submit(
                plan_id=str(body["plan_id"]),
                credential=str(body["credential"]),
                request_revision=int(body["request_revision"]),
                source_state_revision=int(body["source_state_revision"]),
                insert_after=str(body.get("insert_after", "current_segment")),
            )
            self.write_json(result)
        except ServiceError as exc:
            self.write_json(exc.to_dict(), exc.status)
        except Exception as exc:
            self.write_json({"error": "E_INTERNAL", "detail": str(exc)}, 500)


class CancelPendingHandler(BaseHandler):
    def post(self) -> None:
        try:
            body = _decode_body(self)
            if "request_revision" not in body:
                raise ServiceError("E_BAD_REQUEST", 400, "request_revision is required")
            self.write_json(self.core.cancel_pending(int(body["request_revision"])))
        except ServiceError as exc:
            self.write_json(exc.to_dict(), exc.status)
        except Exception as exc:
            self.write_json({"error": "E_INTERNAL", "detail": str(exc)}, 500)


class RunHandler(BaseHandler):
    async def get(self, run_id: str) -> None:
        try:
            self.write_json(await asyncio.to_thread(self.core.get_run, run_id))
        except ServiceError as exc:
            self.write_json(exc.to_dict(), exc.status)
        except Exception as exc:
            self.write_json({"error": "E_INTERNAL", "detail": str(exc)}, 500)


class RunProgressHandler(BaseHandler):
    async def get(self, run_id: str) -> None:
        try:
            self.write_json(await asyncio.to_thread(self.core.run_progress, run_id))
        except ServiceError as exc:
            self.write_json(exc.to_dict(), exc.status)


class TimelineHandler(BaseHandler):
    def post(self) -> None:
        try:
            body = _decode_body(self)
            self.write_json(self.core.timeline(body))
        except ServiceError as exc:
            self.write_json(exc.to_dict(), exc.status)
        except Exception as exc:
            self.write_json({"error": "E_INTERNAL", "detail": str(exc)}, 500)


class PerformHandler(BaseHandler):
    async def post(self) -> None:
        try:
            body = _decode_body(self)
            self.write_json(await asyncio.to_thread(self.core.perform, body))
        except ServiceError as exc:
            self.write_json(exc.to_dict(), exc.status)


class FleetTimelineHandler(BaseHandler):
    async def post(self) -> None:
        try:
            body = _decode_body(self)
            self.write_json(await asyncio.to_thread(self.core.fleet_timeline, body))
        except ServiceError as exc:
            self.write_json(exc.to_dict(), exc.status)
        except Exception as exc:
            self.write_json({"error": "E_INTERNAL", "detail": str(exc)}, 500)


class TextShowHandler(BaseHandler):
    def post(self) -> None:
        try:
            body = _decode_body(self)
            self.write_json(self.core.text_show(body))
        except ServiceError as exc:
            self.write_json(exc.to_dict(), exc.status)
        except Exception as exc:
            self.write_json({"error": "E_INTERNAL", "detail": str(exc)}, 500)


class LatestRunHandler(BaseHandler):
    async def get(self) -> None:
        self.write_json(await asyncio.to_thread(self.core.latest_run))


class ReplayHandler(BaseHandler):
    async def get(self, run_id: str) -> None:
        try:
            raw_hz = self.get_query_argument("hz", "15")
            try:
                hz = int(raw_hz)
            except (TypeError, ValueError):
                hz = 15
            self.write_json(await asyncio.to_thread(self.core.replay_run, run_id, hz))
        except ServiceError as exc:
            self.write_json(exc.to_dict(), exc.status)
        except Exception as exc:
            self.write_json({"error": "E_INTERNAL", "detail": str(exc)}, 500)


class EvaluateHandler(BaseHandler):
    async def post(self) -> None:
        try:
            body = _decode_body(self)
            if "run_id" not in body:
                raise ServiceError("E_BAD_REQUEST", 400, "run_id is required")
            self.write_json(await asyncio.to_thread(self.core.evaluate_run, str(body["run_id"])))
        except ServiceError as exc:
            self.write_json(exc.to_dict(), exc.status)
        except Exception as exc:
            self.write_json({"error": "E_INTERNAL", "detail": str(exc)}, 500)


class VideoCaptureHandler(tornado.web.RequestHandler):
    """Same-origin local video export, using generated names and a fixed root."""
    async def post(self, key=None):
        if key is not None:
            raise tornado.web.HTTPError(405)
        origin = self.request.headers.get("Origin")
        if origin and origin != f"{self.request.protocol}://{self.request.host}":
            raise tornado.web.HTTPError(403)
        body = self.request.body
        if len(body) > 96 * 1024 * 1024:
            raise tornado.web.HTTPError(413)
        if body[:4] != b"\x1a\x45\xdf\xa3" or b"webm" not in body[:256]:
            raise tornado.web.HTTPError(400)
        root = self.application.core.output_root / "video-captures"
        root.mkdir(parents=True, exist_ok=True)
        key = secrets.token_hex(12)
        await asyncio.to_thread((root / (key + ".webm")).write_bytes, body)
        self.set_header("Content-Type", "application/json")
        self.write({"url": "/api/video-captures/" + key + ".webm"})

    async def get(self, key=None):
        if key is None:
            raise tornado.web.HTTPError(405)
        path = self.application.core.output_root / "video-captures" / (key + ".webm")
        if not path.is_file():
            raise tornado.web.HTTPError(404)
        self.set_header("Content-Type", "video/webm")
        self.set_header("X-Content-Type-Options", "nosniff")
        self.set_header("Cache-Control", "no-store")
        self.set_header("Content-Disposition", 'attachment; filename="agnes-actual-full-replay.webm"')
        self.write(await asyncio.to_thread(path.read_bytes))


class WSHandler(tornado.websocket.WebSocketHandler):
    def initialize(self, core: ServiceCore) -> None:
        self.core = core

    def check_origin(self, origin: str) -> bool:
        return True

    async def open(self) -> None:
        self.application.ws_clients.add(self)
        try:
            self.write_message(json.dumps({
                "type": "hello", "service": "drone-show", "protocol": 1}))
            self.write_message(json.dumps({"type": "state", "data": await asyncio.to_thread(self.core.snapshot)}))
        except Exception:
            pass

    def on_close(self) -> None:
        self.application.ws_clients.discard(self)

    def on_message(self, message: str) -> None:
        try:
            parsed = json.loads(message)
        except Exception:
            return
        if isinstance(parsed, dict) and parsed.get("type") == "ping":
            self.write_message(json.dumps({"type": "pong"}))


class WebHandler(tornado.web.RequestHandler):
    def initialize(self, web_dir: str) -> None:
        self.web_dir = Path(web_dir).resolve()

    def get(self, path: str) -> None:
        rel = (path or "index.html").strip("/") or "index.html"
        candidate = (self.web_dir / rel).resolve()
        try:
            candidate.relative_to(self.web_dir)
        except ValueError:
            self.set_status(403)
            self.set_header("Content-Type", "application/json")
            self.write(json.dumps({"error": "E_PATH_TRAVERSAL"}))
            return
        if candidate.is_dir():
            candidate = candidate / "index.html"
        if candidate.is_file():
            ctype = mimetypes.guess_type(str(candidate))[0] or "application/octet-stream"
            self.set_header("Content-Type", ctype)
            self.set_header("Cache-Control", "no-store")
            with open(candidate, "rb") as handle:
                self.write(handle.read())
            return
        if rel in ("index.html", ""):
            self.set_header("Content-Type", "application/json")
            self.write(json.dumps({
                "service": "drone-show",
                "version": __version__,
                "message": "web UI not installed yet; API is available under /api",
            }))
            return
        self.set_status(404)
        self.set_header("Content-Type", "application/json")
        self.write(json.dumps({"error": "E_NOT_FOUND", "path": rel}))


class DroneShowApplication(tornado.web.Application):
    def __init__(self, core: ServiceCore, web_dir: str | Path = "web", **settings: Any):
        self.core = core
        self.web_dir = str(web_dir)
        self.ws_clients: set[WSHandler] = set()
        self.frame_captures: dict[str, bytes] = {}
        handlers = [
            (r"/api/health", HealthHandler, {"core": core}),
            (r"/api/state", StateHandler, {"core": core}),
            (r"/api/formation", FormationHandler, {"core": core}),
            (r"/api/plan", PlanHandler, {"core": core}),
            (r"/api/validate", ValidateHandler, {"core": core}),
            (r"/api/text_show", TextShowHandler, {"core": core}),
            (r"/api/timeline", TimelineHandler, {"core": core}),
            (r"/api/perform", PerformHandler, {"core": core}),
            (r"/api/fleet_timeline", FleetTimelineHandler, {"core": core}),
            (r"/api/submit", SubmitHandler, {"core": core}),
            (r"/api/cancel_pending", CancelPendingHandler, {"core": core}),
            (r"/api/runs/latest", LatestRunHandler, {"core": core}),
            (r"/api/run/([^/]+)/replay", ReplayHandler, {"core": core}),
            (r"/api/run/([^/]+)/progress", RunProgressHandler, {"core": core}),
            (r"/api/run/([^/]+)", RunHandler, {"core": core}),
            (r"/api/evaluate", EvaluateHandler, {"core": core}),
            (r"/api/frame-captures", FrameCaptureHandler),
            (r"/api/frame-captures/([a-f0-9]{24})\.png", FrameCaptureHandler),
            (r"/api/video-captures", VideoCaptureHandler),
            (r"/api/video-captures/([a-f0-9]{24})\.webm", VideoCaptureHandler),
            (r"/ws", WSHandler, {"core": core}),
            (r"/(.*)", WebHandler, {"web_dir": self.web_dir}),
        ]
        super().__init__(handlers, **settings)

    async def broadcast(self) -> None:
        # Runtime IPC can wait for telemetry archival. Never block HTTP, pong,
        # or other sockets on the event-loop thread while waiting for the worker.
        events = await asyncio.to_thread(self.core.drain_events)
        if not self.ws_clients:
            return
        state_msg = json.dumps({"type": "state", "data": await asyncio.to_thread(self.core.snapshot)})
        for client in list(self.ws_clients):
            try:
                client.write_message(state_msg)
            except Exception:
                self.ws_clients.discard(client)
        for event in events:
            event_msg = json.dumps({"type": "event", "data": event})
            for client in list(self.ws_clients):
                try:
                    client.write_message(event_msg)
                except Exception:
                    self.ws_clients.discard(client)


def make_app(core: ServiceCore, web_dir: str | Path = "web") -> DroneShowApplication:
    return DroneShowApplication(core, web_dir=web_dir)


def serve(core: ServiceCore, host: str = "127.0.0.1", port: int = 8765,
          web_dir: str | Path = "web", broadcast_hz: float = 20.0):
    app = make_app(core, web_dir)
    server = tornado.httpserver.HTTPServer(app)
    server.bind(int(port), address=host)
    server.start(1)
    interval = max(10, int(round(1000.0 / max(1.0, float(broadcast_hz)))))
    periodic = tornado.ioloop.PeriodicCallback(app.broadcast, interval)
    periodic.start()
    return app, server, periodic
