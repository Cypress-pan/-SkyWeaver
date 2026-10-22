"""Parent-process proxy that owns a SimRuntime in a separate process.

The HTTP/WebSocket/UI process imports this module only; PyBullet is imported in
the child process by ``_runtime_worker``.  The proxy presents the same public
method surface as ``SimRuntime`` so ServiceCore keeps working unchanged.
"""
from __future__ import annotations

import multiprocessing as mp
import os
import queue as queue_module
import threading
import time
import uuid
from typing import Any

from .errors import RuntimeBusy


def _runtime_worker(command_queue: Any, response_queue: Any,
                    config: dict[str, Any]) -> None:
    runtime = None
    stop_watchdog = threading.Event()
    parent = mp.parent_process()

    def watchdog() -> None:
        while not stop_watchdog.is_set():
            if parent is not None and not parent.is_alive():
                os._exit(1)
            if os.getppid() == 1:
                os._exit(1)
            time.sleep(0.5)

    threading.Thread(target=watchdog, daemon=True).start()
    try:
        from .runtime import SimRuntime
        runtime = SimRuntime(**config)
        runtime.start()
        response_queue.put(("__ready__", True, None))
        while True:
            command = command_queue.get()
            if command is None:
                break
            request_id, method, args, kwargs = command
            try:
                result = getattr(runtime, method)(*args, **kwargs)
                response_queue.put((request_id, True, result))
            except Exception as exc:
                response_queue.put((request_id, False,
                                    (type(exc).__name__, str(exc))))
    except Exception as exc:
        try:
            response_queue.put(("__ready__", False, (type(exc).__name__, str(exc))))
        except Exception:
            pass
    finally:
        stop_watchdog.set()
        if runtime is not None:
            try:
                runtime.close()
            except Exception:
                pass


class RuntimeProxy:
    def __init__(self, **config: Any):
        self.config = dict(config)
        self._context = mp.get_context("spawn")
        self._command_queue = None
        self._response_queue = None
        self._process = None
        self._ready = False
        self._closed = False
        self._lock = threading.RLock()

    def start(self, timeout: float = 120.0) -> None:
        with self._lock:
            if self._process is not None and self._process.is_alive():
                return
            self._command_queue = self._context.Queue()
            self._response_queue = self._context.Queue()
            self._process = self._context.Process(
                target=_runtime_worker,
                args=(self._command_queue, self._response_queue, self.config),
                daemon=True,
            )
            self._process.start()
            try:
                self._wait_ready(timeout)
            except Exception:
                self._abort_process()
                raise
            self._closed = False

    def _wait_ready(self, timeout: float) -> None:
        deadline = time.monotonic() + max(1.0, float(timeout))
        while time.monotonic() < deadline:
            if self._process is None or not self._process.is_alive():
                raise RuntimeError("runtime worker exited before becoming ready")
            try:
                message = self._response_queue.get(timeout=0.1)
            except queue_module.Empty:
                continue
            if message[0] == "__ready__":
                if message[1]:
                    self._ready = True
                    return
                raise RuntimeError("runtime worker failed: " + str(message[2]))
        raise RuntimeError("runtime worker did not become ready within timeout")

    def _call(self, method: str, *args: Any, call_timeout: float = 180.0,
              **kwargs: Any) -> Any:
        with self._lock:
            if not self._ready or self._process is None or not self._process.is_alive():
                raise RuntimeError("runtime proxy is not running")
            request_id = uuid.uuid4().hex
            self._command_queue.put((request_id, method, args, kwargs))
            deadline = time.monotonic() + max(1.0, float(call_timeout))
            while time.monotonic() < deadline:
                try:
                    message = self._response_queue.get(timeout=0.1)
                except queue_module.Empty:
                    if not self._process.is_alive():
                        raise RuntimeError("runtime worker exited")
                    continue
                if message[0] != request_id:
                    continue
                if message[1]:
                    return message[2]
                error_name, detail = message[2]
                if error_name == "RuntimeBusy":
                    raise RuntimeBusy(detail)
                raise RuntimeError(detail)
            raise TimeoutError("runtime proxy call timed out: " + method)

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            if self._process is not None and self._process.is_alive():
                try:
                    self._call("close", call_timeout=30.0)
                except Exception:
                    pass
                self._process.join(timeout=10.0)
                if self._process.is_alive():
                    self._process.terminate()
                    self._process.join(timeout=5.0)
            self._ready = False

    # -- SimRuntime public method surface ---------------------------------
    def snapshot(self) -> dict[str, Any]:
        return self._call("snapshot")

    def ensure_num_drones(self, num_drones: int, timeout: float = 60.0) -> None:
        return self._call("ensure_num_drones", int(num_drones), timeout=timeout,
                          call_timeout=timeout + 10.0)

    def schedule_plan(self, plan: dict[str, Any], run_id: str,
                      insert_after: str = "immediate") -> str:
        return self._call("schedule_plan", plan, run_id, insert_after)

    def cancel_pending(self) -> list[str]:
        return self._call("cancel_pending")

    def amendment_checkpoint(self) -> dict[str, Any]:
        return self._call("amendment_checkpoint")

    def schedule_amendment(self, plan: dict[str, Any], run_id: str,
                           checkpoint: dict[str, Any]) -> str:
        return self._call("schedule_amendment", plan, run_id, checkpoint)

    def emit_event(self, kind: str, detail: dict[str, Any]) -> None:
        return self._call("emit_event", kind, detail)

    def drain_events(self) -> list[dict[str, Any]]:
        return self._call("drain_events")

    def public_run(self, run_id: str) -> dict[str, Any] | None:
        return self._call("public_run", run_id)

    def run_arrays(self, run_id: str) -> tuple[Any, Any, Any] | None:
        return self._call("run_arrays", run_id)

    def latest_run_info(self) -> dict[str, Any]:
        return self._call("latest_run_info")

    def run_replay_data(self, run_id: str) -> dict[str, Any] | None:
        return self._call("run_replay_data", run_id)

    def begin_planning(self) -> None:
        return self._call("begin_planning")

    def end_planning(self) -> None:
        return self._call("end_planning")
