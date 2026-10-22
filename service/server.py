"""CLI entry point for the deterministic local drone-show service.

Run headless with the workspace conda interpreter::

    ./.conda/bin/python service/server.py --port 8765 --num-drones 16

The process serves HTTP JSON under /api, WebSocket under /ws, and static files
from web/ at /.
"""
from __future__ import annotations

import argparse
import os
import signal
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC = PROJECT_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import tornado.ioloop  # noqa: E402

from droneshow.core import ServiceCore  # noqa: E402
from droneshow.http_server import serve  # noqa: E402


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="drone-show local service")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--num-drones", type=int, default=16)
    parser.add_argument("--ctrl-freq", type=int, default=48)
    parser.add_argument("--pyb-freq", type=int, default=240)
    parser.add_argument("--realtime-factor", type=float, default=1.0)
    parser.add_argument("--output-root", default=str(PROJECT_ROOT / "results"))
    parser.add_argument("--web-dir", default=str(PROJECT_ROOT / "web"))
    parser.add_argument("--broadcast-hz", type=float, default=20.0)
    parser.add_argument("--cjk-font", default=None,
                        help="path to a CJK TTF/TTC/OTF font (or use DRONE_SHOW_CJK_FONT)")
    parser.add_argument("--cjk-font-index", type=int, default=None,
                        help="face index for TTC fonts (or use DRONE_SHOW_CJK_FONT_INDEX)")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.cjk_font:
        os.environ["DRONE_SHOW_CJK_FONT"] = str(args.cjk_font)
    if args.cjk_font_index is not None:
        os.environ["DRONE_SHOW_CJK_FONT_INDEX"] = str(args.cjk_font_index)
    core = ServiceCore(num_drones=args.num_drones, ctrl_freq=args.ctrl_freq,
                       pyb_freq=args.pyb_freq,
                       realtime_factor=args.realtime_factor,
                       output_root=args.output_root)
    core.start()
    app, server, periodic = serve(core, host=args.host, port=args.port,
                                  web_dir=args.web_dir,
                                  broadcast_hz=args.broadcast_hz)
    actual_port = args.port
    if actual_port == 0:
        for sock in getattr(server, "_sockets", {}).values():
            try:
                actual_port = sock.getsockname()[1]
                break
            except Exception:
                continue
    print(f"DRONE_SHOW_SERVICE_READY host={args.host} port={actual_port} "
          f"pid={os.getpid()} num_drones={args.num_drones}", flush=True)
    loop = tornado.ioloop.IOLoop.current()
    shutdown_state = {"requested": False}

    def request_shutdown(signum, frame):
        if shutdown_state["requested"]:
            return
        shutdown_state["requested"] = True
        try:
            loop.add_callback_from_signal(loop.stop)
        except Exception:
            try:
                loop.stop()
            except Exception:
                pass

    for signal_name in ("SIGINT", "SIGTERM", "SIGHUP"):
        handler_signal = getattr(signal, signal_name, None)
        if handler_signal is None:
            continue
        try:
            signal.signal(handler_signal, request_shutdown)
        except (ValueError, OSError):
            pass
    try:
        loop.start()
    except KeyboardInterrupt:
        pass
    finally:
        periodic.stop()
        core.close()
        loop.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
