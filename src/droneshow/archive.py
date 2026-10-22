"""JSON run records and immutable telemetry; never deserialize executable objects."""
import json
import os
import re
from pathlib import Path
import numpy as np


def encode(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


class RunArchive:
    def __init__(self, root):
        self.root = Path(root)

    def save(self, run):
        key = run["run_id"]
        if not re.fullmatch(r"[a-zA-Z0-9_-]{1,80}", key):
            raise ValueError("invalid run identifier")
        directory = self.root / ("run_" + key)
        directory.mkdir(parents=True, exist_ok=True)
        record = {k: v for k, v in run.items() if k not in ("_actual", "_target", "_times")}
        record["archive_version"] = 1
        temporary = directory / "record.json.tmp"
        temporary.write_text(json.dumps(record, default=encode, ensure_ascii=False), encoding="utf-8")
        os.replace(temporary, directory / "record.json")

    def load(self, kinematic=False):
        runs = {}
        for path in sorted(self.root.glob("run_*/record.json"), key=lambda p: p.stat().st_mtime):
            try:
                run = json.loads(path.read_text(encoding="utf-8"))
                if run.get("archive_version") != 1 or path.parent.name != "run_" + run["run_id"]:
                    continue
                if bool(run.get("kinematic_preview", False)) != kinematic:
                    continue
                plan = run["_plan"]
                for key in ("_start_positions", "_target_positions"):
                    plan[key] = np.asarray(plan[key], dtype=float)
                for segment in plan["_segments"]:
                    for key in ("start_positions", "end_positions"):
                        segment[key] = np.asarray(segment[key], dtype=float)
                run.update(_actual=None, _target=None, _times=None)
                if run["status"] in ("running", "waiting_checkpoint", "pending"):
                    run.update(status="interrupted", completed=False, error="E_SERVICE_RESTART")
                telemetry = path.parent / "telemetry.npz"
                if telemetry.is_file():
                    with np.load(telemetry, allow_pickle=False) as data:
                        actual, target, times = data["actual"], data["target"], data["time"]
                    if (actual.shape == target.shape == (len(times), run["_num_drones"], 3)
                            and all(np.isfinite(a).all() for a in (actual, target, times))):
                        run.update(_actual=actual, _target=target, _times=times,
                                   telemetry_ref=str(telemetry.resolve()))
                runs[run["run_id"]] = run
            except (ValueError, TypeError, KeyError, OSError):
                continue  # One damaged recording does not hide all the others.
        return runs
