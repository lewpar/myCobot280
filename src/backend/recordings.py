"""Saved arm recordings: named joint-angle timelines captured by the simulator page.

One JSON file per recording in ``recordings/`` at the repo root (per-machine, gitignored):
    {"id": "3f9a0c1b2d4e", "name": "Wave", "created": 1727140000.0,
     "frames": [[t_seconds, j1, j2, j3, j4, j5, j6], ...]}     angles in degrees, URDF convention

The page records and plays back; playback streams goals over /ws/arm, so every pose still goes
through the normal guards (stop state, collision check, limits). This module only stores them.
"""
import json
import os
import re
import secrets
import threading
import time

import arm_model as model

DIR = os.path.join(model.ROOT, "recordings")
ID_RE = re.compile(r"^[0-9a-f]{12}$")
_lock = threading.Lock()


def _path(rid):
    if not ID_RE.match(rid):
        raise KeyError(rid)
    return os.path.join(DIR, rid + ".json")


def _summary(rec):
    frames = rec["frames"]
    return {"id": rec["id"], "name": rec["name"], "created": rec["created"],
            "duration": round(frames[-1][0] - frames[0][0], 2) if frames else 0, "frames": len(frames)}


def list_all():
    out = []
    if os.path.isdir(DIR):
        for fn in os.listdir(DIR):
            rid = fn[:-5]
            if fn.endswith(".json") and ID_RE.match(rid):
                try:
                    out.append(_summary(load(rid)))
                except (OSError, ValueError, KeyError):
                    pass
    return sorted(out, key=lambda r: r["created"], reverse=True)


def load(rid):
    with open(_path(rid)) as f:
        return json.load(f)


def save(name, frames):
    rec = {"id": secrets.token_hex(6), "name": name, "created": time.time(), "frames": frames}
    with _lock:
        os.makedirs(DIR, exist_ok=True)
        tmp = _path(rec["id"]) + ".tmp"
        with open(tmp, "w") as f:
            json.dump(rec, f, separators=(",", ":"))
        os.replace(tmp, _path(rec["id"]))
    return _summary(rec)


def delete(rid):
    with _lock:
        os.remove(_path(rid))
