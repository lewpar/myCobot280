"""The arm's library: recordings, sequences and saved poses, one JSON file per item.

Stored per machine (gitignored) at the repo root:
    recordings/<id>.json  {"id", "name", "created", "return_zero": bool,
                           "frames": [[t_s, j1..j6 deg], ...],
                           "events": [[t_s, "color"|"pixel"|"brightness", [ints]], ...]}
    sequences/<id>.json   {"id", "name", "created", "steps": [{"recording": id, "pause": s}, ...]}
    poses/<id>.json       {"id", "name", "created", "angles": [j1..j6 deg]}

Angles are degrees in the URDF convention. This module only stores and validates; playback lives
in player.py and goes through the same guards as every other motion.
"""
import json
import math
import os
import re
import secrets
import threading
import time

import arm_model as model

ID_RE = re.compile(r"^[0-9a-f]{12}$")
MAX_FRAMES = 36000   # an hour at the page's 10 samples a second
MAX_EVENTS = 20000
MAX_STEPS = 100
EVENT_ARGS = {"color": 3, "pixel": 5, "brightness": 1}


class Invalid(ValueError):
    """Bad data from a client; the message is safe to show."""


class Store:
    def __init__(self, name, summary):
        self.name = name
        self.dir = os.path.join(model.ROOT, name)
        self._summary = summary
        self._lock = threading.Lock()

    def _path(self, rid):
        if not isinstance(rid, str) or not ID_RE.match(rid):
            raise KeyError(rid)
        return os.path.join(self.dir, rid + ".json")

    def list(self):
        out = []
        if os.path.isdir(self.dir):
            for fn in os.listdir(self.dir):
                if fn.endswith(".json") and ID_RE.match(fn[:-5]):
                    try:
                        out.append(self._summary(self.get(fn[:-5])))
                    except (OSError, ValueError, KeyError):
                        pass
        return sorted(out, key=lambda r: r["created"], reverse=True)

    def get(self, rid):
        """The full item; KeyError if there is no such id."""
        try:
            with open(self._path(rid)) as f:
                return json.load(f)
        except FileNotFoundError:
            raise KeyError(rid)

    def _write(self, item):
        os.makedirs(self.dir, exist_ok=True)
        tmp = self._path(item["id"]) + ".tmp"
        with open(tmp, "w") as f:
            json.dump(item, f, separators=(",", ":"))
        os.replace(tmp, self._path(item["id"]))

    def create(self, fields):
        item = {"id": secrets.token_hex(6), "created": time.time(), **fields}
        with self._lock:
            self._write(item)
        return self._summary(item)

    def update(self, rid, change):
        """Apply ``change(item)`` (which edits it in place) and save. KeyError if missing."""
        with self._lock:
            item = self.get(rid)
            change(item)
            self._write(item)
        return self._summary(item)

    def delete(self, rid):
        with self._lock:
            try:
                os.remove(self._path(rid))
            except FileNotFoundError:
                raise KeyError(rid)


# ---- validation ------------------------------------------------------------------------------

def clean_name(name):
    name = (name or "").strip() if isinstance(name, str) else ""
    if not 1 <= len(name) <= 60:
        raise Invalid("A name must be 1 to 60 characters.")
    return name


def _finite(*vals):
    return all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in vals)


def clean_angles(a):
    lims = model.URDF_LIMITS_DEG
    if not (isinstance(a, list) and len(a) == 6 and _finite(*a)):
        raise Invalid("A pose must be six finite joint angles.")
    if any(not lims[j][0] - 1 <= v <= lims[j][1] + 1 for j, v in enumerate(a)):
        raise Invalid("A joint angle is outside the arm's limits.")
    return [round(float(v), 2) for v in a]


def clean_frames(frames):
    """[[t, 6 angles], ...] with times rebased to 0 and never going backwards."""
    if not isinstance(frames, list) or not 2 <= len(frames) <= MAX_FRAMES:
        raise Invalid(f"A recording needs 2 to {MAX_FRAMES} frames.")
    out, last = [], None
    for f in frames:
        if not (isinstance(f, list) and len(f) == 7 and _finite(*f)):
            raise Invalid("Each frame must be [t, six joint angles], all finite numbers.")
        t = f[0] - frames[0][0]
        if last is not None and t < last:
            raise Invalid("Frame times must not go backwards.")
        last = t
        out.append([round(t, 3)] + clean_angles(f[1:]))
    return out


def clean_events(events, t0=0.0):
    """[[t, kind, [ints]], ...] LED cues, sorted by time; ``t0`` is subtracted (frames rebase to 0)."""
    if events is None:
        return []
    if not isinstance(events, list) or len(events) > MAX_EVENTS:
        raise Invalid(f"At most {MAX_EVENTS} LED cues.")
    out = []
    for e in events:
        if not (isinstance(e, list) and len(e) == 3 and _finite(e[0]) and e[1] in EVENT_ARGS
                and isinstance(e[2], list) and len(e[2]) == EVENT_ARGS[e[1]]
                and all(isinstance(v, int) and not isinstance(v, bool) for v in e[2])):
            raise Invalid("Each LED cue must be [t, 'color'|'pixel'|'brightness', [integers]].")
        kind, args = e[1], e[2]
        ok = (all(0 <= v <= 255 for v in args) if kind == "color" else
              all(0 <= v <= 4 for v in args[:2]) and all(0 <= v <= 255 for v in args[2:]) if kind == "pixel" else
              1 <= args[0] <= 100)
        if not ok:
            raise Invalid("An LED cue has a value out of range.")
        out.append([round(max(0.0, e[0] - t0), 3), kind, args])
    return sorted(out, key=lambda e: e[0])


def trim(rec, start, end):
    """Keep [start, end] seconds of a recording (frames and cues), rebased to 0."""
    frames = rec["frames"]
    if not (_finite(start, end) and 0 <= start < end):
        raise Invalid("Trim needs 0 <= start < end.")
    keep = [f for f in frames if start - 1e-6 <= f[0] <= end + 1e-6]
    if len(keep) < 2:
        raise Invalid("That trim leaves less than two frames.")
    t0 = keep[0][0]
    rec["frames"] = [[round(f[0] - t0, 3)] + f[1:] for f in keep]
    rec["events"] = [[round(e[0] - t0, 3), e[1], e[2]] for e in rec.get("events", [])
                     if t0 - 1e-6 <= e[0] <= keep[-1][0] + 1e-6]


# ---- stores ------------------------------------------------------------------------------------

def _rec_summary(r):
    f = r["frames"]
    return {"id": r["id"], "name": r["name"], "created": r["created"],
            "return_zero": bool(r.get("return_zero")), "duration": round(f[-1][0] - f[0][0], 2) if f else 0,
            "frames": len(f), "events": len(r.get("events", []))}


def _seq_summary(s):
    return {"id": s["id"], "name": s["name"], "created": s["created"], "steps": s["steps"]}


def _pose_summary(p):
    return {"id": p["id"], "name": p["name"], "created": p["created"], "angles": p["angles"]}


RECORDINGS = Store("recordings", _rec_summary)
SEQUENCES = Store("sequences", _seq_summary)
POSES = Store("poses", _pose_summary)
STORES = [RECORDINGS, SEQUENCES, POSES]


def clean_steps(steps):
    if not isinstance(steps, list) or not 1 <= len(steps) <= MAX_STEPS:
        raise Invalid(f"A sequence needs 1 to {MAX_STEPS} steps.")
    out = []
    for s in steps:
        if not isinstance(s, dict):
            raise Invalid("Each step must be {recording, pause}.")
        rid, pause = s.get("recording"), s.get("pause", 0)
        if not (_finite(pause) and 0 <= pause <= 600):
            raise Invalid("A pause must be 0 to 600 seconds.")
        try:
            RECORDINGS.get(rid)
        except (KeyError, ValueError, OSError):
            raise Invalid("A step refers to a recording that doesn't exist.")
        out.append({"recording": rid, "pause": round(float(pause), 2)})
    return out
