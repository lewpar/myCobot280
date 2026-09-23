"""Live link between the IK simulator page and the arm, served at /ws/arm.

The page streams joint angles (degrees, URDF convention); this module turns them into servo ticks
using a per-joint zero and direction, sends all six goals in one sync-write packet, and streams the
measured angles back about 10 times a second.

Messages from the page:
    {"type": "goal", "angles": [deg x6], "speed": deg_per_s, "acc": deg_per_s2}
    {"type": "torque", "on": true|false}
    {"type": "set_zero"}                       current pose becomes the kinematic zero
    {"type": "set_dir", "joint": 0-5, "dir": 1|-1}
Message to the page:
    {"type": "state", "angles": [deg|null x6], "torque": bool, "zero": [...], "dir": [...],
     "limits": [[lo_deg, hi_deg] x6]}
"""
import json
import math
import os
import threading
import time

JOINT_IDS = [1, 2, 3, 4, 5, 6]
TICKS_PER_DEG = 4096 / 360
# Joint limits in degrees from the mycobot_280_pi URDF; the servos' own EEPROM limits also apply
URDF_LIMITS = [(-168, 168), (-140, 140), (-150, 150), (-150, 150), (-155, 160), (-180, 180)]
MAX_DPS = 150            # speed cap no matter what the page asks for
HOLD_DPS = 20            # speed used when re-enabling torque at the current pose

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
CALIB_FILE = os.path.join(_ROOT, "ik_calibration.json")
CENTER_FILE = os.path.join(_ROOT, "center_positions.json")   # written by arm_server.py SET_CENTER


def _load_calibration():
    zero, direction = [2048] * 6, [1] * 6
    try:  # seed zeros from arm_server.py's saved centres if there's no IK calibration yet
        with open(CENTER_FILE) as f:
            centers = {int(k): int(v) for k, v in json.load(f).items()}
        zero = [centers.get(sid, 2048) for sid in JOINT_IDS]
    except (OSError, ValueError):
        pass
    try:
        with open(CALIB_FILE) as f:
            c = json.load(f)
        zero = [int(v) for v in c.get("zero", zero)][:6]
        direction = [1 if int(v) >= 0 else -1 for v in c.get("dir", direction)][:6]
    except (OSError, ValueError):
        pass
    return zero, direction


class IKLink:
    def __init__(self, arm):
        self.arm = arm
        self.zero, self.dir = _load_calibration()
        self._lock = threading.Lock()
        self._clients = 0
        self._thread = None
        self._pending_goal = None
        self._pending_torque = None
        self._pending_zero = False
        self._calib_changed = 0.0
        self.ticks = [None] * 6
        self.torque = False

    # -- conversion ----------------------------------------------------------

    def to_ticks(self, j, deg):
        lo, hi = URDF_LIMITS[j]
        deg = max(lo, min(hi, deg))
        return int(round(self.zero[j] + self.dir[j] * deg * TICKS_PER_DEG))

    def to_deg(self, j, ticks):
        return (ticks - self.zero[j]) * self.dir[j] / TICKS_PER_DEG

    def limits_deg(self):
        """URDF limits intersected with each servo's safe EEPROM range, in joint degrees."""
        out = []
        for j, sid in enumerate(JOINT_IDS):
            lo_t, hi_t = self.arm.safe_limits(sid)
            a, b = sorted((self.to_deg(j, lo_t), self.to_deg(j, hi_t)))
            lo, hi = max(a, URDF_LIMITS[j][0]), min(b, URDF_LIMITS[j][1])
            out.append([round(lo, 1), round(hi, 1)] if lo < hi else [0.0, 0.0])
        return out

    def _save(self):
        with open(CALIB_FILE, "w") as f:
            json.dump({"zero": self.zero, "dir": self.dir}, f, indent=2)

    # -- clients ---------------------------------------------------------------

    def add_client(self):
        with self._lock:
            self._clients += 1
            if self._thread is None or not self._thread.is_alive():
                self._thread = threading.Thread(target=self._loop, daemon=True)
                self._thread.start()

    def remove_client(self):
        with self._lock:
            self._clients = max(0, self._clients - 1)
            if self._clients == 0:
                self._pending_goal = None   # nobody is driving: servos just hold their last goal

    def handle(self, msg):
        t = msg.get("type")
        with self._lock:
            if t == "goal":
                # after a calibration change the page re-reads the arm's pose; ignore goals computed
                # with the old calibration, which would otherwise make the arm jump
                if time.monotonic() - self._calib_changed < 0.5:
                    return
                a = msg.get("angles")
                if isinstance(a, list) and len(a) == 6 and all(isinstance(v, (int, float)) for v in a):
                    self._pending_goal = ([float(v) for v in a],
                                          float(msg.get("speed", 60)), float(msg.get("acc", 200)))
            elif t == "torque":
                self._pending_torque = bool(msg.get("on"))
                self._pending_goal = None
            elif t == "set_zero":
                self._pending_zero = True
                self._pending_goal = None
                self._calib_changed = time.monotonic()
            elif t == "set_dir":
                j, d = msg.get("joint"), msg.get("dir")
                if isinstance(j, int) and 0 <= j < 6 and d in (1, -1) and d != self.dir[j]:
                    self.dir[j] = d
                    self._pending_goal = None
                    self._calib_changed = time.monotonic()
                    self._save()

    def state(self):
        with self._lock:
            ticks, torque = list(self.ticks), self.torque
        return {
            "type": "state",
            "angles": [None if t is None else round(self.to_deg(j, t), 2) for j, t in enumerate(ticks)],
            "torque": torque,
            "zero": list(self.zero),
            "dir": list(self.dir),
            "limits": self.limits_deg(),
        }

    # -- bus loop ----------------------------------------------------------------

    def _set_torque(self, on):
        if on:  # hold the current pose: goal = present position before torque comes back
            now = self.arm.read_positions(JOINT_IDS)
            if any(p is None for p in now):
                return
            self.arm.sync_move(dict(zip(JOINT_IDS, now)), int(HOLD_DPS * TICKS_PER_DEG), 10)
        self.arm.sync_torque(JOINT_IDS, on)
        self.torque = on

    def _loop(self):
        states = [self.arm.servo(sid).torque for sid in JOINT_IDS]
        self.torque = all(states)
        while True:
            with self._lock:
                if self._clients == 0:
                    self._thread = None
                    return
                ready = all(t is not None for t in self.ticks)
                tq, self._pending_torque = self._pending_torque, None
                zero, self._pending_zero = self._pending_zero, False
                goal = None
                if ready:   # keep a goal queued until every servo has read back
                    goal, self._pending_goal = self._pending_goal, None
            if zero and ready:
                with self._lock:
                    self.zero = list(self.ticks)
                self._save()
                with self._lock:
                    self._calib_changed = time.monotonic()
            if tq is not None and tq != self.torque:
                self._set_torque(tq)
            if goal is not None:
                if not self.torque:
                    self._set_torque(True)
                angles, dps, dps2 = goal
                speed = int(max(1.0, min(MAX_DPS, dps)) * TICKS_PER_DEG)          # steps/s
                acc = int(max(1, min(254, math.ceil(dps2 * TICKS_PER_DEG / 100))))  # unit: 100 steps/s^2
                self.arm.sync_move({sid: self.to_ticks(j, a) for j, (sid, a) in enumerate(zip(JOINT_IDS, angles))},
                                   speed, acc)
            ticks = self.arm.read_positions(JOINT_IDS)
            with self._lock:
                self.ticks = ticks
            time.sleep(0.02)
