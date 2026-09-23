"""Live link between the IK simulator page and the arm, served at /ws/arm (see main.py).

The page streams joint angles (degrees, URDF convention); this module checks each pose and the path
to it for collisions, turns the angles into servo ticks with the saved calibration, sends all six
goals in one sync-write packet, and streams the measured angles back about 10 times a second.

It also owns the stop state: while stopped, every motion request (from the page, the REST API or
"home all") is refused until someone resumes.

Messages from the page (after the auth message, which main.py handles):
    {"type": "goal", "angles": [deg x6], "speed": deg_per_s, "acc": deg_per_s2}
    {"type": "torque", "on": true|false}
    {"type": "stop"} / {"type": "resume"}
    {"type": "set_zero"}                       current pose becomes the kinematic zero
    {"type": "set_dir", "joint": 0-5, "dir": 1|-1}
    {"type": "set_tool", "mm": 0-150}
Message to the page:
    {"type": "state", "angles": [deg|null x6], "torque": bool, "stopped": bool, "blocked": str|null,
     "calibrated": bool, "zero": [...], "dir": [...], "tool_mm": n, "limits": [[lo, hi] deg x6]}
"""
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import arm_model as model  # noqa: E402

MAX_DPS = 150            # speed cap no matter what the page asks for
HOLD_DPS = 20            # speed used when re-enabling torque at the current pose
IDS = model.JOINT_IDS


class IKLink:
    def __init__(self, arm):
        self.arm = arm
        self.calib = model.load_calibration()
        self._lock = threading.Lock()
        self._clients = 0
        self._thread = None
        self._pending_goal = None
        self._pending_torque = None
        self._pending_zero = False
        self._calib_changed = 0.0
        self.ticks = [None] * 6
        self.torque = False
        self.stopped = False
        self.blocked = None

    # -- helpers used by the REST API too -----------------------------------------

    @property
    def tool_m(self):
        return self.calib["tool_mm"] / 1000

    def check_ticks(self, new_ticks):
        """Collision check for a raw-tick move from the current pose. None if clear."""
        return model.check_tick_move(self.calib, self.arm.read_positions(IDS), new_ticks)

    def stop(self):
        """Hold every servo where it is and refuse motion until resume()."""
        with self._lock:
            self.stopped = True
            self._pending_goal = None
        self.arm.hold(IDS)

    def resume(self):
        with self._lock:
            self.stopped = False
            self.blocked = None
            self._pending_goal = None
            self._calib_changed = time.monotonic()   # the page re-reads the pose before sending again

    def limits_deg(self):
        """URDF limits intersected with each servo's safe EEPROM range, in joint degrees."""
        out = []
        for j, sid in enumerate(IDS):
            lo_t, hi_t = self.arm.safe_limits(sid)
            a, b = sorted((model.ticks_to_deg(self.calib, j, lo_t), model.ticks_to_deg(self.calib, j, hi_t)))
            lo, hi = max(a, model.URDF_LIMITS_DEG[j][0]), min(b, model.URDF_LIMITS_DEG[j][1])
            out.append([round(lo, 1), round(hi, 1)] if lo < hi else [0.0, 0.0])
        return out

    # -- clients -----------------------------------------------------------------

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
        if t == "stop":
            self.stop()
            return
        if t == "resume":
            self.resume()
            return
        with self._lock:
            if t == "goal":
                # after a calibration change or a resume the page re-reads the arm's pose; ignore
                # goals computed before that, which would otherwise make the arm jump
                if self.stopped or time.monotonic() - self._calib_changed < 0.5:
                    return
                a = msg.get("angles")
                speed, acc = msg.get("speed", 60), msg.get("acc", 200)
                if (isinstance(a, list) and len(a) == 6
                        and all(isinstance(v, (int, float)) and math.isfinite(v) for v in a)
                        and isinstance(speed, (int, float)) and isinstance(acc, (int, float))):
                    self._pending_goal = ([float(v) for v in a], float(speed), float(acc))
            elif t == "torque":
                self._pending_torque = bool(msg.get("on"))
                self._pending_goal = None
            elif t == "set_zero":
                self._pending_zero = True
                self._pending_goal = None
                self._calib_changed = time.monotonic()
            elif t == "set_dir":
                j, d = msg.get("joint"), msg.get("dir")
                if isinstance(j, int) and 0 <= j < 6 and d in (1, -1) and d != self.calib["dir"][j]:
                    self.calib["dir"][j] = d
                    self.calib["calibrated"] = True
                    self._pending_goal = None
                    self._calib_changed = time.monotonic()
                    model.save_calibration(self.calib)
            elif t == "set_tool":
                mm = msg.get("mm")
                if isinstance(mm, (int, float)) and 0 <= mm <= 150:
                    self.calib["tool_mm"] = float(mm)
                    model.save_calibration(self.calib)

    def state(self):
        with self._lock:
            ticks, torque, stopped, blocked = list(self.ticks), self.torque, self.stopped, self.blocked
            c = self.calib
        return {
            "type": "state",
            "angles": [None if a is None else round(a, 2) for a in model.pose_from_ticks(c, ticks)],
            "torque": torque,
            "stopped": stopped,
            "blocked": blocked,
            "calibrated": c["calibrated"],
            "zero": list(c["zero"]),
            "dir": list(c["dir"]),
            "tool_mm": c["tool_mm"],
            "limits": self.limits_deg(),
        }

    # -- bus loop --------------------------------------------------------------------

    def _set_torque(self, on):
        if on:  # hold the current pose: goal = present position before torque comes back
            now = self.arm.read_positions(IDS)
            if any(p is None for p in now):
                return
            self.arm.sync_move(dict(zip(IDS, now)), int(HOLD_DPS * model.TICKS_PER_DEG), 10)
        self.arm.sync_torque(IDS, on)
        self.torque = on

    def _send_goal(self, goal):
        angles, dps, dps2 = goal
        current = model.pose_from_ticks(self.calib, self.ticks)
        why = model.check_path(current, angles, self.tool_m)
        with self._lock:
            self.blocked = why
        if why:
            return
        if not self.torque:
            self._set_torque(True)
        c = self.calib
        speed = int(max(1.0, min(MAX_DPS, dps)) * model.TICKS_PER_DEG / c["speed_unit"])
        acc = int(math.ceil(max(1.0, dps2) * model.TICKS_PER_DEG / c["acc_unit"]))
        self.arm.sync_move({sid: model.deg_to_ticks(c, j, a) for j, (sid, a) in enumerate(zip(IDS, angles))},
                           speed, acc)

    def _loop(self):
        states = [self.arm.servo(sid).torque for sid in IDS]
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
                if ready and not self.stopped:   # keep a goal queued until every servo has read back
                    goal, self._pending_goal = self._pending_goal, None
            if zero and ready:
                with self._lock:
                    self.calib["zero"] = list(self.ticks)
                    self.calib["calibrated"] = True
                    self._calib_changed = time.monotonic()
                model.save_calibration(self.calib)
            if tq is not None and tq != self.torque:
                self._set_torque(tq)
            if goal is not None:
                self._send_goal(goal)
            ticks = self.arm.read_positions(IDS)
            with self._lock:
                self.ticks = ticks
            time.sleep(0.02)
