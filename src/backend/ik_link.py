"""Live link between the IK simulator page and the arm, served at /ws/arm (see main.py).

A client either streams joint angles (`goal`, degrees, URDF convention) or sets a `target`: a point for
the tool tip, which this module solves for (ik.py) and drives the arm to along a clear route, re-solving
about 20 times a second on its own thread. Either way every goal is checked for collisions (the pose and
the path to it), turned into servo ticks with the saved calibration and sent to all six servos in one
sync-write packet, and the measured angles are streamed back about 10 times a second.

It also owns the stop state: while stopped, every motion request (from the page, the REST API or
"home all") is refused until someone resumes. It runs recording playback (player.py) in the same
loop, so playback keeps going with no page connected, and it watches for stalled joints: a joint
that stays far from its goal without moving (something in the way) stops the arm.

Protocol (version PROTOCOL). After the auth message (main.py), the backend sends
    {"type": "hello", "protocol": 4}
    {"type": "config", ...}        now, and again whenever it changes (see config())
    {"type": "state", ...}         about 10 times a second (see state())
Messages from the page:
    {"type": "goal", "angles": [deg x6], "speed": 1-360 deg/s, "acc": 1-2000 deg/s², "epoch": n}
                                               epoch = the latest state's; speed is capped at MAX_DPS.
                                               A goal turns torque on if it was off, and ends a target.
    {"type": "target", "xyz": [mm x3], "down": bool, "speed": ..., "acc": ..., "epoch": n}
    {"type": "target", "angles": [deg x6], "speed": ..., "acc": ..., "epoch": n}
                                               Go there and stay (until a goal, stop, torque, a playback or a
                                               new epoch ends it): xyz is solved for the tool tip (down = flange
                                               facing straight down); angles only gets the route. state.ik says
                                               how it's going.
    {"type": "torque", "on": true|false}
    {"type": "stop"} / {"type": "resume"}
    {"type": "set_zero"}                       current pose becomes the kinematic zero
    {"type": "set_dir", "joint": 0-5, "dir": 1|-1}
    {"type": "set_tool", "attachment": "none"|"vacuum"|"custom", "mm": 0-150, "d_mm": 1-60}
                                               (mm/d_mm only matter for "custom")
    {"type": "set_stall_guard", "on": true|false}
    {"type": "set_area", "enabled": bool, "center": -180..180, "span": 30-360, "radius_mm": 0|100-450,
     "base_mm": 0|60-250 (keep-out around the base, below radius_mm; default 150)}
    {"type": "recenter", "joints": [0-5, ...]} re-centre those servos where they are (torque off); answered
                                               with {"type": "recentered", "results": [...]}
A joint that reads well outside what its servo can reach (state.out_of_range says which and why) stops the arm
and refuses every move until it's back in range: it has usually gone past the servo's 0/4095 point, and the
servo would turn it the wrong way round.
A command that can't be applied gets {"type": "error", "code": "bad_request"|"refused"|"internal",
"ref": <its type>, "message": str}; a command that worked shows up in the next config/state.

The epoch goes up whenever the arm's pose has to be re-read before new goals make sense: a resume,
a calibration change (zero or direction) or the end of a playback. Goals from an older epoch are
refused, so a goal computed before the change can never move the arm.
"""
import math
import os
import queue
import sys
import threading
import time
import traceback

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import arm_model as model  # noqa: E402
import ik  # noqa: E402

PROTOCOL = 4
MAX_DPS = 150            # speed cap no matter what the page asks for
GOAL_SPEED = (1, 360)    # deg/s a goal may ask for (capped to MAX_DPS on the arm)
GOAL_ACC = (1, 2000)     # deg/s²
HOLD_DPS = 20            # speed used when re-enabling torque at the current pose
STALL_DEG = 6.0          # a joint this far from its goal...
STALL_S = 1.0            # ...that hasn't moved STALL_MOVE_DEG for this long is stalled
STALL_MOVE_DEG = 0.5
LIMITS_EVERY_S = 1.0     # how often the loop re-reads the servo limits (they're cached once read)
SOLVE_DT = 0.05          # a target is re-solved this often (seconds)
RESEND_S = 0.2           # ...and its next pose re-sent at least this often while it's unchanged
ARRIVED_DEG = 1.0        # state.ik.arrived: every joint this close to the solution
STILL_DEG = 0.2          # a settled target isn't re-solved until the arm moves more than this (reading noise is ~0.1°)
RANGE_TOL_DEG = 10.0     # a joint reading this far outside what its servo can reach isn't moved (see _range_problem)
SEAM_JUMP = 2048         # a reading that jumps this many ticks between reads went past the servo's 0/4095 point
IDS = model.JOINT_IDS
COMMANDS = ("goal", "target", "torque", "stop", "resume", "set_zero", "set_dir", "set_tool", "set_area",
            "set_stall_guard", "recenter")


def _num(v, lo, hi):
    """A finite number (not a bool) in lo..hi."""
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and lo <= v <= hi


def _int(v, lo, hi):
    return isinstance(v, int) and not isinstance(v, bool) and lo <= v <= hi


def _error(code, ref, message):
    return {"type": "error", "code": code, "ref": ref, "message": message}


class IKLink:
    def __init__(self, arm):
        self.arm = arm
        self.calib = model.load_calibration()
        self._lock = threading.Lock()
        self._clients = 0
        self._thread = None
        self._pending_goal = None        # (angles, speed, acc, epoch)
        self._target = None              # the active target: parse_target's dict + speed, acc, epoch
        self._ik = None                  # the last solve for it (state.ik)
        self._ik_q = None                # its solution (radians), where the next solve starts
        self._last_next = None           # (pose queued for the servos, when)
        self._solved = None              # (what was solved: target and settings, from where, the result)
        self._solver = ik.Solver()
        self._solve_thread = None
        self._pending_torque = None
        self._pending_zero = False
        self.epoch = 0                   # see the module docstring
        self.config_rev = 0              # goes up whenever config() would change
        self._limits = None              # _limits_for(calib): joint limits in degrees, None until read
        self.ticks = [None] * 6
        self.torque = False
        self.stopped = False
        self.blocked = None
        self.fault = None
        self.out_of_range = None         # why the arm can't be moved: a joint reads outside its servo's reach
        self.simulated = False           # the arm is the simulated one (main.py sets it; config says so)
        self._recentering = False
        self.stall_guard = True
        self.player = None
        self.play_end = {"n": 0, "message": None}
        self._cmd = None                 # last goal ticks sent to each servo (clamped), None if unknown
        self._ref = [None] * 6           # stall detection: (ticks, time) of each joint's last real movement
        self._quit = False
        self._leds = None
        self._last_error = None

    # -- helpers used by the REST API too -----------------------------------------

    @property
    def tool_m(self):
        return self.calib["tool_mm"] / 1000

    @property
    def tool_r(self):
        return self.calib["tool_d_mm"] / 2000

    @property
    def area(self):
        return self.calib["area"]

    def check_ticks(self, new_ticks):
        """Collision check for a raw-tick move from the current pose. None if clear."""
        now = self.arm.read_positions(IDS)
        return self._range_problem(now) or model.check_tick_move(self.calib, now, new_ticks)

    def range_problem(self):
        """Why the arm mustn't be moved from where it is now (read fresh), or None."""
        return self._range_problem(self.arm.read_positions(IDS))

    def _range_problem(self, ticks):
        """A joint that reads well outside what its servo can reach with this calibration can't be trusted to
        move: usually it has been pushed past the servo's 0/4095 point, so it reads half a turn away and the
        servo would turn it the wrong way round, into the arm, to "get back". Those are only safe to move by
        hand. Returns the reason, or None."""
        with self._lock:
            calib = dict(self.calib, zero=list(self.calib["zero"]), dir=list(self.calib["dir"]))
            lims = self._limits
        for j, t in enumerate(ticks):
            if t is None:
                continue
            lo, hi = lims[j] if lims and lims[j][0] < lims[j][1] else model.URDF_LIMITS_DEG[j]
            a = model.ticks_to_deg(calib, j, t)
            if a < lo - RANGE_TOL_DEG or a > hi + RANGE_TOL_DEG:
                return (f"J{j + 1} reads {model.near_deg(a):.0f}°, outside the {lo:.0f}° to {hi:.0f}° its servo can "
                        f"reach, so the arm won't move it: its servo may have gone past its 0/4095 point and would "
                        f"turn it the wrong way round. Move it back by hand (Hand-guide), or re-centre the servos "
                        f"(Setup, Calibration wizard).")
        return None

    def stop(self, fault=None):
        """Hold every servo where it is and refuse motion until resume()."""
        with self._lock:
            self.stopped = True
            self._pending_goal = None
            self._clear_target_locked()
            if fault:
                self.fault = fault
            self._end_play_locked(fault or "Stopped.")
        held = self.arm.hold(IDS)
        with self._lock:
            self._cmd = held if all(p is not None for p in held) else None

    def resume(self):
        with self._lock:
            self.stopped = False
            self.blocked = None
            self.fault = None
            self._new_epoch_locked()

    def solver_defaults(self):
        """(calibration copy, joint limits in degrees or None) for a solve-only session (/ws/ik)."""
        with self._lock:
            return dict(self.calib, area=dict(self.calib["area"])), (
                [list(l) for l in self._limits] if self._limits else None)

    def set_obstacles(self, obstacles):
        """Replace the obstacles (already cleaned: model.clean_obstacles). Saved, and sent in the next config."""
        with self._lock:
            self.calib["area"] = dict(self.calib["area"], obstacles=obstacles)
            self._pending_goal = None
            self.config_rev += 1
        self._save()

    def forget_goal(self):
        """Something other than this loop (a REST move, torque via REST) changed the servos' goals."""
        with self._lock:
            self._cmd = None

    def shutdown(self):
        """End the bus loop and the solver (the app is closing)."""
        with self._lock:
            self._quit = True
            self.player = None
        for t in (self._thread, self._solve_thread):
            if t and t.is_alive() and t is not threading.current_thread():
                t.join(timeout=2)

    def _new_epoch_locked(self):
        """The page must re-read the pose: goals computed before now are refused."""
        self.epoch += 1
        self._pending_goal = None
        self._clear_target_locked()

    def _clear_target_locked(self):
        self._target = self._ik = self._ik_q = self._last_next = self._solved = None

    # -- playback --------------------------------------------------------------------

    def start_playback(self, pb):
        """Start a player.Playback. Raises RuntimeError if the arm is stopped."""
        with self._lock:
            if self.stopped:
                raise RuntimeError("stopped")
            if self.out_of_range:
                raise RuntimeError(self.out_of_range)
            self.player = pb
            self.blocked = None
            self._pending_goal = None
            self._clear_target_locked()
        self._ensure_thread()

    def stop_playback(self, message="Playback stopped."):
        with self._lock:
            was = self.player is not None
            self._end_play_locked(message)
        if was:
            held = self.arm.hold(IDS)
            with self._lock:
                self._cmd = held if all(p is not None for p in held) else None

    def playback_status(self):
        with self._lock:
            return self.player.status() if self.player else None

    def _end_play_locked(self, message):
        if self.player is not None:
            self.player = None
            self.play_end = {"n": self.play_end["n"] + 1, "message": message}
            self._new_epoch_locked()

    def _fire_leds(self, events):
        """LED cues run on their own thread: an ATOM write takes up to 60 ms and must not delay goals."""
        if self._leds is None:
            self._leds = queue.Queue()
            threading.Thread(target=self._led_worker, daemon=True).start()
        for e in events:
            self._leds.put(e)

    def _led_worker(self):
        atom = self.arm.atom
        while True:
            _, kind, args = self._leds.get()
            try:
                if kind == "color":
                    atom.set_color(*args)
                elif kind == "pixel":
                    atom.pixel(*args)
                elif kind == "brightness":
                    atom.set_brightness(args[0])
            except Exception:
                pass

    def _limits_for(self, calib):
        """URDF limits intersected with each servo's safe EEPROM range, in joint degrees."""
        out = []
        for j, sid in enumerate(IDS):
            lo_t, hi_t = self.arm.safe_limits(sid)
            a, b = sorted((model.ticks_to_deg(calib, j, lo_t), model.ticks_to_deg(calib, j, hi_t)))
            lo, hi = max(a, model.URDF_LIMITS_DEG[j][0]), min(b, model.URDF_LIMITS_DEG[j][1])
            out.append([round(lo, 1), round(hi, 1)] if lo < hi else [0.0, 0.0])
        return out

    def _refresh_limits(self):
        """Re-read the limits (a servo that didn't answer at first may have since)."""
        with self._lock:
            key = (list(self.calib["zero"]), list(self.calib["dir"]))
            calib = dict(self.calib, zero=key[0], dir=key[1])
        lim = self._limits_for(calib)
        with self._lock:
            same = (self.calib["zero"], self.calib["dir"]) == key
            if lim != self._limits and (same or self._limits is None):   # else a newer calibration's are set
                self._limits = lim
                self.config_rev += 1

    def _save(self):
        with self._lock:
            c = dict(self.calib, zero=list(self.calib["zero"]), dir=list(self.calib["dir"]))
        model.save_calibration(c)

    # -- clients -----------------------------------------------------------------

    def add_client(self):
        with self._lock:
            self._clients += 1
        self._ensure_thread()

    def _ensure_thread(self):
        with self._lock:
            if self._thread is None or not self._thread.is_alive():
                self._thread = threading.Thread(target=self._loop, daemon=True)
                self._thread.start()

    def remove_client(self):
        with self._lock:
            self._clients = max(0, self._clients - 1)
            if self._clients == 0:
                self._pending_goal = None   # nobody is driving: servos just hold their last goal
                self._clear_target_locked()

    def handle(self, msg):
        """Apply one message from a page. Returns an error message to send back, or None."""
        t = msg.get("type")
        if t not in COMMANDS:
            return _error("bad_request", t if isinstance(t, str) else None, f"Unknown message type {t!r}.")
        return getattr(self, f"_on_{t}")(msg)

    def _stop_play_for(self, t):
        if self.player is not None:
            self.stop_playback("Playback stopped: torque changed." if t == "torque"
                               else "Playback stopped: calibration changed.")

    def _on_stop(self, msg):
        self.stop()

    def _on_resume(self, msg):
        self.resume()

    def _motion_error(self, t, msg):
        """What's wrong with a goal or target's speed, acc and epoch, or why it can't run now (lock held)."""
        speed, acc, epoch = (msg.get(k) for k in ("speed", "acc", "epoch"))
        if not _num(speed, *GOAL_SPEED):
            return _error("bad_request", t, f"speed must be {GOAL_SPEED[0]}-{GOAL_SPEED[1]} deg/s.")
        if not _num(acc, *GOAL_ACC):
            return _error("bad_request", t, f"acc must be {GOAL_ACC[0]}-{GOAL_ACC[1]} deg/s².")
        if not _int(epoch, 0, 2 ** 53):
            return _error("bad_request", t, "epoch must be the epoch from the latest state.")
        if self.stopped:
            return _error("refused", t, "The arm is stopped. Resume first.")
        if self.player is not None:
            return _error("refused", t, "A playback is running.")
        if epoch != self.epoch:
            return _error("refused", t, f"Stale {t}: re-read the pose (epoch is now {self.epoch}).")
        if self.out_of_range:
            return _error("refused", t, self.out_of_range)
        return None

    def _on_goal(self, msg):
        a = msg.get("angles")
        if not (isinstance(a, list) and len(a) == 6 and all(_num(v, -360, 360) for v in a)):
            return _error("bad_request", "goal", "angles must be 6 joint angles in degrees.")
        with self._lock:
            err = self._motion_error("goal", msg)
            if err:
                return err
            self._clear_target_locked()
            self._pending_goal = ([float(v) for v in a], float(msg["speed"]), float(msg["acc"]), msg["epoch"])

    def _on_target(self, msg):
        tgt, why = ik.parse_target(msg)
        if why:
            return _error("bad_request", "target", why)
        with self._lock:
            err = self._motion_error("target", msg)
            if err:
                return err
            if self._target is None:     # a new stream of targets: solve from where the arm is
                self._ik_q = self._last_next = None
            self._target = dict(tgt, speed=float(msg["speed"]), acc=float(msg["acc"]), epoch=msg["epoch"])
        self._ensure_solve_thread()

    def _on_torque(self, msg):
        on = msg.get("on")
        if not isinstance(on, bool):
            return _error("bad_request", "torque", "on must be true or false.")
        self._stop_play_for("torque")
        with self._lock:
            self._pending_torque = on
            self._pending_goal = None
            self._clear_target_locked()

    def _on_set_zero(self, msg):
        self._stop_play_for("set_zero")
        with self._lock:
            missing = [f"J{j + 1}" for j, t in enumerate(self.ticks) if t is None]
            if missing:
                return _error("refused", "set_zero", f"{', '.join(missing)} not answering, so the zero can't be set.")
            self._pending_zero = True
            self._pending_goal = None

    def _on_set_dir(self, msg):
        j, d = msg.get("joint"), msg.get("dir")
        if not (_int(j, 0, 5) and _int(d, -1, 1) and d != 0):
            return _error("bad_request", "set_dir", "joint must be 0-5 and dir 1 or -1.")
        self._stop_play_for("set_dir")
        with self._lock:
            if d == self.calib["dir"][j]:
                return None
            dirs = list(self.calib["dir"])
            dirs[j] = d
            new = dict(self.calib, dir=dirs)
        lim = self._limits_for(new)
        with self._lock:
            self.calib["dir"] = dirs
            self.calib["calibrated"] = True
            self._limits = lim
            self.config_rev += 1
            self._new_epoch_locked()
        self._save()

    def _on_recenter(self, msg):
        """Re-centre servos where they are (see arm_model.recenter), with torque off. The reply is `recentered`
        (one result per joint); the calibration shifts with it, so angles keep their meaning, and the epoch goes
        up (the readings changed under the page)."""
        joints = msg.get("joints", list(range(6)))
        if not (isinstance(joints, list) and joints and all(_int(j, 0, 5) for j in joints) and len(set(joints)) == len(joints)):
            return _error("bad_request", "recenter", "joints must be a list of joint numbers 0-5.")
        with self._lock:
            if self._recentering:
                return _error("refused", "recenter", "Already re-centring.")
            if self.torque:
                return _error("refused", "recenter", "Turn torque off first (Hand-guide) and hold the arm: "
                                                     "a servo holding a goal would jump when its reading changes.")
            if self.player is not None:
                return _error("refused", "recenter", "A playback is running.")
            calib = dict(self.calib, zero=list(self.calib["zero"]), dir=list(self.calib["dir"]))
            self._recentering = True      # readings jump by up to half a turn meanwhile: not a 0/4095 crossing
        try:
            results = model.recenter(self.arm, calib, joints)
            lim = self._limits_for(calib)
        finally:
            with self._lock:
                self._recentering = False
        with self._lock:
            self.ticks = [None] * 6       # start the readings over (no jump from the old ones)
            self.calib["zero"] = calib["zero"]
            self._limits = lim
            self._cmd = None
            self.config_rev += 1
            self._new_epoch_locked()
        self._save()
        return {"type": "recentered", "results": results}

    def _on_set_tool(self, msg):
        att = msg.get("attachment", self.calib["attachment"])
        spec = model.ATTACHMENTS.get(att) if isinstance(att, str) else None
        if spec is None:
            return _error("bad_request", "set_tool", f"attachment must be one of {', '.join(model.ATTACHMENTS)}.")
        if spec["length_mm"] is not None:     # a known attachment: its own size
            mm, d = spec["length_mm"], spec["diameter_mm"] or model.TOOL_R_DEFAULT * 2000
        else:
            mm, d = msg.get("mm", self.calib["tool_mm"]), msg.get("d_mm", self.calib["tool_d_mm"])
            if not (_num(mm, 0, 150) and _num(d, 1, 60)):
                return _error("bad_request", "set_tool", "mm must be 0-150 and d_mm 1-60.")
        with self._lock:
            self.calib.update(attachment=att, tool_mm=float(mm), tool_d_mm=float(d))
            self._pending_goal = None
            self.config_rev += 1
        self._save()

    def _on_set_area(self, msg):
        with self._lock:
            kept = self.calib["area"].get("obstacles")
        if "obstacles" not in msg and kept:   # the area's sliders don't touch the obstacles
            msg = dict(msg, obstacles=kept)
        a = model.clean_area(msg) if isinstance(msg.get("enabled"), bool) else None
        if a is None:
            return _error("bad_request", "set_area",
                          "Needs enabled (bool), center -180..180, span 30-360, radius_mm 0 or 100-450 "
                          "and base_mm 0 or 60-250 (less than radius_mm), and valid obstacles if any.")
        with self._lock:
            self.calib["area"] = a
            self._pending_goal = None
            self.config_rev += 1
        self._save()

    def _on_set_stall_guard(self, msg):
        on = msg.get("on")
        if not isinstance(on, bool):
            return _error("bad_request", "set_stall_guard", "on must be true or false.")
        with self._lock:
            self.stall_guard = on
            self._ref = [None] * 6
            self.config_rev += 1

    # -- what the page is sent ---------------------------------------------------------

    def hello(self):
        return {"type": "hello", "protocol": PROTOCOL}

    def config(self):
        """Settings that change only when someone changes them. Call with the lock held."""
        c = self.calib
        return {
            "type": "config",
            "calibrated": c["calibrated"],
            "zero": list(c["zero"]),
            "dir": list(c["dir"]),
            "tool_mm": c["tool_mm"],
            "tool_d_mm": c["tool_d_mm"],
            "attachment": c["attachment"],
            "area": dict(c["area"]),
            "limits": [list(l) for l in self._limits],
            "stall_guard": self.stall_guard,
            "simulated": self.simulated,
        }

    def snapshot(self, rev):
        """(config or None if it hasn't changed since ``rev``, current rev, state), read together so
        a state's epoch never arrives before the config it belongs to."""
        if self._limits is None:
            self._refresh_limits()
        with self._lock:
            cfg = self.config() if rev != self.config_rev else None
            return cfg, self.config_rev, self._state_locked()

    def state(self):
        with self._lock:
            return self._state_locked()

    def _state_locked(self):
        return {
            "type": "state",
            "angles": [None if a is None else round(model.near_deg(a), 2) for a in model.pose_from_ticks(self.calib, self.ticks)],
            "ticks": list(self.ticks),
            "torque": self.torque,
            "stopped": self.stopped,
            "blocked": self.blocked,
            "fault": self.fault,
            "out_of_range": self.out_of_range,
            "playback": self.player.status() if self.player else None,
            "play_end": dict(self.play_end),
            "epoch": self.epoch,
            "clients": self._clients,
            "ik": dict(self._ik) if self._ik else None,
        }

    # -- target: solving on its own thread (a rescue can take a while; the bus loop mustn't wait) -----------

    def _ensure_solve_thread(self):
        with self._lock:
            if self._solve_thread is None or not self._solve_thread.is_alive():
                self._solve_thread = threading.Thread(target=self._solve_loop, daemon=True)
                self._solve_thread.start()

    def _solve_loop(self):
        while True:
            t0 = time.monotonic()
            with self._lock:
                if self._quit or self._target is None:
                    self._solve_thread = None
                    return
            try:
                self._solve_once(t0)
            except Exception as e:
                self._fail(e)
            time.sleep(max(0.005, SOLVE_DT - (time.monotonic() - t0)))

    def _solve_once(self, now):
        """Solve for the target from where the arm is, and queue the next pose on the route to it as a goal
        (the bus loop sends it through _send_goal, with every guard, like any other goal)."""
        with self._lock:
            tgt = self._target
            if tgt is None:
                return
            cur = model.pose_from_ticks(self.calib, self.ticks)
            q = self._ik_q
            lim = None
            if self._limits:
                lim = [(lo * ik.DEG, hi * ik.DEG) if lo < hi else u for (lo, hi), u in zip(self._limits, ik.URDF_LIM)]
            settings = (self.tool_m, self.tool_r, dict(self.area), lim)
        if any(a is None for a in cur):
            return
        what = (repr((tgt["xyz"], tgt["angles"], tgt["down"])), repr(settings))
        done = self._solved
        hold = solved = None
        if (done and done[0] == what and done[2]["settled"]
                and max(abs(a - b) for a, b in zip(cur, done[1])) <= STILL_DEG):
            res = dict(done[2])   # nothing has changed since a settled answer: solving again would repeat it
        else:
            self._solver.configure(*settings)
            frm = [a * ik.DEG for a in cur]
            q, res = self._solver.step(q, frm, now, xyz=tgt["xyz"], down=tgt["down"], angles=tgt["angles"])
            solved = (what, cur, res)
            last = self._last_next
            if res["next"] is None and last and self._solver.path(frm, [a * ik.DEG for a in last[0]], every=ik.DEG):
                hold = cur    # no route now, and the rest of the servos' current move isn't clear: stop where they are
        res["arrived"] = (res["next"] is not None and not res["detour"]
                          and all(abs(a - b) <= ARRIVED_DEG for a, b in zip(cur, res["angles"])))
        with self._lock:
            cur_t = self._target
            if cur_t is None or cur_t["epoch"] != tgt["epoch"]:
                return    # ended while solving
            self._ik_q, self._ik = q, res
            if solved:
                self._solved = solved
            if self.stopped or self.player is not None:
                return
            nxt = hold or res["next"]
            if nxt is None:
                return
            last = self._last_next
            if last is None or max(abs(a - b) for a, b in zip(nxt, last[0])) > 0.01 or now - last[1] >= RESEND_S:
                self._pending_goal = (list(nxt), cur_t["speed"], cur_t["acc"], cur_t["epoch"])
                self._last_next = (list(nxt), now)

    # -- bus loop --------------------------------------------------------------------

    def _set_torque(self, on):
        if on:  # hold the current pose: goal = present position before torque comes back
            now = self.arm.read_positions(IDS)
            if any(p is None for p in now):
                return
            self.arm.sync_move(dict(zip(IDS, now)), int(HOLD_DPS * model.TICKS_PER_DEG), 10)
            self._set_cmd(now)
        else:
            with self._lock:
                self._cmd = None
        self.arm.sync_torque(IDS, on)
        self.torque = on

    def _set_cmd(self, targets):
        """Remember the goals just sent (as the servos will clamp them) for the stall check."""
        new = [max(lo, min(hi, t)) for t, (lo, hi) in zip(targets, (self.arm.safe_limits(s) for s in IDS))]
        now = time.monotonic()
        with self._lock:
            old, ticks = self._cmd, self.ticks
            for j in range(6):
                # a new goal for a joint that was at (or near) its old one restarts its stall timer;
                # a joint already far behind keeps its timer, so streaming goals can't hide a stall
                near = old is None or ticks[j] is None or abs(ticks[j] - old[j]) < STALL_DEG * model.TICKS_PER_DEG
                if near and (old is None or new[j] != old[j]):
                    self._ref[j] = (ticks[j], now)
            self._cmd = new

    def _send_goal(self, goal):
        """Collision-check the move from the current pose and send it. Returns the reason if refused.
        ``goal`` is (angles deg, speed deg/s or [deg/s x6], acc deg/s²), already validated. A single speed is
        the fastest joint's; the others get their share of it (see below)."""
        angles, dps, dps2 = goal
        current = model.pose_from_ticks(self.calib, self.ticks)
        why = self._range_problem(self.ticks) or model.check_path(current, angles, self.tool_m, tool_r=self.tool_r,
                                                                  area=self.area)
        with self._lock:
            self.blocked = why
        if why:
            return why
        if not self.torque:
            self._set_torque(True)
        c = self.calib
        reg = lambda d: int(max(1.0, min(MAX_DPS, d)) * model.TICKS_PER_DEG / c["speed_unit"])
        if not isinstance(dps, (list, tuple)):
            # one speed for the move: split it between the joints by how far each has to go, so they arrive
            # together and the arm follows the straight joint-space path check_path looked at. The
            # acceleration stays the same for every joint, so each can still stop quickly.
            far = max(abs(a - b) for a, b in zip(angles, current))
            dps = [min(MAX_DPS, dps) * (abs(a - b) / far if far > 1e-6 else 1.0) for a, b in zip(angles, current)]
        speed = {sid: reg(d) for sid, d in zip(IDS, dps)}
        acc = int(math.ceil(max(1.0, min(GOAL_ACC[1], dps2)) * model.TICKS_PER_DEG / c["acc_unit"]))
        targets = [model.deg_to_ticks(c, j, a) for j, a in enumerate(angles)]
        self.arm.sync_move(dict(zip(IDS, targets)), speed, acc)
        self._set_cmd(targets)
        return None

    def _check_stall(self, ticks, now):
        """A torqued joint far from its goal that hasn't moved for STALL_S: something is in the way."""
        with self._lock:
            cmd = self._cmd
            if not (self.stall_guard and self.torque and not self.stopped and cmd) or None in ticks:
                return None
            for j in range(6):
                ref = self._ref[j]
                if ref is None or ref[0] is None or abs(ticks[j] - ref[0]) > STALL_MOVE_DEG * model.TICKS_PER_DEG:
                    self._ref[j] = ref = (ticks[j], now)
                err = abs(ticks[j] - cmd[j]) / model.TICKS_PER_DEG
                if err > STALL_DEG and now - ref[1] > STALL_S:
                    return (f"J{j + 1} stalled {err:.0f}° short of its goal, so the arm stopped. "
                            f"Check nothing is in the way, then resume.")
        return None

    def _play_tick(self):
        with self._lock:
            pb = self.player
            current = model.pose_from_ticks(self.calib, self.ticks)
        if pb is None or self.stopped:
            return
        act = pb.tick(time.monotonic(), current)
        if act.events:
            self._fire_leds(act.events)
        if act.goal is not None:
            why = self._send_goal((act.goal, act.speeds, pb.acc))
            if why:
                self.stop_playback(f"Playback stopped: {why}.")
                return
        if act.done:
            with self._lock:
                if self.player is pb:
                    self._end_play_locked("Playback finished.")

    def _apply_zero(self):
        with self._lock:
            zero = list(self.ticks)
            new = dict(self.calib, zero=zero)
        lim = self._limits_for(new)
        with self._lock:
            self.calib["zero"] = zero
            self.calib["calibrated"] = True
            self._limits = lim
            self.config_rev += 1
            self._new_epoch_locked()
        self._save()

    def _fail(self, err):
        """Something in the loop raised: stop the arm and say why, rather than let the loop die."""
        msg = f"Arm link error ({type(err).__name__}: {err}), so the arm stopped. Check the bus, then resume."
        if msg != self._last_error:
            traceback.print_exc()
            self._last_error = msg
        try:
            self.stop(msg)
        except Exception:
            pass    # stop() sets the stop state before touching the bus

    def _step(self, next_limits):
        with self._lock:
            ready = all(t is not None for t in self.ticks)
            tq, self._pending_torque = self._pending_torque, None
            zero = ready and self._pending_zero
            if zero:
                self._pending_zero = False
            goal = None
            if ready and not self.stopped and not zero:   # keep a goal queued until every servo reads back
                goal, self._pending_goal = self._pending_goal, None
                if goal is not None and goal[3] != self.epoch:
                    goal = None
        if zero:
            self._apply_zero()
        if tq is not None and tq != self.torque:
            self._set_torque(tq)
        if goal is not None:
            self._send_goal(goal[:3])
        if ready:
            self._play_tick()
        ticks = self.arm.read_positions(IDS)
        with self._lock:
            prev, self.ticks = self.ticks, ticks
        self._check_range(prev, ticks)
        fault = self._check_stall(ticks, time.monotonic())
        if fault:
            self.stop(fault)
        if time.monotonic() >= next_limits:
            self._refresh_limits()
            return time.monotonic() + LIMITS_EVERY_S
        return next_limits

    def _check_range(self, prev, ticks):
        """Keep out_of_range up to date, and stop the arm (if it's holding with torque) when it first goes bad."""
        problem = None
        with self._lock:
            recentering = self._recentering
        for j, (a, b) in enumerate(zip(prev, ticks)):
            if not recentering and a is not None and b is not None and abs(b - a) >= SEAM_JUMP:
                problem = (f"J{j + 1}'s servo went past its 0/4095 point (its reading jumped from {a} to {b}), so the "
                           f"arm stopped. Move it back by hand (Hand-guide), or re-centre the servos (Setup, "
                           f"Calibration wizard).")
        problem = problem or self._range_problem(ticks)
        with self._lock:
            was, self.out_of_range = self.out_of_range, problem
            stop = problem and not was and self.torque and not self.stopped
        if stop:
            self.stop(problem)

    def _loop(self):
        next_limits = 0.0
        try:
            self.torque = all(self.arm.servo(sid).torque for sid in IDS)
        except Exception as e:
            self._fail(e)
        while True:
            with self._lock:
                if self._quit or (self._clients == 0 and self.player is None):
                    self._thread = None
                    return
            try:
                next_limits = self._step(next_limits)
                self._last_error = None
                time.sleep(0.02)
            except Exception as e:
                self._fail(e)
                time.sleep(0.2)
