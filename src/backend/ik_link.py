"""Live link between the IK simulator page and the arm, served at /ws/arm (see main.py).

The page streams joint angles (degrees, URDF convention); this module checks each pose and the path
to it for collisions, turns the angles into servo ticks with the saved calibration, sends all six
goals in one sync-write packet, and streams the measured angles back about 10 times a second.

It also owns the stop state: while stopped, every motion request (from the page, the REST API or
"home all") is refused until someone resumes. It runs recording playback (player.py) in the same
loop, so playback keeps going with no page connected, and it watches for stalled joints: a joint
that stays far from its goal without moving (something in the way) stops the arm.

Protocol (version PROTOCOL). After the auth message (main.py), the backend sends
    {"type": "hello", "protocol": 2}
    {"type": "config", ...}        now, and again whenever it changes (see config())
    {"type": "state", ...}         about 10 times a second (see state())
Messages from the page:
    {"type": "goal", "angles": [deg x6], "speed": 1-360 deg/s, "acc": 1-2000 deg/s², "epoch": n}
                                               epoch = the latest state's; speed is capped at MAX_DPS.
                                               A goal turns torque on if it was off.
    {"type": "torque", "on": true|false}
    {"type": "stop"} / {"type": "resume"}
    {"type": "set_zero"}                       current pose becomes the kinematic zero
    {"type": "set_dir", "joint": 0-5, "dir": 1|-1}
    {"type": "set_tool", "attachment": "none"|"vacuum"|"custom", "mm": 0-150, "d_mm": 1-60}
                                               (mm/d_mm only matter for "custom")
    {"type": "set_stall_guard", "on": true|false}
    {"type": "set_area", "enabled": bool, "center": -180..180, "span": 30-360, "radius_mm": 0|100-450,
     "base_mm": 0|60-250 (keep-out around the base, below radius_mm; default 150)}
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

PROTOCOL = 2
MAX_DPS = 150            # speed cap no matter what the page asks for
GOAL_SPEED = (1, 360)    # deg/s a goal may ask for (capped to MAX_DPS on the arm)
GOAL_ACC = (1, 2000)     # deg/s²
HOLD_DPS = 20            # speed used when re-enabling torque at the current pose
STALL_DEG = 6.0          # a joint this far from its goal...
STALL_S = 1.0            # ...that hasn't moved STALL_MOVE_DEG for this long is stalled
STALL_MOVE_DEG = 0.5
LIMITS_EVERY_S = 1.0     # how often the loop re-reads the servo limits (they're cached once read)
IDS = model.JOINT_IDS
COMMANDS = ("goal", "torque", "stop", "resume", "set_zero", "set_dir", "set_tool", "set_area",
            "set_stall_guard")


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
        return model.check_tick_move(self.calib, self.arm.read_positions(IDS), new_ticks)

    def stop(self, fault=None):
        """Hold every servo where it is and refuse motion until resume()."""
        with self._lock:
            self.stopped = True
            self._pending_goal = None
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

    def forget_goal(self):
        """Something other than this loop (a REST move, torque via REST) changed the servos' goals."""
        with self._lock:
            self._cmd = None

    def shutdown(self):
        """End the bus loop (the app is closing)."""
        with self._lock:
            self._quit = True
            self.player = None
        t = self._thread
        if t and t.is_alive() and t is not threading.current_thread():
            t.join(timeout=2)

    def _new_epoch_locked(self):
        """The page must re-read the pose: goals computed before now are refused."""
        self.epoch += 1
        self._pending_goal = None

    # -- playback --------------------------------------------------------------------

    def start_playback(self, pb):
        """Start a player.Playback. Raises RuntimeError if the arm is stopped."""
        with self._lock:
            if self.stopped:
                raise RuntimeError("stopped")
            self.player = pb
            self.blocked = None
            self._pending_goal = None
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

    def _on_goal(self, msg):
        a, speed, acc, epoch = (msg.get(k) for k in ("angles", "speed", "acc", "epoch"))
        if not (isinstance(a, list) and len(a) == 6 and all(_num(v, -360, 360) for v in a)):
            return _error("bad_request", "goal", "angles must be 6 joint angles in degrees.")
        if not _num(speed, *GOAL_SPEED):
            return _error("bad_request", "goal", f"speed must be {GOAL_SPEED[0]}-{GOAL_SPEED[1]} deg/s.")
        if not _num(acc, *GOAL_ACC):
            return _error("bad_request", "goal", f"acc must be {GOAL_ACC[0]}-{GOAL_ACC[1]} deg/s².")
        if not _int(epoch, 0, 2 ** 53):
            return _error("bad_request", "goal", "epoch must be the epoch from the latest state.")
        with self._lock:
            if self.stopped:
                return _error("refused", "goal", "The arm is stopped. Resume first.")
            if self.player is not None:
                return _error("refused", "goal", "A playback is running.")
            if epoch != self.epoch:
                return _error("refused", "goal", f"Stale goal: re-read the pose (epoch is now {self.epoch}).")
            self._pending_goal = ([float(v) for v in a], float(speed), float(acc), epoch)

    def _on_torque(self, msg):
        on = msg.get("on")
        if not isinstance(on, bool):
            return _error("bad_request", "torque", "on must be true or false.")
        self._stop_play_for("torque")
        with self._lock:
            self._pending_torque = on
            self._pending_goal = None

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
        a = model.clean_area(msg) if isinstance(msg.get("enabled"), bool) else None
        if a is None:
            return _error("bad_request", "set_area",
                          "Needs enabled (bool), center -180..180, span 30-360, radius_mm 0 or 100-450 "
                          "and base_mm 0 or 60-250 (less than radius_mm).")
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
            "angles": [None if a is None else round(a, 2) for a in model.pose_from_ticks(self.calib, self.ticks)],
            "torque": self.torque,
            "stopped": self.stopped,
            "blocked": self.blocked,
            "fault": self.fault,
            "playback": self.player.status() if self.player else None,
            "play_end": dict(self.play_end),
            "epoch": self.epoch,
            "clients": self._clients,
        }

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
        why = model.check_path(current, angles, self.tool_m, tool_r=self.tool_r, area=self.area)
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
            self.ticks = ticks
        fault = self._check_stall(ticks, time.monotonic())
        if fault:
            self.stop(fault)
        if time.monotonic() >= next_limits:
            self._refresh_limits()
            return time.monotonic() + LIMITS_EVERY_S
        return next_limits

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
