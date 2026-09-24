"""Live link between the IK simulator page and the arm, served at /ws/arm (see main.py).

The page streams joint angles (degrees, URDF convention); this module checks each pose and the path
to it for collisions, turns the angles into servo ticks with the saved calibration, sends all six
goals in one sync-write packet, and streams the measured angles back about 10 times a second.

It also owns the stop state: while stopped, every motion request (from the page, the REST API or
"home all") is refused until someone resumes. It runs recording playback (player.py) in the same
loop, so playback keeps going with no page connected, and it watches for stalled joints: a joint
that stays far from its goal without moving (something in the way) stops the arm.

Messages from the page (after the auth message, which main.py handles):
    {"type": "goal", "angles": [deg x6], "speed": deg_per_s, "acc": deg_per_s2}
    {"type": "torque", "on": true|false}
    {"type": "stop"} / {"type": "resume"}
    {"type": "set_zero"}                       current pose becomes the kinematic zero
    {"type": "set_dir", "joint": 0-5, "dir": 1|-1}
    {"type": "set_tool", "attachment": "none"|"vacuum"|"custom", "mm": 0-150, "d_mm": 1-60}
                                               (mm/d_mm only matter for "custom")
    {"type": "set_stall_guard", "on": true|false}
    {"type": "set_area", "enabled": bool, "center": deg, "span": 30-360, "radius_mm": 0|100-450}
Message to the page:
    {"type": "state", "angles": [deg|null x6], "torque": bool, "stopped": bool, "blocked": str|null,
     "calibrated": bool, "zero": [...], "dir": [...], "tool_mm": n, "tool_d_mm": n, "attachment": id, "area": {...}, "limits": [[lo, hi] deg x6],
     "fault": str|null, "stall_guard": bool, "playback": {...}|null, "play_end": {"n", "message"}}
Goals from the page are ignored while a playback runs.
"""
import math
import os
import queue
import sys
import threading
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import arm_model as model  # noqa: E402

MAX_DPS = 150            # speed cap no matter what the page asks for
HOLD_DPS = 20            # speed used when re-enabling torque at the current pose
STALL_DEG = 6.0          # a joint this far from its goal...
STALL_S = 1.0            # ...that hasn't moved STALL_MOVE_DEG for this long is stalled
STALL_MOVE_DEG = 0.5
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
        self.fault = None
        self.stall_guard = True
        self.player = None
        self.play_end = {"n": 0, "message": None}
        self._cmd = None                 # last goal ticks sent to each servo (clamped), None if unknown
        self._ref = [None] * 6           # stall detection: (ticks, time) of each joint's last real movement
        self._quit = False
        self._leds = None

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
            self._pending_goal = None
            self._calib_changed = time.monotonic()   # the page re-reads the pose before sending again

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
            self._calib_changed = time.monotonic()   # the page re-reads the pose before sending again

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
        t = msg.get("type")
        if t == "stop":
            self.stop()
            return
        if t == "resume":
            self.resume()
            return
        if t in ("torque", "set_zero", "set_dir") and self.player is not None:
            self.stop_playback({"torque": "Playback stopped: torque changed.",
                                "set_zero": "Playback stopped: calibration changed.",
                                "set_dir": "Playback stopped: calibration changed."}[t])
        with self._lock:
            if t == "goal":
                # after a calibration change or a resume the page re-reads the arm's pose; ignore
                # goals computed before that, which would otherwise make the arm jump
                if self.stopped or self.player is not None or time.monotonic() - self._calib_changed < 0.5:
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
                att = msg.get("attachment", self.calib["attachment"])
                spec = model.ATTACHMENTS.get(att)
                mm, d = msg.get("mm", self.calib["tool_mm"]), msg.get("d_mm", self.calib["tool_d_mm"])
                if spec and spec["length_mm"] is not None:     # a known attachment: its own size
                    mm, d = spec["length_mm"], spec["diameter_mm"] or model.TOOL_R_DEFAULT * 2000
                ok = lambda v, lo, hi: isinstance(v, (int, float)) and not isinstance(v, bool) and lo <= v <= hi
                if spec and ok(mm, 0, 150) and ok(d, 1, 60):
                    self.calib.update(attachment=att, tool_mm=float(mm), tool_d_mm=float(d))
                    self._pending_goal = None
                    model.save_calibration(self.calib)
            elif t == "set_area":
                a = model.clean_area(msg)
                if a:
                    self.calib["area"] = a
                    self._pending_goal = None
                    model.save_calibration(self.calib)
            elif t == "set_stall_guard":
                self.stall_guard = bool(msg.get("on"))
                self._ref = [None] * 6

    def state(self):
        with self._lock:
            ticks, torque, stopped, blocked = list(self.ticks), self.torque, self.stopped, self.blocked
            fault, guard, play_end = self.fault, self.stall_guard, dict(self.play_end)
            playback = self.player.status() if self.player else None
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
            "tool_d_mm": c["tool_d_mm"],
            "attachment": c["attachment"],
            "area": dict(c["area"]),
            "limits": self.limits_deg(),
            "fault": fault,
            "stall_guard": guard,
            "playback": playback,
            "play_end": play_end,
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
        """Collision-check the move from the current pose and send it. Returns the reason if refused."""
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
        speed = {sid: reg(d) for sid, d in zip(IDS, dps)} if isinstance(dps, (list, tuple)) else reg(dps)
        acc = int(math.ceil(max(1.0, dps2) * model.TICKS_PER_DEG / c["acc_unit"]))
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

    def _loop(self):
        states = [self.arm.servo(sid).torque for sid in IDS]
        self.torque = all(states)
        while True:
            with self._lock:
                if self._quit or (self._clients == 0 and self.player is None):
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
            if ready:
                self._play_tick()
            ticks = self.arm.read_positions(IDS)
            with self._lock:
                self.ticks = ticks
            fault = self._check_stall(ticks, time.monotonic())
            if fault:
                self.stop(fault)
            time.sleep(0.02)
