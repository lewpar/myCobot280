"""Playback of recordings and sequences, run by the IK link's bus loop (ik_link.py).

Pure timing logic: ``Playback.tick(now, current)`` says which goal to send (if any), with what
per-joint speeds, and which LED cues are due. The link sends the goal through its normal path
(collision check of the move, calibration, EEPROM clamps), so playback can't bypass the guards.

Each step goes through phases:
    approach  move to the recording's first pose at the chosen speed and wait until the arm is there
    run       follow the frames; with ``timed`` each joint gets the speed that keeps it on the
              recording's timing, otherwise every joint uses the chosen speed (and may lag)
    finish    wait until the arm reaches the last frame
    zero      (recordings saved with return_zero) move to the zero pose
    pause     (sequences) hold for the step's pause before the next step
"""
import arm_model as model

GOAL_DT = 0.1          # seconds between goals
LEAD = 0.15            # timed mode aims this far (wall time) ahead of "now" on the recording
ARRIVE_DEG = 2.0       # "arrived" tolerance for approach and zero
ARRIVE_GRACE = 3.0     # ...but don't wait longer than the expected travel time plus this
MIN_DPS = 3.0          # never ask a joint for less than this in timed mode


def pose_at(frames, t, hint=0):
    """Linear interpolation of the frames at time t. Returns (angles, index hint)."""
    i = hint if 0 <= hint < len(frames) - 1 and frames[hint][0] <= t else 0
    while i < len(frames) - 2 and frames[i + 1][0] <= t:
        i += 1
    a, b = frames[i], frames[i + 1]
    u = 1.0 if b[0] <= a[0] else max(0.0, min(1.0, (t - a[0]) / (b[0] - a[0])))
    return [a[j] + (b[j] - a[j]) * u for j in range(1, 7)], i


def check_frames(frames, tool_m=0.0, tool_r=model.TOOL_R_DEFAULT, area=None):
    """Collision check of a recording: None if clear, else (t, reason) for the first bad frame.
    Poses less than a degree apart from the last checked one are skipped."""
    last = None
    for k, f in enumerate(frames):
        q = f[1:]
        if last is not None and k < len(frames) - 1 and max(abs(x - y) for x, y in zip(q, last)) < 1.0:
            continue
        why = model.check_pose(q, tool_m, tool_r, area)
        if why:
            return f[0], why
        last = q
    return None


def check_steps(steps, tool_m=0.0, tool_r=model.TOOL_R_DEFAULT, area=None):
    """None if every step is clear, else a message naming the step and time."""
    for n, s in enumerate(steps):
        bad = check_frames(s["frames"], tool_m, tool_r, area)
        where = f'"{s["name"]}"' + (f" (step {n + 1})" if len(steps) > 1 else "")
        if bad:
            return f"{where} at {bad[0]:.1f} s: {bad[1]}"
        if s.get("return_zero"):
            why = model.check_pose([0] * 6, tool_m, tool_r, area)
            if why:
                return f"{where}: the zero pose is blocked ({why})"
    return None


class Action:
    __slots__ = ("goal", "speeds", "events", "done")

    def __init__(self):
        self.goal, self.speeds, self.events, self.done = None, None, [], False


class Playback:
    def __init__(self, name, steps, now, rate=1.0, loop=False, timed=True, speed=60.0, acc=200.0):
        """``steps``: [{"name", "frames", "events", "return_zero", "pause"}]; speed in deg/s, acc deg/s²."""
        self.name, self.steps = name, steps
        self.rate, self.loop, self.timed = rate, loop, timed
        self.speed, self.acc = speed, acc
        self.step = 0
        self.last_goal = -1e9
        self._enter("approach", now)

    def _enter(self, phase, now):
        self.phase, self.since, self.t, self.hint, self.next_event = phase, now, 0.0, 0, 0
        self.arrived_by = None
        self.last_goal = -1e9   # send the new phase's first goal straight away

    @property
    def cur(self):
        return self.steps[self.step]

    def status(self):
        s = self.cur
        return {"name": self.name, "recording": s["name"], "step": self.step, "steps": len(self.steps),
                "phase": self.phase, "t": round(self.t, 2), "duration": s["frames"][-1][0],
                "loop": self.loop, "rate": self.rate, "timed": self.timed}

    def _travel(self, current, goal, now, act):
        """Move to a fixed pose; True once the arm is there (or has had long enough)."""
        if self.arrived_by is None:
            far = max((abs(g - c) for g, c in zip(goal, current) if c is not None), default=0)
            self.arrived_by = now + far / max(self.speed, 1) + ARRIVE_GRACE
        if now - self.last_goal >= GOAL_DT:
            act.goal, act.speeds, self.last_goal = goal, [self.speed] * 6, now
        there = all(c is not None and abs(g - c) <= ARRIVE_DEG for g, c in zip(goal, current))
        return there or now > self.arrived_by

    def _after_step(self, now, act):
        if self.cur.get("pause", 0) > 0 and (self.step + 1 < len(self.steps) or self.loop):
            self._enter("pause", now)
        else:
            self._next_step(now, act)

    def _next_step(self, now, act):
        if self.step + 1 < len(self.steps):
            self.step += 1
        elif self.loop:
            self.step = 0
        else:
            act.done = True
            return
        self._enter("approach", now)

    def tick(self, now, current):
        """``current``: measured angles (deg, None where unknown). Returns an Action."""
        act = Action()
        s = self.cur
        frames = s["frames"]
        if self.phase == "approach":
            if self._travel(current, frames[0][1:], now, act):
                self._enter("run", now)
            return act
        if self.phase == "run":
            end = frames[-1][0]
            self.t = min(end, (now - self.since) * self.rate)
            events = s.get("events") or []
            while self.next_event < len(events) and events[self.next_event][0] <= self.t:
                act.events.append(events[self.next_event])
                self.next_event += 1
            if now - self.last_goal >= GOAL_DT or self.t >= end:
                ahead = min(end, self.t + (LEAD * self.rate if self.timed else 0))
                goal, self.hint = pose_at(frames, ahead, self.hint)
                if self.timed:
                    # the recording's own speed over the next LEAD (wall time), or faster if the
                    # joint has fallen behind and must catch up
                    now_q, _ = pose_at(frames, self.t, self.hint)
                    speeds = [max(MIN_DPS, 1.1 * abs(g - q) / LEAD, (abs(g - c) / LEAD if c is not None else 0))
                              for g, q, c in zip(goal, now_q, current)]
                else:
                    speeds = [self.speed] * 6
                act.goal, act.speeds, self.last_goal = goal, speeds, now
            if self.t >= end:
                self._enter("finish", now)
                self.t, self.last_goal = end, now
            return act
        if self.phase == "finish":
            self.t = frames[-1][0]
            if self._travel(current, frames[-1][1:], now, act):
                self._enter("zero", now) if s.get("return_zero") else self._after_step(now, act)
            return act
        if self.phase == "zero":
            if self._travel(current, [0.0] * 6, now, act):
                self._after_step(now, act)
            return act
        if self.phase == "pause":
            self.t = now - self.since
            if self.t >= s.get("pause", 0):
                self._next_step(now, act)
            return act
        act.done = True
        return act
