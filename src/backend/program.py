"""Motion programs (the page's Motion Studio): a list of blocks compiled into a recording's frames and LED cues,
so a program plays through player.py like any recording, with every guard.

Blocks (library.clean_blocks validates them; every block has a short "id" the page uses to point at it, and may
have a "note", a label for people):
    {"type": "pose",   "angles": [deg x6], "speed": deg/s}        move to a joint pose
    {"type": "point",  "xyz": [mm x3], "down": bool, "speed"}      move the tool tip to a point (solved here)
    {"type": "home",   "speed"}                                     move to the zero pose
    {"type": "wait",   "seconds": s}
    {"type": "led",    "color": [r, g, b]}                          the ATOM's LEDs, all one colour
    {"type": "repeat", "times": n, "blocks": [...]}

A move is a minimum-jerk profile in joint space (the joint going furthest peaks at the block's speed), sampled
FRAME_DT apart; when the straight path isn't clear it goes through the route planner's raised poses
(ik.Solver.plan_move), each leg its own profile. The program starts at its first move's pose (playback's
approach phase gets the arm there). compile() also collision-checks the frames (player.check_frames) and
says which block each problem belongs to.
"""
import math

import ik
import player

FRAME_DT = 0.1
MIN_MOVE_S = 0.3
MAX_S = 3600.0          # longest program, seconds (library.MAX_FRAMES at 10 a second)
PEAK = 1.875            # a minimum-jerk profile's peak speed over its average


def _minjerk(u):
    return u * u * u * (10 - 15 * u + 6 * u * u)


class _Out:
    def __init__(self):
        self.frames, self.events, self.marks, self.problems, self.solved = [], [], [], [], {}
        self.t, self.pose = 0.0, None

    def problem(self, block, message):
        if not any(p["block"] == block["id"] for p in self.problems):
            self.problems.append({"block": block["id"], "message": message})

    def hold(self, seconds):
        self.t += seconds
        if self.pose is not None:
            self.frames.append([round(self.t, 3)] + [round(v, 2) for v in self.pose])

    def move(self, q, speed):
        """A minimum-jerk move from the current pose to q (degrees) at ``speed`` deg/s peak."""
        if self.pose is None:   # the first move: the program starts there
            self.pose = list(q)
            self.frames.append([round(self.t, 3)] + [round(v, 2) for v in q])
            return
        far = max(abs(a - b) for a, b in zip(q, self.pose))
        if far < 1e-6:
            return
        dur = max(MIN_MOVE_S, PEAK * far / speed)
        n = max(1, math.ceil(dur / FRAME_DT))
        start = self.pose
        for k in range(1, n + 1):
            s = _minjerk(k / n)
            self.frames.append([round(self.t + dur * k / n, 3)] + [round(a + (b - a) * s, 2) for a, b in zip(start, q)])
        self.t += dur
        self.pose = list(q)


def compile_program(blocks, tool_m=0.0, tool_r=None, area=None, limits=None):
    """{"frames", "events", "marks": [[t, block id]], "solved": {block id: [deg x6]}, "duration", "problems":
    [{"block", "message"}]}. Frames are empty if nothing moves."""
    tool_r = ik.model.TOOL_R_DEFAULT if tool_r is None else tool_r
    solver = ik.Solver(tool_m, tool_r, area, limits)
    out = _Out()

    def route_to(q, block):
        if out.pose is not None:
            frm, to = [v * ik.DEG for v in out.pose], [v * ik.DEG for v in q]
            vias = solver.plan_move(frm, to)
            if vias is None:
                out.problem(block, f"no clear way there ({solver.path(frm, to)})")
            else:
                for v in vias:
                    out.move([a / ik.DEG for a in v], block["speed"])
        out.move(q, block["speed"])

    def run(bs):
        for b in bs:
            if out.t > MAX_S:
                out.problem(b, f"the program runs past {MAX_S / 60:.0f} minutes")
                return
            out.marks.append([round(out.t, 3), b["id"]])
            kind = b["type"]
            if kind in ("pose", "home"):
                q = [0.0] * 6 if kind == "home" else list(b["angles"])
                why = solver.hit([v * ik.DEG for v in q])
                if why:
                    out.problem(b, f"that pose is blocked: {why}")
                out.solved[b["id"]] = q
                route_to(q, b)
            elif kind == "point":
                q = solve_point(solver, b, out.pose)
                out.solved[b["id"]] = q["angles"]
                if q["problem"]:
                    out.problem(b, q["problem"])
                route_to(q["angles"], b)
            elif kind == "wait":
                out.hold(b["seconds"])
            elif kind == "led":
                out.events.append([round(out.t, 3), "color", list(b["color"])])
            elif kind == "repeat":
                for _ in range(b["times"]):
                    run(b["blocks"])
                    if out.problems and out.t > MAX_S:
                        return

    run(blocks)
    if len(out.frames) == 1:          # a single pose: hold it a moment so it's a playable recording
        out.hold(0.5)
    if out.frames:
        bad = player.check_frames(out.frames, tool_m, tool_r, area)
        if bad:
            t, why = bad
            at = out.marks[0]
            for m in out.marks:           # the last block that had started by then
                if m[0] <= t + 1e-6:
                    at = m
            block = next(b for b in _walk(blocks) if b["id"] == at[1])
            out.problem(block, f"at {t:.1f} s: {why}")
    return {"frames": out.frames, "events": out.events, "marks": out.marks, "solved": out.solved,
            "duration": round(out.t, 2), "problems": out.problems}


def _walk(blocks):
    for b in blocks:
        yield b
        yield from _walk(b.get("blocks", []))


def solve_point(solver, block, start):
    """The joint pose for a point block (degrees), solved from ``start`` (degrees, or the zero pose)."""
    x, y, z = (v / 1000 for v in block["xyz"])
    q = [v * ik.DEG for v in (start or [0.0] * 6)]
    frm = q[:]
    res = None
    for k in range(40):   # a few solves, as the page does frame by frame: time for restarts too
        q, res = solver.step(q, frm, k * 0.3, xyz=(x, y, z), down=block["down"])
        if res["settled"] and res["reached"]:
            break
    problem = None
    if res["outside"]:
        problem = f"that point {res['outside']}"
    elif not res["reached"]:
        problem = (f"out of reach ({res['pos_err_mm']:.0f} mm short)" if res["pos_err_mm"] >= 3 else
                   "reachable, but not with the flange facing down")
    elif res["blocked"] and res["next"] is None and solver.hit(q):
        problem = f"the only way to reach it is blocked: {res['blocked']}"
    return {"angles": [round(v / ik.DEG, 2) for v in q], "problem": problem}
