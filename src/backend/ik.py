"""Inverse kinematics for the arm: where to put the joints so the tool tip reaches a point, and the route
the servos take there. Pure logic (no bus); used by ik_link (the `target` message on /ws/arm, which drives the
arm) and by the solve-only socket /ws/ik (main.py), which the simulator page uses when it isn't driving the arm.

Two engines do the numbers, chosen with MYCOBOT_IK (ENGINES; "native" by default):
- "native": task-priority damped least squares on the geometric Jacobian (position first, "flange facing down"
  in the null space), a few small steps per solve (ITERS), each clamped to the joint limits. About 2 ms a
  solve on a desktop.
- "ikpy": IKPy (https://ikpy.readthedocs.io) on a chain built from arm_model.URDF_JOINTS (make_chain), the joint
  limits as its bounds and the attachment as a fixed last link; "facing down" is its orientation_mode "Z".
  IKPy weighs position and orientation together, so when a facing-down answer misses the point, the position
  alone is solved again and the closer answer wins (position first). Tens of ms a solve on a desktop.
Neither knows about collisions, so around them:
- iterate_clear: from a clear pose, never step into a collision: it stops at the last clear pose (native: the
  last clear step; ikpy: the last clear pose on the way to its answer);
- rescue: when stuck (short of the target, colliding, or with no clear route from where the servos are) it
  restarts from seeded poses, ranked collision-free first, then reachable, then closest; retried up to
  MAX_RETRIES times with fresh random seeds while it stays stuck on one target.

Route (plan_move): straight there in joint space if that path is clear, else through raised poses (J2-J5 at 0:
lift, turn the base, come down; also lifting the shoulder or straightening the elbow first). Every leg gets
the same collision and work-area check (arm_model.check_path).

Angles are radians and lengths metres inside this module; the socket messages (parse_target, Session,
result) use degrees and millimetres.
"""
import math
import os
import sys
import threading
import warnings

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import arm_model as model  # noqa: E402

N = 6
DEG = math.pi / 180
ITERS = 14                    # native: solver steps per solve
RESCUE_S, RETRY_S, MAX_RETRIES = 0.25, 1.0, 6
REACHED_M, REACHED_ORI = 0.003, 3 * DEG   # what counts as "on the target" (and "facing down")
THERE_M, THERE_ORI = 1.5e-4, 2e-3         # close enough that there's nothing left to solve
CLEAR_STEPS = 12                          # iterate_clear: samples on the way to IKPy's answer
IKPY_TOL = 1e-4               # IKPy's convergence tolerance: its default is several times slower for nothing we can see
XYZ_MM = 1000                 # |x|, |y|, |z| of a target, mm
URDF_LIM = [(lo * DEG, hi * DEG) for lo, hi in model.URDF_LIMITS_DEG]
DOWN = [0.0, 0.0, -1.0]
_warn_lock = threading.Lock()   # warnings.catch_warnings isn't thread-safe; solves run on several threads
ENGINES = ("native", "ikpy")
DEFAULT_ENGINE = os.environ.get("MYCOBOT_IK", "native").strip().lower() or "native"
if DEFAULT_ENGINE not in ENGINES:
    print(f"WARNING: MYCOBOT_IK={DEFAULT_ENGINE!r} isn't one of {', '.join(ENGINES)}; using native", flush=True)
    DEFAULT_ENGINE = "native"


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _dot(u, v):
    """Dot product of two 6-vectors, written out (sum() over a generator costs several times more, and the
    solver takes thousands a second)."""
    return u[0] * v[0] + u[1] * v[1] + u[2] * v[2] + u[3] * v[3] + u[4] * v[4] + u[5] * v[5]


def _pinv(J, lam2):
    """Damped pseudo-inverse of a 3x6 matrix (three rows): 6 rows of 3, J^T (J J^T + lam2 I)^-1."""
    r0, r1, r2 = J
    a00 = _dot(r0, r0) + lam2
    a11 = _dot(r1, r1) + lam2
    a22 = _dot(r2, r2) + lam2
    a01 = _dot(r0, r1)
    a02 = _dot(r0, r2)
    a12 = _dot(r1, r2)
    c00, c01, c02 = a11 * a22 - a12 * a12, a02 * a12 - a01 * a22, a01 * a12 - a02 * a11
    c11, c12, c22 = a00 * a22 - a02 * a02, a01 * a02 - a00 * a12, a00 * a11 - a01 * a01
    inv = 1.0 / (a00 * c00 + a01 * c01 + a02 * c02)   # positive: lam2 > 0
    x00, x01, x02, x11, x12, x22 = c00 * inv, c01 * inv, c02 * inv, c11 * inv, c12 * inv, c22 * inv
    return [(u * x00 + v * x01 + w * x02, u * x01 + v * x11 + w * x12, u * x02 + v * x12 + w * x22)
            for u, v, w in zip(r0, r1, r2)]


def make_chain(tool_m=0.0, limits=None):
    """The arm as an IKPy chain: a fixed base, the six URDF joints (bounded by ``limits``, radians) and the
    attachment as a fixed link along the flange normal, so the chain ends at the tool tip. (IKPy is only
    imported when this engine is used.)"""
    import numpy as np
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        from ikpy.chain import Chain
        from ikpy.link import OriginLink, URDFLink
    links = [OriginLink()]
    for k, ((xyz, rpy), (lo, hi)) in enumerate(zip(model.URDF_JOINTS, limits or URDF_LIM)):
        links.append(URDFLink(f"J{k + 1}", origin_translation=np.array(xyz, float), origin_orientation=np.array(rpy, float),
                              rotation=np.array([0.0, 0.0, 1.0]), bounds=(lo, hi), use_symbolic_matrix=False))
    links.append(URDFLink("tool tip", origin_translation=np.array([0.0, 0.0, tool_m]), origin_orientation=np.zeros(3),
                          joint_type="fixed", use_symbolic_matrix=False))
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        return Chain(links, active_links_mask=[False] + [True] * N + [False], name="mycobot280")


class Solver:
    """IK and route planning under one set of settings: the attachment (tool_m long, tool_r radius), the work
    area and the joint limits (radians; the servos' safe range once known). Keeps the rescue state between
    solves, so one Solver belongs to one stream of targets (a page, or the arm's target)."""

    def __init__(self, tool_m=0.0, tool_r=model.TOOL_R_DEFAULT, area=None, limits=None, engine=None):
        self.engine = engine or DEFAULT_ENGINE
        if self.engine not in ENGINES:
            raise ValueError(f"engine must be one of {', '.join(ENGINES)}")
        self.tool_m, self.tool_r, self.area = tool_m, tool_r, area
        self.lim = [tuple(l) for l in (limits or URDF_LIM)]
        self._ikpy_chain = None
        self._chain_key = None
        self.reset()

    def configure(self, tool_m, tool_r, area, limits=None):
        lim = [tuple(l) for l in (limits or URDF_LIM)]
        if (tool_m, tool_r, area, lim) != (self.tool_m, self.tool_r, self.area, self.lim):
            self.tool_m, self.tool_r, self.area, self.lim = tool_m, tool_r, area, lim
            self.reset()

    def reset(self):
        """Forget the rescue state and the cached route (the settings changed)."""
        self.mem = {"key": None, "t": -1e9, "tries": 0}
        self._plan_key = self._plan_out = None
        self.pending = False     # the last solve was stuck with a restart still to come

    # -- collision checks (arm_model works in degrees) ----------------------------------

    def hit(self, q):
        return model.check_pose([v / DEG for v in q], self.tool_m, self.tool_r, self.area)

    def path(self, a, b, steps=16, every=3 * DEG):
        return model.check_path([v / DEG for v in a], [v / DEG for v in b], self.tool_m, steps=steps,
                                tool_r=self.tool_r, area=self.area, every_deg=every / DEG)

    def tcp(self, q):
        _, _, fl, n = model.chain(q)
        t = self.tool_m
        return (fl[0] + n[0] * t, fl[1] + n[1] * t, fl[2] + n[2] * t), n

    # -- solving ----------------------------------------------------------------------------

    def errors(self, q, target, orient):
        """(position error m, angle from facing straight down rad, or 0 when that isn't asked for)."""
        (px, py, pz), n = self.tcp(q)
        return (math.sqrt((target[0] - px) ** 2 + (target[1] - py) ** 2 + (target[2] - pz) ** 2),
                math.acos(_clamp(-n[2], -1.0, 1.0)) if orient else 0.0)

    def _chain(self):
        key = (self.tool_m, tuple(self.lim))
        if self._chain_key != key:
            self._ikpy_chain, self._chain_key = make_chain(self.tool_m, self.lim), key
        return self._ikpy_chain

    def _ikpy(self, q, target, orient):
        lim = self.lim
        x0 = [0.0] + [_clamp(v, lo + 1e-6, hi - 1e-6) for v, (lo, hi) in zip(q, lim)] + [0.0]   # inside the bounds
        kw = {"target_orientation": DOWN, "orientation_mode": "Z"} if orient else {}
        with _warn_lock, warnings.catch_warnings():
            warnings.simplefilter("ignore")
            sol = self._chain().inverse_kinematics(list(target), initial_position=x0, tol=IKPY_TOL, **kw)
        return [_clamp(float(v), lo, hi) for v, (lo, hi) in zip(sol[1:1 + N], lim)]

    def solve(self, q, target, orient):
        """IKPy from q (radians): (its answer, errors). With orient, position still comes first: if the facing-down
        answer misses the point, the point alone is solved too and the closer of the two is kept."""
        q2 = self._ikpy(q, target, orient)
        r = self.errors(q2, target, orient)
        if orient and r[0] > 0.002:
            qp = self._ikpy(q2, target, False)
            rp = self.errors(qp, target, orient)
            if rp[0] < r[0] - 0.001:
                q2, r = qp, rp
        return q2, r

    def iterate(self, q, target, iters, orient):
        """Move q (radians, in place) toward putting the TCP on target; returns (position error m,
        orientation error rad)."""
        tx, ty, tz = target
        lim, tool = self.lim, self.tool_m
        for _ in range(iters):
            pos, axes, fl, (dx, dy, dz) = model.chain(q)
            px, py, pz = fl[0] + dx * tool, fl[1] + dy * tool, fl[2] + dz * tool
            ex, ey, ez = tx - px, ty - py, tz - pz
            pos_err = math.sqrt(ex * ex + ey * ey + ez * ez)
            if pos_err > 0.03:
                s = 0.03 / pos_err
                ex, ey, ez = ex * s, ey * s, ez * s
            ori_err = 0.0
            if orient:
                ori_err = math.acos(_clamp(-dz, -1.0, 1.0))
                ox, oy, oz = -dy, dx, 0.0                 # dir x down
                if ox * ox + oy * oy < 1e-8:
                    ox, oy, oz = 0.0, -dz, dy             # (1,0,0) x dir
                    if oy * oy + oz * oz < 1e-8:
                        ox, oy, oz = 0.0, 1.0, 0.0
                s = min(ori_err, 0.3) / math.sqrt(ox * ox + oy * oy + oz * oz)
                ox, oy, oz = ox * s, oy * s, oz * s
            if pos_err < 1.5e-4 and (not orient or ori_err < 2e-3):
                break
            # geometric Jacobian: position rows axis x (tcp - joint), orientation rows the axes
            jp = ([], [], [])
            for (jx, jy, jz), (ax, ay, az) in zip(pos, axes):
                cx, cy, cz = px - jx, py - jy, pz - jz
                jp[0].append(ay * cz - az * cy)
                jp[1].append(az * cx - ax * cz)
                jp[2].append(ax * cy - ay * cx)
            pp = _pinv(jp, 0.006 * 0.006)
            dq = [a * ex + b * ey + c * ez for a, b, c in pp]
            if orient:
                jo = ([a[0] for a in axes], [a[1] for a in axes], [a[2] for a in axes])
                # the orientation task in the position task's null space: Jo (I - Pp Jp) = Jo - (Jo Pp) Jp,
                # which is 3x3 in the middle instead of 6x6
                pc = ([p[0] for p in pp], [p[1] for p in pp], [p[2] for p in pp])   # columns of Pp
                jon = []
                for jr in jo:
                    m0, m1, m2 = _dot(jr, pc[0]), _dot(jr, pc[1]), _dot(jr, pc[2])
                    jon.append([j - m0 * a - m1 * b - m2 * c for j, a, b, c in zip(jr, jp[0], jp[1], jp[2])])
                e = (ox - _dot(jo[0], dq), oy - _dot(jo[1], dq), oz - _dot(jo[2], dq))
                po = _pinv(jon, 0.05 * 0.05)
                dq = [d + a * e[0] + b * e[1] + c * e[2] for d, (a, b, c) in zip(dq, po)]
            mx = max(abs(v) for v in dq)
            sc = 0.1 / mx if mx > 0.1 else 1.0
            for i in range(N):
                q[i] = _clamp(q[i] + dq[i] * sc, lim[i][0], lim[i][1])
        (px, py, pz), n = self.tcp(q)
        return (math.sqrt((tx - px) ** 2 + (ty - py) ** 2 + (tz - pz) ** 2),
                math.acos(_clamp(-n[2], -1.0, 1.0)) if orient else 0.0)

    def _iterate_clear_native(self, q, target, iters, orient):
        """Like iterate, but q never steps from a clear pose into a collision: it stops at its last clear step
        (the solver itself knows nothing about collisions, and following a target it would happily walk the
        elbow into the base). A q that already collides just iterates."""
        if self.hit(q):
            return self.iterate(q, target, iters, orient)
        r = self.iterate(q, target, 0, orient)
        for _ in range(iters):
            prev = q[:]
            r = self.iterate(q, target, 1, orient)
            if self.hit(q):
                q[:] = prev
                return self.iterate(q, target, 0, orient)
            if r[0] < 1.5e-4 and (not orient or r[1] < 2e-3):
                break
        return r

    def iterate_clear(self, q, target, orient, iters=ITERS):
        """One solve's worth of moving q (radians, in place) toward target, never from a clear pose into a
        collision; returns the errors. ``iters`` is the native engine's step count (IKPy solves in one go)."""
        if self.engine == "native":
            return self._iterate_clear_native(q, target, iters, orient)
        return self._iterate_clear_ikpy(q, target, orient)

    def _iterate_clear_ikpy(self, q, target, orient):
        """Move q (radians, in place) to IKPy's answer and return its errors, but never from a clear pose into a
        collision (IKPy knows nothing about them, and following a target it would happily put the elbow in the
        base): then it walks toward the answer and stops at the last clear pose. A q that already collides just
        takes the answer."""
        q[:] = [_clamp(v, lo, hi) for v, (lo, hi) in zip(q, self.lim)]   # (a seed may start outside the limits)
        r = self.errors(q, target, orient)
        if r[0] < THERE_M and (not orient or r[1] < THERE_ORI):
            return r                                   # already there: nothing to solve
        q2, r2 = self.solve(q, target, orient)
        if self.hit(q) or not self.hit(q2):
            q[:] = q2
            return r2
        best = None
        for k in range(1, CLEAR_STEPS + 1):
            p = [a + (b - a) * k / CLEAR_STEPS for a, b in zip(q, q2)]
            if self.hit(p):
                break
            best = p
        if best is None:
            return r
        q[:] = best
        return self.errors(q, target, orient)

    # -- route --------------------------------------------------------------------------------

    def plan_move(self, frm, to):
        """How the servos get from `frm` to `to` (radians): [] = straight there, else the poses to pass
        through first; None if no route is clear."""
        key = (tuple(frm), tuple(to))
        if key == self._plan_key:
            return self._plan_out
        self._plan_key, self._plan_out = key, None
        legs = {}

        def clear(a, b):
            k = (tuple(a), tuple(b))
            if k not in legs:
                legs[k] = not self.path(a, b)
            return legs[k]

        if clear(frm, to):
            self._plan_out = []
            return []

        def zeroed(q, js):
            return [0.0 if i in js else v for i, v in enumerate(q)]

        raised = lambda q: zeroed(q, (1, 2, 3, 4))
        shoulder_up = lambda q: zeroed(q, (1,))
        unbent = lambda q: zeroed(q, (2, 3, 4))
        out = [[raised(frm)], [shoulder_up(frm), raised(frm)], [unbent(frm), raised(frm)]]
        into = [[raised(to)], [raised(to), shoulder_up(to)], [raised(to), unbent(to)]]
        chains = [[raised(to)], [raised(frm)]] + [a + b for a in out for b in into]
        for vias in chains:
            pts = [frm] + vias + [to]
            if all(not self.hit(pts[k]) and clear(pts[k - 1], pts[k]) for k in range(1, len(pts))):
                self._plan_out = vias
                return vias
        return None

    def trouble(self, q, frm):
        """2: the pose collides; 1: clear, but the servos have no clear route to it from `frm`; 0: fine."""
        if self.hit(q):
            return 2
        return 1 if frm is not None and self.plan_move(frm, q) is None else 0

    def rescue(self, q_cur, target, orient, frm=None, tries=0):
        """Restart from seeded poses; the best {q, r, trouble} or None. `frm` (where the servos are) makes poses
        they can't get to lose. tries > 0 uses random seeds instead of the fixed ones (which would only find the
        same pose again), different for each try but repeatable."""
        yaw = math.atan2(target[1], target[0])
        seeds = []
        native = self.engine == "native"
        if not tries:   # (an IKPy seed costs tens of ms: fewer, well-spread ones for it)
            bends = [(0.5, 0.9, 0.9), (0.2, 1.3, 0.9), (0.9, 0.4, 1.2), (-0.3, 1.6, 1.0)] if native else \
                [(0.5, 0.9, 0.9), (-0.3, 1.6, 1.0)]
            yaws = (yaw, yaw + math.pi / 2, yaw - math.pi / 2, yaw + math.pi) if native else (yaw, yaw + math.pi)
            for y0 in yaws:
                y = math.atan2(math.sin(y0), math.cos(y0))
                for b in bends:
                    for sgn in (1, -1):
                        seeds.append([y, sgn * b[0], sgn * b[1], sgn * b[2], 0.0, 0.0])
        else:
            rs = tries * 7919 + 1

            def rand():
                nonlocal rs
                rs = (rs * 1103515245 + 12345) % 2147483648
                return rs / 2147483648

            for _ in range(24 if native else 6):
                seeds.append([yaw + (rand() - 0.5) * 1.6 if i == 0 else (lo + (hi - lo) * rand()) * 0.8
                              for i, (lo, hi) in enumerate(self.lim)])
        # and from the arm pointing straight up (facing the target, and at zero), stepped clear of collisions: what
        # solving from the zero pose finds, and a pose the route through raised poses always reaches
        starts = [] if tries else [([yaw, 0.0, 0.0, 0.0, 0.0, 0.0], True), ([0.0] * N, True)]
        found = []
        for q, upright in starts + [(s, False) for s in seeds]:
            if upright:
                r = self.iterate_clear(q, target, orient, iters=240)
            elif native:
                r = self.iterate(q, target, 80, orient)
            else:
                q[:], r = self.solve(q, target, orient)
            hit = bool(self.hit(q))
            move = sum(abs(a - b) for a, b in zip(q, q_cur))
            found.append({"q": q, "r": r, "trouble": 2 if hit else 0,
                          "score": r[0] * 1000 + r[1] * 20 + move * 0.5 + (1e6 if hit else 0)})
        found.sort(key=lambda c: c["score"])
        if frm is None or not found:
            return found[0] if found else None
        # the best clear pose the servos have a route to; failing that, the best clear one; failing that, the best
        for c in found[:8]:
            if c["trouble"]:
                break
            if self.plan_move(frm, c["q"]) is not None:
                return c
            c["trouble"] = 1
        return found[0]

    def solve_frame(self, q, target, orient, frm, now, rescue=True):
        """One solve: moves q (radians, in place) to IKPy's answer for target and returns its errors, stepping
        clear of collisions. When q is stuck it restarts from seeded poses: RESCUE_S after the target changes, then up
        to MAX_RETRIES more times RETRY_S apart with new random seeds while it stays stuck on the same target
        (an unreachable one stops costing time). A restart wins if it's in less trouble (clear beats colliding,
        reachable beats unreachable), or as clear and closer. `now` is in seconds."""
        r = self.iterate_clear(q, target, orient)
        t = self.trouble(q, frm)
        mem = self.mem
        self.pending = False
        if not (t > 0 or r[0] > 0.002 or (orient and r[1] > 2 * DEG)):
            mem["key"] = None
            return r
        key = (tuple(round(v, 4) for v in target), orient)
        again = key == mem["key"]
        due = mem["tries"] < MAX_RETRIES and now - mem["t"] >= RETRY_S if again else now - mem["t"] >= RESCUE_S
        if rescue and due:
            mem["tries"] = mem["tries"] + 1 if again else 0
            mem["key"], mem["t"] = key, now
            b = self.rescue(q, target, orient, frm, mem["tries"])
            if b and (b["trouble"] < t or (b["trouble"] == t and (b["r"][0] < r[0] - 0.001 or
                                                                 (b["r"][0] < 0.002 and b["r"][1] < r[1] - DEG)))):
                q[:] = b["q"]
                r = b["r"]
        # still stuck: another restart will come (this target's first, or one of its retries)
        self.pending = rescue and (mem["key"] != key or mem["tries"] < MAX_RETRIES)
        return r

    # -- one request ---------------------------------------------------------------------------

    def step(self, q, frm, now, xyz=None, down=False, angles=None, rescue=True, restart=False):
        """Solve for a target and plan the route to it; returns (solution q, result dict for the socket).

        q: where to start solving (radians), or None to start from `frm` (or the zero pose).
        frm: where the servos are (radians), or None to skip the route.
        xyz: the target for the TCP (metres), with `down` asking for the flange to face straight down; or
        angles: a joint pose (radians) to go to without solving (only the route is planned)."""
        if angles is not None:
            q = [_clamp(a, lo, hi) for a, (lo, hi) in zip(angles, self.lim)]
            r = (0.0, 0.0)
            target = None
            settled = True
        else:
            q = list(q if q is not None else frm if frm is not None else [0.0] * N)
            start = q[:]
            target = (xyz[0], xyz[1], max(model.TCP_MIN_Z, xyz[2]))
            if restart:
                b = self.rescue(q, target, down, frm)
                if b:
                    q[:] = b["q"]
            r = self.solve_frame(q, target, down, frm, now, rescue)
            # solving again from here would change nothing: q didn't move and no restart is due
            settled = not self.pending and max(abs(a - b) for a, b in zip(q, start)) < 1e-5
        blocked = self.hit(q)
        nxt, detour = None, False
        if frm is not None and not blocked:
            route = self.plan_move(frm, q)
            if route is None:
                blocked = self.path(frm, q)
            else:
                nxt, detour = (route[0] if route else q), bool(route)
        outside = None
        if target is not None and self.area and self.area.get("enabled"):
            outside = model._outside_area(target, self.area)
        res = {
            "target": None if target is None else [round(v * 1000, 2) for v in target],
            "down": bool(down) if target is not None else False,
            "angles": [round(v / DEG, 4) for v in q],
            "next": None if nxt is None else [round(v / DEG, 4) for v in nxt],
            "detour": detour,
            "reached": r[0] < REACHED_M and (not down or target is None or r[1] < REACHED_ORI),
            "pos_err_mm": round(r[0] * 1000, 2),
            "ori_err_deg": round(r[1] / DEG, 2),
            "blocked": blocked,
            "outside": outside,
            "settled": settled,
        }
        return q, res


# ---- message parsing, shared by /ws/arm `target` and /ws/ik `solve` ---------------------------------

def _num(v, lo, hi):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and lo <= v <= hi


def _pose(v):
    return isinstance(v, list) and len(v) == N and all(_num(a, -360, 360) for a in v)


def parse_target(msg):
    """The target in a message: ({"xyz": metres | None, "angles": radians | None, "down": bool}, None), or
    (None, what's wrong). Exactly one of xyz (mm) and angles (degrees) must be given."""
    xyz, angles, down = msg.get("xyz"), msg.get("angles"), msg.get("down", False)
    if (xyz is None) == (angles is None):
        return None, "Give either xyz (mm) or angles (degrees)."
    if not isinstance(down, bool):
        return None, "down must be true or false."
    if xyz is not None:
        if not (isinstance(xyz, list) and len(xyz) == 3 and all(_num(v, -XYZ_MM, XYZ_MM) for v in xyz)):
            return None, f"xyz must be 3 numbers in mm, each -{XYZ_MM}..{XYZ_MM}."
        return {"xyz": [v / 1000 for v in xyz], "angles": None, "down": down}, None
    if not _pose(angles):
        return None, "angles must be 6 joint angles in degrees."
    return {"xyz": None, "angles": [a * DEG for a in angles], "down": False}, None


def _error(code, ref, message):
    return {"type": "error", "code": code, "ref": ref, "message": message}


class Session:
    """One /ws/ik connection: its own Solver, settings and last solution. handle() takes a message and returns
    the reply (every message gets one). Starts with the settings saved for the arm (attachment, work area) and
    the arm's joint limits if it's connected."""

    def __init__(self, calib=None, limits_deg=None):
        c = calib or model.load_calibration()
        self.settings = {"tool_mm": float(c["tool_mm"]), "tool_d_mm": float(c["tool_d_mm"]),
                         "area": dict(c["area"]),
                         "limits": [list(l) if l[0] < l[1] else list(u)    # [0, 0]: the servo's range is unusable
                                    for l, u in zip(limits_deg or model.URDF_LIMITS_DEG, model.URDF_LIMITS_DEG)]}
        self.solver = Solver()
        self._apply()
        self.q = None
        self._last = None        # (request, reply) of the last solve, repeated as is while it's settled

    def _apply(self):
        s = self.settings
        self.solver.configure(s["tool_mm"] / 1000, s["tool_d_mm"] / 2000, s["area"],
                              [(lo * DEG, hi * DEG) for lo, hi in s["limits"]])

    def settings_msg(self):
        return {"type": "settings", **self.settings, "engine": self.solver.engine}

    def handle(self, msg, now):
        t = msg.get("type") if isinstance(msg, dict) else None
        if t == "settings":
            return self._on_settings(msg)
        if t == "solve":
            return self._on_solve(msg, now)
        return _error("bad_request", t if isinstance(t, str) else None,
                      f"Unknown message type {t!r}." if msg is not None else 'Messages must be JSON objects with a "type".')

    def _on_settings(self, msg):
        s = dict(self.settings)
        if "tool_mm" in msg or "tool_d_mm" in msg:
            mm, d = msg.get("tool_mm", s["tool_mm"]), msg.get("tool_d_mm", s["tool_d_mm"])
            if not (_num(mm, 0, 150) and _num(d, 1, 60)):
                return _error("bad_request", "settings", "tool_mm must be 0-150 and tool_d_mm 1-60.")
            s["tool_mm"], s["tool_d_mm"] = float(mm), float(d)
        if "area" in msg:
            a = msg["area"]
            a = model.clean_area(a) if isinstance(a, dict) and isinstance(a.get("enabled"), bool) else None
            if a is None:
                return _error("bad_request", "settings",
                              "area needs enabled (bool), center -180..180, span 30-360, radius_mm 0 or 100-450 "
                              "and base_mm 0 or 60-250 (less than radius_mm).")
            s["area"] = a
        if "limits" in msg:
            lim = msg["limits"]
            ok = isinstance(lim, list) and len(lim) == N and all(
                isinstance(l, list) and len(l) == 2 and _num(l[0], lo, hi) and _num(l[1], lo, hi) and l[0] < l[1]
                for l, (lo, hi) in zip(lim, model.URDF_LIMITS_DEG))
            if not ok:
                return _error("bad_request", "settings", "limits must be 6 [lo, hi] pairs in degrees, inside the URDF limits.")
            s["limits"] = [[float(l[0]), float(l[1])] for l in lim]
        self.settings = s
        self._apply()
        self._last = None
        return self.settings_msg()

    def _on_solve(self, msg, now):
        tgt, why = parse_target(msg)
        if why:
            return _error("bad_request", "solve", why)
        q, frm = msg.get("q"), msg.get("from")
        for name, v in (("q", q), ("from", frm)):
            if v is not None and not _pose(v):
                return _error("bad_request", "solve", f"{name} must be 6 joint angles in degrees.")
        for name in ("rescue", "restart"):
            if not isinstance(msg.get(name, False), bool):
                return _error("bad_request", "solve", f"{name} must be true or false.")
        rid = msg.get("id")
        if rid is not None and not (isinstance(rid, (str, int)) and not isinstance(rid, bool)):
            return _error("bad_request", "solve", "id must be a string or an integer.")
        # the same request again, and solving it again would change nothing: the same answer, without the work
        key = (repr(tgt), q if q is not None else self.q, frm, msg.get("rescue", True))
        if self._last and self._last[0] == key and self._last[1]["settled"] and not msg.get("restart", False):
            return {"type": "ik", "id": rid, **self._last[1]}
        start = [a * DEG for a in q] if q is not None else self.q
        frm = [a * DEG for a in frm] if frm is not None else None
        self.q, res = self.solver.step(start, frm, now, xyz=tgt["xyz"], down=tgt["down"], angles=tgt["angles"],
                                       rescue=msg.get("rescue", True), restart=msg.get("restart", False))
        self._last = (key, res)
        return {"type": "ik", "id": rid, **res}
