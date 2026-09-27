"""The backend's IK (ik.py) and the solve-only socket /ws/ik.

The solver runs as the arm's target loop does (ik_link: a solve every 50 ms from where the servos are, the
servos moving along straight joint-space lines toward the next pose on the route). Targets reachable from the
zero pose must be reached when dragged there too, and mostly from arbitrary poses; the servos must never
collide on the way (the work area's soft edges may be grazed between samples)."""
import math
import random

import pytest
from starlette.websockets import WebSocketDisconnect

import arm_model as model
import ik
from conftest import PASSWORD

DEG = ik.DEG
DT, VMAX = 0.05, 90 * DEG * 0.05      # 20 solves a second, servos at 90°/s


def area_depth(q, area):
    """How far (m) the tip is past the nearest work-area edge (for grazes)."""
    p = model.fk([v / DEG for v in q])["tcp"]
    r, d = math.hypot(p[0], p[1]), 0.0
    if r < area["base_mm"] / 1000 and p[2] < model.BASE_KEEPOUT_TOP:
        d = max(d, min(area["base_mm"] / 1000 - r, model.BASE_KEEPOUT_TOP - p[2]))
    off = abs((math.degrees(math.atan2(p[1], p[0])) - area["center"] + 180) % 360 - 180) - area["span"] / 2
    if off > 0 and r >= model.AREA_CORE:
        d = max(d, r * math.sin(math.radians(min(off, 90))))
    return d


def run(q0, target, how, orient, out, steps=140, engine="native"):
    """Drive simulated servos to target from q0 ('jump' there, or 'drag' the target from where the TCP is)."""
    s = ik.Solver(area=dict(model.DEFAULT_AREA), engine=engine)
    servo, q = list(q0), None
    start = model.fk([v / DEG for v in q0])["tcp"]
    for f in range(steps):
        a = min(1.0, (f + 1) / 20) if how == "drag" else 1.0
        tg = [p + (b - p) * a for p, b in zip(start, target)]
        q, res = s.step(q, servo, f * DT, xyz=tg, down=orient)
        nxt = res["next"]
        if nxt is not None:
            nxt = [v * DEG for v in nxt]
            far = max(abs(b - c) for b, c in zip(nxt, servo))
            for i in range(6):
                e = nxt[i] - servo[i]
                st = VMAX * (abs(e) / far if far > 0 else 1)
                servo[i] += e if abs(e) < st else math.copysign(st, e)
        hit = s.hit(servo)
        if hit:
            if "work area" not in hit and "close to the base" not in hit:
                return {"ok": False, "collided": hit}
            out["graze_mm"] = max(out["graze_mm"], area_depth(servo, s.area) * 1000)
        if a == 1.0 and res["reached"] and nxt is not None and not res["detour"] and max(
                abs(b - c) for b, c in zip(nxt, servo)) < 1e-9:
            break
    tcp, n = s.tcp(servo)
    ori = math.acos(max(-1.0, min(1.0, -n[2])))
    return {"ok": math.dist(tcp, target) < 0.003 and (not orient or ori < 3 * DEG), "collided": None}


# both engines (ik.ENGINES); IKPy on fewer targets, since each of its solves takes tens of ms
@pytest.mark.parametrize("engine,targets", [("native", 10), ("ikpy", 4)])
def test_solver_gets_there_without_collisions(engine, targets):
    rnd = random.Random(12345)

    def point():
        a, r, z = (rnd.random() - 0.5) * math.pi * 0.9, 0.08 + rnd.random() * 0.2, 0.02 + rnd.random() * 0.25
        return (math.cos(a) * r, math.sin(a) * r, z)

    def random_pose():
        while True:
            q = [(rnd.random() * 2 - 1) * min(hi, 2.2) for _, hi in ik.URDF_LIM]
            if not model.check_pose([v / DEG for v in q], area=dict(model.DEFAULT_AREA)):
                return q

    out = {"cases": {}, "collided": [], "graze_mm": 0.0}
    for _ in range(targets):
        target, zero, pose = point(), [0.0] * 6, random_pose()
        for orient in (True, False):
            if not run(zero, target, "jump", orient, out, engine=engine)["ok"]:
                continue      # only targets reachable from the zero pose
            for name, q0, how in (("zero/drag", zero, "drag"), ("pose/jump", pose, "jump"), ("pose/drag", pose, "drag")):
                r = run(q0, target, how, orient, out, engine=engine)
                c = out["cases"].setdefault(name, {"n": 0, "ok": 0})
                c["n"] += 1
                c["ok"] += r["ok"]
                if r["collided"]:
                    out["collided"].append((name, orient, r["collided"]))
    assert out["collided"] == [], out["collided"]
    assert out["graze_mm"] < 2, out["graze_mm"]
    c = out["cases"]
    assert c["zero/drag"]["n"] >= 0.8 * targets and c["zero/drag"]["ok"] == c["zero/drag"]["n"], c
    for k in ("pose/jump", "pose/drag"):   # the rest need a route out that the work area forbids
        assert c[k]["ok"] >= 0.8 * c[k]["n"], c


def test_route_goes_up_and_over():
    """A move whose straight joint-space path would put J6 into the table goes up and around instead."""
    s = ik.Solver(area=dict(model.DEFAULT_AREA))
    a = [v * DEG for v in (28, -84, -89, 82, 66, 113)]
    b = [v * DEG for v in (19, -82, 46, -83, -87, -11)]
    assert not s.hit(a) and not s.hit(b) and "hit" in s.path(a, b)
    route = s.plan_move(a, b)
    assert route
    pts = [a] + route + [b]
    assert not any(s.path(p, q) for p, q in zip(pts, pts[1:]))
    _, res = s.step(None, a, 0.0, angles=b)
    assert res["detour"] and res["next"] == [round(v / DEG, 4) for v in route[0]] and res["target"] is None


def test_step_reports_blocked_and_outside():
    s = ik.Solver(area=dict(model.DEFAULT_AREA))
    _, res = s.step(None, [0.0] * 6, 0.0, xyz=(-0.15, -0.1, 0.1))           # behind: outside the front half
    assert res["outside"] == "would leave the work area"
    _, res = s.step(None, [0.0] * 6, 0.0, angles=[0, 130 * DEG, 130 * DEG, 0, 0, 0])   # folded into the table
    assert res["blocked"] and res["next"] is None


@pytest.mark.parametrize("msg, part", [
    ({}, "either xyz"),
    ({"xyz": [1, 2, 3], "angles": [0] * 6}, "either xyz"),
    ({"xyz": [1, 2]}, "xyz"),
    ({"xyz": [1, 2, 5000]}, "xyz"),
    ({"xyz": [1, 2, float("nan")]}, "xyz"),
    ({"xyz": [1, 2, 3], "down": 1}, "down"),
    ({"angles": [0] * 5}, "angles"),
    ({"xyz": [100, 0, 100], "from": [0] * 5}, "from"),
    ({"xyz": [100, 0, 100], "q": ["x"] * 6}, "q"),
    ({"xyz": [100, 0, 100], "restart": "yes"}, "restart"),
    ({"xyz": [100, 0, 100], "id": [1]}, "id"),
])
def test_session_rejects_bad_solves(msg, part):
    r = ik.Session(dict(model.DEFAULT_CALIB)).handle({"type": "solve", **msg}, 0.0)
    assert (r["type"], r["code"], r["ref"]) == ("error", "bad_request", "solve") and part in r["message"], r


def test_session_settings():
    s = ik.Session(dict(model.DEFAULT_CALIB, tool_mm=80.0, tool_d_mm=25.0))
    assert s.settings_msg()["tool_mm"] == 80 and s.solver.tool_m == 0.08
    r = s.handle({"type": "settings", "tool_mm": 40, "tool_d_mm": 10, "area": {**model.DEFAULT_AREA, "enabled": False},
                  "limits": [[-90, 90]] + [list(l) for l in model.URDF_LIMITS_DEG[1:]]}, 0.0)
    assert r["type"] == "settings" and r["tool_mm"] == 40 and r["area"]["enabled"] is False
    assert s.solver.tool_r == 0.005 and s.solver.lim[0] == (-90 * DEG, 90 * DEG)
    for bad in ({"tool_mm": 400}, {"area": {"enabled": "yes"}}, {"limits": [[0, 0]] * 6}, {"limits": [[-200, 10]] + [[-1, 1]] * 5}):
        r = s.handle({"type": "settings", **bad}, 0.0)
        assert (r["type"], r["ref"]) == ("error", "settings"), bad
    assert s.handle({"type": "nope"}, 0.0)["code"] == "bad_request"


def test_engine_setting():
    assert ik.DEFAULT_ENGINE in ik.ENGINES and ik.Solver().engine == ik.DEFAULT_ENGINE
    with pytest.raises(ValueError):
        ik.Solver(engine="nope")
    assert ik.Session(dict(model.DEFAULT_CALIB)).settings_msg()["engine"] == ik.DEFAULT_ENGINE


@pytest.mark.parametrize("engine", ik.ENGINES)
def test_session_limits_hold_the_solution(engine, monkeypatch):
    """The solver never leaves the joint limits it's given (the servos' safe range)."""
    monkeypatch.setattr(ik, "DEFAULT_ENGINE", engine)
    s = ik.Session(dict(model.DEFAULT_CALIB, area={**model.DEFAULT_AREA, "enabled": False}))
    s.handle({"type": "settings", "limits": [[-20, 20]] + [list(l) for l in model.URDF_LIMITS_DEG[1:]]}, 0.0)
    for k in range(10):
        r = s.handle({"type": "solve", "xyz": [0, 200, 120], "from": [0] * 6}, k * 0.3)
    assert -20 <= r["angles"][0] <= 20 and not r["reached"]


def login(c, path="/ws/ik", password=PASSWORD):
    cm = c.websocket_connect(path)
    w = cm.__enter__()
    w.send_json({"type": "auth", "password": password})
    return cm, w


def test_ws_ik_needs_the_password(client):
    cm, w = login(client, password="wrong")
    assert w.receive_json()["code"] == "auth"
    with pytest.raises(WebSocketDisconnect) as e:
        w.receive_json()
    assert e.value.code == 4401
    cm.__exit__(None, None, None)


@pytest.mark.parametrize("which", ["client", "no_arm"])
def test_ws_ik_solves_with_or_without_the_arm(which, request):
    c = request.getfixturevalue(which)
    cm, w = login(c)
    try:
        assert w.receive_json() == {"type": "hello", "protocol": 4}
        st = w.receive_json()
        assert st["type"] == "settings" and st["area"]["enabled"] is False and len(st["limits"]) == 6
        w.send_json({"type": "solve", "id": "a", "xyz": [180, -40, 110], "down": True, "from": [0] * 6})
        r = w.receive_json()
        assert r["type"] == "ik" and r["id"] == "a" and r["reached"] and r["next"] == r["angles"] and not r["detour"]
        tcp = model.fk(r["angles"])["tcp"]
        assert math.dist(tcp, (0.18, -0.04, 0.11)) < 0.003
        # every message gets one reply, in order
        w.send_json({"type": "settings", "tool_mm": 80, "tool_d_mm": 25})
        w.send_json({"type": "solve", "xyz": [180, -40, 110], "down": True})
        w.send_bytes(b"\x00")
        assert w.receive_json()["tool_mm"] == 80
        r2 = w.receive_json()
        assert r2["type"] == "ik" and r2["next"] is None and r2["angles"] != r["angles"]   # the tip moved 80 mm
        assert w.receive_json()["code"] == "bad_request"
    finally:
        cm.__exit__(None, None, None)


def test_settled_says_when_solving_again_would_change_nothing():
    s = ik.Solver(area=dict(model.DEFAULT_AREA))
    q, res = s.step(None, [0.0] * 6, 0.0, xyz=(0.18, -0.04, 0.11), down=True)
    for k in range(1, 20):                                # a far target takes a few solves
        if res["settled"]:
            break
        q, res = s.step(q, [0.0] * 6, k * 0.05, xyz=(0.18, -0.04, 0.11), down=True)
    assert res["settled"] and res["reached"]
    _, again = s.step(q, [0.0] * 6, 1.0, xyz=(0.18, -0.04, 0.11), down=True)
    assert again["angles"] == res["angles"] and again["settled"]
    assert s.step(None, [0.0] * 6, 0.0, angles=[0.0] * 6)[1]["settled"]       # nothing to solve
    # out of reach: not settled while restarts are still to come, settled once they've run out
    far = ik.Solver()
    q, t, seen = None, 0.0, []
    for _ in range(80):
        q, res = far.step(q, [0.0] * 6, t, xyz=(0.6, 0.0, 0.1))
        seen.append(res["settled"])
        t += 0.3
    assert not res["reached"] and seen[-1] and not all(seen)


def test_session_repeats_a_settled_answer_without_solving(monkeypatch):
    s = ik.Session(dict(model.DEFAULT_CALIB, area={**model.DEFAULT_AREA, "enabled": False}))
    calls = {"n": 0}
    real = s.solver.step

    def counting(*a, **k):
        calls["n"] += 1
        return real(*a, **k)
    monkeypatch.setattr(s.solver, "step", counting)
    msg = {"type": "solve", "xyz": [180, -40, 110], "down": True, "from": [0] * 6}
    r = s.handle({**msg, "id": 1}, 0.0)
    while not r["settled"]:
        r = s.handle({**msg, "q": r["angles"]}, 0.0)
    n = calls["n"]
    again = s.handle({**msg, "q": r["angles"], "id": 9}, 0.0)
    assert calls["n"] == n and again["id"] == 9 and again["angles"] == r["angles"]
    s.handle({**msg, "q": r["angles"], "from": [1] + [0] * 5}, 0.0)     # the servos moved: solved again
    assert calls["n"] == n + 1
