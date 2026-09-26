"""The work area: a slice of the circle around the base the tool tip must stay inside (default: the front half),
keeping clear of the base."""
import json
import os
import random
import time

import arm_model as model
from conftest import H, wait_for
from helpers import frames_line, joint_deg
from wsclient import ArmWS

FRONT = model.DEFAULT_AREA
VAC = (0.08, 0.0125)
REACH_FRONT = [0, -40, -60, 10, 0, 0]       # reaching forward (+X), flange down: tip at about -15°
REACH_SIDE = [60, -40, -60, 10, 0, 0]       # turned to the left, tip still in the front half
REACH_BACK = [-135, -40, -60, 10, 0, 0]     # turned back to the right: tip at about -150°


def use_default_area(data_dir):
    os.remove(data_dir / "ik_calibration.json")


def test_default_is_the_front_half():
    assert FRONT == {"enabled": True, "center": 0.0, "span": 180.0, "radius_mm": 0.0, "base_mm": 150.0}
    assert model.check_pose([0] * 6, 0, 0.01, FRONT) is None                 # zero pose: arm straight up
    assert model.check_pose([0] * 6, *VAC, FRONT) is None
    assert model.check_pose(REACH_FRONT, *VAC, FRONT) is None
    assert model.check_pose(REACH_SIDE, *VAC, FRONT) is None
    assert model.check_pose(REACH_BACK, *VAC, FRONT) == "the attachment's tip would leave the work area"
    assert model.check_pose(REACH_BACK, 0, 0.01, FRONT) == "the flange would leave the work area"
    assert model.check_pose(REACH_BACK, *VAC, {**FRONT, "enabled": False}) is None


def test_only_the_tip_counts():
    """The rest of the arm may cross the edge as long as the tip stays inside."""
    rnd = random.Random(3)
    crossing = 0
    for _ in range(20000):
        q = [rnd.uniform(lo, hi) for lo, hi in model.URDF_LIMITS_DEG]
        k = model.fk(q, VAC[0])
        tip_in = model._outside_area(k["tcp"], FRONT) is None
        body_out = any(model._outside_area(p, FRONT) for p in k["joints"][2:5])
        why = model.check_pose(q, *VAC, FRONT) or ""
        assert ("work area" in why or "close to the base" in why) == (not tip_in and not model.check_pose(q, *VAC)), (q, why)
        crossing += tip_in and body_out and why == ""
    assert crossing > 100          # poses with the elbow or wrist outside, tip inside: allowed


def test_other_areas():
    right = {**FRONT, "center": -90.0}
    assert model.check_pose([-90, -40, -60, 10, 0, 0], *VAC, right) is None
    assert "work area" in model.check_pose(REACH_SIDE, *VAC, right)
    full = {**FRONT, "span": 360.0}
    assert model.check_pose(REACH_BACK, *VAC, full) is None
    k = model.fk(REACH_FRONT, VAC[0])
    reach = (k["tcp"][0] ** 2 + k["tcp"][1] ** 2) ** 0.5
    short = {**full, "radius_mm": round(reach * 1000 - 5)}
    assert "past the work area" in model.check_pose(REACH_FRONT, *VAC, short)
    assert model.check_pose(REACH_FRONT, *VAC, {**short, "radius_mm": round(reach * 1000 + 5)}) is None


def test_keep_clear_of_the_base():
    """A low tip close to the base folds the wrist back onto the arm (the ATOM into the column): refused."""
    folded = [123.1, 73.3, 49.7, 147.0, 0.0, -25.9]       # tip 130 mm from the axis, 72 mm up
    full = {**FRONT, "span": 360.0}
    assert model.check_pose(folded, 0, 0.01, full) == "the flange would come too close to the base"
    assert model.check_pose(folded, 0, 0.01, {**full, "base_mm": 0.0}) is None      # 0 turns it off
    assert model.check_pose(folded, 0, 0.01, {**full, "base_mm": 120.0}) is None    # smaller than 130 mm
    assert model.check_pose([0] * 6, 0, 0.01, full) is None     # straight up: above the keep-out, on the axis
    assert model.check_pose(REACH_FRONT, *VAC, FRONT) is None   # normal work, further out


def test_clean_area():
    assert model.clean_area({"enabled": 1, "center": "10", "span": 90}) == {
        "enabled": True, "center": 10.0, "span": 90.0, "radius_mm": 0.0, "base_mm": 150.0}   # older saves get the default
    assert model.clean_area({"enabled": True, "center": 0, "span": 90, "base_mm": 0})["base_mm"] == 0
    for bad in (None, {}, {"enabled": True, "center": 200, "span": 90}, {"enabled": True, "center": 0, "span": 10},
                {"enabled": True, "center": 0, "span": 90, "radius_mm": 50}, {"enabled": True, "center": "x", "span": 90},
                {"enabled": True, "center": 0, "span": 90, "base_mm": 30}, {"enabled": True, "center": 0, "span": 90, "base_mm": 300},
                {"enabled": True, "center": 0, "span": 90, "radius_mm": 150, "base_mm": 200}):
        assert model.clean_area(bad) is None, bad


def test_backend_default_and_set_area(data_dir, bus):
    use_default_area(data_dir)
    from conftest import main, TestClient
    with TestClient(main.app) as client:
        with ArmWS(client) as a:
            assert a.wait_config(lambda c: True)["area"] == FRONT
            a.send(type="set_area", enabled=True, center=0, span=400)    # invalid: refused
            assert a.error()["ref"] == "set_area"
            a.send(type="set_area", enabled=True, center=45, span=120, radius_mm=300)
            m = a.wait_config(lambda c: c["area"]["center"] == 45)
            assert m["area"] == {"enabled": True, "center": 45.0, "span": 120.0, "radius_mm": 300.0, "base_mm": 150.0}
        assert json.load(open(model.CALIB_FILE))["area"]["center"] == 45
        main.link.shutdown()


def test_backend_refuses_moves_out_of_the_area(data_dir, bus):
    use_default_area(data_dir)
    from conftest import main, TestClient
    c = model.load_calibration()
    with TestClient(main.app) as client:
        for sid, a in zip(model.JOINT_IDS, REACH_FRONT):
            bus.servos[sid].pos = bus.servos[sid].goal = model.deg_to_ticks(c, sid - 1, a)
        # a REST move turning J1 so the tip ends up behind, on the right
        r = client.post("/api/servo/1/move", headers=H, json={"position": model.deg_to_ticks(c, 0, -135)})
        assert r.status_code == 409 and "work area" in r.json()["detail"]
        with ArmWS(client) as a:
            a.ready()
            a.goal(REACH_BACK)
            m = a.state(lambda m: m["blocked"])
            assert "work area" in m["blocked"]
            assert abs(joint_deg(bus, 0)) < 1
            a.goal(REACH_SIDE)
            assert wait_for(lambda: abs(joint_deg(bus, 0) - 60) < 1, 5)
        # playback that takes the tip out is refused up front
        fr = frames_line(0, 0, -150, 2, base=REACH_FRONT)
        rid = client.post("/api/recordings", headers=H, json={"name": "Sweep", "frames": fr}).json()["id"]
        r = client.post("/api/playback", headers=H, json={"recording": rid})
        assert r.status_code == 409 and "work area" in r.json()["detail"]
        main.link.shutdown()
