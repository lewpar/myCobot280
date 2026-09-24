"""The work area: a slice of the circle around the base the arm must stay inside (default: the right half)."""
import json
import os
import time

import arm_model as model
from conftest import H, PASSWORD, wait_for
from helpers import frames_line, joint_deg

RIGHT = model.DEFAULT_AREA
REACH_RIGHT = [-90, -40, -60, 10, 0, 0]     # reaching out to the arm's right (-Y), flange down
REACH_FRONT = [0, -40, -60, 10, 0, 0]       # the same reach straight ahead (+X): on the edge of the right half
REACH_LEFT = [90, -40, -60, 10, 0, 0]


def use_default_area(data_dir):
    os.remove(data_dir / "ik_calibration.json")


def test_default_is_the_right_half():
    assert RIGHT == {"enabled": True, "center": -90.0, "span": 180.0, "radius_mm": 0.0}
    assert model.check_pose([0] * 6, 0, 0.01, RIGHT) is None                 # zero pose: arm straight up
    assert model.check_pose([0] * 6, 0.08, 0.0125, RIGHT) is None            # ...also with the vacuum tool
    assert model.check_pose(REACH_RIGHT, 0.08, 0.0125, RIGHT) is None
    assert "work area" in model.check_pose(REACH_LEFT, 0.08, 0.0125, RIGHT)
    assert "work area" in model.check_pose(REACH_FRONT, 0.08, 0.0125, RIGHT)
    assert model.check_pose(REACH_LEFT, 0.08, 0.0125, {**RIGHT, "enabled": False}) is None


def test_other_areas():
    left = {**RIGHT, "center": 90.0}
    assert model.check_pose(REACH_LEFT, 0, 0.01, left) is None and model.check_pose(REACH_RIGHT, 0, 0.01, left)
    full = {**RIGHT, "span": 360.0}
    assert model.check_pose(REACH_LEFT, 0, 0.01, full) is None
    k = model.fk(REACH_RIGHT, 0.08)
    reach = (k["tcp"][0] ** 2 + k["tcp"][1] ** 2) ** 0.5
    short = {**full, "radius_mm": round(reach * 1000 - 20)}
    assert "past the work area" in model.check_pose(REACH_RIGHT, 0.08, 0.0125, short)


def test_clean_area():
    assert model.clean_area({"enabled": 1, "center": "10", "span": 90}) == {
        "enabled": True, "center": 10.0, "span": 90.0, "radius_mm": 0.0}
    for bad in (None, {}, {"enabled": True, "center": 200, "span": 90}, {"enabled": True, "center": 0, "span": 10},
                {"enabled": True, "center": 0, "span": 90, "radius_mm": 50}, {"enabled": True, "center": "x", "span": 90}):
        assert model.clean_area(bad) is None, bad


def test_backend_default_and_set_area(data_dir, bus):
    use_default_area(data_dir)
    from conftest import main, TestClient
    with TestClient(main.app) as client:
        with client.websocket_connect("/ws/arm") as w:
            w.send_json({"type": "auth", "password": PASSWORD})
            m = w.receive_json()
            assert m["area"] == RIGHT
            w.send_json({"type": "set_area", "enabled": True, "center": 0, "span": 400})    # invalid: ignored
            w.send_json({"type": "set_area", "enabled": True, "center": 45, "span": 120, "radius_mm": 300})
            while m["area"]["center"] != 45:
                m = w.receive_json()
            assert m["area"] == {"enabled": True, "center": 45.0, "span": 120.0, "radius_mm": 300.0}
        assert json.load(open(model.CALIB_FILE))["area"]["center"] == 45
        main.link.shutdown()


def test_backend_refuses_moves_out_of_the_area(data_dir, bus):
    use_default_area(data_dir)
    from conftest import main, TestClient
    c = model.load_calibration()
    with TestClient(main.app) as client:
        # a REST move swinging J1 so the bent arm points left
        for sid, a in zip(model.JOINT_IDS, REACH_RIGHT):
            bus.servos[sid].pos = bus.servos[sid].goal = model.deg_to_ticks(c, sid - 1, a)
        r = client.post("/api/servo/1/move", headers=H, json={"position": model.deg_to_ticks(c, 0, 90)})
        assert r.status_code == 409 and "work area" in r.json()["detail"]
        # WebSocket goals out of the area are blocked and reported
        with client.websocket_connect("/ws/arm") as w:
            w.send_json({"type": "auth", "password": PASSWORD})
            w.receive_json()
            time.sleep(0.6)
            w.send_json({"type": "goal", "angles": REACH_LEFT, "speed": 120, "acc": 1000})
            m = w.receive_json()
            while not m.get("blocked"):
                m = w.receive_json()
            assert "work area" in m["blocked"]
            assert abs(joint_deg(bus, 0) + 90) < 1
            # inside the area is fine
            w.send_json({"type": "goal", "angles": [-60, -40, -60, 10, 0, 0], "speed": 120, "acc": 1000})
            assert wait_for(lambda: abs(joint_deg(bus, 0) + 60) < 1, 5)
        # playback that leaves the area is refused up front
        fr = frames_line(0, -90, 90, 2, base=REACH_RIGHT)
        rid = client.post("/api/recordings", headers=H, json={"name": "Sweep", "frames": fr}).json()["id"]
        r = client.post("/api/playback", headers=H, json={"recording": rid})
        assert r.status_code == 409 and "work area" in r.json()["detail"]
        main.link.shutdown()
