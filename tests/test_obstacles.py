"""Obstacles (the Workspace view): the shapes' geometry, and every motion path keeping clear of them."""
import math
import time

import pytest

import arm_model as model
import main
from conftest import H, wait_for
from helpers import joint_deg
from wsclient import ArmWS

BOX = {"id": "box1", "name": "Box 1", "shape": "box", "pos": [0, 0, 0], "size": [100, 60, 40], "rot": [0, 0, 0]}


def prep(o):
    return model._prep_obstacle(model.clean_obstacles([o])[0])


@pytest.mark.parametrize("o, inside, outside, d_out", [
    (BOX, (0.049, 0.029, 0.019), (0.07, 0, 0), 0.02),
    ({**BOX, "rot": [0, 0, 90]}, (0.029, 0.049, 0), (0.05, 0, 0), 0.02),           # turned: x and y swap
    ({**BOX, "shape": "cylinder", "size": [80, 1, 100]}, (0.039, 0, 0.049), (0, 0.05, 0), 0.01),
    ({**BOX, "shape": "cylinder", "size": [80, 1, 100], "rot": [90, 0, 0]}, (0, 0.049, 0), (0, 0, 0.05), 0.01),   # lying down
    ({**BOX, "shape": "sphere", "size": [100, 1, 1]}, (0.03, 0.03, 0.02), (0, 0, 0.08), 0.03),
])
def test_shapes(o, inside, outside, d_out):
    p = prep(o)
    assert model.obstacle_distance(inside, p) < 0
    assert abs(model.obstacle_distance(outside, p) - d_out) < 1e-9


def test_validation():
    ok = model.clean_obstacles([{**BOX, "color": "#AABBCC", "name": "  "}])
    assert ok[0]["color"] == "#aabbcc" and ok[0]["name"] == "Box"
    for bad in ([{**BOX, "shape": "cone"}], [{**BOX, "size": [1, 60, 40]}], [{**BOX, "pos": [0, 0, 2000]}],
                [BOX, BOX], [{**BOX, "id": "a b"}], [{**BOX, "color": "red"}], [BOX] * 51):
        assert model.clean_obstacles(bad) is None


def test_the_arm_keeps_clear_even_with_the_area_off():
    reach = [0, -40, -60, 10, 0, 0]                       # reaching forward and down
    tip = model.fk(reach)["tcp"]
    area = {**model.DEFAULT_AREA, "enabled": False}
    assert model.check_pose(reach, area=area) is None
    wall = {**BOX, "pos": [round(tip[0] * 1000), round(tip[1] * 1000), round(tip[2] * 1000)], "size": [40, 40, 40]}
    area["obstacles"] = model.clean_obstacles([wall])
    assert model.check_pose(reach, area=area) == "J6 would hit Box 1" or "would hit Box 1" in model.check_pose(reach, area=area)
    assert model.check_pose([0] * 6, area=area) is None   # straight up is well clear


def test_the_links_as_drawn_count():
    """The upper arm runs ~68 mm beside the J2-J3 line: a shape touching it (but 45 mm off the joint line) is
    caught, and it's still caught once the arm turns and bends (the body moves with its joints)."""
    area = {**model.DEFAULT_AREA, "enabled": False}
    post = {**BOX, "name": "Post", "pos": [0, -115, 195], "size": [40, 40, 40]}   # 4 mm off the tube, 95 mm off the joint line
    area["obstacles"] = model.clean_obstacles([post])
    assert model.check_pose([0] * 6, area=area) == "the upper arm would hit Post"
    area["obstacles"] = model.clean_obstacles([{**post, "pos": [0, -140, 195]}])      # 29 mm off the tube: clear
    assert model.check_pose([0] * 6, area=area) is None
    q = [90, 30, 0, 0, 0, 0]                            # the same spot on the arm, turned and leaning
    f = model.joint_frames([math.radians(v) for v in q])
    p = model._apply(f[1], model.BODY[0][3][4])         # a point on the upper arm's side
    area["obstacles"] = model.clean_obstacles([{**BOX, "name": "Post", "pos": [round(v * 1000) for v in p],
                                                "size": [10, 10, 10]}])
    assert model.check_pose(q, area=area) == "the upper arm would hit Post"


def test_rest_ws_and_playback_respect_them(client):
    r = client.put("/api/obstacles", headers=H, json={"obstacles": [{**BOX, "shape": "nope"}]})
    assert r.status_code == 422
    tip = model.fk([30, -40, -60, 10, 0, 0])["tcp"]
    block = {**BOX, "name": "Crate", "pos": [round(v * 1000) for v in tip], "size": [60, 60, 60]}
    r = client.put("/api/obstacles", headers=H, json={"obstacles": [block]})
    assert r.status_code == 200 and r.json()["obstacles"][0]["name"] == "Crate"
    assert client.get("/api/obstacles", headers=H).json()["obstacles"][0]["id"] == "box1"
    with ArmWS(client) as a:
        c = a.wait_config(lambda c: c["area"].get("obstacles"))
        assert c["area"]["obstacles"][0]["name"] == "Crate"
        a.ready()
        a.goal([30, -40, -60, 10, 0, 0])                  # into the crate: refused
        m = a.state(lambda m: m["blocked"])
        assert "Crate" in m["blocked"]
        time.sleep(0.3)
        assert abs(joint_deg(client.bus, 0)) < 1
        # the area's sliders don't drop the obstacles
        a.send(type="set_area", enabled=False, center=0, span=180)
        c = a.wait_config(lambda c: c["area"]["span"] == 180 and c["area"]["enabled"] is False)
        assert c["area"]["obstacles"][0]["name"] == "Crate"
    # a motion through it is refused up front
    blocks = [{"id": "a", "type": "home", "speed": 60}, {"id": "b", "type": "pose", "angles": [30, -40, -60, 10, 0, 0], "speed": 60}]
    out = client.post("/api/programs/compile", headers=H, json={"blocks": blocks}).json()
    assert out["problems"] and "Crate" in out["problems"][0]["message"]
    # saved with the calibration
    assert model.load_calibration()["area"]["obstacles"][0]["name"] == "Crate"


def test_saved_without_the_arm(no_arm):
    r = no_arm.put("/api/obstacles", headers=H, json={"obstacles": [BOX]})
    assert r.status_code == 200
    assert no_arm.get("/api/obstacles", headers=H).json()["obstacles"][0]["id"] == "box1"


def test_the_route_goes_over_one():
    """A post in the way of a sweep from right to left: the straight move hits it, the route lifts over it."""
    import ik
    right, left, front = [-40, -40, -60, 10, 0, 0], [40, -40, -60, 10, 0, 0], [0, -40, -60, 10, 0, 0]
    tip = model.fk(front)["tcp"]
    post = model.clean_obstacles([{"id": "p", "name": "the post", "shape": "cylinder", "pos": [round(tip[0] * 1000), round(tip[1] * 1000), 90],
                                   "size": [40, 40, 180], "rot": [0, 0, 0]}])    # right where the tip passes, a bit higher
    s = ik.Solver(area={**model.DEFAULT_AREA, "enabled": False, "obstacles": post})
    D = ik.DEG
    a, b = [v * D for v in right], [v * D for v in left]
    assert not s.hit(a) and not s.hit(b) and "the post" in s.path(a, b)
    route = s.plan_move(a, b)
    assert route
    pts = [a] + route + [b]
    assert not any(s.path(p, q) for p, q in zip(pts, pts[1:]))
