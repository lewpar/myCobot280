"""Motion Studio programs: validation and storage, compiling blocks into frames (program.py), and playing a
program on the (fake) arm."""
import math

import pytest

import arm_model as model
import main
from conftest import H, wait_for
from helpers import colliding_pose, joint_deg

A = [10, 20, 20, 20, 0, 0]
B = [-20, 10, 30, 10, 20, 30]


def compile_(client, blocks):
    r = client.post("/api/programs/compile", headers=H, json={"blocks": blocks})
    assert r.status_code == 200, r.text
    return r.json()


def test_crud_and_validation(client):
    blocks = [{"id": "a", "type": "pose", "angles": A, "speed": 40},
              {"id": "r", "type": "repeat", "times": 2, "blocks": [{"id": "w", "type": "wait", "seconds": 0.5}]}]
    r = client.post("/api/programs", headers=H, json={"name": "Wave", "blocks": blocks})
    assert r.status_code == 200 and r.json()["blocks"] == 3
    pid = r.json()["id"]
    assert client.get(f"/api/programs/{pid}", headers=H).json()["blocks"][1]["blocks"][0]["seconds"] == 0.5
    r = client.put(f"/api/programs/{pid}", headers=H, json={"name": "Wave 2", "blocks": blocks[:1]})
    assert r.status_code == 200 and r.json()["name"] == "Wave 2" and r.json()["blocks"] == 1
    assert [p["name"] for p in client.get("/api/programs", headers=H).json()] == ["Wave 2"]
    for bad in ([{"id": "a", "type": "pose", "angles": [0] * 5}],
                [{"id": "a", "type": "pose", "angles": A, "speed": 0}],
                [{"id": "a", "type": "wait", "seconds": 5}, {"id": "a", "type": "wait", "seconds": 5}],
                [{"id": "a", "type": "fly"}],
                [{"id": "a", "type": "led", "color": [0, 300, 0]}],
                [{"id": "a", "type": "repeat", "times": 0, "blocks": []}],
                [{"id": "a b", "type": "home"}],
                [{"id": "a", "type": "home", "note": "x" * 81}],
                [{"id": "a", "type": "home", "note": 5}]):
        assert client.post("/api/programs", headers=H, json={"name": "x", "blocks": bad}).status_code == 422, bad
    assert client.delete(f"/api/programs/{pid}", headers=H).status_code == 200
    assert client.get(f"/api/programs/{pid}", headers=H).status_code == 404


def test_compile_blocks_into_frames(client):
    out = compile_(client, [
        {"id": "a", "type": "pose", "angles": A, "speed": 60},
        {"id": "w", "type": "wait", "seconds": 1},
        {"id": "c", "type": "led", "color": [0, 80, 255]},
        {"id": "r", "type": "repeat", "times": 2, "blocks": [
            {"id": "b", "type": "pose", "angles": B, "speed": 60},
            {"id": "a2", "type": "pose", "angles": A, "speed": 60}]},
        {"id": "h", "type": "home", "speed": 30}])
    f = out["frames"]
    assert not out["problems"] and f[0] == [0.0] + A and f[-1][1:] == [0.0] * 6
    assert all(b[0] > a[0] for a, b in zip(f, f[1:]))
    assert out["events"] == [[1.0, "color", [0, 80, 255]]]
    marks = dict((b, t) for t, b in reversed(out["marks"]))   # first time each block starts
    assert marks["a"] == 0 and marks["w"] == 0 and marks["c"] == 1.0 and marks["b"] == 1.0
    assert [m[1] for m in out["marks"]].count("b") == 2 and out["solved"]["h"] == [0.0] * 6
    # the joint going furthest peaks at the block's speed (a little over: the profile is sampled)
    peak = max(max(abs(y - x) for x, y in zip(p[1:], q[1:])) / (q[0] - p[0]) for p, q in zip(f, f[1:]))
    assert 50 < peak < 66
    assert abs(out["duration"] - f[-1][0]) < 1e-6


def test_point_blocks_are_solved(client):
    out = compile_(client, [{"id": "p", "type": "point", "xyz": [180, -40, 110], "down": True, "speed": 60},
                            {"id": "q", "type": "point", "xyz": [160, 60, 90], "down": True, "speed": 60}])
    assert not out["problems"], out["problems"]
    for bid, xyz in (("p", (0.18, -0.04, 0.11)), ("q", (0.16, 0.06, 0.09))):
        k = model.fk(out["solved"][bid])
        assert math.dist(k["tcp"], xyz) < 0.003 and k["normal"][2] < -0.99
    far = compile_(client, [{"id": "f", "type": "point", "xyz": [900, 0, 100], "down": False, "speed": 60}])
    assert far["problems"][0]["block"] == "f" and "reach" in far["problems"][0]["message"]


def test_problems_point_at_their_block(client):
    out = compile_(client, [{"id": "ok", "type": "pose", "angles": A, "speed": 60},
                            {"id": "bad", "type": "pose", "angles": colliding_pose(), "speed": 60}])
    assert [p["block"] for p in out["problems"]] == ["bad"]


def test_moves_go_around_like_the_arm_does(client):
    """A move whose straight path would hit the table goes up and over (the route planner), never colliding."""
    a, b = [28, -84, -89, 82, 66, 113], [19, -82, 46, -83, -87, -11]
    out = compile_(client, [{"id": "a", "type": "pose", "angles": a, "speed": 90},
                            {"id": "b", "type": "pose", "angles": b, "speed": 90}])
    assert not out["problems"], out["problems"]
    assert not any(model.check_pose(fr[1:]) for fr in out["frames"])
    assert any(abs(fr[3]) < 1 and abs(fr[4]) < 1 for fr in out["frames"])   # passes the raised pose


def test_play_a_program_on_the_arm(client):
    blocks = [{"id": "a", "type": "pose", "angles": [15, 10, 10, 10, 0, 0], "speed": 120},
              {"id": "c", "type": "led", "color": [255, 0, 0]},
              {"id": "b", "type": "pose", "angles": [-15, 10, 10, 10, 0, 0], "speed": 120}]
    pid = client.post("/api/programs", headers=H, json={"name": "Swing", "blocks": blocks}).json()["id"]
    r = client.post("/api/playback", headers=H, json={"program": pid, "speed": 120, "acc": 1000})
    assert r.status_code == 200 and r.json()["playback"]["name"] == "Swing"
    assert wait_for(lambda: client.get("/api/playback", headers=H).json()["playback"] is None, 15)
    assert abs(joint_deg(client.bus, 0) + 15) < 2 and client.bus.atom.rgb == [255, 0, 0]
    # one with a problem is refused before anything moves
    bad = client.post("/api/programs", headers=H, json={"name": "Bad", "blocks": [
        {"id": "x", "type": "pose", "angles": colliding_pose(), "speed": 60}]}).json()["id"]
    r = client.post("/api/playback", headers=H, json={"program": bad})
    assert r.status_code == 409 and "blocked" in r.json()["detail"]
    assert client.post("/api/playback", headers=H, json={"program": pid, "recording": "x"}).status_code == 422
    assert client.post("/api/playback", headers=H, json={"program": "000000000000"}).status_code == 404


def test_block_notes_are_kept(client):
    blocks = [{"id": "a", "type": "home", "speed": 30, "note": "  start straight up "}, {"id": "b", "type": "wait", "seconds": 1, "note": ""}]
    pid = client.post("/api/programs", headers=H, json={"name": "Notes", "blocks": blocks}).json()["id"]
    got = client.get(f"/api/programs/{pid}", headers=H).json()["blocks"]
    assert got[0]["note"] == "start straight up" and "note" not in got[1]
