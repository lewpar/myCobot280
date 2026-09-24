"""Attachments: the collision model for a tool on the flange, and choosing one over /ws/arm."""
import json
import math
import random

import arm_model as model
from conftest import H, PASSWORD, wait_for
from helpers import frames_line

VAC = (0.08, 0.0125)


def test_vacuum_pick_pose_is_clear_and_tip_may_touch():
    # flange straight down, arm reaching forward: the usual picking pose
    for q in ([0, -40, -60, 10, 0, 0], [30, -50, -50, 10, 0, 0]):
        k = model.fk(q, VAC[0])
        assert k["normal"][2] < -0.95, "test pose should point the flange down"
    q = [0, -40, -60, 10, 0, 0]
    assert model.check_pose(q, *VAC) is None


def _tool_points(q, tool_m):
    k = model.fk(q, tool_m)
    n = max(2, math.ceil(tool_m / model.TOOL_STEP))
    return k, [model._mid(k["flange"], k["tcp"], i / n) for i in range(1, n + 1)]


def test_no_allowed_pose_puts_the_attachment_inside_the_arm():
    """Whatever check catches it, a pose with the tube inside the upper arm or forearm is never allowed."""
    rnd = random.Random(7)
    inside = 0
    for _ in range(20000):
        q = [rnd.uniform(lo, hi) for lo, hi in model.URDF_LIMITS_DEG]
        k, pts = _tool_points(q, VAC[0])
        j = k["joints"]
        if any(model._seg_dist(p, j[1], j[2]) < model.UPPER_ARM_R + VAC[1] or
               model._seg_dist(p, j[2], j[3]) < model.FOREARM_R + VAC[1] for p in pts):
            inside += 1
            assert model.check_pose(q, *VAC) is not None, q
    assert inside > 100          # the sample did fold the tool into the arm


def test_attachment_only_collisions_exist():
    """Some poses are fine for the bare flange and refused only because of the tube."""
    rnd = random.Random(8)
    only = set()
    for _ in range(20000):
        q = [rnd.uniform(lo, hi) for lo, hi in model.URDF_LIMITS_DEG]
        if model.check_pose(q, 0) is None:
            why = model.check_pose(q, *VAC)
            if why:
                only.add(why)
    assert "the attachment would hit the upper arm" in only and any("table" in w for w in only), only


def test_tilted_tool_counts_its_radius_against_the_table():
    """Straight down, the tip may touch; tilted, the rim of the tube's end reaches lower than its centre."""
    rnd = random.Random(9)
    found = 0
    for _ in range(300000):
        q = [rnd.uniform(lo, hi) for lo, hi in model.URDF_LIMITS_DEG]
        k = model.fk(q, VAC[0])
        rim = VAC[1] * math.sqrt(max(0, 1 - k["normal"][2] ** 2))
        if model.TCP_MIN_Z <= k["tcp"][2] < model.TCP_MIN_Z + rim - 0.001 and model.check_pose(q, VAC[0], 1e-6) is None:
            assert "table" in (model.check_pose(q, *VAC) or ""), q
            found += 1
            if found >= 3:
                return
    raise AssertionError(f"only {found} tilted low poses found")


def test_choose_attachment_over_ws(client):
    with client.websocket_connect("/ws/arm") as w:
        w.send_json({"type": "auth", "password": PASSWORD})
        m = w.receive_json()
        assert m["attachment"] == "custom" and m["tool_mm"] == 0     # defaults
        w.send_json({"type": "set_tool", "attachment": "vacuum", "mm": 5, "d_mm": 5})
        m = w.receive_json()
        while m.get("attachment") != "vacuum":
            m = w.receive_json()
        assert (m["tool_mm"], m["tool_d_mm"]) == (80, 25)            # a known attachment keeps its own size
        w.send_json({"type": "set_tool", "attachment": "custom", "mm": 200, "d_mm": 10})   # too long: ignored
        w.send_json({"type": "set_tool", "attachment": "laser", "mm": 10, "d_mm": 10})      # unknown: ignored
        w.send_json({"type": "set_tool", "attachment": "custom", "mm": 40, "d_mm": 12})
        m = w.receive_json()
        while m.get("tool_mm") != 40:
            m = w.receive_json()
        assert m["attachment"] == "custom" and m["tool_d_mm"] == 12
    saved = json.load(open(model.CALIB_FILE))
    assert (saved["attachment"], saved["tool_mm"], saved["tool_d_mm"]) == ("custom", 40, 12)
    assert model.load_calibration()["tool_d_mm"] == 12


def test_playback_checked_with_the_attachment(client):
    # a recording that's fine for the bare flange but drives the vacuum tip into the table
    low = None
    for j2 in range(-10, -90, -2):
        q = [0, j2, -90 - j2 + 0, 0, 0, 0]
        q = [0, j2, -60, -(90 + j2 - 60) + 0, 0, 0]
        if model.check_pose(q, 0) is None and model.check_pose(q, *VAC):
            low = q
            break
    assert low, "no pose that only the attachment collides in"
    fr = frames_line(0, 0, 1, 1, base=[0, 0, 0, 0, 0, 0])[:3] + [[1.0] + low]
    rid = client.post("/api/recordings", headers=H, json={"name": "Low", "frames": fr}).json()["id"]
    main_link = __import__("main").link
    assert client.post("/api/playback", headers=H, json={"recording": rid}).status_code == 200
    wait_for(lambda: client.get("/api/playback", headers=H).json()["playback"] is None, 10)
    main_link.calib.update(attachment="vacuum", tool_mm=80.0, tool_d_mm=25.0)
    r = client.post("/api/playback", headers=H, json={"recording": rid})
    assert r.status_code == 409 and "attachment" in r.json()["detail"]
