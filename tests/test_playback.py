"""Playback on the (fake) arm through /api/playback: guards, cues, sequences, stop."""
import time

import arm_model as model
import main
from conftest import H, wait_for
from helpers import colliding_pose, frames_line, joint_deg


def save(client, name, frames, **kw):
    r = client.post("/api/recordings", headers=H, json={"name": name, "frames": frames, **kw})
    assert r.status_code == 200, r.text
    return r.json()["id"]


def play(client, **body):
    return client.post("/api/playback", headers=H, json={"speed": 120, "acc": 1000, **body})


def finished(client):
    st = client.get("/api/playback", headers=H).json()
    return st["playback"] is None and st["last"]["message"]


def test_plays_recording_with_leds_and_no_page(client):
    fr = frames_line(0, 0, 20, 1.5)
    rid = save(client, "Sweep", fr, events=[[0.2, "color", [0, 0, 200]], [1.0, "pixel", [2, 2, 255, 0, 0]]])
    r = play(client, recording=rid)
    assert r.status_code == 200 and r.json()["playback"]["phase"] == "approach"
    msg = wait_for(lambda: finished(client), 15)
    assert msg == "Playback finished."
    client.bus.settle(0.5)
    assert abs(joint_deg(client.bus, 0) - 20) < 1.5 and abs(joint_deg(client.bus, 1) - 20) < 1.5
    assert wait_for(lambda: client.bus.atom.pixels[12] == [255, 0, 0], 2)
    assert client.bus.atom.pixels[0] == [0, 0, 200]


def test_timed_playback_tracks_the_recording(client):
    fr = frames_line(0, 0, 45, 1.5)                      # 30°/s
    rid = save(client, "Fast", fr)
    play(client, recording=rid)
    lags = []
    while not finished(client):
        p = client.get("/api/playback", headers=H).json()["playback"]
        if p and p["phase"] == "run" and 0.2 < p["t"] < 1.4:
            lags.append(p["t"] * 30 - joint_deg(client.bus, 0))
        time.sleep(0.02)
    assert lags and max(abs(x) for x in lags) < 6, lags     # within ~0.2 s of the recording


def test_return_zero(client):
    rid = save(client, "Out", frames_line(0, 0, 25, 1), return_zero=True)
    play(client, recording=rid, rate=2)
    assert wait_for(lambda: finished(client), 15) == "Playback finished."
    client.bus.settle(0.5)
    assert all(abs(joint_deg(client.bus, j)) < 2.5 for j in range(6))


def test_sequence(client):
    a = save(client, "A", frames_line(0, 0, 15, 1))
    b = save(client, "B", frames_line(0, 15, -15, 1))
    s = client.post("/api/sequences", headers=H, json={"name": "AB", "steps": [
        {"recording": a, "pause": 0.3}, {"recording": b}]}).json()
    seen = set()
    play(client, sequence=s["id"], rate=2)
    end = time.monotonic() + 15
    while time.monotonic() < end and not finished(client):
        p = client.get("/api/playback", headers=H).json()["playback"]
        if p:
            seen.add((p["step"], p["phase"]))
        time.sleep(0.03)
    assert finished(client) == "Playback finished."
    assert (0, "pause") in seen and any(st == 1 for st, _ in seen)
    client.bus.settle(0.5)
    assert abs(joint_deg(client.bus, 0) + 15) < 2


def test_refusals(client):
    rid = save(client, "A", frames_line(0, 0, 10, 1))
    assert play(client).status_code == 422
    assert play(client, recording=rid, sequence=rid).status_code == 422
    assert play(client, recording="0123456789ab").status_code == 404
    assert play(client, recording=rid, rate=10).status_code == 422
    assert play(client, recording=rid, speed=0).status_code == 422
    bad = frames_line(0, 0, 10, 1)[:5] + [[0.6] + colliding_pose()]
    r = play(client, recording=save(client, "Bad", bad))
    assert r.status_code == 409 and "at 0.6 s" in r.json()["detail"]
    client.post("/api/stop", headers=H)
    assert play(client, recording=rid).status_code == 423


def test_stop_and_rest_moves_during_playback(client):
    rid = save(client, "Long", frames_line(0, 0, 40, 6))
    play(client, recording=rid, loop=True)
    assert wait_for(lambda: (client.get("/api/playback", headers=H).json()["playback"] or {}).get("phase") == "run", 5)
    assert client.post("/api/servo/2/move", headers=H, json={"position": 2100}).status_code == 409
    assert client.post("/api/servos/center_all", headers=H).status_code == 409
    time.sleep(0.5)
    client.post("/api/playback/stop", headers=H)
    assert finished(client) == "Playback stopped."
    before = joint_deg(client.bus, 0)
    client.bus.settle(0.5)
    assert abs(joint_deg(client.bus, 0) - before) < 1.5            # held where it was
    # Stop (the big red button) also ends playback
    play(client, recording=rid)
    time.sleep(0.3)
    client.post("/api/stop", headers=H)
    assert finished(client) == "Stopped."


def test_stall_ends_playback(client):
    rid = save(client, "Far", frames_line(0, 0, 60, 2))
    client.bus.servos[1].stop_at = 2048 + int(10 * model.TICKS_PER_DEG)
    play(client, recording=rid)
    assert wait_for(lambda: main.link.stopped, 10)
    assert "stalled" in finished(client) and "J1" in main.link.fault


def test_ws_goals_ignored_during_playback(client):
    from conftest import PASSWORD
    rid = save(client, "Hold", frames_line(0, 0, 5, 3))
    with client.websocket_connect("/ws/arm") as w:
        w.send_json({"type": "auth", "password": PASSWORD})
        w.receive_json()
        play(client, recording=rid)
        for _ in range(5):
            w.send_json({"type": "goal", "angles": [-60, 0, 0, 0, 0, 0], "speed": 150, "acc": 1000})
            time.sleep(0.1)
        m = w.receive_json()
        while m.get("playback") is None:
            m = w.receive_json()
        assert m["playback"]["name"] == "Hold"
        assert joint_deg(client.bus, 0) > -2
        client.post("/api/playback/stop", headers=H)
