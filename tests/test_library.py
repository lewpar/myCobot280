"""Recordings, sequences and saved poses over REST (no motion)."""
import main
from conftest import H
from helpers import frames_line


def rec(client, name="Wave", **kw):
    body = {"name": name, "frames": kw.pop("frames", frames_line(0, 0, 20, 1)), **kw}
    r = client.post("/api/recordings", headers=H, json=body)
    assert r.status_code == 200, r.text
    return r.json()


def test_password_required_and_lockout(client):
    assert client.get("/api/recordings").status_code == 401          # a missing password counts as a failure
    for _ in range(main.FAIL_LIMIT - 2):
        assert client.get("/api/poses", headers={"X-Arm-Password": "nope"}).status_code == 401
    assert client.get("/api/poses", headers=H).status_code == 200    # still under the limit
    client.get("/api/poses", headers={"X-Arm-Password": "nope"})
    assert client.get("/api/poses", headers=H).status_code == 429    # locked out, even with the right one


def test_recording_roundtrip(client):
    fr = [[5 + f[0]] + f[1:] for f in frames_line(0, 0, 20, 2)]
    ev = [[5.5, "color", [255, 0, 0]], [6.0, "pixel", [1, 2, 0, 255, 0]], [5.2, "brightness", [40]]]
    r = rec(client, " Wave ", frames=fr, events=ev, return_zero=True)
    assert r["name"] == "Wave" and r["return_zero"] and r["frames"] == 21 and r["events"] == 3 and r["duration"] == 2
    full = client.get(f"/api/recordings/{r['id']}", headers=H).json()
    assert full["frames"][0][0] == 0 and full["frames"][-1][0] == 2
    assert [e[0] for e in full["events"]] == [0.2, 0.5, 1.0]     # rebased with the frames, sorted
    assert [x["id"] for x in client.get("/api/recordings", headers=H).json()] == [r["id"]]


def test_recording_validation(client):
    good = frames_line(0, 0, 20, 1)
    bad = [
        {"name": "", "frames": good}, {"name": "   ", "frames": good}, {"name": "x" * 61, "frames": good},
        {"name": "a", "frames": good[:1]}, {"name": "a", "frames": [[0, 1, 2], [1, 1, 2]]},
        {"name": "a", "frames": [[1] + [0] * 6, [0] + [0] * 6]},                   # time goes backwards
        {"name": "a", "frames": [[0] + [0] * 6, [1, 400, 0, 0, 0, 0, 0]]},          # past a limit
        {"name": "a", "frames": good, "events": [[0, "explode", [1]]]},
        {"name": "a", "frames": good, "events": [[0, "pixel", [5, 0, 1, 1, 1]]]},
        {"name": "a", "frames": good, "events": [[0, "brightness", [0]]]},
        {"name": "a", "frames": good, "events": [[0, "color", [1, 2]]]},
    ]
    for b in bad:
        assert client.post("/api/recordings", headers=H, json=b).status_code == 422, b
    for path in ("/api/recordings/..%2F..%2Fetc", "/api/recordings/zzzzzzzzzzzz", "/api/recordings/0123456789ab"):
        assert client.get(path, headers=H).status_code == 404


def test_recording_edit_and_trim(client):
    r = rec(client, frames=frames_line(0, 0, 30, 3), events=[[0.5, "color", [1, 2, 3]], [2.5, "color", [4, 5, 6]]])
    rid = r["id"]
    e = client.patch(f"/api/recordings/{rid}", headers=H, json={"name": "Renamed", "return_zero": True}).json()
    assert e["name"] == "Renamed" and e["return_zero"]
    e = client.patch(f"/api/recordings/{rid}", headers=H, json={"trim": [1.0, 2.0]}).json()
    assert e["duration"] == 1.0 and e["frames"] == 11 and e["events"] == 0
    full = client.get(f"/api/recordings/{rid}", headers=H).json()
    assert full["frames"][0] == [0.0, 10.0, 20, 20, 20, 0, 0] and full["frames"][-1][1] == 20.0
    for bad in ({"trim": [2, 1]}, {"trim": [0.01, 0.02]}, {"trim": [-1, 1]}, {"name": ""}):
        assert client.patch(f"/api/recordings/{rid}", headers=H, json=bad).status_code == 422, bad
    assert client.patch("/api/recordings/0123456789ab", headers=H, json={"name": "x"}).status_code == 404


def test_sequences(client):
    a, b = rec(client, "A"), rec(client, "B")
    steps = [{"recording": a["id"], "pause": 1.5}, {"recording": b["id"]}]
    s = client.post("/api/sequences", headers=H, json={"name": "Show", "steps": steps}).json()
    assert s["steps"] == [{"recording": a["id"], "pause": 1.5}, {"recording": b["id"], "pause": 0.0}]
    u = client.put(f"/api/sequences/{s['id']}", headers=H, json={"name": "Show 2", "steps": steps[::-1]}).json()
    assert u["name"] == "Show 2" and u["steps"][0]["recording"] == b["id"]
    for bad in ({"name": "x", "steps": []}, {"name": "x", "steps": [{"recording": "0123456789ab"}]},
                {"name": "x", "steps": [{"recording": a["id"], "pause": -1}]}, {"name": "x", "steps": ["nope"]}):
        assert client.post("/api/sequences", headers=H, json=bad).status_code == 422, bad
    # a recording used by a sequence can't be deleted until the sequence stops using it
    r = client.delete(f"/api/recordings/{a['id']}", headers=H)
    assert r.status_code == 409 and "Show 2" in r.json()["detail"]
    assert client.delete(f"/api/sequences/{s['id']}", headers=H).status_code == 200
    assert client.delete(f"/api/recordings/{a['id']}", headers=H).status_code == 200
    assert client.get("/api/sequences", headers=H).json() == []


def test_poses(client):
    p = client.post("/api/poses", headers=H, json={"name": "Pick", "angles": [10, 20, 30, 40, 50, 60]}).json()
    assert p["angles"] == [10, 20, 30, 40, 50, 60]
    assert [x["name"] for x in client.get("/api/poses", headers=H).json()] == ["Pick"]
    assert client.post("/api/poses", headers=H, json={"name": "x", "angles": [0] * 5}).status_code == 422
    assert client.post("/api/poses", headers=H, json={"name": "x", "angles": [999] + [0] * 5}).status_code == 422
    assert client.delete(f"/api/poses/{p['id']}", headers=H).status_code == 200
    assert client.delete(f"/api/poses/{p['id']}", headers=H).status_code == 404


def test_library_without_arm(no_arm):
    """Recordings and poses are files: they work with no arm; playback needs one."""
    r = no_arm.post("/api/recordings", headers=H, json={"name": "A", "frames": frames_line(0, 0, 10, 1)})
    assert r.status_code == 200
    assert no_arm.post("/api/playback", headers=H, json={"recording": r.json()["id"]}).status_code == 503
