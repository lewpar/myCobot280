"""REST motion endpoints against the fake bus: guards, clamps, stop/resume."""
import arm_model as model
from conftest import H
from helpers import colliding_pose


def test_health_and_servos(client):
    h = client.get("/api/health", headers=H).json()
    assert h["connected"] and h["servo_ids"] == [1, 2, 3, 4, 5, 6]
    s = client.get("/api/servos", headers=H).json()
    assert [x["id"] for x in s] == [1, 2, 3, 4, 5, 6]
    assert s[0]["limits_min"] == 150 and s[0]["limits_max"] == 3946       # EEPROM limits minus the buffer
    assert s[5]["limits_min"] == 50 and s[5]["limits_max"] == 4045        # J6 reports 0,0: full range


def test_no_arm(no_arm):
    assert no_arm.get("/api/health", headers=H).json()["connected"] is False
    assert no_arm.get("/api/servos", headers=H).status_code == 503


def test_move_and_clamp(client):
    r = client.post("/api/servo/1/move", headers=H, json={"position": 2300, "speed": 3000, "accel": 100}).json()
    assert r["success"] and abs(client.bus.servos[1].pos - 2300) <= 10
    # beyond the EEPROM limit: clamped to limit - 50
    client.post("/api/servo/1/move", headers=H, json={"position": 4000, "speed": 4000, "accel": 254})
    assert client.bus.servos[1].goal == 3946


def test_rejects_unlimited_speed_and_accel(client):
    for body in ({"position": 2100, "speed": 0}, {"position": 2100, "accel": 0},
                 {"position": 2100, "speed": 4001}, {"position": 5000}):
        assert client.post("/api/servo/1/move", headers=H, json=body).status_code == 422, body


def test_collision_refused(client):
    q = colliding_pose()
    c = model.load_calibration()
    # J2 and J3 into the table: moving J2 alone first may already collide; either way nothing moves
    client.bus.servos[3].pos = client.bus.servos[3].goal = model.deg_to_ticks(c, 2, q[2])
    r = client.post("/api/servo/2/move", headers=H, json={"position": model.deg_to_ticks(c, 1, q[1])})
    assert r.status_code == 409 and "Move refused" in r.json()["detail"]
    assert client.bus.servos[2].goal == 2048


def test_stop_blocks_moves_until_resume(client):
    assert client.post("/api/stop", headers=H).json()["stopped"]
    assert client.get("/api/safety", headers=H).json()["stopped"]
    assert client.post("/api/servo/1/move", headers=H, json={"position": 2100}).status_code == 423
    assert client.post("/api/servos/center_all", headers=H).status_code == 423
    client.post("/api/resume", headers=H)
    assert client.post("/api/servo/1/move", headers=H, json={"position": 2100, "speed": 3000}).status_code == 200


def test_home_positions(client):
    client.bus.servos[1].pos = 2200
    home = client.post("/api/servos/home", headers=H).json()["home"]
    assert home["1"] == 2200 if "1" in home else home[1] == 2200
    client.bus.servos[1].pos = client.bus.servos[1].goal = 2048
    r = client.post("/api/servos/center_all", headers=H).json()
    assert r["success"] and abs(client.bus.servos[1].pos - 2200) <= 10


def test_torque(client):
    client.post("/api/servos/torque_all", headers=H, json={"enabled": False})
    assert all(s.regs[40] == 0 for s in client.bus.servos.values())
    client.post("/api/servo/3/torque", headers=H, json={"enabled": True})
    assert client.bus.servos[3].regs[40] == 1


def test_atom(client):
    assert client.post("/api/atom/color", headers=H, json={"r": 1, "g": 2, "b": 3}).json()["acked"]
    assert client.post("/api/atom/pixel", headers=H, json={"x": 4, "y": 0, "r": 9}).json()["acked"]
    st = client.get("/api/atom/state", headers=H).json()
    assert st["pixels"][4] == [9, 0, 0] and st["pixels"][0] == [1, 2, 3]
    assert client.post("/api/atom/pixel", headers=H, json={"x": 5, "y": 0}).status_code == 422


def test_echoing_adapter(client):
    """Reads still work when the adapter echoes every request back."""
    client.bus.echo = True
    assert client.get("/api/servo/1", headers=H).json()["position"] == 2048
