"""A joint past its servo's 0/4095 point (or otherwise far outside what its servo can reach): the arm stops and
won't move it, whoever asks; re-centring the servos where they are (the `recenter` command)."""
import json
import time

import arm_model as model
import main
from conftest import H, wait_for
from helpers import frames_line, joint_deg
from wsclient import ArmWS

TPD = model.TICKS_PER_DEG


def calibrate(data_dir, **kw):
    p = data_dir / "ik_calibration.json"
    c = json.loads(p.read_text())
    c.update(kw, calibrated=True)
    p.write_text(json.dumps(c))


def test_power_on_past_the_wrap_point_never_moves_the_arm(data_dir, bus):
    """What happened on the real arm: J4's zero sits near its servo's 0/4095 point (tick 573, direction -1) and
    it was powered up bent past it (tick 3063, reading -219°). Connecting must not move it."""
    calibrate(data_dir, zero=[2048, 2048, 2048, 573, 2048, 2048], dir=[1, 1, 1, -1, 1, 1])
    bus.servos[4].pos = bus.servos[4].goal = 3063
    from conftest import TestClient
    with TestClient(main.app) as client:
        with ArmWS(client) as a:
            m = a.state(lambda m: m["stopped"])
            assert "J4 reads 141°" in m["out_of_range"] and m["fault"] == m["out_of_range"]
            assert abs(m["angles"][3] - 141.2) < 0.5 and m["ticks"][3] == 3063     # the short way round
            # what the page does on connect: hold the pose it adopted (clamped into range)
            a.send(type="resume")
            m = a.state(lambda m: not m["stopped"])
            a.target(angles=[0, 0, 0, 46, 0, 0])
            e = a.error()
            assert (e["code"], e["ref"]) == ("refused", "target") and "J4 reads" in e["message"]
            a.goal([0, 0, 0, 0, 0, 0])
            assert "J4 reads" in a.error()["message"]
            r = client.post("/api/servo/1/move", headers=H, json={"position": 2100})
            assert r.status_code == 409 and "J4 reads" in r.json()["detail"]
            rid = client.post("/api/recordings", headers=H, json={"name": "x", "frames": frames_line(0, 0, 5, 1)}).json()["id"]
            r = client.post("/api/playback", headers=H, json={"recording": rid})
            assert r.status_code == 409 and "J4 reads" in r.json()["detail"]
            time.sleep(0.4)
            assert bus.servos[4].pos == 3063 and bus.servos[1].pos == 2048    # nothing moved
            # turned back by hand (torque off), it can be driven again
            a.send(type="torque", on=False)
            a.state(lambda m: not m["torque"])
            bus.servos[4].pos = 573
            m = a.state(lambda m: m["out_of_range"] is None)
            a.goal([10, 0, 0, 0, 0, 0], epoch=m["epoch"])
            assert wait_for(lambda: abs(joint_deg(bus, 0) - 10) < 1, 5)
        main.link.shutdown()


def test_going_past_the_wrap_point_stops_at_once(client):
    with ArmWS(client) as a:
        a.ready()
        client.bus.servos[3].pos = client.bus.servos[3].goal = 300      # J3 near the 0/4095 point...
        a.state(lambda m: m["ticks"][2] == 300 or m["stopped"])
        client.bus.servos[3].pos = client.bus.servos[3].goal = 4000     # ...and past it
        m = a.state(lambda m: m["stopped"])
        assert "0/4095" in m["fault"] and "J3" in m["fault"]


def test_recenter(data_dir, bus):
    calibrate(data_dir, zero=[2048, 2048, 2048, 573, 2048, 2048], dir=[1, 1, 1, -1, 1, 1])
    (data_dir / "center_positions.json").write_text(json.dumps({"4": 600, "1": 2000}))
    bus.servos[4].pos = bus.servos[4].goal = 573 - int(10 * TPD)          # J4 at +10°
    from conftest import TestClient
    with TestClient(main.app) as client:
        with ArmWS(client) as a:
            m = a.ready()
            assert abs(m["angles"][3] - 10) < 0.2
            a.send(type="recenter", joints=[3])
            e = a.error()
            assert (e["code"], e["ref"]) == ("refused", "recenter") and "torque" in e["message"]
            a.send(type="recenter", joints=[9])
            assert a.error()["code"] == "bad_request"
            a.send(type="torque", on=False)
            a.state(lambda m: not m["torque"])
            before = m["epoch"]
            a.send(type="recenter", joints=[3])
            r = next(x for x in iter(a.recv, None) if x["type"] == "recentered")
            (res,) = r["results"]
            assert res["ok"] and res["joint"] == 3 and abs(res["after"] - 2048) <= 1
            assert bus.servos[4].corr == res["before"] - 2048 and bus.servos[4].regs[55] == 1   # locked again
            c = a.wait_config(lambda c: c["zero"][3] != 573)
            assert c["zero"][3] == (573 + res["after"] - res["before"]) % 4096
            m = a.state(lambda m: m["epoch"] > before and m["angles"][3] is not None)
            assert abs(m["angles"][3] - 10) < 0.2                         # the same pose, the same angle
            assert model.load_centers() == {4: (600 + res["after"] - res["before"]) % 4096, 1: 2000}
            log = json.loads((data_dir / "servo_centres_log.json").read_text())
            assert log[-1]["results"][0]["correction_before"] == 0
            # torque back on holds the pose: no jump, although the goal register still has the old number
            a.send(type="torque", on=True)
            a.state(lambda m: m["torque"])
            time.sleep(0.3)
            assert abs(bus.servos[4].pos - 2048) <= 2
        main.link.shutdown()
