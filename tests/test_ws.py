"""The /ws/arm link: auth, hello/config/state, goals and epochs, errors, stop, stall guard."""
import time

import pytest
from starlette.websockets import WebSocketDisconnect

import arm_model as model
import main
from conftest import H, wait_for
from helpers import joint_deg
from wsclient import ArmWS


def test_ws_rejects_wrong_password(client):
    with client.websocket_connect("/ws/arm") as w:
        w.send_json({"type": "auth", "password": "wrong"})
        assert w.receive_json()["code"] == "auth"
        with pytest.raises(WebSocketDisconnect) as e:
            w.receive_json()
        assert e.value.code == 4401


def test_ws_rejects_a_non_json_login(client):
    with client.websocket_connect("/ws/arm") as w:
        w.send_bytes(b"\x00\x01")
        assert w.receive_json()["code"] == "auth"


def test_ws_no_arm_close_code(no_arm):
    from conftest import PASSWORD
    with no_arm.websocket_connect("/ws/arm") as w:
        w.send_json({"type": "auth", "password": PASSWORD})
        assert w.receive_json()["code"] == "no_arm"
        with pytest.raises(WebSocketDisconnect) as e:
            w.receive_json()
        assert e.value.code == 4503


def test_ws_hello_config_then_state(client):
    with ArmWS(client) as a:
        assert a.hello == {"type": "hello", "protocol": 2}
        first = a.recv()
        assert first["type"] == "config"
        assert {"calibrated", "zero", "dir", "tool_mm", "tool_d_mm", "attachment", "area", "limits",
                "stall_guard"} <= set(first)
        m = a.ready()
        assert m["angles"] == [0.0] * 6 and m["torque"] and not m["stopped"]
        assert {"fault", "playback", "play_end", "epoch", "clients"} <= set(m) and "zero" not in m
        assert m["clients"] == 1
        # config is only re-sent when it changes
        for _ in range(5):
            assert a.recv()["type"] == "state"
        a.send(type="set_stall_guard", on=False)
        assert a.wait_config(lambda c: c["stall_guard"] is False)


def test_ws_state_and_goal(client):
    with ArmWS(client) as a:
        a.ready()
        a.goal([15, 10, 10, 10, 0, 0])
        assert wait_for(lambda: abs(joint_deg(client.bus, 0) - 15) < 1, 5)
        assert abs(joint_deg(client.bus, 1) - 10) < 1
        assert not a.errors


@pytest.mark.parametrize("change, part", [
    ({"angles": [1, 2, 3]}, "angles"),
    ({"angles": [0, 0, 0, 0, 0, "x"]}, "angles"),
    ({"speed": 0}, "speed"),
    ({"speed": True}, "speed"),
    ({"acc": 5000}, "acc"),
    ({"epoch": None}, "epoch"),
])
def test_bad_goals_are_answered(client, change, part):
    with ArmWS(client) as a:
        m = a.ready()
        goal = {"type": "goal", "angles": [5, 0, 0, 0, 0, 0], "speed": 60, "acc": 200, "epoch": m["epoch"]}
        a.send(**{**goal, **change})
        e = a.error()
        assert (e["code"], e["ref"]) == ("bad_request", "goal") and part in e["message"]


def test_bad_messages_keep_the_link_up(client):
    with ArmWS(client) as a:
        a.ready()
        a.w.send_text('{"type":"goal","angles":[5,0,0,0,0,0],"speed":60,"acc":Infinity,"epoch":0}')
        assert a.error()["code"] == "bad_request"
        a.w.send_bytes(b"\x00")
        assert a.error()["code"] == "bad_request"
        a.w.send_text("not json")
        assert a.error()["code"] == "bad_request"
        a.send(type="set_tool", attachment=["x"])
        assert a.error()["ref"] == "set_tool"
        a.send(type="set_dir", joint=True, dir=True)
        assert a.error()["ref"] == "set_dir"
        a.send(type="torque", on="yes")
        assert a.error()["ref"] == "torque"
        a.send(type="dance")
        e = a.error()
        assert (e["code"], e["ref"]) == ("bad_request", "dance")
        # the bus loop is still alive and still moves the arm
        a.goal([15, 0, 0, 0, 0, 0])
        assert wait_for(lambda: abs(joint_deg(client.bus, 0) - 15) < 1, 5)
        assert main.link._thread.is_alive() and model.load_calibration()["dir"] == [1] * 6


def test_ws_stop_and_resume(client):
    with ArmWS(client) as a:
        m = a.ready()
        old = m["epoch"]
        a.send(type="stop")
        assert a.state(lambda m: m["stopped"])["stopped"]
        a.goal([30, 0, 0, 0, 0, 0])
        e = a.error()
        assert (e["code"], e["ref"]) == ("refused", "goal") and "stopped" in e["message"]
        a.send(type="resume")
        m = a.state(lambda m: not m["stopped"])
        assert m["epoch"] > old
        a.goal([30, 0, 0, 0, 0, 0], epoch=old)                 # computed before the resume
        e = a.error()
        assert e["code"] == "refused" and "Stale" in e["message"]
        time.sleep(0.3)
        assert abs(joint_deg(client.bus, 0)) < 0.5
        a.goal([30, 0, 0, 0, 0, 0], epoch=m["epoch"])          # after re-reading the pose
        assert wait_for(lambda: abs(joint_deg(client.bus, 0) - 30) < 1, 5)


def test_calibration_change_bumps_epoch_and_config(client):
    with ArmWS(client) as a:
        m = a.ready()
        rev = a.config
        a.send(type="set_dir", joint=0, dir=-1)
        c = a.wait_config(lambda c: c["dir"][0] == -1)
        assert c["calibrated"] and c is not rev
        assert a.state()["epoch"] > m["epoch"]


def test_set_zero_waits_for_every_servo(client):
    with ArmWS(client) as a:
        a.ready()
        before = list(main.link.calib["zero"])
        client.bus.alive.discard(4)
        a.state(lambda m: m["angles"][3] is None)
        a.send(type="set_zero")
        e = a.error()
        assert (e["code"], e["ref"]) == ("refused", "set_zero") and "J4" in e["message"]
        client.bus.alive.add(4)
        a.ready()
        client.bus.servos[2].pos = client.bus.servos[2].goal = 2248
        a.state(lambda m: abs(m["angles"][1]) > 10)
        m = a.last
        a.send(type="set_zero")
        c = a.wait_config(lambda c: c["zero"] != before)
        assert abs(c["zero"][1] - 2248) <= 1 and a.state()["epoch"] > m["epoch"]


def test_a_bus_error_stops_the_arm_instead_of_killing_the_loop(client, monkeypatch):
    with ArmWS(client) as a:
        a.ready()
        real, calls = main.link.arm.read_positions, {"n": 0}

        def flaky(ids):
            calls["n"] += 1
            if calls["n"] == 3:
                raise OSError("bus gone")
            return real(ids)
        monkeypatch.setattr(main.link.arm, "read_positions", flaky)
        m = a.state(lambda m: m["stopped"])
        assert "bus gone" in m["fault"]
        a.send(type="resume")
        m = a.state(lambda m: not m["stopped"] and m["fault"] is None)
        a.goal([10, 0, 0, 0, 0, 0], epoch=m["epoch"])
        assert wait_for(lambda: abs(joint_deg(client.bus, 0) - 10) < 1, 5)


def test_two_pages_see_each_other(client):
    with ArmWS(client) as a:
        a.ready()
        with ArmWS(client) as b:
            b.ready()
            assert a.state(lambda m: m["clients"] == 2)
        assert a.state(lambda m: m["clients"] == 1)


def test_torque_on_holds_current_pose(client):
    with ArmWS(client) as a:
        a.ready()
        a.send(type="torque", on=False)
        a.state(lambda m: not m["torque"])
        client.bus.servos[1].pos = 2300.0                      # moved by hand while limp
        client.bus.servos[1].goal = 1800                       # stale goal left in the servo
        a.send(type="torque", on=True)
        a.state(lambda m: m["torque"])
        client.bus.settle(0.3)
        assert abs(client.bus.servos[1].pos - 2300) <= 2       # didn't jump to the stale goal


def test_stall_stops_the_arm(client):
    with ArmWS(client) as a:
        a.ready()
        client.bus.servos[1].stop_at = 2048 + int(5 * model.TICKS_PER_DEG)     # blocked 5° in
        a.goal([40, 0, 0, 0, 0, 0])
        m = a.state(lambda m: m["stopped"], n=80)
        assert m["fault"] and "J1 stalled" in m["fault"]
        a.send(type="resume")
        assert a.state(lambda m: not m["stopped"])["fault"] is None


def test_no_stall_on_normal_moves_or_with_guard_off(client):
    with ArmWS(client) as a:
        a.ready()
        # a slow but unobstructed move: far from the goal for a while, but always moving
        a.goal([25, 0, 0, 0, 0, 0], speed=15, acc=40)
        time.sleep(2.5)
        assert not main.link.stopped and main.link.fault is None
        a.send(type="set_stall_guard", on=False)
        client.bus.servos[2].stop_at = 2048 + int(3 * model.TICKS_PER_DEG)
        a.goal([25, 30, 0, 0, 0, 0])
        time.sleep(2)
        assert not main.link.stopped


def test_rest_move_does_not_trip_stall_guard(client):
    with ArmWS(client) as a:
        a.ready()
        a.goal([5, 0, 0, 0, 0, 0])
        time.sleep(0.8)
        # a REST move takes J1 far from the IK loop's last goal and leaves it there
        client.post("/api/servo/1/move", headers=H, json={"position": 2600, "speed": 3000, "accel": 100})
        time.sleep(2)
        assert not main.link.stopped, main.link.fault
