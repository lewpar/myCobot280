"""The /ws/arm link: auth, state, goals, stop, stall guard."""
import time

import pytest
from starlette.websockets import WebSocketDisconnect

import arm_model as model
import main
from conftest import PASSWORD, wait_for
from helpers import joint_deg


def connect(client):
    ws = client.websocket_connect("/ws/arm")
    w = ws.__enter__()
    w.send_json({"type": "auth", "password": PASSWORD})
    return ws, w


def next_state(w, cond=lambda m: True, n=60):
    for _ in range(n):
        m = w.receive_json()
        if m.get("type") == "state" and cond(m):
            return m
    raise AssertionError("no matching state")


def test_ws_rejects_wrong_password(client):
    with client.websocket_connect("/ws/arm") as w:
        w.send_json({"type": "auth", "password": "wrong"})
        assert w.receive_json()["code"] == "auth"
        with pytest.raises(WebSocketDisconnect):
            w.receive_json()


def test_ws_state_and_goal(client):
    ws, w = connect(client)
    try:
        m = next_state(w, lambda m: all(a is not None for a in m["angles"]))
        assert m["angles"] == [0.0] * 6 and m["torque"] and not m["stopped"]
        assert {"fault", "stall_guard", "playback", "play_end", "limits"} <= set(m)
        time.sleep(0.6)
        w.send_json({"type": "goal", "angles": [15, 10, 10, 10, 0, 0], "speed": 120, "acc": 1000})
        assert wait_for(lambda: abs(joint_deg(client.bus, 0) - 15) < 1, 5)
        assert abs(joint_deg(client.bus, 1) - 10) < 1
    finally:
        ws.__exit__(None, None, None)


def test_ws_stop_and_resume(client):
    ws, w = connect(client)
    try:
        next_state(w, lambda m: all(a is not None for a in m["angles"]))
        w.send_json({"type": "stop"})
        assert next_state(w, lambda m: m["stopped"])["stopped"]
        w.send_json({"type": "goal", "angles": [30, 0, 0, 0, 0, 0], "speed": 120, "acc": 1000})
        time.sleep(0.5)
        assert abs(joint_deg(client.bus, 0)) < 0.5            # ignored while stopped
        w.send_json({"type": "resume"})
        next_state(w, lambda m: not m["stopped"])
        w.send_json({"type": "goal", "angles": [30, 0, 0, 0, 0, 0], "speed": 120, "acc": 1000})
        time.sleep(0.2)
        assert abs(joint_deg(client.bus, 0)) < 0.5            # still inside the 0.5 s resume handshake
    finally:
        ws.__exit__(None, None, None)


def test_torque_on_holds_current_pose(client):
    ws, w = connect(client)
    try:
        next_state(w, lambda m: all(a is not None for a in m["angles"]))
        w.send_json({"type": "torque", "on": False})
        next_state(w, lambda m: not m["torque"])
        client.bus.servos[1].pos = 2300.0                      # moved by hand while limp
        client.bus.servos[1].goal = 1800                       # stale goal left in the servo
        w.send_json({"type": "torque", "on": True})
        next_state(w, lambda m: m["torque"])
        client.bus.settle(0.3)
        assert abs(client.bus.servos[1].pos - 2300) <= 2       # didn't jump to the stale goal
    finally:
        ws.__exit__(None, None, None)


def test_stall_stops_the_arm(client):
    ws, w = connect(client)
    try:
        next_state(w, lambda m: all(a is not None for a in m["angles"]))
        time.sleep(0.6)
        client.bus.servos[1].stop_at = 2048 + int(5 * model.TICKS_PER_DEG)     # blocked 5° in
        w.send_json({"type": "goal", "angles": [40, 0, 0, 0, 0, 0], "speed": 120, "acc": 1000})
        m = next_state(w, lambda m: m["stopped"], n=80)
        assert m["fault"] and "J1 stalled" in m["fault"]
        w.send_json({"type": "resume"})
        assert next_state(w, lambda m: not m["stopped"])["fault"] is None
    finally:
        ws.__exit__(None, None, None)


def test_no_stall_on_normal_moves_or_with_guard_off(client):
    ws, w = connect(client)
    try:
        next_state(w, lambda m: all(a is not None for a in m["angles"]))
        time.sleep(0.6)
        # a slow but unobstructed move: far from the goal for a while, but always moving
        w.send_json({"type": "goal", "angles": [25, 0, 0, 0, 0, 0], "speed": 15, "acc": 40})
        time.sleep(2.5)
        assert not main.link.stopped and main.link.fault is None
        w.send_json({"type": "set_stall_guard", "on": False})
        client.bus.servos[2].stop_at = 2048 + int(3 * model.TICKS_PER_DEG)
        w.send_json({"type": "goal", "angles": [25, 30, 0, 0, 0, 0], "speed": 120, "acc": 1000})
        time.sleep(2)
        assert not main.link.stopped
    finally:
        ws.__exit__(None, None, None)


def test_rest_move_does_not_trip_stall_guard(client):
    ws, w = connect(client)
    try:
        next_state(w, lambda m: all(a is not None for a in m["angles"]))
        time.sleep(0.6)
        w.send_json({"type": "goal", "angles": [5, 0, 0, 0, 0, 0], "speed": 120, "acc": 1000})
        time.sleep(0.8)
        # a REST move takes J1 far from the IK loop's last goal and leaves it there
        from conftest import H
        client.post("/api/servo/1/move", headers=H, json={"position": 2600, "speed": 3000, "accel": 100})
        time.sleep(2)
        assert not main.link.stopped, main.link.fault
    finally:
        ws.__exit__(None, None, None)
