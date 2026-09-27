"""Simulated mode (./run.sh --sim, MYCOBOT_SIM=1): the backend drives the simulated arm (simbus.SimBus), never
the serial port, and keeps the simulated arm's calibration apart from the real one's."""
import json

import serial

import arm_model as model
import main
from conftest import H, wait_for
from wsclient import ArmWS


def test_simulated_mode(data_dir, monkeypatch, tmp_path):
    def no_port(*a, **k):
        raise AssertionError("simulated mode opened the serial port")
    monkeypatch.setattr(serial, "Serial", no_port)
    monkeypatch.setattr(main, "SIMULATED", True)
    monkeypatch.setattr(main, "SIM_DIR", str(tmp_path / "sim_data"))
    real_calib = (data_dir / "ik_calibration.json").read_text()
    main._failures.clear()
    from conftest import TestClient
    with TestClient(main.app) as client:
        h = client.get("/api/health", headers=H).json()
        assert h["connected"] and h["simulated"] and h["serial_port"] == "simulated" and h["servo_ids"] == [1, 2, 3, 4, 5, 6]
        with ArmWS(client) as a:
            c = a.wait_config(lambda c: True)
            assert c["simulated"] and c["calibrated"] and c["zero"] == [2048] * 6
            m = a.ready()
            a.target(xyz=[180, -40, 110], down=True)
            m = a.state(lambda m: m["ik"] and m["ik"]["arrived"], n=100)
            assert m["ik"]["reached"]                    # the simulated arm really got there
            a.send(type="set_dir", joint=0, dir=-1)
            a.wait_config(lambda c: c["dir"][0] == -1)
        main.link.shutdown()
    # its calibration went to sim_data/, and the real one wasn't touched
    assert json.load(open(tmp_path / "sim_data" / "ik_calibration.json"))["dir"][0] == -1
    assert (data_dir / "ik_calibration.json").read_text() == real_calib
