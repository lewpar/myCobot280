"""Shared fixtures: every test gets a fresh fake bus, a fresh backend and its own data directory.

Nothing here opens a real serial port: ``serial.Serial`` is replaced before the library is imported.
"""
import os
import sys
import time

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path[:0] = [ROOT, os.path.join(ROOT, "src", "backend"), os.path.dirname(__file__)]
os.environ["MYCOBOT_PASSWORD"] = PASSWORD = "test-pw"
os.environ["MYCOBOT_PORT"] = "/dev/fake"

import serial  # noqa: E402
from fakebus import FakeBus  # noqa: E402

_current = {"bus": None}
serial.Serial = lambda *a, **k: _current["bus"]

import arm_model  # noqa: E402
import library  # noqa: E402
import main  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

H = {"X-Arm-Password": PASSWORD}


@pytest.fixture
def data_dir(tmp_path, monkeypatch):
    """Calibration, home positions and the library all live in tmp_path."""
    monkeypatch.setattr(arm_model, "CALIB_FILE", str(tmp_path / "ik_calibration.json"))
    monkeypatch.setattr(arm_model, "CENTER_FILE", str(tmp_path / "center_positions.json"))
    for store in library.STORES:
        monkeypatch.setattr(store, "dir", str(tmp_path / store.name))
    return tmp_path


@pytest.fixture
def bus(data_dir):
    b = FakeBus()
    _current["bus"] = b
    yield b
    _current["bus"] = None


@pytest.fixture
def client(bus):
    main._failures.clear()
    with TestClient(main.app) as c:
        c.bus = bus
        yield c
        if main.link:
            main.link.shutdown()
    main._failures.clear()


@pytest.fixture
def no_arm(data_dir):
    """Backend with no arm on the serial port."""
    _current["bus"] = None
    main._failures.clear()
    orig = serial.Serial
    serial.Serial = lambda *a, **k: (_ for _ in ()).throw(serial.SerialException("no such port"))
    try:
        with TestClient(main.app) as c:
            yield c
    finally:
        serial.Serial = orig


def wait_for(cond, timeout=5.0, step=0.02):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        v = cond()
        if v:
            return v
        time.sleep(step)
    return cond()
