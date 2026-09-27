"""tools/recenter_servos.py on the fake bus: reads only without --write, re-centres and sets the zero, undoes."""
import argparse
import json
import os
import sys

import arm_model as model
from mycobot280 import MyCobot280

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
import recenter_servos as tool  # noqa: E402


def args(**kw):
    return argparse.Namespace(**{"write": False, "joints": None, "keep_zero": False, "undo": False, **kw})


def test_dry_run_writes_nothing(bus):
    arm = MyCobot280("/dev/fake")
    lines = []
    assert tool.run(args(), arm, ask=lambda _: "yes", out=lines.append) == 0
    assert "Nothing written" in lines[-1] and all(s.eeprom_writes == 0 for s in bus.servos.values())


def test_recenter_sets_zero_and_undo_puts_it_back(bus, data_dir):
    # J4 and J5 zeroed near their 0/4095 point; the arm is held in the zero pose
    c = json.loads((data_dir / "ik_calibration.json").read_text())
    (data_dir / "ik_calibration.json").write_text(json.dumps({**c, "zero": [2048, 2048, 2048, 573, 970, 2048]}))
    bus.servos[4].pos = bus.servos[4].goal = 573
    bus.servos[5].pos = bus.servos[5].goal = 970
    arm = MyCobot280("/dev/fake")
    lines = []
    assert tool.run(args(write=True, joints="4,5"), arm, ask=lambda _: "yes", out=lines.append) == 0
    assert any("its 0/4095 point is inside" in l for l in lines)
    cal = model.load_calibration()
    assert cal["zero"][3] == 2048 and cal["zero"][4] == 2048 and cal["calibrated"]
    assert bus.servos[4].corr == 573 - 2048 and bus.servos[1].eeprom_writes == 0
    assert all(bus.servos[s].regs[40] == 1 for s in (4, 5))              # torque back on, holding
    assert abs(bus.servos[4].goal - 2048) <= 1
    assert tool.run(args(undo=True), arm, ask=lambda _: "yes", out=lines.append) == 0
    assert bus.servos[4].corr == 0 and abs(bus.servos[4].pos - 573) <= 1
    assert model.load_calibration()["zero"][3] == 573


def test_says_no_unless_told_yes(bus):
    arm = MyCobot280("/dev/fake")
    assert tool.run(args(write=True), arm, ask=lambda _: "no", out=lambda *_: None) == 1
    assert all(s.eeprom_writes == 0 for s in bus.servos.values()) and all(s.regs[40] == 1 for s in bus.servos.values())
