"""The simulator page (ik_sim.html), tested with node: its collision model and Player must match the
Python ones exactly, and the whole page must work in jsdom against a fake backend.

Skipped when node or tests/js/node_modules is missing (./run_tests.sh installs them)."""
import json
import os
import random
import shutil
import subprocess

import pytest

import arm_model as model
import player
from helpers import frames_line

JS = os.path.join(os.path.dirname(__file__), "js")
pytestmark = pytest.mark.skipif(not shutil.which("node") or not os.path.isdir(os.path.join(JS, "node_modules")),
                                reason="needs node and tests/js/node_modules (cd tests/js && npm install)")


def node(script, data=None, timeout=60):
    r = subprocess.run(["node", script], cwd=JS, input=json.dumps(data) if data is not None else None,
                       capture_output=True, text=True, timeout=timeout)
    assert r.returncode == 0, r.stdout[-3000:] + r.stderr[-3000:]
    return r.stdout


@pytest.mark.parametrize("tool_mm,tool_d_mm", [(0, 20), (80, 25), (150, 60)])
def test_collision_model_matches(tool_mm, tool_d_mm):
    rnd = random.Random(280 + tool_mm)
    poses = [[round(rnd.uniform(lo - 3, hi + 3), 2) for lo, hi in model.URDF_LIMITS_DEG] for _ in range(3000)]
    # plus poses near the table and the base, where the checks are close calls
    poses += [[rnd.uniform(-180, 180), rnd.uniform(40, 140), rnd.uniform(-150, 150), rnd.uniform(-150, 150),
               rnd.uniform(-150, 150), 0] for _ in range(2000)]
    js = json.loads(node("parity.js", {"poses": poses, "tool_mm": tool_mm, "tool_d_mm": tool_d_mm}))["blocked"]
    py = [model.check_pose(q, tool_mm / 1000, tool_d_mm / 2000) is not None for q in poses]
    bad = [q for q, a, b in zip(poses, js, py) if a != b]
    assert not bad, f"{len(bad)} mismatches, e.g. {bad[:3]}"
    assert 0.1 < sum(py) / len(py) < 0.9        # the sample actually exercises both outcomes


def test_attachments_match():
    """The page's attachment list matches arm_model.ATTACHMENTS (ids and sizes)."""
    js = json.loads(subprocess.run(
        ["node", "-e", "const {pureBlocks}=require('./page');process.stdout.write(JSON.stringify(pureBlocks().ATTACHMENTS))"],
        cwd=JS, capture_output=True, text=True, check=True).stdout)
    assert set(js) == set(model.ATTACHMENTS)
    for k, a in model.ATTACHMENTS.items():
        assert (js[k]["length"], js[k]["diameter"]) == (a["length_mm"], a["diameter_mm"]), k


def run_py(steps, opts, arm, dt):
    pb = player.Playback("x", steps, 0.0, **opts)
    arm, goals, t = list(arm), [], 0.0
    while t < 60:
        act = pb.tick(t, list(arm))
        if act.goal is not None:
            goals.append([t, pb.phase, act.goal, act.speeds])
            arm[:] = act.goal
        if act.done:
            break
        t = round(t + dt, 6)
    return goals


@pytest.mark.parametrize("opts", [dict(rate=1, loop=False, timed=True, speed=60),
                                  dict(rate=2, loop=False, timed=False, speed=45)])
def test_player_matches(opts):
    steps = [{"name": "A", "frames": frames_line(0, 10, 30, 1.5), "events": [], "return_zero": True, "pause": 0.4},
             {"name": "B", "frames": frames_line(1, 20, 40, 1), "events": [], "pause": 0}]
    arm, dt = [0, 20, 20, 20, 0, 0], 0.02
    py = run_py(steps, opts, arm, dt)
    js = json.loads(node("parity.js", {"poses": [], "player": {"steps": steps, "opts": opts, "arm": arm, "dt": dt}}))["goals"]
    assert len(py) == len(js), (len(py), len(js))
    for a, b in zip(py, js):
        assert abs(a[0] - b[0]) < 1e-6 and a[1] == b[1], (a[:2], b[:2])
        assert all(abs(x - y) < 1e-6 for x, y in zip(a[2] + a[3], b[2] + b[3])), (a, b)


def test_page_smoke():
    out = node("smoke.js", timeout=300)
    assert "FAIL" not in out, out
