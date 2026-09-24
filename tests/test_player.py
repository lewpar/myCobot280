"""player.Playback timing logic with a simulated clock and an arm that is always where it's told."""
import player
from helpers import frames_line


def run(pb, until=60.0, dt=0.02, arm=None):
    """Drive the playback; the fake arm jumps to every goal. Returns (goals, events, end time)."""
    arm = arm or [0.0] * 6
    goals, events, t = [], [], 0.0
    while t < until:
        act = pb.tick(t, list(arm))
        if act.goal is not None:
            goals.append((t, pb.phase, list(act.goal), list(act.speeds)))
            arm[:] = act.goal
        events += [(t, e) for e in act.events]
        if act.done:
            return goals, events, t
        t += dt
    raise AssertionError("playback never finished")


def test_pose_at_interpolates():
    fr = frames_line(0, 0, 10, 1)
    assert abs(player.pose_at(fr, 0.55)[0][0] - 5.5) < 1e-9
    assert player.pose_at(fr, 99)[0][0] == 10
    q, hint = player.pose_at(fr, 0.3)
    assert player.pose_at(fr, 0.1, hint)[0][0] == 1.0          # a stale hint past t is ignored


def test_timed_playback_keeps_time_and_fires_cues():
    fr = frames_line(0, 0, 20, 2)
    ev = [[0.5, "color", [1, 2, 3]], [1.5, "pixel", [0, 0, 9, 9, 9]]]
    pb = player.Playback("A", [{"name": "A", "frames": fr, "events": ev}], 0.0, speed=60)
    goals, events, end = run(pb, arm=[0, 20, 20, 20, 0, 0])
    run_goals = [g for g in goals if g[1] == "run"]
    start = run_goals[0][0]
    assert 1.9 < end - start < 2.3                                    # ~2 s at rate 1
    assert [round(t - start, 1) for t, _ in events] == [0.5, 1.5]
    assert goals[-1][2][0] == 20
    # timed: speeds follow the recording (10°/s), not the 60°/s setting
    mid = [g for g in run_goals if 0.5 < g[0] - start < 1.5]
    assert all(8 < g[3][0] < 14 for g in mid), [g[3][0] for g in mid]
    assert all(g[3][1] == player.MIN_DPS for g in mid)                # joints that don't move


def test_rate_and_untimed():
    fr = frames_line(0, 0, 20, 2)
    pb = player.Playback("A", [{"name": "A", "frames": fr}], 0.0, rate=2, timed=False, speed=45)
    goals, _, end = run(pb, arm=[0, 20, 20, 20, 0, 0])
    run_goals = [g for g in goals if g[1] == "run"]
    assert 0.9 < end - run_goals[0][0] < 1.3
    assert all(g[3] == [45] * 6 for g in goals)


def test_approach_then_return_zero():
    fr = frames_line(0, 30, 40, 1)
    pb = player.Playback("A", [{"name": "A", "frames": fr, "return_zero": True}], 0.0)
    goals, _, _ = run(pb, arm=[0.0] * 6)
    phases = [g[1] for g in goals]
    assert phases[0] == "approach" and goals[0][2][0] == 30
    assert "zero" in phases and goals[-1][2] == [0.0] * 6


def test_sequence_with_pause_and_loop():
    a, b = frames_line(0, 0, 10, 1), frames_line(0, 10, 0, 1)
    steps = [{"name": "A", "frames": a, "pause": 1.0}, {"name": "B", "frames": b, "pause": 0}]
    pb = player.Playback("S", steps, 0.0)
    goals, _, end = run(pb, arm=[0, 20, 20, 20, 0, 0])
    assert pb.step == 1 and 3.0 < end < 4.0                           # 1 s + 1 s pause + 1 s (+ settling)
    looped = player.Playback("S", steps, 0.0, loop=True)
    arm, t, seen = [0, 20, 20, 20, 0, 0], 0.0, []
    while t < 10:
        act = looped.tick(t, list(arm))
        assert not act.done
        if act.goal:
            arm[:] = act.goal
        seen.append(looped.step)
        t += 0.02
    assert seen.count(0) and seen[-1] in (0, 1) and seen.index(1) < len(seen) - seen[::-1].index(0)


def test_approach_gives_up_waiting_on_a_lagging_arm():
    fr = frames_line(0, 10, 20, 1)
    pb = player.Playback("A", [{"name": "A", "frames": fr}], 0.0, speed=100)
    t = 0.0
    while pb.phase == "approach":
        pb.tick(t, [9.0, 20, 20, 20, 0, 0])                 # stuck 1° short... within tolerance
        t += 0.02
    assert t < 0.1
    pb = player.Playback("A", [{"name": "A", "frames": fr}], 0.0, speed=100)
    t = 0.0
    while pb.phase == "approach" and t < 10:
        pb.tick(t, [0.0, 20, 20, 20, 0, 0])                 # never gets there
        t += 0.02
    assert 3.0 < t < 3.5                                     # 10°/100°/s + 3 s grace


def test_check_steps():
    from helpers import colliding_pose
    good = frames_line(0, 0, 20, 1)
    assert player.check_steps([{"name": "A", "frames": good}]) is None
    q = colliding_pose()
    bad = good[:5] + [[0.5] + q] + [[0.6] + good[-1][1:]]
    msg = player.check_steps([{"name": "A", "frames": good}, {"name": "B", "frames": bad}])
    assert msg.startswith('"B" (step 2) at 0.5 s:')
