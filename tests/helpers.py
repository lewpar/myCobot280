"""Small helpers shared by the tests."""
import arm_model as model


def frames_line(j, a0, a1, secs, hz=10, base=None):
    """Frames moving joint j (0-5) from a0 to a1 degrees over secs, other joints at ``base``."""
    base = base or [0, 20, 20, 20, 0, 0]
    n = int(secs * hz)
    out = []
    for k in range(n + 1):
        q = list(base)
        q[j] = a0 + (a1 - a0) * k / n
        out.append([k / hz] + q)
    return out


def joint_deg(bus, j, calib=None):
    """The fake servo's present angle for joint j (default calibration: zero 2048, dir +1)."""
    c = calib or model.load_calibration()
    return model.ticks_to_deg(c, j, bus.servos[j + 1].pos)


def colliding_pose():
    """A pose the collision model refuses (arm folded down into the table)."""
    for a in range(60, 141, 5):
        q = [0, a, a, 0, 0, 0]
        if model.check_pose(q):
            return q
    raise AssertionError("no colliding pose found")
