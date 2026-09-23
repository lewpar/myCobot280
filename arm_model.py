"""
arm_model — shared model of the myCobot280 used by the backend, the TCP server and the tools.

* Kinematics from Elephant Robotics' mycobot_280_pi URDF (base frame: metres, Z up).
* Calibration: per-joint zero tick and direction that map servo ticks to URDF joint angles,
  plus the tool length and the servo speed/acceleration units (ik_calibration.json).
* Saved centre ("home") positions in raw ticks (center_positions.json).
* A simple collision check: table, base/shoulder column, and wrist-vs-upper-arm.

The IK simulator page (src/backend/static/ik_sim.html) carries a copy of the same collision
model so it can refuse poses before sending them; the backend checks again before moving.
"""

import json
import math
import os
import threading

ROOT = os.path.dirname(os.path.abspath(__file__))
CALIB_FILE = os.path.join(ROOT, "ik_calibration.json")
CENTER_FILE = os.path.join(ROOT, "center_positions.json")

JOINT_IDS = [1, 2, 3, 4, 5, 6]
TICKS_PER_DEG = 4096 / 360

# (xyz, rpy) of each joint origin relative to the previous joint frame; every joint turns about local Z
URDF_JOINTS = [
    ((0, 0, 0.13956), (0, 0, 0)),
    ((0, 0, -0.001), (0, 1.5708, -1.5708)),
    ((-0.1104, 0, 0), (0, 0, 0)),
    ((-0.096, 0, 0.06462), (0, 0, -1.5708)),
    ((0, -0.07318, -0.001), (1.5708, -1.5708, 0)),
    ((0, 0.0456, 0), (-1.5708, 0, 0)),
]
URDF_LIMITS_DEG = [(-168, 168), (-140, 140), (-150, 150), (-150, 150), (-155, 160), (-180, 180)]

# ---- collision model (metres) -------------------------------------------------------------------
# Keep these in sync with COLLISION in ik_sim.html.
FLOOR_MARGIN = 0.005     # clearance kept between any link and the table
TCP_MIN_Z = 0.003        # the tool tip itself may come this close to the table
BASE_R, BASE_TOP = 0.075, 0.12      # pedestal + J1 housing, checked against J3 and beyond
COLUMN_R, COLUMN_TOP = 0.05, 0.19   # shoulder column, checked against the wrist
WRIST_TO_UPPER_ARM = 0.05           # minimum distance from the wrist to the J2-J3 link

DEFAULT_CALIB = {"zero": [2048] * 6, "dir": [1] * 6, "tool_mm": 0.0,
                 "speed_unit": 1.0,    # servo speed register: steps/s per unit (STS: 1)
                 "acc_unit": 100.0}    # servo acceleration register: steps/s^2 per unit (STS: 100)

_file_lock = threading.Lock()


# ---- small matrix helpers ------------------------------------------------------------------------

def _mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _origin(xyz, rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    # URDF rpy: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
            [-sp, cp * sr, cp * cr, xyz[2]],
            [0, 0, 0, 1]]


def _rotz(q):
    c, s = math.cos(q), math.sin(q)
    return [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


_FIXED = [_origin(x, r) for x, r in URDF_JOINTS]


def fk(q_deg, tool_m=0.0):
    """Joint origins, flange centre, tool tip and flange normal in the base frame."""
    t = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    joints = []
    for fixed, q in zip(_FIXED, q_deg):
        t = _mul(t, fixed)
        joints.append((t[0][3], t[1][3], t[2][3]))
        t = _mul(t, _rotz(math.radians(q)))
    flange = (t[0][3], t[1][3], t[2][3])
    normal = (t[0][2], t[1][2], t[2][2])
    tcp = tuple(f + n * tool_m for f, n in zip(flange, normal))
    return {"joints": joints, "flange": flange, "tcp": tcp, "normal": normal}


# ---- collision checks ----------------------------------------------------------------------------

def _mid(a, b, f=0.5):
    return tuple(x + (y - x) * f for x, y in zip(a, b))


def _seg_dist(p, a, b):
    ab = [y - x for x, y in zip(a, b)]
    ap = [y - x for x, y in zip(a, p)]
    L = sum(v * v for v in ab)
    f = 0.0 if L == 0 else max(0.0, min(1.0, sum(u * v for u, v in zip(ap, ab)) / L))
    c = [x + v * f for x, v in zip(a, ab)]
    return math.dist(p, c)


def check_pose(q_deg, tool_m=0.0):
    """None if the pose is clear, otherwise a short reason."""
    for j, (q, (lo, hi)) in enumerate(zip(q_deg, URDF_LIMITS_DEG)):
        if not lo - 0.5 <= q <= hi + 0.5:
            return f"J{j + 1} would pass its {lo}..{hi} degree limit"
    k = fk(q_deg, tool_m)
    j = k["joints"]
    # (name, point, radius against the table, radius against the base, is part of the wrist)
    body = [
        ("the elbow (J3)", j[2], 0.03, 0.03, False),
        ("the forearm", _mid(j[2], j[3]), 0.028, 0.028, False),
        ("J4", j[3], 0.026, 0.026, False),
        ("the wrist (J5)", j[4], 0.024, 0.024, True),
        ("J6", j[5], 0.022, 0.022, True),
        ("the flange", k["flange"], 0.0, 0.02, True),
    ]
    if tool_m > 0.002:
        body.append(("the tool", _mid(k["flange"], k["tcp"]), 0.0, 0.01, True))
    for name, p, r_floor, r_side, wrist in body:
        if p[2] - r_floor < FLOOR_MARGIN:
            return f"{name} would hit the table"
        rad = math.hypot(p[0], p[1])
        if rad < BASE_R + r_side and p[2] - r_side < BASE_TOP:
            return f"{name} would hit the base"
        if wrist and rad < COLUMN_R + r_side and p[2] - r_side < COLUMN_TOP:
            return f"{name} would hit the shoulder"
        if wrist and _seg_dist(p, j[1], j[2]) < WRIST_TO_UPPER_ARM:
            return f"{name} would hit the upper arm"
    if k["tcp"][2] < TCP_MIN_Z:
        return "the tool tip would go below the table"
    return None


def check_path(q_from, q_to, tool_m=0.0, steps=16):
    """Check poses along a straight joint-space move (an approximation of what the servos do)."""
    if any(v is None for v in q_from) or check_pose(q_from, tool_m):
        return check_pose(q_to, tool_m)   # already in contact (or unknown): allow moving to any clear pose
    for s in range(1, steps + 1):
        f = s / steps
        why = check_pose([a + (b - a) * f for a, b in zip(q_from, q_to)], tool_m)
        if why:
            return why + (" on the way there" if s < steps else "")
    return None


# ---- calibration --------------------------------------------------------------------------------

def load_centers():
    try:
        with open(CENTER_FILE) as f:
            return {int(k): int(v) for k, v in json.load(f).items()}
    except (OSError, ValueError, TypeError):
        return {}


def save_centers(centers):
    with _file_lock:
        tmp = CENTER_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump({str(k): int(v) for k, v in sorted(centers.items())}, f, indent=2)
        os.replace(tmp, CENTER_FILE)


def load_calibration():
    c = json.loads(json.dumps(DEFAULT_CALIB))
    centers = load_centers()
    if centers:  # seed zeros from the saved centres until a proper calibration exists
        c["zero"] = [centers.get(sid, 2048) for sid in JOINT_IDS]
    c["calibrated"] = False
    try:
        with open(CALIB_FILE) as f:
            saved = json.load(f)
        if isinstance(saved.get("zero"), list) and len(saved["zero"]) == 6:
            c["zero"] = [int(v) for v in saved["zero"]]
            c["calibrated"] = bool(saved.get("calibrated", True))
        if isinstance(saved.get("dir"), list) and len(saved["dir"]) == 6:
            c["dir"] = [1 if int(v) >= 0 else -1 for v in saved["dir"]]
        for key in ("tool_mm", "speed_unit", "acc_unit"):
            if isinstance(saved.get(key), (int, float)) and saved[key] >= 0:
                c[key] = float(saved[key])
    except (OSError, ValueError, TypeError):
        pass
    return c


def save_calibration(c):
    data = {k: c[k] for k in ("zero", "dir", "tool_mm", "speed_unit", "acc_unit", "calibrated")}
    with _file_lock:
        tmp = CALIB_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(data, f, indent=2)
        os.replace(tmp, CALIB_FILE)


def ticks_to_deg(c, j, ticks):
    return (ticks - c["zero"][j]) * c["dir"][j] / TICKS_PER_DEG


def deg_to_ticks(c, j, deg):
    lo, hi = URDF_LIMITS_DEG[j]
    deg = max(lo, min(hi, deg))
    return int(round(c["zero"][j] + c["dir"][j] * deg * TICKS_PER_DEG))


def pose_from_ticks(c, ticks):
    return [None if t is None else ticks_to_deg(c, j, t) for j, t in enumerate(ticks)]


def check_tick_move(c, current_ticks, new_ticks):
    """Collision check for a raw-tick move (REST / TCP single-servo moves).

    ``current_ticks`` and ``new_ticks`` are lists of 6 (None where unknown). Returns None if clear."""
    if any(t is None for t in current_ticks):
        return "not every servo answered, so the move can't be checked"
    q0 = pose_from_ticks(c, current_ticks)
    q1 = pose_from_ticks(c, [n if n is not None else t for n, t in zip(new_ticks, current_ticks)])
    return check_path(q0, q1, c["tool_mm"] / 1000)
