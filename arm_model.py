"""
arm_model — shared model of the myCobot280 used by the backend and the tools.

* Kinematics from Elephant Robotics' mycobot_280_pi URDF (base frame: metres, Z up).
* Calibration: per-joint zero tick and direction that map servo ticks to URDF joint angles,
  plus the tool length and the servo speed/acceleration units (ik_calibration.json).
* Saved centre ("home") positions in raw ticks (center_positions.json).
* A simple collision check: table, base/shoulder column, and wrist-vs-upper-arm.

The IK simulator page (src/backend/static/sim/js/collision.js) carries a copy of the same collision
model so it can refuse poses before sending them; the backend checks again before moving.
"""

import json
import math
import os
import re
import threading

ROOT = os.path.dirname(os.path.abspath(__file__))
CALIB_FILE = os.path.join(ROOT, "ik_calibration.json")
CENTER_FILE = os.path.join(ROOT, "center_positions.json")
RECENTER_LOG = os.path.join(ROOT, "servo_centres_log.json")   # every re-centring, with the values to undo it

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
# Keep these in sync with COLLISION in sim/js/collision.js.
FLOOR_MARGIN = 0.005     # clearance kept between any link and the table
TCP_MIN_Z = 0.003        # the tool tip itself may come this close to the table
BASE_R, BASE_TOP = 0.075, 0.12      # pedestal + J1 housing, checked against J3 and beyond
COLUMN_R, COLUMN_TOP = 0.05, 0.19   # shoulder column, checked against the wrist
WRIST_TO_UPPER_ARM = 0.05           # minimum distance from the wrist to the J2-J3 link
# The attachment on the flange is a cylinder along the flange normal, sampled every TOOL_STEP.
# Its points are checked like the wrist, plus against the arm's own links (it can fold back into them).
TOOL_STEP = 0.015
UPPER_ARM_R = 0.03       # J2-J3 link radius + margin, for attachment points
FOREARM_R = 0.028        # J3-J4 link radius + margin
TOOL_R_DEFAULT = 0.01    # radius assumed when only a length is known (a 20 mm custom tool)
# The ATOM head (5x5 LED matrix) sits behind the J5 body on the J6 axis, facing away from the flange:
# (distance behind the flange face, radius) of the spheres that cover it. Checked like the wrist, and
# against the forearm, which it can fold back into.
ATOM_SPHERES = ((0.050, 0.017), (0.066, 0.016))

# Work area: a slice of the circle around the base the tool tip (the TCP: flange centre, or the attachment's
# tip) must stay inside; the rest of the arm may cross its edges. Keep in sync with the page.
# center: direction of the slice's middle in degrees (0 = +X, the way the flange points at the zero pose,
# away from the Pi's ports; -90 = -Y, the arm's right). span: its width in degrees (360 = the whole circle).
# radius_mm: outer limit, 0 for none. base_mm: a keep-out cylinder of that radius around the base axis, up to
# BASE_KEEPOUT_TOP high, that the tip may not enter (0 for none): working that close in makes the arm fold its
# wrist back onto itself. Above it (the zero pose, the raised poses routes go through) a tip within AREA_CORE
# of the axis is always inside.
AREA_CORE = 0.06
BASE_KEEPOUT_TOP = 0.25
DEFAULT_AREA = {"enabled": True, "center": 0.0, "span": 180.0, "radius_mm": 0.0, "base_mm": 150.0}

# Attachments the page offers (keep in sync with ATTACHMENTS in sim/js/kinematics.js): length and diameter in mm.
ATTACHMENTS = {
    "none": {"name": "No attachment", "length_mm": 0, "diameter_mm": 0},
    "vacuum": {"name": "Vacuum suction", "length_mm": 80, "diameter_mm": 25},
    "custom": {"name": "Custom", "length_mm": None, "diameter_mm": None},
}

DEFAULT_CALIB = {"zero": [2048] * 6, "dir": [1] * 6, "tool_mm": 0.0,
                 "tool_d_mm": TOOL_R_DEFAULT * 2000, "attachment": "custom", "area": dict(DEFAULT_AREA),
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


# each fixed joint origin as a flat 3x4 (rotation rows, then the translation column)
_FIXED34 = [(f[0][0], f[0][1], f[0][2], f[0][3], f[1][0], f[1][1], f[1][2], f[1][3], f[2][0], f[2][1], f[2][2], f[2][3])
            for f in _FIXED]


def chain(q_rad):
    """Forward kinematics, written out for speed (the IK solver calls it thousands of times a second).
    Returns (joint origins, joint axes, flange position, flange normal); joint i turns about axes[i]."""
    a, b, c, x, d, e, f, y, g, h, i, z = 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0
    pos, axes = [], []
    for (m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23), q in zip(_FIXED34, q_rad):
        # T = T * fixed
        a, b, c, x = (a * m00 + b * m10 + c * m20, a * m01 + b * m11 + c * m21, a * m02 + b * m12 + c * m22,
                      a * m03 + b * m13 + c * m23 + x)
        d, e, f, y = (d * m00 + e * m10 + f * m20, d * m01 + e * m11 + f * m21, d * m02 + e * m12 + f * m22,
                      d * m03 + e * m13 + f * m23 + y)
        g, h, i, z = (g * m00 + h * m10 + i * m20, g * m01 + h * m11 + i * m21, g * m02 + h * m12 + i * m22,
                      g * m03 + h * m13 + i * m23 + z)
        pos.append((x, y, z))
        axes.append((c, f, i))
        # T = T * rotz(q): only the first two columns change
        cq, sq = math.cos(q), math.sin(q)
        a, b = a * cq + b * sq, b * cq - a * sq
        d, e = d * cq + e * sq, e * cq - d * sq
        g, h = g * cq + h * sq, h * cq - g * sq
    return pos, axes, (x, y, z), (c, f, i)


def joint_frames(q_rad):
    """Each joint's frame after its own rotation, as (rotation rows, origin): what the page's rotGroups[k] is."""
    a, b, c, x, d, e, f, y, g, h, i, z = 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0
    out = []
    for (m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23), q in zip(_FIXED34, q_rad):
        a, b, c, x = (a * m00 + b * m10 + c * m20, a * m01 + b * m11 + c * m21, a * m02 + b * m12 + c * m22,
                      a * m03 + b * m13 + c * m23 + x)
        d, e, f, y = (d * m00 + e * m10 + f * m20, d * m01 + e * m11 + f * m21, d * m02 + e * m12 + f * m22,
                      d * m03 + e * m13 + f * m23 + y)
        g, h, i, z = (g * m00 + h * m10 + i * m20, g * m01 + h * m11 + i * m21, g * m02 + h * m12 + i * m22,
                      g * m03 + h * m13 + i * m23 + z)
        cq, sq = math.cos(q), math.sin(q)
        a, b = a * cq + b * sq, b * cq - a * sq
        d, e = d * cq + e * sq, e * cq - d * sq
        g, h = g * cq + h * sq, h * cq - g * sq
        out.append((((a, b, c), (d, e, f), (g, h, i)), (x, y, z)))
    return out


def _body_parts():
    """The body as points in each joint's own frame: [(name, joint, radius, local points, local centre, reach)]."""
    zero = joint_frames([0.0] * 6)

    def to_local(j, p):   # inverse of the joint's zero-pose frame: R^T (p - t)
        (R, t) = zero[j]
        v = [p[k] - t[k] for k in range(3)]
        return tuple(R[0][k] * v[0] + R[1][k] * v[1] + R[2][k] * v[2] for k in range(3))

    parts = []
    for name, j, r, pts in BODY_LINKS:
        samples = []
        for p0, p1 in zip(pts, pts[1:]):
            n = max(1, math.ceil(math.dist(p0, p1) / BODY_STEP))
            samples += [_mid(p0, p1, k / n) for k in range(n)]
        samples.append(pts[-1])
        parts.append((name, j, r, samples))
    for name, j, c, ax, r, length in BODY_HOUSINGS:
        parts.append((name, j, r, [tuple(c[k] + ax[k] * length / 2 * s for k in range(3)) for s in (-1, 0, 1)]))
    out = []
    for name, j, r, pts in parts:
        loc = [to_local(j, p) for p in pts]
        cen = tuple(sum(p[k] for p in loc) / len(loc) for k in range(3))
        out.append((name, j, r, loc, cen, max(math.dist(p, cen) for p in loc) + r))
    return out


def fk(q_deg, tool_m=0.0):
    """Joint origins, flange centre, tool tip and flange normal in the base frame."""
    joints, _, flange, normal = chain([math.radians(q) for q in q_deg])
    tcp = (flange[0] + normal[0] * tool_m, flange[1] + normal[1] * tool_m, flange[2] + normal[2] * tool_m)
    return {"joints": joints, "flange": flange, "tcp": tcp, "normal": normal}


# ---- collision checks ----------------------------------------------------------------------------

def _mid(a, b, f=0.5):
    return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f)


def _seg_dist(p, a, b):
    """Distance from point p to the segment a-b. Written out (no lists or generators): check_pose calls it
    a dozen times a pose, and the IK solver calls check_pose at every step."""
    ax, ay, az = a
    dx, dy, dz = b[0] - ax, b[1] - ay, b[2] - az
    L = dx * dx + dy * dy + dz * dz
    f = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy + (p[2] - az) * dz) / L))
    return math.dist(p, (ax + dx * f, ay + dy * f, az + dz * f))


# ---- obstacles (the page's Workspace view): boxes, cylinders and spheres the whole arm keeps clear of ------------
# They live in the work area dict (area["obstacles"]), so they reach every check the area does, but unlike the
# area's own limits they apply even with the area switched off. Keep in sync with collision.js.
OBST_MARGIN = 0.01       # clearance kept from an obstacle, on top of each part's radius
OBST_MAX = 50
OBST_SHAPES = ("box", "cylinder", "sphere")
# The arm's body as the page draws it, checked against obstacles (the kinematic line alone is up to ~70 mm off
# the real links: the upper arm runs beside J2-J3, not along it). Base frame, metres, at the zero pose; each part
# moves with joint `j` (0-based). Keep in sync with BODY_LINKS / BODY_HOUSINGS in kinematics.js (test_page.py).
# The column (J1-J2, and the J2 servo on it) only turns about itself: a shape there is flagged by the page.
BODY_LINKS = (   # (name, joint, radius, polyline): the round tubes between the servos
    ("the upper arm", 1, 0.023, ((0, -0.035, 0.1386), (0, -0.068, 0.1386), (0, -0.068, 0.249), (0, -0.031, 0.249))),
    ("the forearm", 2, 0.021, ((0, -0.031, 0.249), (0, 0, 0.249), (0, 0, 0.345), (0, -0.029, 0.345))),
    ("the wrist", 3, 0.020, ((0, -0.029, 0.345), (0, -0.0636, 0.345), (0, -0.0636, 0.381))),
    ("the wrist", 4, 0.019, ((0, -0.0636, 0.381), (0, -0.0636, 0.4181))),
)
BODY_HOUSINGS = (   # (name, joint, centre, axis, radius, length): the servos
    ("the elbow", 1, (0, -0.031, 0.249), (0, 1, 0), 0.028, 0.032),
    ("J4", 2, (0, -0.029, 0.345), (0, 1, 0), 0.024, 0.028),
    ("the wrist", 3, (0, -0.0636, 0.381), (0, 0, 1), 0.0205, 0.014),
    ("J6", 4, (0.023, -0.0636, 0.4181), (1, 0, 0), 0.021, 0.034),
)
BODY_STEP = 0.015   # points along a tube this far apart
BODY = None         # _body_parts(), built at the end of the module
_OBST_ID = re.compile(r"^[A-Za-z0-9_-]{1,24}$")
_HEX = re.compile(r"^#[0-9a-fA-F]{6}$")


def euler_xyz(rx, ry, rz):
    """Rotation matrix rows for Euler angles (radians) in three.js's "XYZ" order, so the page and this agree."""
    a, b, c, d, e, f = math.cos(rx), math.sin(rx), math.cos(ry), math.sin(ry), math.cos(rz), math.sin(rz)
    ae, af, be, bf = a * e, a * f, b * e, b * f
    return ((c * e, -c * f, d), (af + be * d, ae - bf * d, -b * c), (bf - ae * d, be + af * d, a * c))


def clean_obstacles(obs):
    """A list of obstacles, checked and normalised (sizes and positions in mm, rotations in degrees), or None.
    Each: {"id", "name", "shape": box|cylinder|sphere, "pos": [x, y, z] (centre), "size": [x, y, z],
    "rot": [x, y, z], "color": "#rrggbb"}; a cylinder's axis is its local z and its diameter size[0], a
    sphere's diameter size[0]."""
    if not isinstance(obs, list) or len(obs) > OBST_MAX:
        return None
    out, seen = [], set()
    fin = lambda v, lo, hi: isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and lo <= v <= hi
    for o in obs:
        if not isinstance(o, dict) or o.get("shape") not in OBST_SHAPES:
            return None
        oid, name, pos, size, rot = o.get("id"), o.get("name", ""), o.get("pos"), o.get("size"), o.get("rot", [0, 0, 0])
        if not (isinstance(oid, str) and _OBST_ID.match(oid)) or oid in seen or not isinstance(name, str) or len(name) > 40:
            return None
        if not all(isinstance(v, list) and len(v) == 3 for v in (pos, size, rot)):
            return None
        if not (all(fin(v, -1000, 1000) for v in pos) and all(fin(v, -1e6, 1e6) for v in size) and all(fin(v, -360, 360) for v in rot)):
            return None
        size = [float(v) for v in size]
        if o["shape"] == "cylinder":      # (the sizes a shape doesn't use follow the ones it does)
            size[1] = size[0]
        elif o["shape"] == "sphere":
            size = [size[0]] * 3
        if not all(5 <= v <= 1000 for v in size):
            return None
        color = o.get("color", "#8a94a6")
        if not (isinstance(color, str) and _HEX.match(color)):
            return None
        seen.add(oid)
        out.append({"id": oid, "name": name.strip() or o["shape"].capitalize(), "shape": o["shape"],
                    "pos": [round(float(v), 1) for v in pos], "size": [round(v, 1) for v in size],
                    "rot": [round(float(v), 2) for v in rot], "color": color.lower()})
    return out


def _prep_obstacle(o):
    """(name, shape, centre (m), rotation rows, shape sizes (m), bounding radius) for the checks."""
    c = tuple(v / 1000 for v in o["pos"])
    R = euler_xyz(*(math.radians(v) for v in o["rot"]))
    sx, sy, sz = (v / 2000 for v in o["size"])
    if o["shape"] == "box":
        dims, bound = (sx, sy, sz), math.sqrt(sx * sx + sy * sy + sz * sz)
    elif o["shape"] == "cylinder":
        dims, bound = (sx, sz), math.hypot(sx, sz)
    else:
        dims, bound = (sx,), sx
    return (o["name"], o["shape"], c, R, dims, bound)


_prep_cache = {}


def _prepared(obs):
    key = tuple((o["shape"], *o["pos"], *o["size"], *o["rot"], o["name"]) for o in obs)
    got = _prep_cache.get(key)
    if got is None:
        if len(_prep_cache) > 64:
            _prep_cache.clear()
        got = _prep_cache[key] = [_prep_obstacle(o) for o in obs]
    return got


def obstacle_distance(p, prep):
    """Signed distance (m) from point p to a prepared obstacle: negative inside."""
    _, shape, c, R, dims, _ = prep
    dx, dy, dz = p[0] - c[0], p[1] - c[1], p[2] - c[2]
    # into the obstacle's own frame: R transposed
    lx = R[0][0] * dx + R[1][0] * dy + R[2][0] * dz
    ly = R[0][1] * dx + R[1][1] * dy + R[2][1] * dz
    lz = R[0][2] * dx + R[1][2] * dy + R[2][2] * dz
    if shape == "box":
        qx, qy, qz = abs(lx) - dims[0], abs(ly) - dims[1], abs(lz) - dims[2]
        out = math.sqrt(max(qx, 0.0) ** 2 + max(qy, 0.0) ** 2 + max(qz, 0.0) ** 2)
        return out + min(max(qx, qy, qz), 0.0)
    if shape == "cylinder":
        rad, ax = math.hypot(lx, ly) - dims[0], abs(lz) - dims[1]
        return math.hypot(max(rad, 0.0), max(ax, 0.0)) + min(max(rad, ax), 0.0)
    return math.sqrt(lx * lx + ly * ly + lz * lz) - dims[0]


def _apply(frame, p):
    R, t = frame
    return (R[0][0] * p[0] + R[0][1] * p[1] + R[0][2] * p[2] + t[0], R[1][0] * p[0] + R[1][1] * p[1] + R[1][2] * p[2] + t[1],
            R[2][0] * p[0] + R[2][1] * p[1] + R[2][2] * p[2] + t[2])


def _hits_obstacle(obs, q_deg, extra):
    """The first "<part> would hit <obstacle>", or None. The body (BODY_*) at pose q_deg, and extra: [(name, point,
    radius)] (the ATOM, the attachment, the flange)."""
    frames = joint_frames([math.radians(v) for v in q_deg])
    for prep in _prepared(obs):
        name, _, c, _, _, bound = prep
        for part, j, r, loc, cen, reach in BODY:
            if math.dist(_apply(frames[j], cen), c) > bound + reach + OBST_MARGIN:
                continue                              # nowhere near: skip its points
            for p in loc:
                if obstacle_distance(_apply(frames[j], p), prep) < r + OBST_MARGIN:
                    return f"{part} would hit {name}"
        for part, p, r in extra:
            if math.dist(p, c) <= bound + r + OBST_MARGIN and obstacle_distance(p, prep) < r + OBST_MARGIN:
                return f"{part} would hit {name}"
    return None


def clean_area(a):
    """A valid work area dict, or None if ``a`` isn't one. Its obstacles (if it has the key) are cleaned too."""
    try:
        out = {"enabled": bool(a["enabled"]), "center": float(a["center"]), "span": float(a["span"]),
               "radius_mm": float(a.get("radius_mm", 0)), "base_mm": float(a.get("base_mm", DEFAULT_AREA["base_mm"]))}
    except (KeyError, TypeError, ValueError):
        return None
    if "obstacles" in a:
        obs = clean_obstacles(a["obstacles"])
        if obs is None:
            return None
        out["obstacles"] = obs
    ok = (-180 <= out["center"] <= 180 and 30 <= out["span"] <= 360
          and (out["radius_mm"] == 0 or 100 <= out["radius_mm"] <= 450)
          and (out["base_mm"] == 0 or 60 <= out["base_mm"] <= 250)
          and (out["radius_mm"] == 0 or out["base_mm"] < out["radius_mm"]))
    return out if ok and all(math.isfinite(v) for k, v in out.items() if k not in ("enabled", "obstacles")) else None


def _outside_area(p, area):
    """None if point p is inside the work area, else what's wrong."""
    rad = math.hypot(p[0], p[1])
    lim = area["radius_mm"] / 1000
    if lim > 0 and rad > lim:
        return "would reach past the work area"
    if rad < area.get("base_mm", 0) / 1000 and p[2] < BASE_KEEPOUT_TOP:
        return "would come too close to the base"
    if area["span"] >= 360 or rad < AREA_CORE:
        return None
    off = abs((math.degrees(math.atan2(p[1], p[0])) - area["center"] + 180) % 360 - 180)
    return "would leave the work area" if off > area["span"] / 2 else None


def check_pose(q_deg, tool_m=0.0, tool_r=TOOL_R_DEFAULT, area=None):
    """None if the pose is clear, otherwise a short reason. The attachment is ``tool_m`` long with
    radius ``tool_r`` (metres); ``area`` is a work area dict (see DEFAULT_AREA) or None for no limit. Its
    obstacles are checked whether or not the area is enabled."""
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
    atom = [(tuple(f - n * d for f, n in zip(k["flange"], k["normal"])), r) for d, r in ATOM_SPHERES]
    body += [("the ATOM", p, r, r, True) for p, r in atom]
    tool = []
    if tool_m > 0.002:
        # lowest point of the cylinder's cross-section: its full radius when level, nothing when vertical
        r_floor = tool_r * math.sqrt(max(0.0, 1 - k["normal"][2] ** 2))
        n = max(2, math.ceil(tool_m / TOOL_STEP))
        tool = [_mid(k["flange"], k["tcp"], i / n) for i in range(1, n + 1)]
        body += [("the attachment", p, r_floor, tool_r, True) for p in tool[:-1]]
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
    for p, r in atom:
        if _seg_dist(p, j[2], j[3]) < FOREARM_R + r:
            return "the ATOM would hit the forearm"
    for p in tool:   # the attachment folding back into the arm's own links
        if _seg_dist(p, j[1], j[2]) < UPPER_ARM_R + tool_r:
            return "the attachment would hit the upper arm"
        if _seg_dist(p, j[2], j[3]) < FOREARM_R + tool_r:
            return "the attachment would hit the forearm"
    if area and area.get("obstacles"):
        extra = ([("the flange", k["flange"], 0.02)] + [("the ATOM", p, r) for p, r in atom]
                 + [("the attachment", p, tool_r) for p in tool])
        why = _hits_obstacle(area["obstacles"], q_deg, extra)
        if why:
            return why
    if tool and k["tcp"][2] - r_floor < TCP_MIN_Z:
        return "the attachment's tip would go below the table"
    if k["tcp"][2] < TCP_MIN_Z:
        return "the tool tip would go below the table"
    if area and area["enabled"]:
        why = _outside_area(k["tcp"], area)
        if why:
            return ("the attachment's tip " if tool else "the flange ") + why
    return None


def check_path(q_from, q_to, tool_m=0.0, steps=16, tool_r=TOOL_R_DEFAULT, area=None, every_deg=3.0):
    """Check poses along a straight joint-space move (ik_link synchronises the servos' speeds so they follow
    it, roughly). At least ``steps`` samples, and one every ``every_deg`` of the joint that moves most."""
    if any(v is None for v in q_from) or check_pose(q_from, tool_m, tool_r, area):
        return check_pose(q_to, tool_m, tool_r, area)   # already in contact (or unknown): allow moving to any clear pose
    steps = max(steps, math.ceil(max(abs(b - a) for a, b in zip(q_from, q_to)) / every_deg))
    for s in range(1, steps + 1):
        f = s / steps
        why = check_pose([a + (b - a) * f for a, b in zip(q_from, q_to)], tool_m, tool_r, area)
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
        for key in ("tool_mm", "tool_d_mm", "speed_unit", "acc_unit"):
            if isinstance(saved.get(key), (int, float)) and saved[key] >= 0:
                c[key] = float(saved[key])
        if saved.get("attachment") in ATTACHMENTS:
            c["attachment"] = saved["attachment"]
        if clean_area(saved.get("area")):
            c["area"] = clean_area(saved["area"])
    except (OSError, ValueError, TypeError):
        pass
    return c


def save_calibration(c):
    data = {k: c[k] for k in ("zero", "dir", "tool_mm", "tool_d_mm", "attachment", "area", "speed_unit",
                              "acc_unit", "calibrated")}
    with _file_lock:
        tmp = CALIB_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(data, f, indent=2)
        os.replace(tmp, CALIB_FILE)


def recenter(arm, calib, joints, now=None):
    """Re-centre the servos of ``joints`` (0-5) where they are now: each reads 2048 afterwards (Feetech's
    "calibrate the middle", stored in the servo's EEPROM), which puts its 0/4095 point half a turn away. Torque
    must be off (the goal register keeps its number, so a torqued servo would jump). The calibration's zero and
    the saved home positions shift by the same amount, so every angle still means the same pose; ``calib`` is
    updated in place (the caller saves it). Every re-centring is appended to RECENTER_LOG with the old
    corrections, so it can be undone (tools/recenter_servos.py --undo). Returns one result per joint."""
    import time as _time
    results = []
    centers = load_centers()
    for j in joints:
        sid = JOINT_IDS[j]
        before = arm.read_positions([sid])[0]
        corr = arm.position_correction(sid)
        r = {"joint": j, "id": sid, "before": before, "correction_before": corr, "after": None,
             "correction_after": None, "ok": False}
        results.append(r)
        if before is None or corr is None:
            r["message"] = f"J{j + 1} didn't answer."
            continue
        r["after"] = after = arm.recenter(sid)
        r["correction_after"] = arm.position_correction(sid)
        if after is None or abs(after - 2048) > 8:
            r["message"] = f"J{j + 1} didn't take the new centre (it reads {after})."
            continue
        r["ok"] = True
        d = after - before
        calib["zero"][j] = (calib["zero"][j] + d) % 4096
        if sid in centers:
            centers[sid] = (centers[sid] + d) % 4096
    if any(r["ok"] for r in results):
        save_centers(centers)
        try:
            with open(RECENTER_LOG) as f:
                log = json.load(f)
        except (OSError, ValueError):
            log = []
        log.append({"time": now if now is not None else _time.time(), "results": results})
        with _file_lock:
            with open(RECENTER_LOG, "w") as f:
                json.dump(log, f, indent=2)
    return results


def ticks_to_deg(c, j, ticks):
    return (ticks - c["zero"][j]) * c["dir"][j] / TICKS_PER_DEG


def near_deg(a):
    """The same joint angle within -180..180: how far a joint is really turned from its zero. Only differs
    from ``a`` for a reading more than half a turn from the zero, i.e. a servo that has gone past its 0/4095
    point (the arm model and every check keep using the reading itself)."""
    return (a + 180.0) % 360.0 - 180.0


def deg_to_ticks(c, j, deg):
    lo, hi = URDF_LIMITS_DEG[j]
    deg = max(lo, min(hi, deg))
    return int(round(c["zero"][j] + c["dir"][j] * deg * TICKS_PER_DEG))


def pose_from_ticks(c, ticks):
    return [None if t is None else ticks_to_deg(c, j, t) for j, t in enumerate(ticks)]


def check_tick_move(c, current_ticks, new_ticks):
    """Collision check for a raw-tick move (REST single-servo moves).

    ``current_ticks`` and ``new_ticks`` are lists of 6 (None where unknown). Returns None if clear."""
    if any(t is None for t in current_ticks):
        return "not every servo answered, so the move can't be checked"
    q0 = pose_from_ticks(c, current_ticks)
    q1 = pose_from_ticks(c, [n if n is not None else t for n, t in zip(new_ticks, current_ticks)])
    return check_path(q0, q1, c["tool_mm"] / 1000, tool_r=c["tool_d_mm"] / 2000, area=c["area"])


BODY = _body_parts()
