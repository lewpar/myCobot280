#!/usr/bin/env python3
"""
check_servo_units.py — measure what the servo speed and acceleration registers really mean.

The IK link converts degrees/s into register values assuming STS units (speed: 1 step/s per unit,
acceleration: 100 steps/s² per unit). This times two moves of one joint and prints the units your
servos actually use, so the sim and the real arm move at the same pace.

    python3 tools/check_servo_units.py              # measures J6 (wrist roll: no load from gravity)
    python3 tools/check_servo_units.py --write      # also saves the result to ik_calibration.json

Stop the backend first (only one program can hold the serial port).
The joint turns about 45 degrees each way from where it is now and comes back.
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import arm_model as model  # noqa: E402
from mycobot280 import MyCobot280  # noqa: E402

TOL = 8  # ticks


def settle(arm, sid, timeout=5.0):
    """Wait until the servo has actually stopped (two readings 100 ms apart agree)."""
    t0, last = time.monotonic(), None
    while time.monotonic() - t0 < timeout:
        p = arm.read_positions([sid])[0]
        if p is not None and last is not None and abs(p - last) <= 1:
            return
        last = p
        time.sleep(0.1)


def timed_move(arm, sid, target, speed, accel, timeout=15.0):
    """Send one move and return seconds until the servo is within TOL ticks of the target."""
    arm.sync_move({sid: target}, speed, accel)
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        p = arm.read_positions([sid])[0]
        if p is not None and abs(p - target) <= TOL:
            return time.monotonic() - t0
    raise RuntimeError(f"servo {sid} did not reach {target} within {timeout} s")


def trapezoid_time(d, v, a):
    """Time for a move of d steps at cruise speed v and acceleration a, until it is within TOL."""
    v_end = min(v, math.sqrt(2 * a * TOL))
    up, down = v * v / (2 * a), (v * v - v_end * v_end) / (2 * a)
    if up + down > d - TOL:   # never reaches cruise speed
        vp = math.sqrt(a * (d - TOL) + v_end * v_end / 2)
        return vp / a + (vp - v_end) / a
    return v / a + (v - v_end) / a + (d - TOL - up - down) / v


def solve_accel(d, v, t):
    """Acceleration that makes trapezoid_time(d, v, a) == t (bisection on a log scale)."""
    lo, hi = 1.0, 1e7
    for _ in range(100):
        mid = math.sqrt(lo * hi)
        if trapezoid_time(d, v, mid) > t:
            lo = mid
        else:
            hi = mid
    return math.sqrt(lo * hi)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=os.environ.get("MYCOBOT_PORT", "/dev/ttyAMA0"))
    ap.add_argument("--servo", type=int, default=6, help="servo ID to test (default 6, the wrist roll)")
    ap.add_argument("--speed", type=int, default=300, help="speed register value for the test")
    ap.add_argument("--write", action="store_true", help="save the measured units to ik_calibration.json")
    ap.add_argument("--yes", action="store_true", help="don't ask before moving")
    args = ap.parse_args()

    with MyCobot280(args.port) as arm:
        sid = args.servo
        start = arm.get_position(sid)
        if start is None:
            sys.exit(f"servo {sid} did not answer")
        lo, hi = arm.get_limits(sid)
        span = int(45 * model.TICKS_PER_DEG)
        a, b = start - span, start + span
        if a < lo or b > hi:
            sys.exit(f"servo {sid} is at {start}; it needs {span} ticks of room each way "
                     f"inside its safe range {lo}..{hi}. Move it nearer the middle first.")
        if sid in model.JOINT_IDS:
            calib = model.load_calibration()
            now = arm.read_positions(model.JOINT_IDS)
            for t in (a, b):
                why = model.check_tick_move(calib, now, [t if s == sid else None for s in model.JOINT_IDS])
                if why:
                    sys.exit(f"refusing: {why}")
        if not args.yes and input(f"Servo {sid} will turn ±45° from {start} and come back. Go? [y/N] ").lower() != "y":
            return

        arm.sync_torque([sid], True)
        d = b - a
        try:
            timed_move(arm, sid, a, 400, 50)
            settle(arm, sid)
            # 1) near-instant ramps (max accel) -> the time is almost all cruising
            t1 = timed_move(arm, sid, b, args.speed, 254)
            v = (d - TOL) / t1                             # steps/s actually achieved
            timed_move(arm, sid, a, 400, 50)
            settle(arm, sid)
            # 2) slow ramps: t = d/v + v/acc for a trapezoid that reaches cruise speed
            acc_reg = 5
            t2 = timed_move(arm, sid, b, args.speed, acc_reg)
            acc = solve_accel(d, v, t2)
        finally:
            timed_move(arm, sid, start, 400, 50)

    speed_unit = v / args.speed
    acc_unit = acc / acc_reg
    print(f"\nspeed register {args.speed} -> {v:.0f} steps/s   => speed_unit = {speed_unit:.3f} steps/s per unit"
          f"   (STS nominal: 1.0)")
    print(f"accel register {acc_reg}   -> {acc:.0f} steps/s²  => acc_unit   = {acc_unit:.1f} steps/s² per unit"
          f"   (STS nominal: 100)")
    if not 0.8 < speed_unit < 1.25 or not 70 < acc_unit < 140:
        print("These differ noticeably from the STS defaults; the IK link will use the measured values if saved.")
    if args.write:
        c = model.load_calibration()
        c["speed_unit"], c["acc_unit"] = round(speed_unit, 4), round(acc_unit, 2)
        model.save_calibration(c)
        print(f"saved to {model.CALIB_FILE}")


if __name__ == "__main__":
    main()
