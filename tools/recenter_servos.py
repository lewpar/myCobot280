#!/usr/bin/env python3
"""
recenter_servos.py — move each servo's 0/4095 point out of the way of its joint.

Every servo counts 0-4095 over one turn. If a joint's zero pose sits near 0 or 4095, part of its travel is
on the other side of that point: the joint then reads half a turn away and the servo drives it the wrong way
round, into the arm. Re-centring makes the servo read 2048 where it is now (Feetech's own "calibrate the
middle", stored in its EEPROM), so with the arm in its zero pose every joint's 0/4095 point is half a turn
away, behind it.

    python3 tools/recenter_servos.py               # show where each servo's centre is (reads only)
    python3 tools/recenter_servos.py --write       # re-centre them (asks first; you hold the arm)
    python3 tools/recenter_servos.py --write --joints 4,5
    python3 tools/recenter_servos.py --undo        # put back the corrections from before the last re-centring

The calibration (ik_calibration.json) and the saved home positions shift with it, so angles keep their
meaning; with --write it also sets the zero to this pose (the arm straight up, flange facing +X), unless
--keep-zero. Each re-centring is logged to servo_centres_log.json with the values --undo needs.

Stop the backend first (only one program can hold the serial port). The Calibration wizard in the sim
(Setup tab) does the same through the backend.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import arm_model as model  # noqa: E402

PORT = os.environ.get("MYCOBOT_PORT", "/dev/ttyAMA0")
FAR_DEG = 60     # a zero this far from the servo's centre has its 0/4095 point within reach


def table(arm, calib, out):
    ticks = arm.read_positions(model.JOINT_IDS)
    out(f"{'joint':6} {'reads':>6} {'zero':>6} {'zero from centre':>17} {'correction':>11}")
    for j, sid in enumerate(model.JOINT_IDS):
        z = calib["zero"][j]
        off = (z - 2048) / model.TICKS_PER_DEG
        corr = arm.position_correction(sid)
        warn = "  <- its 0/4095 point is inside the joint's travel" if abs(off) > FAR_DEG else ""
        out(f"J{j + 1:<5} {str(ticks[j]):>6} {z:>6} {off:>16.0f}° {str(corr):>11}{warn}")
    return ticks


def hold_and_torque_on(arm, ids):
    """Torque back on without a jump: goal = where each servo is now, first."""
    now = arm.read_positions(ids)
    arm.sync_move({sid: p for sid, p in zip(ids, now) if p is not None}, 200, 10)
    arm.sync_torque(ids, True)


def run(args, arm, ask=input, out=print):
    calib = model.load_calibration()
    if args.undo:
        return undo(arm, calib, ask, out)
    joints = sorted({int(j) - 1 for j in args.joints.split(",")}) if args.joints else list(range(6))
    if any(not 0 <= j <= 5 for j in joints):
        out("Joints are 1-6.")
        return 2
    table(arm, calib, out)
    if not args.write:
        out("\nNothing written. Run with --write to re-centre (you'll be asked first).")
        return 0
    ids = [model.JOINT_IDS[j] for j in joints]
    out(f"\nTorque goes OFF on {', '.join(f'J{j + 1}' for j in joints)}: the arm will drop unless you hold it.")
    if ask("Holding the arm? Type yes: ").strip().lower() != "yes":
        out("Nothing done.")
        return 1
    arm.sync_torque(ids, False)
    out("Pose the arm at its zero: straight up, the flange facing +X (away from the Pi's ports). Turn any joint")
    out("that's folded back past where it should be the short way round, by hand.")
    if ask("In the zero pose? Type yes to re-centre: ").strip().lower() != "yes":
        hold_and_torque_on(arm, ids)
        out("Nothing written; torque is back on.")
        return 1
    results = model.recenter(arm, calib, joints)
    for r in results:
        out(f"J{r['joint'] + 1}: " + (f"{r['before']} -> {r['after']} (correction {r['correction_before']} -> "
                                      f"{r['correction_after']})" if r["ok"] else r["message"]))
    if not args.keep_zero:
        now = arm.read_positions(model.JOINT_IDS)
        if all(p is not None for p in now):
            calib["zero"] = list(now)
            calib["calibrated"] = True
            out("Zero set to this pose.")
    model.save_calibration(calib)
    hold_and_torque_on(arm, ids)
    out("Torque is back on. Start the backend and check the sim shows the arm straight up.")
    return 0 if all(r["ok"] for r in results) else 1


def undo(arm, calib, ask, out):
    try:
        with open(model.RECENTER_LOG) as f:
            log = json.load(f)
    except (OSError, ValueError):
        out("No re-centring logged.")
        return 1
    done = [r for r in log[-1]["results"] if r["ok"]]
    if not done:
        out("The last re-centring changed nothing.")
        return 1
    out("Puts back: " + ", ".join(f"J{r['joint'] + 1} correction {r['correction_after']} -> {r['correction_before']}"
                                  for r in done))
    if ask("Torque goes off on those joints while it's written: hold the arm, then type yes: ").strip().lower() != "yes":
        out("Nothing done.")
        return 1
    ids = [r["id"] for r in done]
    arm.sync_torque(ids, False)
    centers = model.load_centers()
    for r in done:
        now = arm.read_positions([r["id"]])[0]
        if not arm.set_position_correction(r["id"], r["correction_before"]):
            out(f"J{r['joint'] + 1} didn't answer.")
            continue
        d = arm.read_positions([r["id"]])[0] - now
        calib["zero"][r["joint"]] = (calib["zero"][r["joint"]] + d) % 4096
        if r["id"] in centers:
            centers[r["id"]] = (centers[r["id"]] + d) % 4096
        out(f"J{r['joint'] + 1}: correction {arm.position_correction(r['id'])}")
    model.save_calibration(calib)
    model.save_centers(centers)
    log.pop()
    with open(model.RECENTER_LOG, "w") as f:
        json.dump(log, f, indent=2)
    hold_and_torque_on(arm, ids)
    out("Done; torque is back on.")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--write", action="store_true", help="re-centre (asks first)")
    ap.add_argument("--joints", help="which joints, e.g. 4,5 (default: all six)")
    ap.add_argument("--keep-zero", action="store_true", help="don't set the zero to this pose")
    ap.add_argument("--undo", action="store_true", help="put back the corrections from before the last re-centring")
    ap.add_argument("--port", default=PORT)
    args = ap.parse_args(argv)
    from mycobot280 import MyCobot280
    arm = MyCobot280(args.port)
    try:
        return run(args, arm)
    finally:
        arm.close()


if __name__ == "__main__":
    sys.exit(main())
