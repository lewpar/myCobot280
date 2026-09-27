"""The backend's IK sessions (ik.Session, as /ws/ik serves them) over stdin/stdout, one JSON message per line,
so the jsdom smoke test's fake backend solves with the real solver. Each message carries "_s" (which session);
the reply carries it back."""
import json
import os
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path[:0] = [ROOT, os.path.join(ROOT, "src", "backend")]
import arm_model as model  # noqa: E402
import ik  # noqa: E402

sessions = {}
try:
    for line in sys.stdin:
        msg = json.loads(line)
        sid = msg.pop("_s", 0)
        if sid not in sessions:
            sessions[sid] = ik.Session(json.loads(json.dumps(model.DEFAULT_CALIB)))
        reply = sessions[sid].handle(msg, time.monotonic())
        sys.stdout.write(json.dumps({"_s": sid, **reply}) + "\n")
        sys.stdout.flush()
except (BrokenPipeError, KeyboardInterrupt):   # the test finished and closed the pipe
    sys.stderr.close()
