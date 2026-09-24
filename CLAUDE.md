# CLAUDE.md

Context for working on this repo with Claude Code. Read this before changing anything that can move the arm.

## What this is

Control software for a **myCobot 280 Pi** (six-axis desk arm, Raspberry Pi 4 in the base) that we drive
**directly over the Feetech servo bus**, bypassing Elephant Robotics' firmware and pymycobot API.
The **FastAPI backend** is the only interface to the arm:

- **FastAPI backend** (`src/backend/`), REST API + a WebSocket IK link + serves the IK simulator page.
- **IK simulator** (`src/backend/static/ik_sim.html`, served at `/sim`), a Three.js page that solves
  inverse kinematics, simulates the servos, and streams joint angles to the real arm.

## Hardware facts

- Bus: Pi 4 UART `/dev/ttyAMA0`, **1,000,000 baud**, half-duplex TTL through the base board's buffers.
- Servos: Feetech **STS-style**, IDs **1–6** (J1 base … J6 wrist roll/flange). 0–4095 ticks per turn
  (4096/360 ticks per degree), **little-endian words, bit 15 = sign** on position reads.
- ATOM (ESP32 in the end effector, 5×5 LEDs) is **reflashed** with `atom_led_matrix/atom_led_matrix.ino`
  to act as Feetech **ID 7** on the same bus. Pins: bus RX GPIO 19, TX GPIO 22, LEDs GPIO 27.
  It answers WRITE-style "pings" (addr 0), not the PING instruction, and ignores broadcast (0xFE).
- EEPROM angle limits live at registers 9/11. J6 reports `0,0` (treated as full range with the buffer).

Register map used (STS): 9/11 min/max limit, 31 position correction, 40 torque enable, 41 acceleration
(unit 100 steps/s²), 42–43 goal position, 44–45 goal time, 46–47 goal speed (steps/s), 55 lock,
56–57 present position. Instructions: PING 0x01, READ 0x02, WRITE 0x03, SYNC WRITE 0x83 (to 0xFE).

## Repo map

| Path | Role |
|------|------|
| `mycobot280.py` | Servo/ATOM library: packet building, echo-tolerant checksum-verified reads, bus lock, limits, `move`, `sync_move`, `move_all`, `hold`, `read_positions`, `sync_torque` |
| `arm_model.py` | **Shared model**: URDF kinematics (`fk`), collision checks (`check_pose`, `check_path`, `check_tick_move`), calibration store (`ik_calibration.json`), home store (`center_positions.json`) |
| `src/backend/main.py` | FastAPI app: password middleware, validated REST endpoints, motion guard, `/api/stop` `/api/resume`, `/ws/arm`, `/sim` |
| `src/backend/ik_link.py` | Bus loop behind `/ws/arm`: streams goals, owns the **stop state** used by REST, runs **playback**, and the **stall guard** |
| `src/backend/player.py` | Playback timing (phases, per-joint speeds, LED cues) and the up-front path check; pure logic, ticked by `ik_link` |
| `src/backend/library.py` | Recordings, sequences, saved poses: validation + one JSON file each in `recordings/`, `sequences/`, `poses/` (gitignored) |
| `src/backend/static/ik_sim.html` | The simulator page (single self-contained file, Three.js r147 UMD from jsDelivr) |
| `atom_led_matrix/atom_led_matrix.ino` | ATOM firmware (frame parser in `feed_byte`) |
| `tools/check_servo_units.py` | Times moves to measure real speed/accel register units; `--write` saves them |
| `tools/diagnostics/` | Old bring-up scripts, not used by anything |
| `tests/` | pytest suite on a fake bus (`fakebus.py`), plus `tests/js/` node tests for the page (parity + jsdom smoke) |
| `run_tests.sh` | Runs every test: `./run_tests.sh [pytest args]` |
| `run.sh` | Starts the backend: `./run.sh [--port /dev/ttyX] [--host A] [--http-port N] [--dev]`; sets up `venv/`, pre-flight checks, `exec`s uvicorn |

## Running

```bash
pip install -r src/backend/requirements.txt      # backend (fastapi, uvicorn, pyserial, dotenv)
cp src/backend/.env.example src/backend/.env      # set MYCOBOT_PASSWORD
./run.sh                                          # API :8000, simulator at http://<pi>:8000/sim
```

`./run.sh --dev` (or `MYCOBOT_DEV=1`) turns uvicorn `--reload` on (off by default on purpose; it excludes `venv/`).
The serial port comes from `--port`, then `$MYCOBOT_PORT`, then `src/backend/.env`; run.sh never prompts.
Env vars: `MYCOBOT_PORT`, `MYCOBOT_BAUD`, `MYCOBOT_PASSWORD`, `MYCOBOT_CORS_ORIGINS`.

## Invariants: don't break these

1. **One program owns the serial port.** The library opens it with `exclusive=True`; a second opener
   gets a `RuntimeError`. Never work around this. Stop the backend before running tools.
2. **Hold `_bus._lock` per transaction, never across a wait.** Long operations (move polling, IK loop)
   take the lock for each packet so REST, the IK stream and the ATOM can interleave.
3. **Every motion path goes through the guards**: stop state → collision check (`arm_model`) → clamp to
   EEPROM limits minus 50 ticks → register-range clamp. That applies to REST moves, Home All, IK goals
   and playback (which sends every goal through `_send_goal`). New motion code must do the same (see
   `_guard_motion` in `main.py`, `_send_goal` in `ik_link.py`). REST moves are refused (409) during playback.
4. **Three things exist twice** and must stay identical, enforced by `tests/test_page.py`:
   the kinematics and collision model (`arm_model.py`: `URDF_JOINTS`, `URDF_LIMITS_DEG`, the collision
   constants, `check_pose` ↔ `ik_sim.html`: `JOINTS`, `COLLISION`, `checkPose`), the player
   (`player.py` `Playback` ↔ `ik_sim.html` `Player`, used for offline playback) and the recording
   pre-check (`check_frames`/`check_steps` ↔ `checkFrames`/`checkSteps`). Change both, run the tests.
5. **Speed 0 and acceleration 0 mean "unlimited" on STS servos.** Never send them. Valid: speed 1–4000,
   accel 1–254 (`SPEED_*`, `ACCEL_*` in `mycobot280.py`). The API rejects out-of-range values (422).
6. **Password on everything.** REST: header `X-Arm-Password`. `/ws/arm`: first message
   `{"type":"auth","password":...}` (browsers can't set WS headers).
   Compare with `hmac.compare_digest`. Failed attempts are rate-limited (5/min per IP → 429).
7. **Re-enabling torque must first set goal = present position** (see `IKLink._set_torque`), or the arm
   jumps to whatever stale goal the servo holds.
8. **After any calibration change or resume, the page re-reads the real pose before sending goals**, and
   the backend ignores goals for 0.5 s (`_calib_changed`). Keep both halves of that handshake. The same
   applies when a backend playback ends (`_end_play_locked` sets `_calib_changed`; the page adopts the pose).
9. **Anything outside the IK loop that changes servo goals calls `link.forget_goal()`** (REST moves go
   through `_guard_motion`, which does it; torque endpoints do it). Otherwise the stall guard compares
   the arm against a goal it no longer has and stops it for no reason.

## Kinematics and calibration

- Base frame: metres, Z up, origin at the base. Joint angles in degrees in the URDF convention
  (from Elephant's `mycobot_280_pi` URDF). Zero pose: arm straight up, flange facing +X.
- `ik_calibration.json`: `zero` (tick per joint at the URDF zero), `dir` (±1), `tool_mm`,
  `speed_unit`, `acc_unit`, `calibrated`. Created by the sim's "Set zero" / "Reverse" controls.
  Until it exists, zeros are seeded from `center_positions.json` and `calibrated` is false.
- ticks = zero + dir × deg × 4096/360 (`arm_model.deg_to_ticks` / `ticks_to_deg`).
- `center_positions.json`: raw-tick "home" per servo (`/api/servos/home`, `/api/servos/center_all`).
  Both JSON files are per-machine; `ik_calibration.json` is gitignored.

## Collision model (`arm_model.check_pose`)

Sphere-ish points on J3, forearm midpoint, J4, J5, J6, flange (+ tool midpoint) checked against:
the table (`FLOOR_MARGIN` 5 mm, tool tip `TCP_MIN_Z` 3 mm), the base cylinder (r 75 mm, top 120 mm),
the shoulder column for wrist points (r 50 mm, top 190 mm), and wrist-to-upper-arm distance ≥ 50 mm.
`check_path` samples 16 poses along a **straight joint-space** path (an approximation of what the
servos do). If the start pose already collides, only the target is checked so you can move out.
It's deliberately conservative and approximate; it is not a substitute for watching the arm.

## Simulator page internals (`ik_sim.html`)

- Layout: top bar (link chip, theme toggle, Stop), 3D viewport (camera presets, legend), tabbed inspector
  (Motion / Joints / Robot / ATOM / Record / Play) and a status bar. The script finds everything by element id, so keep the
  ids when moving markup around. The canvas sizes to `#stage` (ResizeObserver), not the window.
- IK: damped least squares on the geometric Jacobian, **task priority** (position first, "flange
  facing down" in the null space), step scaled uniformly, joint limits clamped, and `ikRescue`
  restarts from seeded poses when stuck or colliding (collisions add a 1e6 score penalty).
- `qIK` = solver output; `qCmd` = what the servos are told. **Only collision-free poses and paths are
  copied from qIK to qCmd.** The simulated servos use a trapezoidal velocity profile toward qCmd.
- Hand-guide mode: torque off, qIK/qCmd follow the measured pose. Stop: freeze qCmd, send `stop`.
- ATOM LED panel: talks to `/api/atom/*` over REST (not the WebSocket), with the backend host from
  the WS address field and the password field. Requests go one at a time; a 401/429 drops the rest of the
  queue so a drag can't trip the lockout. The 3D ATOM's LEDs mirror the panel (index row×5+x, seen from behind).
- Record tab: samples `measured` (or the sim servos when offline) at 10 Hz into `[t, deg x6]` frames and
  logs ATOM panel changes as LED cues (`recEvent`), trims still ends, saves via `/api/recordings`.
  Waypoints build a minimum-jerk recording through added poses (`wpFrames`).
- Play tab: lists recordings/sequences, draws the selected path (`pathLine`, violet) and pre-checks it
  (Play is disabled if it collides). **Connected**: Play posts `/api/playback`; while the state's
  `playback` is set (`remotePlay`) the page follows the measured pose like hand-guide and sends no goals;
  moving the target sends `/api/playback/stop`. **Offline**: `Player` ticks in the frame loop, writes goals
  into **qIK** (so collision check → qCmd applies) and per-joint speeds into `simSpeeds` for the sim servos.
  Local playback ends on Stop, hand-guide, a blocked pose, a resync, or when `homeLock` is cleared.
- Motion tab also has Jog (tool X/Y/Z moves the target; joint jog moves qIK with `homeLock`) and saved poses
  (`goPose`). Robot tab has the stall-guard toggle; a `fault` from the backend shows in the status bar.
- The page auto-fills `ws://<host>/ws/arm` when served from `/sim`. It stores the password in
  sessionStorage (localStorage only if "remember" is ticked).
- A copy was also published as a claude.ai artifact; **the repo file is the source of truth**.

## Protocols

**`/ws/arm` messages.** Page → backend: `auth`, `goal {angles[6] deg, speed deg/s, acc deg/s²}`,
`torque {on}`, `stop`, `resume`, `set_zero`, `set_dir {joint 0-5, dir ±1}`, `set_tool {mm 0-150}`,
`set_stall_guard {on}`. Goals are ignored while a playback runs.
Backend → page (~10 Hz): `state {angles[6]|null, torque, stopped, blocked, calibrated, zero, dir,
tool_mm, limits[6][lo,hi], fault, stall_guard, playback{name, recording, step, steps, phase, t, duration,
loop, rate, timed}|null, play_end{n, message}}`, or `error {code: auth|locked|no_arm, message}`.

**REST** (all under `/api`, all need the header): `auth`, `health`, `safety`, `stop`, `resume`,
`servos[?rescan=true]`, `servos/status`, `servos/home` (GET/POST), `servos/center_all`,
`servos/torque_all`, `servo/{id}` and `/move`, `/move_rel`, `/center`, `/torque`, `/ping`,
`atom/{color,pixel,brightness,ping,state}`,
`recordings` (GET list / POST `{name, frames[[t, deg x6]], events[[t, color|pixel|brightness, [ints]]], return_zero}`),
`recordings/{id}` (GET / PATCH `{name?, return_zero?, trim?[start, end]}` / DELETE, 409 if a sequence uses it),
`sequences` (GET / POST `{name, steps[{recording, pause}]}`), `sequences/{id}` (GET/PUT/DELETE),
`poses` (GET / POST `{name, angles}`), `poses/{id}` (DELETE),
`playback` (GET status / POST `{recording|sequence, rate 0.25-4, loop, timed, speed 1-150, acc}`), `playback/stop`.
Refusals: 409 collision (or playback running), 423 stopped, 422 bad values, 404 unknown id.
ATOM writes return `acked` (false = no reply; the flashed firmware may predate the reply-on-write parser).

## Testing without the arm

`./run_tests.sh` runs everything (about a minute and a half; installs `tests/requirements.txt` into `venv/`
and `tests/js` npm packages on first run). `./run_tests.sh -k playback -x` passes args to pytest.

- `tests/fakebus.py`: a drop-in `serial.Serial` emulating STS servos 1–6 (register file, READ/WRITE/PING/
  SYNC WRITE, trapezoidal motion from the speed/accel registers, asserts speed/accel are never 0), the
  ATOM on ID 7, optional TX echo, and hooks (`servos[id].pos` to move a limp joint, `servos[id].stop_at`
  for an obstruction). `conftest.py` injects it and points every data file at a temp dir.
- `test_motion_api.py`, `test_ws.py` (auth, goals, stop/resume handshake, torque-on hold, stall guard),
  `test_library.py`, `test_player.py` (timing with a simulated clock), `test_playback.py` (on the fake arm).
- `test_page.py` runs node: `checkPose` vs `check_pose` on 10,000 poses (0 mismatches), `Player` vs
  `Playback` goal-for-goal, and `tests/js/smoke.js` (jsdom, fake WebGL/WebSocket/backend) through record,
  save, edit, trim, import/export, waypoints, sequences, local and backend playback, poses, jog, faults.
- ATOM parser (not in the suite yet): extract the `// ---- Frame parser` … `// ---- end frame parser`
  block and compile it on the host with g++ and a small harness.

## Working with the real arm

- **Ask before running anything that opens the serial port or moves the arm.** Prefer testing against
  the fake bus first.
- First moves after a change: Max speed ~30°/s, targets a few cm away, clear of the table, hand on Esc.
- Hand-guide mode drops torque; the arm falls unless someone is holding it.
- Current status: the user has the backend running on the real arm and has started calibrating.

## Not verified on hardware yet

- Speed/accel register units (run `tools/check_servo_units.py --write`).
- URDF link lengths vs this arm (measure flange position at a few targets; see README).
- The collision margins against the real housings.
- The ATOM firmware change (parser rewrite) has only been host-tested, not flashed.
- Playback with per-joint speeds (timed mode), and the stall guard thresholds (`STALL_DEG` 6°, `STALL_S`
  1 s in `ik_link.py`) against real load: gravity sag must stay under 6° or it will false-trip.

## Known limitations / ideas

- REST moves work in raw ticks and can fight the IK stream if both are used at once.
- Plain HTTP: the password stops casual use, not traffic capture. HTTPS would need a reverse proxy.
- The collision path check assumes straight joint-space motion; real servos finish at different times.
- LED cues played on the arm change the real ATOM, but the page's LED panel doesn't mirror them (press Read back).
- Backend playback keeps running with no page open (by design); stop it with Stop, `/api/stop` or `/api/playback/stop`.
