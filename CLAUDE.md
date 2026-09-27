# CLAUDE.md

Context for working on this repo with Claude Code. Read this before changing anything that can move the arm.

## What this is

Control software for a **myCobot 280 Pi** (six-axis desk arm, Raspberry Pi 4 in the base) that we drive
**directly over the Feetech servo bus**, bypassing Elephant Robotics' firmware and pymycobot API.
The **FastAPI backend** is the only interface to the arm:

- **FastAPI backend** (`src/backend/`), REST API + two WebSockets (`/ws/arm` drives the arm, `/ws/ik` only
  solves) + serves the IK simulator page. **The inverse kinematics lives here only** (`ik.py`).
- **IK simulator** (`src/backend/static/sim/`, served at `/sim/`), a Three.js page that sends targets to the
  backend, shows its solutions, simulates the servos, and drives the real arm through `/ws/arm`.

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
| `src/backend/main.py` | FastAPI app: password middleware, validated REST endpoints, motion guard, `/api/stop` `/api/resume`, `/ws/arm`, `/ws/ik`, `/sim` |
| `src/backend/ik.py` | **The IK**: `Solver` (the native solver, IKPy or Pink for the numbers, `MYCOBOT_IK`; wrapped in collision-clear stepping, seeded restarts and `plan_move` routes), `parse_target`, and `Session` (one `/ws/ik` connection); pure logic |
| `src/backend/ik_link.py` | Bus loop behind `/ws/arm`: streams goals, solves an active **target** on its own thread, owns the **stop state** used by REST, runs **playback**, and the **stall guard** |
| `src/backend/player.py` | Playback timing (phases, per-joint speeds, LED cues) and the up-front path check; pure logic, ticked by `ik_link` |
| `src/backend/library.py` | Recordings, sequences, saved poses: validation + one JSON file each in `recordings/`, `sequences/`, `poses/` (gitignored) |
| `src/backend/static/sim/` | The simulator page: `index.html`, `style.css`, native ES modules in `js/` (no build step; three.js r147 from jsDelivr via an import map) |
| `atom_led_matrix/atom_led_matrix.ino` | ATOM firmware (frame parser in `feed_byte`) |
| `tools/check_servo_units.py` | Times moves to measure real speed/accel register units; `--write` saves them |
| `tools/recenter_servos.py` | Re-centres servos (EEPROM) so their wrap point is behind the joint; `--undo`; backend stopped |
| `tools/diagnostics/` | Old bring-up scripts, not used by anything |
| `tests/` | pytest suite on a fake bus (`fakebus.py`), plus `tests/js/` node tests for the page (parity + jsdom smoke) |
| `WEBSOCKET.md` | Reference for `/ws/arm` and `/ws/ik` (protocol 3) |
| `run_tests.sh` | Runs every test: `./run_tests.sh [pytest args]` |
| `run.sh` | Starts the backend: `./run.sh [--ik native\|ikpy\|pink] [--password PW] [--no-prompt] [--port /dev/ttyX] [--host A] [--http-port N] [--dev]`; sets up `venv/`, asks for the solver and password in a terminal, pre-flight checks, `exec`s uvicorn |

## Running

```bash
pip install -r src/backend/requirements.txt      # backend (fastapi, uvicorn, pyserial, dotenv, ikpy + numpy/scipy)
cp src/backend/.env.example src/backend/.env      # optional: a saved password, the serial port
./run.sh                                          # asks for the solver and password; API :8000, /sim
```

`./run.sh --dev` (or `MYCOBOT_DEV=1`) turns uvicorn `--reload` on (off by default on purpose; it excludes `venv/`).
The serial port comes from `--port`, then `$MYCOBOT_PORT`, then `src/backend/.env`. In a terminal run.sh asks which
IK solver to use and the password for the session (Enter keeps the saved one, or with none saved the backend makes
one up); `--ik`, `--password` or `--no-prompt` skip the questions, and without a terminal it never asks. The answers
reach the backend as `MYCOBOT_IK`/`MYCOBOT_PASSWORD` in its environment (never on its command line), which win over
`.env` (`load_dotenv` doesn't override).
Env vars: `MYCOBOT_PORT`, `MYCOBOT_BAUD`, `MYCOBOT_PASSWORD`, `MYCOBOT_CORS_ORIGINS`, `MYCOBOT_IK` (`native`, the
default, `ikpy` or `pink`; shown in `/api/health` and `/ws/ik`'s settings).

## Invariants: don't break these

1. **One program owns the serial port.** The library opens it with `exclusive=True`; a second opener
   gets a `RuntimeError`. Never work around this. Stop the backend before running tools.
2. **Hold `_bus._lock` per transaction, never across a wait.** Long operations (move polling, IK loop)
   take the lock for each packet so REST, the IK stream and the ATOM can interleave.
3. **Every motion path goes through the guards**: stop state → range guard (no joint reading more than 10°
   outside its limits, `IKLink._range_problem`) → collision check (`arm_model`) → clamp to EEPROM limits minus
   50 ticks and never within 228 ticks (20°) of the servo's 0/4095 wrap point (`_SEAM_MARGIN`) → register-range clamp. That applies to REST moves, Home All, IK goals
   and playback (which sends every goal through `_send_goal`). New motion code must do the same (see
   `_guard_motion` in `main.py`, `_send_goal` in `ik_link.py`; a `target`'s solver thread only queues goals
   for the bus loop, which sends them through `_send_goal`). REST moves are refused (409) during playback.
4. **Three things exist twice** and must stay identical, enforced by `tests/test_page.py`:
   the kinematics and collision model (`arm_model.py`: `URDF_JOINTS`, `URDF_LIMITS_DEG`, the collision
   constants, `check_pose` ↔ `sim/js/kinematics.js` `JOINTS` and `sim/js/collision.js` `COLLISION`,
   `checkPose`), the player (`player.py` `Playback` ↔ `sim/js/player.js` `Player`, used for offline playback),
   the recording pre-check (`check_frames`/`check_steps` ↔ `collision.js` `checkFrames`/`checkSteps`), the
   attachment list (`arm_model.ATTACHMENTS` ↔ `kinematics.js` `ATTACHMENTS`), and the work area
   (`DEFAULT_AREA`, `_outside_area` ↔ `collision.js` `DEFAULT_AREA`, `outsideArea`). Change both, run the tests.
5. **Speed 0 and acceleration 0 mean "unlimited" on STS servos.** Never send them. Valid: speed 1–4000,
   accel 1–254 (`SPEED_*`, `ACCEL_*` in `mycobot280.py`). The API rejects out-of-range values (422).
6. **Password on everything.** REST: header `X-Arm-Password`. `/ws/arm` and `/ws/ik`: first message
   `{"type":"auth","password":...}` (browsers can't set WS headers).
   Compare with `hmac.compare_digest`. Failed attempts are rate-limited (5/min per IP → 429).
7. **Re-enabling torque must first set goal = present position** (see `IKLink._set_torque`), or the arm
   jumps to whatever stale goal the servo holds.
8. **After any calibration change or resume, the page re-reads the real pose before sending goals.** The
   backend enforces it with an **epoch** (`IKLink._new_epoch_locked`): it goes up on resume, `set_zero`,
   `set_dir` and the end of a backend playback, goals and targets carrying an older epoch are refused, and an
   active target ends. The page takes
   the new epoch only in the state it adopts the pose from. Anything new that re-maps angles or moves the arm
   behind the page's back must bump the epoch too.
9. **Anything outside the IK loop that changes servo goals calls `link.forget_goal()`** (REST moves go
   through `_guard_motion`, which does it; torque endpoints do it). Otherwise the stall guard compares
   the arm against a goal it no longer has and stops it for no reason.
10. **Never move a joint that reads out of range, and never change a servo's reading with torque on.** A joint
   past its servo's wrap point reads half a turn away; driving it "back" turns it the wrong way, into the arm
   (this happened on the real arm: J4 zeroed at tick 573, powered up at 3063). `recenter` (EEPROM position
   correction) is refused unless torque is off, because the goal register keeps its number and would jump.

## Kinematics and calibration

- Base frame: metres, Z up, origin at the base. Joint angles in degrees in the URDF convention
  (from Elephant's `mycobot_280_pi` URDF). Zero pose: arm straight up, flange facing +X.
- `ik_calibration.json`: `zero` (tick per joint at the URDF zero), `dir` (±1), `tool_mm`, `tool_d_mm`,
  `attachment` (`none`/`vacuum`/`custom`), `area` (work area), `speed_unit`, `acc_unit`, `calibrated`. Created by the sim's "Set zero" / "Reverse" controls.
  Until it exists, zeros are seeded from `center_positions.json` and `calibrated` is false.
- ticks = zero + dir × deg × 4096/360 (`arm_model.deg_to_ticks` / `ticks_to_deg`).
- `center_positions.json`: raw-tick "home" per servo (`/api/servos/home`, `/api/servos/center_all`).
  Both JSON files are per-machine; `ik_calibration.json` is gitignored.

## Collision model (`arm_model.check_pose`)

Sphere-ish points on J3, forearm midpoint, J4, J5, J6, flange (+ tool midpoint) checked against:
the table (`FLOOR_MARGIN` 5 mm, tool tip `TCP_MIN_Z` 3 mm), the base cylinder (r 75 mm, top 120 mm),
the shoulder column for wrist points (r 50 mm, top 190 mm), and wrist-to-upper-arm distance ≥ 50 mm.
The **attachment** is a cylinder (`tool_mm` long, `tool_d_mm` wide) along the flange normal, sampled every
15 mm (`TOOL_STEP`). Its points count as wrist points with the tube's radius, plus: against the table only
its lowest cross-section point counts (full radius when level, 0 when vertical, so a downward tip may
touch down to `TCP_MIN_Z`), and they must clear the upper arm (J2–J3, `UPPER_ARM_R` + r) and forearm
(J3–J4, `FOREARM_R` + r). The **ATOM head** (behind the J5 body, on the J6 axis) is two spheres
(`ATOM_SPHERES`: 50 mm / r 17 and 66 mm / r 16 behind the flange face), checked like wrist points plus the forearm.
The forearm check uses the kinematic J3–J4 line; the 3D model's forearm tube sits up to ~30 mm off it, which
is one reason close-in folded poses are kept out by `base_mm` instead.

The **work area** (`area`: `enabled`, `center` deg, `span` deg, `radius_mm`, `base_mm`; default the front
half, center 0°, span 180°, base keep-out 150 mm) is enforced in `check_pose` too, but **only for the tool tip** (the TCP: flange
centre or attachment tip); the rest of the arm may cross the edges (`_outside_area`). A tip within
`AREA_CORE` (60 mm) of the base axis is always inside, except that **`base_mm`** keeps the tip out of a
cylinder that radius around the base axis up to `BASE_KEEPOUT_TOP` (250 mm): reaching in that close and low
folds the wrist back onto the arm (the ATOM into the column). It stops below 250 mm so the zero pose and the
raised poses routes go through stay allowed. Saved areas without `base_mm` get the default. 0° is +X, where the flange points at the zero pose
(away from the Pi's ports); −90° is −Y, the arm's right. Everything that checks collisions passes
`tool_m`, `tool_r` and `area`. The test fixture turns the area off; `test_work_area.py` uses the default.
`check_path` samples a **straight joint-space** path: at least 16 poses, and one every 3° of the joint that
moves most. The servos are made to follow that line: `ik_link._send_goal` splits a goal's speed between the
joints by how far each goes (the acceleration stays the same for all, so stopping stays quick), and the page's
simulated servos do the same. If the start pose already collides, only the target is checked so you can move out.
It's deliberately conservative and approximate; it is not a substitute for watching the arm.

## Simulator page internals (`static/sim/`)

- No framework, no build step: `index.html` has the markup and an import map (`three`, `three/addons/`
  → jsDelivr, r147); `js/main.js` is the entry point. Modules, by job:
  `kinematics.js` (`JOINTS`, `fk`, `LIM`, attachment size), `collision.js` (`checkPose`, `checkPath`, work area,
  recording pre-check), `player.js` (`Player`): pure, imported directly by the parity tests. `solve.js` is the
  client of the backend's solver (`/ws/ik`, or the arm's `state.ik` while driving); there is no IK in the page.
  `wizard.js` is the calibration wizard (below).
  `scene.js` (every three.js object; throws if WebGL fails, which shows `#fail`), `state.js` (shared state),
  `api.js` (REST + password), `link.js` (`/ws/arm`, Stop/Resume, hand-guide, calibration), `settings.js`
  (attachment + work area), `motion.js`, `atom.js`, `record.js`, `play.js` (one per tab), `chrome.js`
  (tabs, foldable cards + help notes, popovers, theme, views, link chip, joint overlay), `main.js` (wiring + frame loop).
- **Modules don't touch the page when imported**; each has an `init*()` that `main.js` calls in order. Keep it
  that way: the modules import each other in cycles, which is only safe because nothing runs at import time.
- **Shared mutable state lives in `S`** (`state.js`: `stopped`, `homeLock`, `measured`, `limp`, `play`,
  `remotePlay`, `ikRestart`, …) because an imported `let` can't be assigned from another module. `toolLen`/`toolR` and
  `area` are live `let` exports changed only through `setToolSize` / `setAreaModel`. Everything else is private
  to its module. `qIK`, `qCmd`, `servo`, `target` and `LIM` are shared arrays/vectors, mutated in place.
- Browsers won't load modules from `file://`: use the backend (or any static server). The backend sends the
  files with `Cache-Control: no-cache`, so an update never mixes old and new modules.
- Layout: top bar (link chip → **connection popover** with address/password/"drive the servos", Hand-guide
  pill shown only while connected, theme toggle, Stop), 3D viewport (camera presets, **View menu** with the
  display toggles: ghost, real arm, trail, envelope, joint overlay; legend; **joint overlay** with the
  per-joint bars), tabbed inspector (Move / Record / Play / ATOM / Setup) and a status bar.
  Cards with `data-fold` fold from their header (remembered in localStorage `mycobot-fold`); a `p.note.help`
  is hidden behind an ⓘ button that `chrome.js` adds. Keep explanations in `.note.help`, not always-on text.
  A folded card with a non-empty `.callout.warn` shows an amber dot. The script finds everything by element id, so keep the
  ids when moving markup around. The canvas sizes to `#stage` (ResizeObserver), not the window.
- IK (backend, `ik.py`), three engines chosen with `MYCOBOT_IK` (`Solver.engine`):
  - **native** (default): damped least squares on the geometric Jacobian, **task priority** (position first,
    "flange facing down" in the null space), `ITERS` small steps per solve clamped to the joint limits;
    `iterate_clear` stops at the last clear step instead of walking into a collision. ~2 ms a solve on a desktop.
  - **ikpy**: IKPy (`make_chain`: the six URDF joints bounded by the joint limits, the attachment as a fixed last
    link; "facing down" is `orientation_mode="Z"`, `IKPY_TOL`; imported only when chosen). It weighs position
    and orientation together, so `Solver.solve` re-solves the position alone when a facing-down answer misses
    and keeps the closer one; `iterate_clear` walks from a clear start toward its answer and stops at the last
    clear pose. ~50 ms a solve on a desktop.
  - **pink**: Pink on Pinocchio, differential IK stepped like native (`_iterate_pink`, one QP per step with daqp)
    on a model built from a generated URDF (`pink_urdf`: `URDF_JOINTS`, the limits, `PINK_VEL` so a step is at
    most 0.1 rad like native, a fixed `tool_tip` frame). A frame task with orientation cost `PINK_ORI`, its
    target turned only so the flange axis points down. ~10 ms a solve. Optional: not in requirements.txt
    (Pinocchio has no 32-bit Arm wheels); run.sh installs `pin-pink daqp` when it's chosen, and its tests skip
    without it.
  When stuck (short of the target, colliding, or with no route from the servos) `rescue` restarts from seeds
  (native and pink: 32 plus two upright ones, 24 random on retries; ikpy: 8 + 2, 6 random), ranked collision-free first,
  then reachable, then closest; up to 6 retries (`Solver.mem`; `configure` with new settings starts over). `plan_move` is straight there
  if that's clear, else through raised poses (J2–J5 at 0: lift, turn the base, come down; also lifting the
  shoulder or straightening the elbow first). `arm_model.chain` is the fast FK the checks use.
  On a desktop a solve while dragging is ~2 ms native / ~10 ms Pink / ~50 ms IKPy, a rescue ~40 ms / ~250 ms /
  ~0.9 s; several times that on the Pi, which is why native is the default. Every answer says
  whether it's **`settled`** (solving again would change nothing), and a target already reached isn't solved
  again at all.
- Where it runs: a `/ws/arm` **target** is re-solved every 50 ms on `IKLink`'s solver thread from the measured
  pose, and the next pose on the route is queued as a goal (re-sent when it changes or every 0.2 s). It skips
  the solve while the last answer is settled and nothing moved (`STILL_DEG`). The page, when not driving, sends
  `/ws/ik` **solve** requests (one in flight; `q` = qIK, `from` = its simulated servos), each answered with the
  solution and the next pose on the route; it doesn't repeat a request whose answer was settled, and
  `ik.Session` answers a repeated settled request from its last reply. `solve.js` reconnects on its own, but never
  retries a password the backend rejected (it would count towards the lockout).
- `qIK` = the backend's solution (taken only if it started from the qIK the page still has); `qCmd` = what the
  servos are told. **Only collision-free poses reach qCmd, along a clear route**: the answer's `next`
  ("going up and around" in the status bar when it's a detour), used only when the answer is for the current
  qIK. When qIK is set directly (`homeLock`: jog, saved pose, zero, playback) the page asks for a route to that
  joint pose (`angles`) instead. Local playback never detours and doesn't ask (it must follow its recording).
  Each frame the rest of the servos' current move is re-checked finely (every 1°) and they hold where they
  are if it's no longer clear. The simulated servos use a trapezoidal velocity profile toward qCmd,
  synchronised like the real ones.
- Driving the arm (`link.armDriving()`: connected, all servos reading, "drive the arm" on, not hand-guide,
  stopped, resyncing or playing): `link.js` sends the Move target as `target` every 100 ms when it changes
  (`xyz` + `down`, or `angles` under `homeLock`) and each state's `ik` becomes the page's answer; it sends no
  `goal`s and no `/ws/ik` solves. A state with `ik: null` (the backend ended the target) makes it send again.
- **Calibration wizard** (`wizard.js`, `#wiz` floating over the 3D view; opened from Setup → Calibration or the
  top bar's Calibrate pill, shown when the arm isn't calibrated or `state.out_of_range` is set): check the link →
  torque off → pose at zero (the sim glides to the zero pose; hovering a joint's hint rings it in 3D) →
  re-centre far-off servos (`recenter`, from `state.ticks`) → `set_zero` → torque on (and Resume) → which way
  each servo turns → summary. The direction check can't be automatic (a reversed servo's reading is reversed
  too, so numbers always agree with the model): by default the arm turns each joint `TEST_DEG` (15°) from where
  it is and back at 15°/s with plain `goal`s (every guard applies) while the model mirrors it with a ring and
  arrow (`scene.setJointFx`), and the user answers same way / opposite way (`set_dir` flips it). Or, by hand:
  torque off, the model wiggles the joint, the user turns it the arrow's way, and a negative reading means
  reversed. While `S.wizard` is set, `wizardFrame` gives the sim arm's pose, `armDriving()` is off (only the
  direction test's goals move the arm), and the target, gizmo and ghost are hidden; the picture is shifted right
  (`scene.setViewShift`) so the arm isn't behind the panel. Closing adopts the measured pose.
- Hand-guide mode, a backend playback, and a joint out of range (`S.armRange`): the sim mirrors the measured
  pose, unclamped (so a fold past the limits shows as it is). Stop: freeze qCmd, send `stop`.
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
- Setup tab, Attachment card: picks `attachment` (`setAttachment`), which sets `toolLen`/`toolR` (the TCP moves to the
  tip, and `checkPose` uses both) and shows its 3D model on the flange (`vacuumG`, or `toolStub` for custom;
  `envelope` draws the collision cylinder). Sent as `set_tool`; on connect the backend's saved attachment wins.
- Setup tab has the Work area card (`setArea`; presets, direction, width, max reach). The slice is drawn on
  the floor (`areaG`); Figure-8 and Random centre themselves in it (`areaDir`). A target outside it says so
  in the status bar (the solver would otherwise report "out of reach"). Sent as `set_area`; on connect
  the backend's saved area wins, like the attachment.
- Move tab also has Jog (tool X/Y/Z moves the target; joint jog moves qIK with `homeLock`) and saved poses
  (`goPose`). Setup tab has the stall-guard toggle (Safety) and Calibration; a `fault` from the backend shows in the status bar.
- The page auto-fills `ws://<host>/ws/arm` when served from `/sim/`. It stores the password in
  sessionStorage (localStorage only if "remember" is ticked).
- An older single-file copy was published as a claude.ai artifact; it no longer matches (protocol 1).
  **The repo is the source of truth.**

## Protocols

**`/ws/arm` (protocol 4; full reference in `WEBSOCKET.md`, short form in the `ik_link.py` docstring).** The page sends `auth` first; the
backend answers `hello {protocol}`, then `config` (now and whenever it changes), then `state` about 10 times a second.
Page → backend: `goal {angles[6] deg, speed 1-360 deg/s (capped at 150), acc 1-2000 deg/s², epoch}` (turns
torque on if it was off; ends a target), `target {xyz[3] mm + down | angles[6], speed, acc, epoch}` (solved and
driven until a goal, stop, torque, playback, new epoch or the last client leaving ends it), `torque {on}`, `stop`, `resume`, `set_zero`, `set_dir {joint 0-5, dir ±1}`,
`set_tool {attachment, mm 0-150, d_mm 1-60}` (a known attachment keeps its own size), `set_stall_guard {on}`,
`set_area {enabled, center -180..180, span 30..360, radius_mm 0|100..450, base_mm 0|60..250}`, `recenter {joints}` (torque
off; answered with `recentered {results}`, the only success reply). Booleans must be JSON booleans and
numbers finite (NaN/Infinity are rejected at the socket).
Backend → page: `config {calibrated, zero, dir, tool_mm, tool_d_mm, attachment, area, limits[6][lo,hi],
stall_guard}`; `state {angles[6]|null, torque, stopped, blocked, fault, playback{name, recording, step, steps,
phase, t, duration, loop, rate, timed}|null, play_end{n, message}, epoch, clients, ik, ticks[6], out_of_range}`, where `ik` is null or
`{target, down, angles, next, detour, reached, pos_err_mm, ori_err_deg, blocked, outside, arrived}`;
`error {code, ref, message}`. Fatal codes (the socket closes): `auth` (4401), `locked` (4429), `no_arm` (4503).
Non-fatal, answering one command (`ref` = its type): `bad_request` (malformed or out of range), `refused`
(valid but not now: stopped, playback running, stale epoch, a servo not answering for `set_zero`),
`internal` (the handler raised). Success has no reply; it shows up in the next config/state.
An exception in the bus loop stops the arm with a `fault` instead of killing the loop.
Several pages may connect (`clients`); their goals and targets simply overwrite each other, last one wins.

**`/ws/ik` (solve-only, no arm needed).** Same auth; then `hello`, `settings`. Every message gets one reply in
order: `settings {tool_mm, tool_d_mm, area, limits}` → `settings`; `solve {id?, xyz + down | angles, q?, from?,
rescue?, restart?}` → `ik {id, …the fields of state.ik except arrived}`; else `error`. One `ik.Session` per
connection; at most 2 solves run at once (`IK_SLOTS`), in threads.

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
- `test_work_area.py` (default front half, only the tip counts, other slices, REST/WS/playback refusals),
  `test_attachments.py` (tool collision rules, choosing one over WS, playback refused with the tool on),
  `test_motion_api.py`, `test_ws.py` (auth and close codes, hello/config/state, goal validation and epochs, error replies, bad input and bus errors not killing the link, stop/resume, torque-on hold, stall guard); `tests/wsclient.py` is the test client,
  `test_library.py`, `test_player.py` (timing with a simulated clock), `test_playback.py` (on the fake arm).
- `test_range_guard.py` (the real incident replayed: nothing moves, the way out by hand; wrap-point
  crossing; `recenter`), `test_recenter_tool.py` (the tool: dry run, write, undo). The fake bus models the
  position correction (`corr`), the EEPROM lock and the 128-to-torque "calibrate the middle".
- `test_ik.py`: the solver as the target loop runs it, on every engine (Pink and IKPy on fewer targets; Pink skipped
  if it isn't installed), and Pink's generated URDF against `arm_model.fk` (fixed targets reached after drags and from arbitrary
  poses, no collisions on the way, the up-and-over detour), `Session` validation and settings, `/ws/ik` with
  and without the arm. `test_ws.py` has the `target` tests (drives the fake arm to a point, joint target with a
  detour, blocked targets, what ends and refuses one).
- `test_page.py` runs node: `checkPose` vs `check_pose` on 10,000 poses (0 mismatches), `Player` vs
  `Playback` goal-for-goal, and `tests/js/smoke.js` (jsdom, fake WebGL/WebSocket/backend) through record,
  save, edit, trim, import/export, waypoints, sequences, local and backend playback, poses, jog, faults, and
  driving the arm by target, the out-of-range state, Hand-guide mirroring and the whole calibration wizard. Its fake `/ws/ik` (and the fake arm's target) run the real solver through
  `tests/js/ik_stdio.py` (`PYTHON` is set by `test_page.py`).
  `tests/js/page.js` bundles the page's modules with esbuild (a test-only dependency) so node and jsdom can
  run them; `tests/js/three-shim.js` swaps in a WebGLRenderer that draws nothing. `test_sim_files.py` checks
  `/sim/` is served (redirect, no password, `no-cache`, modules as JavaScript, every relative import exists).
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
- The collision margins against the real housings, and the vacuum attachment's size and mounting
  (assumed centred on the flange axis, 25 × 80 mm from the flange face to the cup).
- The ATOM firmware change (parser rewrite) has only been host-tested, not flashed.
- Synchronised joint speeds on the real servos (does the arm follow the straight joint-space line?), the
  ATOM head spheres against the real head, and the 150 mm base keep-out against how you actually work.
- Solving on the Pi: how many solves a second the target loop and `/ws/ik` manage with each engine, rescue
  times (IKPy's likely several seconds), and whether solving delays the bus loop (both share the GIL).
- Playback with per-joint speeds (timed mode), and the stall guard thresholds (`STALL_DEG` 6°, `STALL_S`
  1 s in `ik_link.py`) against real load: gravity sag must stay under 6° or it will false-trip.

## Known limitations / ideas

- REST moves work in raw ticks and can fight the IK stream if both are used at once.
- The page can't solve without the backend (and the password): opened from a plain static server it shows
  the scene, but dragging the target doesn't move the model.
- Plain HTTP: the password stops casual use, not traffic capture. HTTPS would need a reverse proxy.
- The collision path check assumes straight joint-space motion. The servos are synchronised to follow it,
  but with a shared acceleration their ramps differ slightly, and discrete samples can clip an edge (the
  solver test allows 2 mm on the work area's soft edges, none on real collisions).
- LED cues played on the arm change the real ATOM, but the page's LED panel doesn't mirror them (press Read back).
- Backend playback keeps running with no page open (by design); stop it with Stop, `/api/stop` or `/api/playback/stop`.
