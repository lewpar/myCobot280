# WebSocket API (protocol 4)

Two sockets, both on the backend (`ws://<pi>:8000`), both behind the arm password:

- **`/ws/arm`**, the live link to the arm. Send it a **`target`** (a point for the tool tip) and the backend
  solves the inverse kinematics, plans a collision-free route and moves the arm there; or stream joint-angle
  **`goal`s** yourself. Every move is collision-checked, converted to servo ticks with the saved calibration
  and sent to all six servos in one sync-write. The measured pose comes back about 10 times a second.
- **[`/ws/ik`](#wsik-solve-only)**, solve-only: the same solver without moving anything (or needing the arm).
  The simulator page uses it when it isn't driving the arm.

The IK exists only on the backend (`src/backend/ik.py`); the simulator page at `/sim/` is a client of these
sockets like any other.

Implementation: `src/backend/main.py` (`ws_arm`, `ws_ik`: auth, framing, the send loop),
`src/backend/ik_link.py` (`IKLink`: command handling, the bus loop, the target's solver thread, stop state,
playback, stall guard) and `src/backend/ik.py` (the solver, route planning and the `/ws/ik` session).
The docstring at the top of `ik_link.py` is the short-form reference; this file is the long form.

- [Connecting](#connecting)
- [Message framing](#message-framing)
- [Backend → client](#backend--client): `hello`, `config`, `state`, `error`
- [Client → backend](#client--backend): `target`, `goal`, `torque`, `stop`, `resume`, `set_zero`, `set_dir`,
  `set_tool`, `set_area`, `set_stall_guard`, `recenter`
- [Joints the arm won't move](#joints-the-arm-wont-move)
- [The epoch](#the-epoch)
- [Errors and close codes](#errors-and-close-codes)
- [`/ws/ik` (solve-only)](#wsik-solve-only)
- [Example session](#example-session)

## Connecting

URL: `ws://<pi>:8000/ws/arm` (plain HTTP/WS; there is no TLS unless you add a reverse proxy). `/ws/ik` logs
in the same way.

Browsers can't set headers on a WebSocket, so the password goes in the **first message**, which must
arrive within **5 seconds** of the connection opening:

```json
{"type": "auth", "password": "..."}
```

The password is `MYCOBOT_PASSWORD` (from the environment or `src/backend/.env`), compared with
`hmac.compare_digest`. On a random password run, the backend prints it at startup.

What happens next:

| Situation | Backend sends | Then |
|---|---|---|
| 5+ failed attempts from this IP in the last 60 s (REST and WS count together) | `error` `locked` | closes with **4429** |
| Wrong/missing password, not JSON, wrong `type`, or nothing within 5 s | `error` `auth` | closes with **4401** (counts as a failed attempt) |
| Password right but the serial port didn't open | `error` `no_arm` | closes with **4503** |
| Password right | `hello`, then `config`, then `state` ~10 Hz | stays open |

Each failed attempt also waits 0.5 s before replying, to slow down guessing.

Several clients may be connected at once (`state.clients` counts them). They all see the same state,
and their goals and targets overwrite each other: the last one received wins.

## Message framing

- Every message in both directions is a **JSON object in a text frame** with a string `type`.
- Binary frames, non-objects, and JSON containing `NaN`/`Infinity` are rejected with
  `error` `bad_request` (`ref: null`); the connection stays open.
- Booleans must be real JSON booleans (`true`/`false`, not `1`/`0`), and numbers must be finite
  numbers (not booleans, not strings).
- Successful commands get **no reply**. Their effect shows up in the next `config` or `state`.
  Only failures are answered, with an `error` whose `ref` is the command's `type`.

## Backend → client

### `hello`

Sent once, right after a successful auth.

```json
{"type": "hello", "protocol": 4}
```

Protocol 3 added `target`, `state.ik` and `/ws/ik`. Protocol 4 added `state.ticks`, `state.out_of_range`
and `recenter`, and refuses to move a joint that reads outside what its servo can reach. Everything else
is unchanged since protocol 2.

Check `protocol` and refuse to drive the arm if it isn't one you understand.

### `config`

Sent once after `hello`, and again whenever any field changes (calibration, attachment, work area,
stall guard, or the servos' limits being re-read). It is always sent **before** the `state` whose
epoch belongs to it, so a client can adopt a new epoch knowing it has the matching calibration.

```json
{
  "type": "config",
  "calibrated": true,
  "zero": [2048, 2051, 2040, 2060, 2048, 2048],
  "dir": [1, -1, 1, 1, -1, 1],
  "tool_mm": 80.0,
  "tool_d_mm": 25.0,
  "attachment": "vacuum",
  "area": {"enabled": true, "center": 0.0, "span": 180.0, "radius_mm": 0.0, "base_mm": 150.0},
  "limits": [[-165.0, 165.0], [-135.0, 135.0], [-150.0, 150.0], [-145.0, 145.0], [-150.0, 155.0], [-175.0, 175.0]],
  "stall_guard": true
}
```

| Field | Meaning |
|---|---|
| `calibrated` | `false` until a zero or direction has been set (zeros are then seeded from `center_positions.json`) |
| `zero` | Servo tick per joint (J1…J6) at the URDF zero pose (arm straight up, flange facing +X) |
| `dir` | `1` or `-1` per joint. `ticks = zero + dir × deg × 4096/360` |
| `tool_mm`, `tool_d_mm` | Attachment length (along the flange normal) and diameter, mm |
| `attachment` | `"none"`, `"vacuum"` (80 × 25 mm) or `"custom"` |
| `area` | Work area for the tool tip, see [`set_area`](#set_area) |
| `limits` | Per joint `[lo, hi]` in degrees: the URDF limits intersected with the servo's EEPROM range minus the 50-tick buffer, under the current calibration. `[0, 0]` means the ranges don't overlap (bad calibration) |
| `stall_guard` | Whether the [stall guard](#the-stall-guard) is on |

### `state`

About every 100 ms.

```json
{
  "type": "state",
  "angles": [0.12, -3.4, 10.0, 5.5, null, 0.0],
  "ticks": [2049, 2087, 2162, 2111, null, 2048],
  "torque": true,
  "stopped": false,
  "blocked": null,
  "fault": null,
  "out_of_range": null,
  "playback": null,
  "play_end": {"n": 0, "message": null},
  "epoch": 3,
  "clients": 1,
  "ik": null
}
```

| Field | Meaning |
|---|---|
| `angles` | Measured joint angles in degrees (URDF convention, 2 decimals), within −180…180 (a joint more than half a turn from its zero reads the short way round). An entry is `null` if that servo didn't answer the last read |
| `ticks` | The raw servo readings, 0–4095 (2048 is the servo's own centre), `null` where a servo didn't answer |
| `torque` | Whether torque is on (as last set by this link) |
| `stopped` | The stop state. While `true`, every motion request (WS goals, REST moves, Home All, playback) is refused until someone resumes |
| `blocked` | `null`, or the reason the **last goal** was refused by the collision/limit/work-area check, e.g. `"the elbow (J3) would hit the table"`. Cleared by the next goal that passes, a new playback, or resume |
| `fault` | `null`, or why the backend stopped the arm by itself (stall guard, a bus error, a joint out of range). Cleared on resume |
| `out_of_range` | `null`, or why the arm won't be moved at all: see [joints the arm won't move](#joints-the-arm-wont-move). Clears by itself once every joint is back in range |
| `playback` | `null`, or the backend playback's status (below) |
| `play_end` | `{n, message}`: `n` goes up by one each time a playback ends, `message` says why (`"Playback finished."`, `"Stopped."`, `"Playback stopped: …"`). Watch `n` to notice an end you missed between states |
| `epoch` | See [the epoch](#the-epoch). Goals must carry this value |
| `clients` | Number of connected WebSocket clients |
| `ik` | `null`, or how the active [`target`](#target) is going (below) |

`playback` while a recording or sequence is playing (started via REST `POST /api/playback`):

```json
{"name": "demo", "recording": "wave", "step": 0, "steps": 2, "phase": "run",
 "t": 1.52, "duration": 4.8, "loop": false, "rate": 1.0, "timed": true}
```

`name` is the recording or sequence, `recording` the one currently playing, `step`/`steps` the position in a
sequence, `phase` one of `approach` (moving to the first frame), `run`, `finish`, `zero` (returning to the
zero pose) or `pause` (between sequence steps), `t`/`duration` seconds into the current recording. While
it's set, the backend is driving the arm: goals from clients are refused, and a client should just follow
`angles`.

`ik` while a target is set (re-solved about 20 times a second):

```json
{"target": [180.0, -40.0, 110.0], "down": true,
 "angles": [7.66, -26.6, -121.71, 58.31, 0.0, 20.16], "next": [7.66, -26.6, -121.71, 58.31, 0.0, 20.16],
 "detour": false, "reached": true, "pos_err_mm": 0.0, "ori_err_deg": 0.0,
 "blocked": null, "outside": null, "settled": true, "arrived": true}
```

| Field | Meaning |
|---|---|
| `target` | The point being solved for, mm (z raised to 3 mm if it was lower); `null` for a joint-pose target |
| `down` | Whether the flange is asked to face straight down |
| `angles` | The solution: joint angles in degrees. For a point out of reach, the closest pose found |
| `next` | The pose the servos are being sent to now: `angles` itself, or a raised pose on the way round (`detour`). `null` when there's no clear route (see `blocked`) |
| `detour` | The straight joint-space move isn't clear, so the arm goes up and around (lift, turn the base, come down) |
| `reached` | The solution puts the tool tip within 3 mm of `target` (and, with `down`, within 3° of straight down) |
| `pos_err_mm`, `ori_err_deg` | How far the solution is from that |
| `blocked` | `null`, or why the arm isn't moving toward the solution: the solution collides, or no route to it is clear |
| `outside` | `null`, or why `target` itself is outside the work area (`"would leave the work area"`, `"would come too close to the base"`, `"would reach past the work area"`). The solver then stops at the area's edge |
| `settled` | Solving again from here would give the same answer: the solution stopped changing and no restart is still to come. `false` while it's still converging on a far point, or stuck with restarts left |
| `arrived` | The arm is at the solution: every joint within 1°, with no detour left |

`reached` is about the solution, `arrived` about the arm: wait for both before treating a move as done.

### `error`

```json
{"type": "error", "code": "refused", "ref": "goal", "message": "The arm is stopped. Resume first."}
```

See [errors and close codes](#errors-and-close-codes). `message` is human-readable and may change;
branch on `code` and `ref`.

## Client → backend

### `target`

Go to a point (or a joint pose) and stay there. The backend solves, plans the route and drives the arm.

```json
{"type": "target", "xyz": [180, -40, 110], "down": true, "speed": 30, "acc": 200, "epoch": 3}
{"type": "target", "angles": [0, 20, -40, 20, 0, 0], "speed": 30, "acc": 200, "epoch": 3}
```

| Field | Valid | Meaning |
|---|---|---|
| `xyz` | 3 finite numbers, −1000…1000 | The tool tip's target in mm, base frame: Z up from the table, 0° (+X) where the flange points at the zero pose, −Y the arm's right. The tool tip is the flange centre, or the attachment's tip if one is set (`set_tool`) |
| `down` | boolean (default `false`, `xyz` only) | Also keep the flange facing straight down (for picking things up) |
| `angles` | 6 finite numbers, −360…360 | Instead of `xyz`: a joint pose in degrees. Nothing is solved; the arm just takes a clear route there |
| `speed`, `acc`, `epoch` | as for [`goal`](#goal) | |

Give exactly one of `xyz` and `angles`. Refused (`refused`) if the arm is stopped, a playback is running, or
`epoch` is stale.

While a target is set, a solver thread on the backend runs about 20 times a second, from where the arm
actually is:

1. **Solve** (for `xyz`): damped least squares from the previous solution (or from the arm's pose, for the
   first target), position first and "facing down" second. It never steps from a clear pose into a
   collision. When it's stuck (short of the point, colliding, or with no clear route), it restarts from
   seeded poses a quarter of a second after the point changes, then up to 6 more times a second apart with
   new random seeds while the point stays the same. The numbers come from the engine set with `MYCOBOT_IK`
   on the backend (`/api/health` says which): `native` (the default, a couple of ms a solve), `pink` (Pink,
   about 10 ms) or `ikpy` (IKPy, tens of ms a solve and around a second a restart on a desktop). Several times
   that on the Pi, so with IKPy the loop manages fewer than 20 a second there.
2. **Plan**: straight to the solution if that joint-space path is clear, otherwise through raised poses
   (J2–J5 at 0: lift, turn the base, come down; or lift the shoulder or straighten the elbow first). If no
   route is clear, the arm isn't sent anywhere, and if the rest of its current move has stopped being clear
   it holds where it is.
3. **Drive**: the next pose on the route goes out as a goal, with all of the [`goal`](#goal) checks below.

Once the answer is `settled` and the arm is still (within 0.2° of where it was solved from), the loop stops
re-solving until the point, the settings or the arm move, so a target the arm is holding costs next to
nothing.

Sending another `target` replaces the point and carries on from the current solution, so a client can stream
targets as fast as it likes (only the latest counts). A target stays set, holding the arm at the solution,
until a `goal`, `stop`, `torque`, a playback starting, a new epoch, or the last client disconnecting ends it
(`state.ik` goes back to `null`). `set_tool` and `set_area` keep it and re-solve under the new settings.

### `goal`

Move the arm to a pose along a straight joint-space path (no solving, no detours). Ends any target.

```json
{"type": "goal", "angles": [0, 20, -40, 20, 0, 0], "speed": 30, "acc": 200, "epoch": 3}
```

| Field | Valid | Meaning |
|---|---|---|
| `angles` | 6 finite numbers, −360…360 | Target joint angles, degrees, URDF convention |
| `speed` | 1–360 | deg/s of the joint that moves furthest. **Capped at 150** on the arm |
| `acc` | 1–2000 | deg/s², the same for every joint |
| `epoch` | integer ≥ 0 | The `epoch` from the latest `state` |

Refused (`refused`) if the arm is stopped, a playback is running, or `epoch` is stale.

Otherwise the goal is queued; only the **latest** queued goal is kept, so streaming at the frame rate
is fine. The bus loop (about every 20 ms) takes it and:

1. Waits until all six servos have read back at least once (the goal stays queued until then).
2. Checks the straight joint-space path from the measured pose to the goal (`arm_model.check_path`:
   joint limits, table, base, shoulder column, self-collision, attachment, ATOM head, work area). If the
   start pose already collides, only the target is checked, so you can move out of a bad pose.
   A failing goal is **dropped silently** apart from `state.blocked`, which says why; it is not an `error`.
3. Turns torque on if it was off (holding the current pose first).
4. Splits `speed` between the joints in proportion to how far each has to go, so they arrive together
   and the arm follows the path that was checked.
5. Clamps each target to the servo's EEPROM limits minus 50 ticks, and sends all six in one sync-write.

Queued goals are discarded by `stop`, `torque`, `set_zero`, `set_dir`, `set_tool`, `set_area`, a new
epoch, a playback starting, and the last client disconnecting (the servos then hold their last goal).

### `torque`

```json
{"type": "torque", "on": false}
```

Torque off makes every joint limp (**the arm falls unless someone holds it**; this is hand-guide mode).
Torque on first sets each servo's goal to its present position, so the arm holds where it is instead of
jumping. Stops any playback and discards a queued goal. Not refused while stopped.

### `stop`

```json
{"type": "stop"}
```

Holds every servo where it is (goal = present position), ends any playback, discards a queued goal and
sets `stopped`. Everything that moves the arm is then refused until `resume`. Always allowed.

### `resume`

```json
{"type": "resume"}
```

Clears `stopped`, `blocked` and `fault`, and **bumps the epoch**: re-read `angles` from the next state
before sending goals.

### `set_zero`

```json
{"type": "set_zero"}
```

The current measured pose becomes the URDF zero (arm straight up, flange facing +X) for every joint.
Saved to `ik_calibration.json`, sets `calibrated`, recomputes `limits`, bumps the epoch, and stops any
playback. Refused if any servo isn't answering.

### `set_dir`

```json
{"type": "set_dir", "joint": 1, "dir": -1}
```

`joint` 0–5 (J1…J6), `dir` `1` or `-1`. Reverses which way that joint's angle counts. Saved, sets
`calibrated`, recomputes `limits`, bumps the epoch, stops any playback. Sending the direction it already
has does nothing.

### `set_tool`

```json
{"type": "set_tool", "attachment": "custom", "mm": 60, "d_mm": 20}
```

| Field | Valid | Meaning |
|---|---|---|
| `attachment` | `"none"`, `"vacuum"`, `"custom"` (default: the current one) | Which attachment is on the flange |
| `mm` | 0–150 | Length from the flange face to the tip (custom only; default the current value) |
| `d_mm` | 1–60 | Diameter (custom only; default the current value) |

A known attachment uses its own size and ignores `mm`/`d_mm` (`none` = 0 mm, `vacuum` = 80 × 25 mm).
The tool tip becomes the TCP for the work-area check, and the attachment is collision-checked as a
cylinder. Saved; discards a queued goal.

### `set_area`

```json
{"type": "set_area", "enabled": true, "center": 0, "span": 180, "radius_mm": 0, "base_mm": 150}
```

| Field | Valid | Meaning |
|---|---|---|
| `enabled` | boolean (required) | Enforce the work area |
| `center` | −180…180 | Direction of the middle of the slice, degrees. 0° is +X (where the flange points at the zero pose), −90° is the arm's right |
| `span` | 30…360 | Width of the slice, degrees (360 = all round) |
| `radius_mm` | `0` or 100…450 (default 0) | Maximum reach of the tip; 0 = no limit |
| `base_mm` | `0` or 60…250, less than `radius_mm` if that is set (default 150) | Keep the tip out of a cylinder this radius around the base axis, up to 250 mm high; 0 = off |

Only the tool tip is checked against the area; the rest of the arm may cross its edges. A tip within
60 mm of the base axis counts as inside the slice. Saved; discards a queued goal.

### `recenter`

```json
{"type": "recenter", "joints": [3, 4]}
```

Re-centre the servos of those joints (0–5, default all six) where they are now: each reads 2048 afterwards.
This is Feetech's own "calibrate the middle", stored in the servo's EEPROM. It puts the servo's wrap point
(where its count goes from 4095 back to 0) half a turn from here, so done in the zero pose, the wrap point ends
up behind the joint, out of its reach. The calibration's zero and the saved home positions shift with it, so
every angle keeps its meaning. The epoch goes up, and each re-centring is logged to `servo_centres_log.json`
(`tools/recenter_servos.py --undo` puts the last one back).

Refused unless **torque is off**: the servo's goal register keeps its number, so a servo holding a goal
would jump when its reading changes. Hold the arm (Hand-guide) first. Also refused during a playback.

This is the one command with a reply on success:

```json
{"type": "recentered", "results": [
  {"joint": 3, "id": 4, "ok": true, "before": 573, "after": 2048, "correction_before": 0, "correction_after": -1475}]}
```

A joint that didn't answer, or didn't take the new centre, has `"ok": false` and a `message`.

### `set_stall_guard`

```json
{"type": "set_stall_guard", "on": true}
```

Turns the stall guard on or off (on by default, not saved across restarts).

#### The stall guard

While torque is on and the arm isn't stopped, a joint that is more than **6°** from its goal and hasn't
moved **0.5°** in **1 s** is taken to be blocked by something. The backend then stops the arm, sets
`fault` (e.g. `"J3 stalled 12° short of its goal, so the arm stopped. …"`) and needs a `resume`.
Goals sent for a target count the same way.

## Joints the arm won't move

Each servo counts 0–4095 over one turn and wraps round at the ends. If a joint's zero sits near that wrap point,
part of the joint's travel lies beyond it, and a joint pushed there (by hand, or left there at power-off) reads
half a turn away. Asked to "move back", the servo then turns the wrong way round, into the arm.

So whenever a joint reads more than 10° outside its `limits`, the backend:

- sets `state.out_of_range` to say which joint and why, and stops the arm (with that `fault`) if it had torque on;
- refuses every `goal` and `target` (`refused`), REST move (409) and playback (409) until the joint is back
  in range;
- stops at once if a reading jumps by half a turn or more between reads (the wrap point was crossed).

The way out is by hand: torque off (`torque`, Hand-guide), turn the joint back the short way, then `resume`.
To stop it happening again, re-centre the servos in the zero pose (`recenter`, or the page's calibration
wizard). Goals are also never sent within 228 ticks (20°) of a servo's wrap point, whatever the servo's own
angle limits say; `config.limits` includes that margin.

## The epoch

`state.epoch` goes up whenever the arm's pose has to be re-read before new goals make sense:

- `resume`
- `set_zero`, `set_dir`, `recenter` (the angle ↔ tick mapping changed)
- the end of a backend playback (the arm moved behind the client's back)

A `goal` or `target` whose `epoch` isn't the current one is refused (`refused`, `ref` its type); a queued
goal is discarded and an active target ended when the epoch changes. So nothing computed from an old pose
or old calibration can move the arm.

The intended client behaviour: when `state.epoch` changes, take `angles` from **that same state** as the
new starting pose (reset your target to it), and only then send goals or targets with the new epoch. A
`config` for a calibration change always arrives before the state carrying the new epoch.

## Errors and close codes

`error.code` values:

| Code | Fatal | When |
|---|---|---|
| `auth` | yes, close 4401 | Wrong or missing password in the first message |
| `locked` | yes, close 4429 | Too many failed passwords from this IP (5 per minute) |
| `no_arm` | yes, close 4503 | The backend couldn't open the serial port |
| `bad_request` | no | Malformed or out-of-range command, unknown `type`, or not a JSON object |
| `refused` | no | Valid but not allowed now: stopped, playback running, stale epoch, a joint out of range (for `goal` and `target`), a servo not answering for `set_zero`, torque on for `recenter` |
| `internal` | no | The command handler raised; `message` has the exception |

Fatal errors have no `ref`. Non-fatal ones have `ref` = the command's `type` (or `null` if it had none).

Other close codes: **1011** if the backend's send loop fails. A client should reconnect and
re-authenticate.

An exception inside the bus loop (e.g. a serial error) doesn't close the socket: the arm is stopped,
`fault` explains, and `resume` retries.

## `/ws/ik` (solve-only)

The same solver and route planner, without moving anything. It doesn't need the arm (no `no_arm`), so it
works for planning and simulation too. Log in as for `/ws/arm`; the backend then sends
`{"type": "hello", "protocol": 4}` and the session's `settings`. Every message after that gets **exactly one
reply, in order**. Each connection has its own solver, settings and last solution.

### `settings`

```json
{"type": "settings", "tool_mm": 80, "tool_d_mm": 25,
 "area": {"enabled": true, "center": 0, "span": 180, "radius_mm": 0, "base_mm": 150},
 "limits": [[-168, 168], [-140, 140], [-150, 150], [-150, 150], [-155, 160], [-180, 180]]}
```

All fields are optional: `tool_mm` 0–150 and `tool_d_mm` 1–60 (the attachment, as in `set_tool`), `area` (as in
[`set_area`](#set_area)), `limits` (per joint `[lo, hi]` in degrees, inside the URDF limits: the solver never
leaves them). The reply also says which `engine` solves (read-only; set on the backend with `MYCOBOT_IK`).
They start as the arm's saved attachment and work area, and the servos' limits if the arm is
connected (the URDF limits if not). The reply is the full `settings` message, or an `error` (`ref: "settings"`).

### `solve`

```json
{"type": "solve", "id": 7, "xyz": [180, -40, 110], "down": true,
 "q": [0, 0, 0, 0, 0, 0], "from": [0, 20, 20, 20, 0, 0], "rescue": true, "restart": false}
```

| Field | Valid | Meaning |
|---|---|---|
| `xyz` + `down`, or `angles` | as for [`target`](#target) | What to solve for, or a joint pose to plan a route to |
| `q` | 6 angles, degrees (optional) | Where to start solving: your current solution. Default: this connection's last solution, else `from`, else the zero pose |
| `from` | 6 angles, degrees (optional) | Where the servos are. Without it there's no route (`next` is `null`) |
| `rescue` | boolean (default `true`) | Allow restarts from seeded poses when stuck (off: iterate only, always quick) |
| `restart` | boolean (default `false`) | Restart from seeded poses now, before iterating: for a fresh start far from `q` |
| `id` | string or integer (optional) | Echoed in the reply |

One `solve` is one step of what the arm's target loop does: 14 solver iterations plus any restart, then the
route. Call it repeatedly (with `q` = the last `angles`) to converge on a far target, as the page does every
frame. The reply is `{"type": "ik", "id": ..., ...}` with the same fields as [`state.ik`](#state) apart from
`arrived`, or an `error` (`ref: "solve"`).

When a reply is `settled`, sending the same request again (same point, `q`, `from` and settings) returns the same
answer; the backend replies from its last answer without solving. A client can simply stop asking until
something changes, as the page does.

## Example session

```text
→ {"type":"auth","password":"hunter2"}
← {"type":"hello","protocol":4}
← {"type":"config","calibrated":true,"zero":[...],"dir":[...],...,"stall_guard":true}
← {"type":"state","angles":[0,0,0,0,0,0],"torque":true,"stopped":false,"blocked":null,...,"epoch":0,"clients":1}
→ {"type":"goal","angles":[10,20,-30,10,0,0],"speed":30,"acc":200,"epoch":0}
← {"type":"state","angles":[0.4,0.9,-1.3,0.4,0,0],...}          (moving)
→ {"type":"goal","angles":[0,-150,0,0,0,0],"speed":30,"acc":200,"epoch":0}
← {"type":"state",...,"blocked":"J2 would pass its -140..140 degree limit"...}   (or a collision reason)
→ {"type":"stop"}
← {"type":"state",...,"stopped":true,...}
→ {"type":"goal","angles":[0,0,0,0,0,0],"speed":30,"acc":200,"epoch":0}
← {"type":"error","code":"refused","ref":"goal","message":"The arm is stopped. Resume first."}
→ {"type":"resume"}
← {"type":"state",...,"stopped":false,"epoch":1,...}
→ {"type":"target","xyz":[180,-40,110],"down":true,"speed":30,"acc":200,"epoch":1}
← {"type":"state","angles":[...],...,"ik":{"target":[180,-40,110],"reached":true,"arrived":false,"detour":false,...}}
← {"type":"state","angles":[...],...,"ik":{"target":[180,-40,110],"reached":true,"arrived":true,...}}
```

A minimal Python client (uses the `websockets` package) that moves the tool tip to a point and waits:

```python
import asyncio, json, websockets

async def move_to(xyz_mm, down=True):
    async with websockets.connect("ws://raspberrypi.local:8000/ws/arm") as ws:
        await ws.send(json.dumps({"type": "auth", "password": "hunter2"}))
        assert json.loads(await ws.recv())["protocol"] == 4
        sent = False
        async for raw in ws:
            m = json.loads(raw)
            if m["type"] == "error":
                print("error:", m)
            elif m["type"] == "state" and not sent and None not in m["angles"]:
                await ws.send(json.dumps({"type": "target", "xyz": xyz_mm, "down": down,
                                          "speed": 20, "acc": 100, "epoch": m["epoch"]}))
                sent = True
            elif m["type"] == "state" and sent and m["ik"]:
                ik = m["ik"]
                if ik["next"] is None:                 # no clear route: the arm won't move
                    print("blocked:", ik["blocked"])
                    return
                if ik["arrived"]:
                    if ik["outside"]:
                        print("the point is outside the work area:", ik["outside"])
                    print("there" if ik["reached"] else f"as close as it gets: {ik['pos_err_mm']} mm off")
                    return

asyncio.run(move_to([180, -40, 110]))
```

Per `CLAUDE.md`: try new client code against the fake bus (`tests/wsclient.py` is the test suite's client)
before pointing it at the real arm, and keep speeds low for the first moves.
