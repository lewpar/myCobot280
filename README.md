# myCobot280 Arm Control

FastAPI backend and browser IK simulator for controlling the myCobot280 robotic arm and its ATOM ESP32.

## Hardware Architecture

```
RPi (/dev/ttyAMA0, 1M baud)
  └─ half-duplex TTL UART bus ─┐
      ├─ Servo 1 (base rotation)
      ├─ Servo 2 (arm joint)
      ├─ Servo 3 (arm joint)
      ├─ Servo 4 (arm joint)
      ├─ Servo 5 (arm joint)
      ├─ Servo 6 (end effector)
      └─ ATOM ESP32 (ID 7, LED matrix)
```

All devices share the same half-duplex UART bus. Two protocols coexist on the wire, distinguished by header byte:

| Protocol | Header | Addressing | Target |
|----------|--------|------------|--------|
| Feetech SCS | `FF FF` | Servo ID (1-6) | Servos |
| Feetech SCS | `FF FF` | Servo ID 7 | ATOM ESP32 |

## Setup

```
pip install -r src/backend/requirements.txt
cp src/backend/.env.example src/backend/.env
```

Flash `atom_led_matrix/atom_led_matrix.ino` to the ATOM ESP32. Default UART pins in the sketch:

| Pin | Function |
|-----|----------|
| GPIO 27 | LED data (NeoPixel) |
| GPIO 19 | Bus RX |
| GPIO 22 | Bus TX |

Adjust `BUS_RX` / `BUS_TX` at the top of the `.ino` if your ATOM uses different pins.

### Password

Everything that can move the arm asks for a password: the IK simulator, the WebSocket link and the
REST API. Set it in `src/backend/.env`:

```
MYCOBOT_PASSWORD=choose-something
```

If it's empty, the backend makes up a random password at startup and print it.

- REST: send it in the `X-Arm-Password` header on every `/api` request.
- WebSocket `/ws/arm`: browsers can't set headers on a WebSocket, so the first message is
  `{"type": "auth", "password": "..."}`.

Five wrong passwords from one address within a minute lock that address out for the rest of the minute.
The connection is plain HTTP, so the password keeps casual users on the network out; it doesn't
protect against someone capturing traffic. Put the backend behind HTTPS if that matters.

## Usage

Only one program can have the serial port open at a time (the backend or one of the tools, not both).
The second one to start exits with a "port is already in use" message.

```
./run.sh              # API on :8000, docs at /docs, simulator at http://<pi>:8000/sim
```

The first run creates `venv/` and installs the backend's requirements (again whenever
`src/backend/requirements.txt` changes). The script prints the simulator's address for each network
interface and warns about a missing serial port, missing permissions on it, a missing `.env` or password,
or something already using the HTTP port. The serial port comes from `--port`, `$MYCOBOT_PORT` or
`src/backend/.env`, in that order.

```
./run.sh --port /dev/ttyUSB0      # another serial port
./run.sh --http-port 8080         # another HTTP port (--host to bind one address)
./run.sh --dev                    # restart on code changes (uvicorn --reload); not while the arm moves
./run.sh --help
```

### Python API (mycobot280)

The `mycobot280` module is a self-contained library that talks directly to the arm over serial. It can be used standalone or imported by other scripts.

```python
from mycobot280 import MyCobot280

arm = MyCobot280("/dev/ttyAMA0")

# Servos
s1 = arm.servo(1)
print(s1.position)          # read current position
s1.move(2048)               # absolute move → (True, 2048)
s1.move_rel(-100)           # move relative to current
s1.center()                 # go to 2048
s1.torque = False           # disable torque
s1.ping()                   # check responsiveness

# ATOM
arm.atom.ping()                       # reachable?
arm.atom.color = (255, 0, 0)          # set all LEDs red
arm.atom.set_color(0, 255, 0)         # same, explicit method
arm.atom.pixel(2, 2, 0, 0, 255)      # single pixel blue

# Several servos at once (one sync-write packet)
arm.sync_move({1: 2048, 2: 1900}, speed=600, accel=20)   # non-blocking
arm.move_all({1: 2048, 2: 1900})       # waits; returns {id: (arrived, position)}
arm.read_positions([1, 2, 3])          # [2048, 1900, 2050] (None where no reply)
arm.hold([1, 2, 3, 4, 5, 6])           # stop where they are
arm.sync_torque([1, 2, 3, 4, 5, 6], False)

# Convenience methods on the arm itself
arm.move(2, 1500, speed=600)
arm.get_position(3)
arm.scan()                # re-scan, returns [1, 2, 3, 4, 5, 6]
arm.servo_ids             # cached ID list
arm.servo_count           # 6

# Cleanup
arm.close()
# or use a context manager:
with MyCobot280("/dev/ttyAMA0") as arm:
    arm.servo(1).move(2048)
```

## Servo IDs

| ID | Joint            |
|----|------------------|
| 1  | Base rotation    |
| 2  | Arm joint        |
| 3  | Arm joint        |
| 4  | Arm joint        |
| 5  | Arm joint        |
| 6  | End effector     |
| 7  | ATOM (LED, I/O)  |

## Feetech Protocol

Every frame on the bus is `LEN + 4` bytes:

```
FF FF <ID> <LEN> <INSTR> [params...] <CHKSUM>
```

- `LEN` = 2 + number of parameter bytes (includes the INSTR byte)
- `CHKSUM` = `~(ID + LEN + INSTRUCTION + sum(params)) & 0xFF`

  E.g. for `FF FF 01 05 03 2A 00 08` (servo 1 write position 2048):
  Sum of bytes after the headers = `01 + 05 + 03 + 2A + 00 + 08` = 59 (0x3B).
  `~0x3B` masked to 8 bits = `0xC4`.

Example — WRITE position 2048 to servo 1 (`FF FF 01 05 03 2A 00 08 C4`):

| Byte | Value  | Meaning |
|------|--------|---------|
| 0    | `FF`   | Header |
| 1    | `FF`   | Header |
| 2    | `01`   | Servo ID 1 |
| 3    | `05`   | LEN = 5 (INSTR + ADDR + 2 data bytes) |
| 4    | `03`   | WRITE instruction |
| 5    | `2A`   | Register 0x2A = goal position |
| 6    | `00`   | Position low byte |
| 7    | `08`   | Position high byte (0x0800 = 2048, little-endian) |
| 8    | `C4`   | Checksum |

Example — set ATOM LED to red via ID 7 (`FF FF 07 06 03 01 FF 00 00 EF`):

| Byte | Value  | Meaning |
|------|--------|---------|
| 0    | `FF`   | Header |
| 1    | `FF`   | Header |
| 2    | `07`   | ID 7 (ATOM) |
| 3    | `06`   | LEN = 6 (INSTR + ADDR + 3 data bytes) |
| 4    | `03`   | WRITE instruction |
| 5    | `01`   | ADDR 0x01 = SET_COLOR |
| 6    | `FF`   | Red = 255 |
| 7    | `00`   | Green = 0 |
| 8    | `00`   | Blue = 0 |
| 9    | `EF`   | Checksum |

| Instruction | Code | Purpose |
|-------------|------|---------|
| `PING`      | 0x01 | Check if device responds |
| `READ`      | 0x02 | Read register(s) |
| `WRITE`     | 0x03 | Write register(s) |

**Status response** (from device to host):
```
FF FF <ID> <LEN> <ERR> [params...] <CHKSUM>
```

Key servo registers:

| Address | Register            | Bytes |
|---------|---------------------|-------|
| 9       | Min angle limit     | 2     |
| 11      | Max angle limit     | 2     |
| 40      | Torque enable       | 1     |
| 41      | Acceleration        | 1     |
| 42      | Goal position       | 2     |
| 46      | Goal speed          | 2     |
| 56      | Present position    | 2     |

## Safety

- **Limits:** each servo's min/max angle limits are read from EEPROM. Every move is clamped to
  `[min+50, max-50]`; servos with limits `0,0` (J6) get 50–4045. The IK link also keeps each joint
  inside the URDF limits.
- **Register ranges:** speed must be 1–4000 steps/s and acceleration 1–254. The API
  rejects values outside these ranges, and the library clamps them rather than letting them wrap.
- **Collisions:** every move (IK, REST, Home All) is checked against the table, the base and
  shoulder column, and the wrist folding into the upper arm, both at the target and along the way
  (a straight joint-space path). Refused moves say what would have hit (HTTP 409).
  The model lives in `arm_model.py`; the simulator carries a copy so it refuses the same poses before
  sending anything. The check uses the IK calibration, so calibrate first (see below).
- **Stop:** `POST /api/stop`, or the Stop button / Esc in the simulator. Every joint holds where it is (torque stays on, so nothing drops) and all motion is
  refused until you resume.

## ATOM Protocol

ATOM commands use standard Feetech WRITE packets addressed to ID 7:

```
FF FF 07 <LEN> 03 <ADDR> <DATA> <CHK>
```

| Address | Command    | Data                  |
|---------|------------|-----------------------|
| `0x00`  | PING       | none                  |
| `0x01`  | SET_COLOR  | R G B                 |
| `0x02`  | SET_PIXEL  | X Y R G B (0-4 grid)  |

On boot the ATOM runs a rainbow animation on the 5×5 LED matrix until the first command arrives.

The simulator's **ATOM** tab has an LED matrix panel: pick a colour and click or drag across the 5×5 grid,
fill or clear all LEDs, set the brightness, or read the current LEDs back. It uses the REST endpoints
(`/api/atom/pixel`, `color`, `brightness`, `state`) with the password from the connection box, so the
arm link doesn't have to be connected. Brightness is capped at half the NeoPixel range by the firmware.

## IK simulator and live link

The backend serves a 3D inverse-kinematics simulator at `http://<pi>:8000/sim` and a WebSocket at
`/ws/arm` that streams joint angles to the servos (one sync-write packet per update) and measured
positions back (~10 Hz).

1. `./run.sh backend`, open `http://<pi>:8000/sim`, go to the **Robot** tab, enter the password and press **Connect**.
   The chip in the top bar shows the link state; **Stop** (or Esc) is always in the top-right corner.
2. Still in **Robot**, turn on **Hand-guide mode**, pose the arm like the sim's zero pose (arm straight up), press
   **Set zero to the arm's current pose**.
3. Bend each joint by hand. If the green (measured) pose turns the other way, tick **Reverse** for it.
4. Set **Tool length** if something is mounted on the flange; the collision check includes it.

Calibration is saved to `ik_calibration.json` (zeros seeded from `center_positions.json` until
then). If a zero sits far from the middle of a servo's travel, the page says how much range that
joint has lost.

### Recording and playback

The **Record** tab captures a motion and plays it back. Press **Record**, move the arm (turn on
**Hand-guide mode** from the same tab to move it by hand, or drive it from Motion), then press
**Stop recording**, name it and **Save**. While connected it records the arm's measured pose
10 times a second; offline it records the simulated arm. Still time at either end is trimmed.

Pick a saved recording and press **Play** (or double-click it). The arm first moves to the recording's
start pose, then follows it at the chosen playback speed, optionally looping. Playback feeds the same
path as the IK target, so every pose is collision-checked on the page and the backend and the Max speed
and Acceleration from Motion apply. Stop/Esc, hand-guide mode, or moving the target ends it.

Recordings are stored on the backend as JSON in `recordings/` (gitignored) through
`GET/POST /api/recordings`, `GET/DELETE /api/recordings/{id}`.

Home positions (`GET`/`POST /api/servos/home`, `POST /api/servos/center_all`) are stored in
`center_positions.json`. Home All moves every joint together after a collision check.

## Checking the model against your arm

Two things can't be known from the code alone:

- **Servo speed/acceleration units.** The IK link assumes STS units (speed 1 step/s per unit,
  acceleration 100 steps/s² per unit). With the backend stopped, run
  `python3 tools/check_servo_units.py` to time two moves of the wrist roll and print the real units;
  `--write` saves them to `ik_calibration.json`, and the IK link uses them from then on.
- **Link lengths.** They come from Elephant Robotics' URDF. After calibrating, set a target in the
  simulator with the flange facing down, let the arm get there, and measure from the centre of the
  base to the centre of the flange. Try three or four points spread around the workspace. Errors
  of more than a few millimetres that grow with reach mean a link length in `arm_model.py`
  (`URDF_JOINTS`) and in the simulator's `JOINTS` table needs adjusting.

## Tools

- `tools/check_servo_units.py`: measures the servo speed and acceleration units (above).
- `tools/diagnostics/`: bring-up scripts kept for reference (see its README).
