# myCobot280 Arm Control

TCP client/server for controlling the myCobot280 robotic arm and its ATOM ESP32.

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
pip install -r requirements.txt
```

Flash `atom_led_matrix/atom_led_matrix.ino` to the ATOM ESP32. Default UART pins in the sketch:

| Pin | Function |
|-----|----------|
| GPIO 27 | LED data (NeoPixel) |
| GPIO 32 | Bus RX |
| GPIO 26 | Bus TX |

Adjust `BUS_RX` / `BUS_TX` at the top of the `.ino` if your ATOM uses different pins.

## Usage

### TCP client/server

**On the robot:**
```
python3 arm_server.py
```
Only one client at a time. The server sends keepalive pings every 15 seconds and drops unresponsive clients.

**On a client machine:**
```
python3 arm_client.py
```
Prompts for server IP and port, then shows the interactive menu.

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

Each servo's min/max angle limits are read from EEPROM on startup. All moves are clamped to `[min+50, max-50]`. Servos with limits set to `0,0` (continuous rotation, e.g. end effector) get the full 50–4045 range. Enforced on both server and client.

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
