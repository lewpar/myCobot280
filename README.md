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
