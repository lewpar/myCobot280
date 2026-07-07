# myCobot280 Arm Control

TCP-based client/server system for controlling the myCobot280 robotic arm over serial.

## Architecture

- **`arm_server.py`** - Runs on the robot (e.g. Raspberry Pi). Communicates with Feetech-family servos over `/dev/ttyAMA0` at 1M baud, exposes a TCP control interface.
- **`arm_client.py`** - Connects to the server from any machine. Provides an interactive CLI and one-shot command-line usage for controlling the arm.

## Setup

```
pip install -r requirements.txt
```

## Usage

**On the robot** (start the server):
```
python3 arm_server.py
```

**On a client machine** (interactive mode):
```
python3 arm_client.py --host <robot-ip>
```

One-shot commands:
```
python3 arm_client.py STATUS
python3 arm_client.py MOVE 1 2048
```

## Servo IDs

| ID | Joint             | Range    |
|----|-------------------|----------|
| 1  | Base rotation     | 0-4095   |
| 2  | Arm joint         | 0-4095   |
| 3  | Arm joint         | 0-4095   |
| 4  | Arm joint         | 0-4095   |
| 5  | Arm joint         | 0-4095   |
| 6  | End effector      | 0-4095   |

## Protocol Commands

| Command                              | Description                    |
|--------------------------------------|--------------------------------|
| `SCAN`                               | Re-scan bus for servos         |
| `COUNT`                              | Number of servos detected      |
| `POS <id>`                           | Read current position          |
| `INFO <id>`                          | Read position + limits         |
| `LIMITS <id>`                        | Read min/max angle limits      |
| `MOVE <id> <pos> [speed] [accel]`    | Absolute move to position      |
| `MOVE_REL <id> <delta> [speed] [accel]` | Relative move               |
| `TORQUE <id> <0\|1>`                 | Enable/disable torque          |
| `CENTER <id> [pos] [speed] [accel]`  | Move servo to center (default 2048) |
| `PING <id>`                          | Ping a servo                   |
| `STATUS`                             | Show all servo positions       |
| `QUIT`                               | Disconnect                     |

Position values are on the native 0-4095 scale.
