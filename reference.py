#!/usr/bin/env python3
"""
mycobot_bus_probe.py (interactive menu version)

A raw pyserial diagnostic tool to figure out which protocol/bus is actually
present on a given serial line: Feetech-style servo protocol (0xFF 0xFF ...)
or Elephant Robotics' internal proprietary framing (0xFE 0xFE ...).

Run it with no arguments:

    python3 mycobot_bus_probe.py

You'll get an interactive menu to pick the port, baud rate, and mode
instead of passing command-line flags. As soon as the port opens, the
script automatically scans IDs 1-50 for connected Feetech-family servos.
Every mode that needs a servo ID then makes you pick one from that
detected list (with an option to rescan) instead of guessing/typing one
blindly.

Modes:
    sniff      Just open the port and print any raw bytes that show up.
               Useful if the mycobot's own controller is powered on and
               periodically talking, or if you can trigger movement from
               another app while this listens.

    scan       (Also runs automatically on startup.) Sweeps IDs 1-50 and
               reports which ones respond as real Feetech-family servos.
               Re-run this any time you plug in/power a new servo so it
               shows up in the picker used by the other modes.

    ping       Send a Feetech-style PING packet to a servo (picked from
               the scanned list) and print exactly what comes back, byte
               for byte, whether or not it looks like a valid Feetech
               status packet.

    ping-fe    Sends a raw Elephant-Robotics-style framed packet
               (0xFE 0xFE header) instead of a Feetech one, to see if
               *that* framing gets a cleaner response on this line.

    position   Read-only: pings the servo, then reads and reports its
               configured Min Angle Limit, Max Angle Limit, and current
               Present Position (all on the servo's native 0-4095 scale).
               Never writes anything to the servo.

    move       Pings the servo first and REFUSES to proceed unless a
               valid Feetech-family reply comes back. Reads the servo's
               current position, then moves it by a relative +/- amount
               you enter (clamped to the 0-4095 range) at a conservative
               speed/acceleration. Good for a one-off "does this ID move,
               and which way" test.

    jog        Same safety check as move, but instead of a single shot it
               drops you into a loop where you can repeatedly send '+' or
               '-' to nudge the same servo back and forth by a step size
               (which you can change on the fly), so you can watch a joint
               and dial in direction/range interactively. Type 'q' to exit
               the loop.

    zero       Pings the servo first and REFUSES to proceed unless a
               valid Feetech-family (0xFF 0xFF ...) reply comes back.
               If confirmed, moves the servo to its center position
               (2048 by default, i.e. the Feetech "zero"/mid reference)
               at a conservative speed/acceleration. Always asks for an
               interactive confirmation before moving anything.

    set-zero   Pings the servo first and REFUSES to proceed unless a
               valid Feetech-family reply comes back. Does NOT move the
               servo - instead it rewrites the Position Correction
               EEPROM register so wherever the servo physically is
               RIGHT NOW becomes the new reported center (default 2048).
               This is the same operation Feetech's own SDK calls
               CalibrationOfs / "set center position". Verifies the
               result afterward and warns if it doesn't match.

This script does not depend on the Feetech STServo_Python SDK - it talks
directly over pyserial so we can see truly raw bytes without any SDK layer
rejecting or reinterpreting them.
"""

import glob
import os
import platform
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("This script needs pyserial. Install with: pip install pyserial --break-system-packages")
    sys.exit(1)


# ---------------------------------------------------------------------------
# Low-level protocol helpers (unchanged from the CLI version)
# ---------------------------------------------------------------------------

def hexdump(data: bytes) -> str:
    if not data:
        return "(no bytes received)"
    return " ".join(f"{b:02X}" for b in data)


def feetech_checksum(payload_after_id: bytes) -> int:
    # Feetech checksum = ~(sum of all bytes after the two 0xFF header bytes) & 0xFF
    return (~sum(payload_after_id)) & 0xFF


def build_feetech_ping(servo_id: int) -> bytes:
    # Standard Feetech/Dynamixel-style PING packet:
    # 0xFF 0xFF <ID> <LEN=2> <INSTR=0x01 PING> <CHECKSUM>
    body = bytes([servo_id, 0x02, 0x01])
    chk = feetech_checksum(body)
    return bytes([0xFF, 0xFF]) + body + bytes([chk])


# Feetech SMS/STS control table addresses (match what was confirmed against
# the mycobot's servo bus: PID gains at 21/22/23, EEPROM lock at 55, etc.)
ADDR_MIN_ANGLE_LIMIT = 9        # EEPROM, 2 bytes, unsigned, native 0-4095 scale
ADDR_MAX_ANGLE_LIMIT = 11       # EEPROM, 2 bytes, unsigned, native 0-4095 scale
ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41
ADDR_GOAL_POSITION = 42
ADDR_GOAL_SPEED = 46
ADDR_POSITION_CORRECTION = 31   # EEPROM, 2 bytes, sign-magnitude, range -2047..+2047
ADDR_LOCK = 55
ADDR_PRESENT_POSITION = 56


def build_feetech_write(servo_id: int, address: int, data: bytes) -> bytes:
    # 0xFF 0xFF <ID> <LEN> <INSTR=0x03 WRITE> <ADDR> <DATA...> <CHECKSUM>
    body = bytes([servo_id, 3 + len(data), 0x03, address]) + data
    chk = feetech_checksum(body)
    return bytes([0xFF, 0xFF]) + body + bytes([chk])


def build_feetech_read(servo_id: int, address: int, length: int) -> bytes:
    # 0xFF 0xFF <ID> <LEN=4> <INSTR=0x02 READ> <ADDR> <LENGTH> <CHECKSUM>
    body = bytes([servo_id, 0x04, 0x02, address, length])
    chk = feetech_checksum(body)
    return bytes([0xFF, 0xFF]) + body + bytes([chk])


def parse_status_params(resp: bytes):
    """Pull the parameter bytes out of a Feetech status packet, if it's shaped right."""
    if len(resp) >= 6 and resp[0] == 0xFF and resp[1] == 0xFF:
        length = resp[3]
        params = resp[5:5 + (length - 2)]
        return params
    return None


def build_fe_style_ping(servo_id: int) -> bytes:
    # Best-effort guess at Elephant Robotics' internal framing style,
    # modeled after documented raw examples like:
    #   FE FE 06 21 01 23 28 14 FA   (set J1 angle command)
    # This is NOT confirmed as a real "ping" - it's just to see if this
    # framing gets *any* structured-looking response at all on this line.
    body = bytes([servo_id, 0x02, 0x01])
    chk = (~sum(body)) & 0xFF
    return bytes([0xFE, 0xFE]) + body + bytes([chk])


def read_response(ser: serial.Serial, wait: float = 0.05) -> bytes:
    time.sleep(wait)
    n = ser.in_waiting
    if n:
        return ser.read(n)
    return b""


# ---------------------------------------------------------------------------
# Mode implementations (unchanged behavior, just called from the menu)
# ---------------------------------------------------------------------------

def do_sniff(ser: serial.Serial, seconds: float):
    print(f"Sniffing raw serial traffic for {seconds:.0f}s. "
          f"Trigger movement / commands from elsewhere now if you can...")
    end = time.time() + seconds
    saw_anything = False
    while time.time() < end:
        n = ser.in_waiting
        if n:
            data = ser.read(n)
            saw_anything = True
            print(f"[{time.time():.3f}] {len(data)} bytes: {hexdump(data)}")
        time.sleep(0.02)
    if not saw_anything:
        print("No traffic seen at all during the sniff window.")


def do_ping(ser: serial.Serial, servo_id: int, verbose: bool = True, quiet: bool = False):
    packet = build_feetech_ping(servo_id)
    ser.reset_input_buffer()
    ser.write(packet)
    resp = read_response(ser)

    if quiet:
        # Used by the bus scanner: just report yes/no, no per-ID chatter.
        return len(resp) >= 6 and resp[0] == 0xFF and resp[1] == 0xFF

    if verbose:
        print(f"ID {servo_id:3d} | sent: {hexdump(packet)}")

    if not resp:
        print(f"ID {servo_id:3d} | no response (timeout)")
        return False

    print(f"ID {servo_id:3d} | RAW RESPONSE ({len(resp)} bytes): {hexdump(resp)}")

    # Quick sanity check against Feetech's expected status packet shape:
    # 0xFF 0xFF <ID> <LEN> <ERROR> [params...] <CHECKSUM>
    if len(resp) >= 6 and resp[0] == 0xFF and resp[1] == 0xFF:
        print(f"ID {servo_id:3d} | header matches Feetech 0xFF 0xFF - looks like a real Feetech-family reply")
        return True
    elif len(resp) >= 2 and resp[0] == 0xFE and resp[1] == 0xFE:
        print(f"ID {servo_id:3d} | header is 0xFE 0xFE - this looks like Elephant Robotics' own framing, "
              f"not raw Feetech servo protocol")
        return False
    else:
        print(f"ID {servo_id:3d} | header does not match either known framing - "
              f"could be noise, wrong baud, or garbled bytes")
        return False


def scan_servos(ser: serial.Serial, id_start: int = 1, id_end: int = 50):
    """Sweep a range of servo IDs and return the ones that give a real
    Feetech-shaped reply. This is the single source of truth the menu uses
    to build 'pick a servo' lists, so users never have to guess an ID."""
    print(f"\nScanning bus for servos (IDs {id_start}-{id_end})...")
    print("  ('.' = no reply, '!' = servo found)")
    print("  ", end="", flush=True)

    found = []
    for servo_id in range(id_start, id_end + 1):
        ok = do_ping(ser, servo_id, quiet=True)
        print("!" if ok else ".", end="", flush=True)
        if ok:
            found.append(servo_id)
        time.sleep(0.02)
    print()  # end the progress line

    if found:
        print(f"Found {len(found)} servo(s): {found}")
    else:
        print("No servos responded. Check wiring, power, port, and baud rate.")

    return found


def feetech_write(ser: serial.Serial, servo_id: int, address: int, data: bytes, wait: float = 0.05) -> bytes:
    packet = build_feetech_write(servo_id, address, data)
    ser.reset_input_buffer()
    ser.write(packet)
    return read_response(ser, wait)


def feetech_read(ser: serial.Serial, servo_id: int, address: int, length: int, wait: float = 0.05) -> bytes:
    packet = build_feetech_read(servo_id, address, length)
    ser.reset_input_buffer()
    ser.write(packet)
    return read_response(ser, wait)


def read_unsigned16(ser: serial.Serial, servo_id: int, address: int):
    """Read a plain (non sign-magnitude) little-endian 16-bit register."""
    resp = feetech_read(ser, servo_id, address, 2)
    params = parse_status_params(resp)
    if params and len(params) >= 2:
        return params[0] | (params[1] << 8)
    return None


def read_present_position(ser: serial.Serial, servo_id: int):
    return read_unsigned16(ser, servo_id, ADDR_PRESENT_POSITION)


def read_min_angle_limit(ser: serial.Serial, servo_id: int):
    return read_unsigned16(ser, servo_id, ADDR_MIN_ANGLE_LIMIT)


def read_max_angle_limit(ser: serial.Serial, servo_id: int):
    return read_unsigned16(ser, servo_id, ADDR_MAX_ANGLE_LIMIT)


SERVO_POSITION_MIN = 0
SERVO_POSITION_MAX = 4095


def move_to_absolute_position(ser: serial.Serial, servo_id: int, target: int, speed: int, accel: int,
                               settle_seconds: float = 1.5):
    """Write torque/accel/speed/goal-position to move a servo to an absolute
    position on the 0-4095 scale. Assumes the caller has already confirmed
    this is a real Feetech-family device. Returns the position read back
    after the move settles (or None if it couldn't be read)."""
    target = max(SERVO_POSITION_MIN, min(SERVO_POSITION_MAX, target))

    print("\nEnabling torque...")
    feetech_write(ser, servo_id, ADDR_TORQUE_ENABLE, bytes([1]))

    print(f"Setting acceleration = {accel}...")
    feetech_write(ser, servo_id, ADDR_ACCELERATION, bytes([accel & 0xFF]))

    print(f"Setting goal speed = {speed}...")
    feetech_write(ser, servo_id, ADDR_GOAL_SPEED, bytes([speed & 0xFF, (speed >> 8) & 0xFF]))

    print(f"Writing goal position = {target}...")
    feetech_write(ser, servo_id, ADDR_GOAL_POSITION, bytes([target & 0xFF, (target >> 8) & 0xFF]))

    print("Waiting for the move to settle...")
    time.sleep(settle_seconds)

    return read_present_position(ser, servo_id)


def do_read_position(ser: serial.Serial, servo_id: int):
    """Read-only: report a servo's configured min/max angle limits and its
    current position. Never writes anything, so no move/EEPROM confirmation
    is needed - just the usual ping safety check before trusting the reply."""
    print(f"--- position mode: servo ID {servo_id} ---")
    print("Step 1: pinging servo to confirm it's a real Feetech-family device...")
    ok = do_ping(ser, servo_id)

    if not ok:
        print(f"\nABORTING: ID {servo_id} did not return a valid Feetech-shaped (0xFF 0xFF ...) reply.")
        print("Refusing to trust register reads from an unconfirmed device.")
        return

    print(f"\nConfirmed: ID {servo_id} is responding as a Feetech-family servo.\n")

    min_limit = read_min_angle_limit(ser, servo_id)
    max_limit = read_max_angle_limit(ser, servo_id)
    current = read_present_position(ser, servo_id)

    def fmt(v):
        return str(v) if v is not None else "unknown (no/garbled reply)"

    print(f"Min angle limit:   {fmt(min_limit)}")
    print(f"Max angle limit:   {fmt(max_limit)}")
    print(f"Current position:  {fmt(current)}")
    print("(All values are on the servo's native 0-4095 position scale, not degrees.)")

    if min_limit == 0 and max_limit == 0:
        print("\nNote: min and max angle limits both read as 0. On Feetech servos this")
        print("usually means angle limits are disabled (continuous/wheel rotation mode)")
        print("rather than a real zero-width range, so there's no meaningful min/max here.")
    elif None not in (min_limit, max_limit, current):
        if min_limit <= current <= max_limit:
            span = max_limit - min_limit
            pct = ((current - min_limit) / span * 100) if span else 0.0
            print(f"\nCurrent position is within the configured range (~{pct:.0f}% of the way from min to max).")
        else:
            print("\nWarning: current position is outside the configured min/max angle limits.")


def do_zero(ser: serial.Serial, servo_id: int, center: int, speed: int, accel: int):
    print(f"--- zero mode: servo ID {servo_id} ---")
    print("Step 1: pinging servo to confirm it's a real Feetech-family device...")
    ok = do_ping(ser, servo_id)

    if not ok:
        print(f"\nABORTING: ID {servo_id} did not return a valid Feetech-shaped (0xFF 0xFF ...) reply.")
        print("Refusing to send any write/move commands to an unconfirmed device.")
        return

    print(f"\nConfirmed: ID {servo_id} is responding as a Feetech-family servo. Proceeding.\n")

    current = read_present_position(ser, servo_id)
    if current is not None:
        print(f"Current reported position: {current} (0-4095 scale, center is usually 2048)")
    else:
        print("Could not read current position (no/garbled response) - continuing anyway.")

    resp = input(f"\nThis will move servo ID {servo_id} to position {center} "
                 f"(speed={speed}, accel={accel}). Continue? [y/N] ").strip().lower()
    if resp != "y":
        print("Aborted by user.")
        return

    new_pos = move_to_absolute_position(ser, servo_id, center, speed, accel)

    if new_pos is not None:
        print(f"New reported position: {new_pos} (target was {center})")
        if abs(new_pos - center) <= 5:
            print("Looks centered/zeroed successfully.")
        else:
            print("Position doesn't match target closely - check mechanical limits, "
                  "torque, or whether the servo is under load.")
    else:
        print("Could not read back position after the move.")


def do_move_relative(ser: serial.Serial, servo_id: int, delta: int, speed: int, accel: int):
    """Move a servo by a relative amount (+/-) from wherever it currently is.
    Useful for quickly sanity-checking that a given servo ID actually moves,
    and in which direction, without needing to know its absolute zero point."""
    direction = "increase" if delta >= 0 else "decrease"
    print(f"--- move mode: servo ID {servo_id}, {direction} by {abs(delta)} ---")
    print("Step 1: pinging servo to confirm it's a real Feetech-family device...")
    ok = do_ping(ser, servo_id)

    if not ok:
        print(f"\nABORTING: ID {servo_id} did not return a valid Feetech-shaped (0xFF 0xFF ...) reply.")
        print("Refusing to send any write/move commands to an unconfirmed device.")
        return

    print(f"\nConfirmed: ID {servo_id} is responding as a Feetech-family servo. Proceeding.\n")

    current = read_present_position(ser, servo_id)
    if current is None:
        print("Could not read current position - can't safely compute a relative move.")
        print("Try 'ping' first to make sure the bus/baud/ID are correct, or use 'zero' "
              "mode instead, which targets an absolute position.")
        return

    target = current + delta
    clamped = max(SERVO_POSITION_MIN, min(SERVO_POSITION_MAX, target))
    print(f"Current position: {current}")
    if clamped != target:
        print(f"Requested target {target} is outside the 0-4095 range - clamping to {clamped}.")
    print(f"Target position:  {clamped}")

    resp = input(f"\nThis will move servo ID {servo_id} from {current} to {clamped} "
                 f"(speed={speed}, accel={accel}). Continue? [y/N] ").strip().lower()
    if resp != "y":
        print("Aborted by user.")
        return

    new_pos = move_to_absolute_position(ser, servo_id, clamped, speed, accel)

    if new_pos is not None:
        print(f"New reported position: {new_pos} (target was {clamped})")
        actual_delta = new_pos - current
        print(f"Actual change: {actual_delta:+d} (requested: {delta:+d})")
        if abs(new_pos - clamped) <= 5:
            print("Move looks like it landed on target.")
        else:
            print("Position doesn't match target closely - check mechanical limits, "
                  "torque, or whether the servo is under load.")
    else:
        print("Could not read back position after the move.")


def do_jog(ser: serial.Serial, servo_id: int, step: int, speed: int, accel: int):
    """Interactive jog loop: repeatedly nudge one servo by +/- step so you can
    watch which physical joint moves and confirm the direction, without
    re-entering the ID or re-pinging every time."""
    print(f"--- jog mode: servo ID {servo_id} (step size {step}) ---")
    print("Step 1: pinging servo to confirm it's a real Feetech-family device...")
    ok = do_ping(ser, servo_id)

    if not ok:
        print(f"\nABORTING: ID {servo_id} did not return a valid Feetech-shaped (0xFF 0xFF ...) reply.")
        print("Refusing to send any write/move commands to an unconfirmed device.")
        return

    current = read_present_position(ser, servo_id)
    if current is None:
        print("Could not read current position - aborting jog session.")
        return

    print(f"\nConfirmed: ID {servo_id} is responding as a Feetech-family servo.")
    print(f"Starting position: {current}")
    print("\nCommands: '+' move positive by step, '-' move negative by step,")
    print("          a number to set a new step size, 'q' to quit jogging this servo.\n")

    while True:
        raw = input(f"[ID {servo_id} @ {current}, step {step}] jog> ").strip().lower()

        if raw == "q":
            print("Exiting jog mode for this servo.")
            return

        if raw == "+":
            delta = step
        elif raw == "-":
            delta = -step
        elif raw.lstrip("+-").isdigit():
            # Typing a bare number changes the step size rather than moving,
            # so you can dial in a bigger/smaller nudge without leaving jog mode.
            step = int(raw)
            print(f"Step size set to {step}.")
            continue
        else:
            print("Unrecognized input. Use '+', '-', a number for step size, or 'q' to quit.")
            continue

        target = max(SERVO_POSITION_MIN, min(SERVO_POSITION_MAX, current + delta))
        if target == current:
            print("Already at a limit in that direction (0 or 4095) - no move sent.")
            continue

        new_pos = move_to_absolute_position(ser, servo_id, target, speed, accel, settle_seconds=0.6)
        if new_pos is not None:
            print(f"  -> now at {new_pos} (requested {target})")
            current = new_pos
        else:
            print("  -> could not read back position; assuming move happened as requested.")
            current = target


def encode_signed_11bit(value: int) -> bytes:
    """Feetech-style sign-magnitude 16-bit field: bit 11 = sign, bits 0-10 = magnitude."""
    value = max(-2047, min(2047, value))
    if value < 0:
        word = 0x0800 | (-value)
    else:
        word = value & 0x07FF
    return bytes([word & 0xFF, (word >> 8) & 0xFF])


def decode_signed_11bit(low: int, high: int) -> int:
    word = low | (high << 8)
    magnitude = word & 0x07FF
    if word & 0x0800:
        return -magnitude
    return magnitude


def read_position_correction(ser: serial.Serial, servo_id: int):
    resp = feetech_read(ser, servo_id, ADDR_POSITION_CORRECTION, 2)
    params = parse_status_params(resp)
    if params and len(params) >= 2:
        return decode_signed_11bit(params[0], params[1])
    return None


def do_set_zero(ser: serial.Serial, servo_id: int, target_center: int):
    print(f"--- set-zero mode: servo ID {servo_id} ---")
    print("This does NOT move the servo. It rewrites the servo's internal Position")
    print("Correction register so wherever it is RIGHT NOW gets reported as the new")
    print(f"center ({target_center}), the same operation Feetech's own SDK calls")
    print("CalibrationOfs / 'set center position'.\n")

    print("Step 1: pinging servo to confirm it's a real Feetech-family device...")
    ok = do_ping(ser, servo_id)
    if not ok:
        print(f"\nABORTING: ID {servo_id} did not return a valid Feetech-shaped (0xFF 0xFF ...) reply.")
        print("Refusing to write EEPROM calibration data to an unconfirmed device.")
        return

    print(f"\nConfirmed: ID {servo_id} is responding as a Feetech-family servo. Proceeding.\n")

    current_pos = read_present_position(ser, servo_id)
    old_correction = read_position_correction(ser, servo_id)

    if current_pos is None or old_correction is None:
        print("Could not read current position and/or existing Position Correction value.")
        print("Aborting - it's not safe to compute a new offset without both readings.")
        return

    print(f"Current reported position:      {current_pos}")
    print(f"Existing Position Correction:   {old_correction}")

    # Present Position already reflects the OLD correction. We want the physical
    # position to newly report as `target_center` without moving anything, so:
    #   new_correction = old_correction + (current_pos - target_center)
    new_correction = old_correction + (current_pos - target_center)
    new_correction = max(-2047, min(2047, new_correction))

    print(f"Calculated new Position Correction: {new_correction}")
    print("\nNOTE: this register's sign convention is reconstructed from Feetech's")
    print("documented control table, not verified against original SDK source. After")
    print("writing, this script re-reads the position and tells you whether it landed")
    print(f"near the target ({target_center}) - if it's off (e.g. moved further away, or")
    print("landed at roughly double the expected offset), the sign convention on this")
    print("servo model is likely flipped - stop and don't repeat the write blindly.\n")

    resp = input(f"Write new Position Correction = {new_correction} to ID {servo_id}'s EEPROM? [y/N] ").strip().lower()
    if resp != "y":
        print("Aborted by user.")
        return

    print("\nUnlocking EEPROM (Lock = 0)...")
    feetech_write(ser, servo_id, ADDR_LOCK, bytes([0]))

    print(f"Writing Position Correction = {new_correction}...")
    feetech_write(ser, servo_id, ADDR_POSITION_CORRECTION, encode_signed_11bit(new_correction))

    print("Re-locking EEPROM (Lock = 1)...")
    feetech_write(ser, servo_id, ADDR_LOCK, bytes([1]))

    time.sleep(0.2)
    verify_pos = read_present_position(ser, servo_id)
    verify_correction = read_position_correction(ser, servo_id)

    print(f"\nAfter write: reported position = {verify_pos}, Position Correction = {verify_correction}")
    if verify_pos is not None:
        if abs(verify_pos - target_center) <= 5:
            print(f"Success: position now reports as ~{target_center}, matching target.")
        else:
            print(f"MISMATCH: expected ~{target_center}, got {verify_pos}.")
            print("Do not blindly repeat this write. Consider re-checking the sign")
            print("convention or reverting Position Correction to the old value:")
            print(f"    old value was: {old_correction}")


def do_ping_fe(ser: serial.Serial, servo_id: int):
    packet = build_fe_style_ping(servo_id)
    ser.reset_input_buffer()
    ser.write(packet)
    resp = read_response(ser)
    print(f"Sent 0xFE-style packet to ID {servo_id}: {hexdump(packet)}")
    if not resp:
        print("No response.")
        return
    print(f"RAW RESPONSE ({len(resp)} bytes): {hexdump(resp)}")


# ---------------------------------------------------------------------------
# Menu / interactive input helpers
# ---------------------------------------------------------------------------

COMMON_BAUD_RATES = [9600, 19200, 38400, 57600, 115200, 500000, 1000000]


def detect_os() -> str:
    """Return a short, human-friendly OS label: 'Windows', 'Linux', 'macOS', or 'Unknown'."""
    system = platform.system()
    if system == "Darwin":
        return "macOS"
    if system in ("Windows", "Linux"):
        return system
    return system or "Unknown"


def discover_serial_ports():
    """Return a list of (device_path, description) tuples for serial ports
    that actually exist on this machine right now. Combines pyserial's
    cross-platform enumeration (which reliably finds USB-attached adapters
    on Windows/Linux/macOS) with OS-specific device-node globs, since ports
    like a Raspberry Pi's onboard UART (/dev/ttyAMA0) or a motherboard COM
    header often don't show up in USB-based enumeration at all."""
    found = {}

    # 1) pyserial's own enumeration - works well for USB-to-serial adapters
    #    (FTDI, CP210x, CH340, etc.) on every platform, and for Windows COM
    #    ports in general.
    try:
        for p in list_ports.comports():
            desc = p.description if p.description and p.description != "n/a" else ""
            found[p.device] = desc
    except Exception:
        pass

    system = platform.system()

    if system == "Linux":
        # Onboard/UART devices and other real device nodes that pyserial's
        # USB-based enumeration frequently misses (e.g. Raspberry Pi GPIO
        # UART is /dev/ttyAMA0, or /dev/serial0 symlinked to it).
        patterns = [
            "/dev/ttyUSB*",
            "/dev/ttyACM*",
            "/dev/ttyAMA*",
            "/dev/serial0",
            "/dev/serial1",
            "/dev/serial/by-id/*",
        ]
        for pattern in patterns:
            for path in glob.glob(pattern):
                real_path = os.path.realpath(path)
                if os.path.exists(real_path) and real_path not in found:
                    found[real_path] = found.get(path, "")
                if path not in found and os.path.exists(path):
                    found[path] = ""

    elif system == "Darwin":
        for pattern in ("/dev/cu.*", "/dev/tty.*"):
            for path in glob.glob(pattern):
                if path not in found and os.path.exists(path):
                    found[path] = ""

    # On Windows, list_ports.comports() is the reliable source of truth for
    # COM ports, so there's no extra glob step - device nodes like /dev/*
    # don't apply there.

    return sorted(found.items())


def select_port() -> str:
    os_label = detect_os()
    print(f"\nDetected OS: {os_label}")

    ports = discover_serial_ports()
    options_display = []
    for device, desc in ports:
        label = f"{device} - {desc}" if desc else device
        options_display.append(label)
    options_display.append("Type in a port manually")

    if not ports:
        print("No serial ports were auto-detected. Make sure the device is plugged in / "
              "powered, or enter a path manually below.")

    print("\nSelect the serial port")
    for i, opt in enumerate(options_display, start=1):
        print(f"  {i}) {opt}")

    while True:
        raw = input("Select an option: ").strip()
        if not raw.isdigit():
            print("Please enter a number from the list.")
            continue
        idx = int(raw)
        if 1 <= idx <= len(ports):
            return ports[idx - 1][0]
        if idx == len(options_display):
            default_hint = "/dev/ttyAMA0" if os_label == "Linux" else ("COM3" if os_label == "Windows" else "/dev/cu.usbserial-XXXX")
            return input(f"Enter the port (e.g. {default_hint}): ").strip()
        print("Out of range, try again.")


def prompt_choice(title: str, options: list, allow_custom: bool = False, custom_prompt: str = "Enter custom value: "):
    """Print a numbered menu and return the chosen value (as a string)."""
    print(f"\n{title}")
    for i, opt in enumerate(options, start=1):
        print(f"  {i}) {opt}")
    if allow_custom:
        print(f"  {len(options) + 1}) Other / type it in")

    while True:
        raw = input("Select an option: ").strip()
        if not raw.isdigit():
            print("Please enter a number from the list.")
            continue
        idx = int(raw)
        if 1 <= idx <= len(options):
            return options[idx - 1]
        if allow_custom and idx == len(options) + 1:
            return input(custom_prompt).strip()
        print("Out of range, try again.")


def prompt_int(prompt: str, default: int) -> int:
    raw = input(f"{prompt} [{default}]: ").strip()
    if raw == "":
        return default
    try:
        return int(raw)
    except ValueError:
        print(f"Not a valid number, using default ({default}).")
        return default


def prompt_float(prompt: str, default: float) -> float:
    raw = input(f"{prompt} [{default}]: ").strip()
    if raw == "":
        return default
    try:
        return float(raw)
    except ValueError:
        print(f"Not a valid number, using default ({default}).")
        return default


def prompt_yes_no(prompt: str, default_no: bool = True) -> bool:
    suffix = "[y/N]" if default_no else "[Y/n]"
    raw = input(f"{prompt} {suffix}: ").strip().lower()
    if raw == "":
        return not default_no
    return raw == "y"


def select_baud() -> int:
    choice = prompt_choice(
        "Select baud rate",
        [str(b) for b in COMMON_BAUD_RATES],
        allow_custom=True,
        custom_prompt="Enter custom baud rate: ",
    )
    try:
        return int(choice)
    except ValueError:
        print("Invalid baud rate entered, defaulting to 1000000.")
        return 1_000_000


def select_servo_id(ser: serial.Serial, scan_state: dict, purpose: str) -> int:
    """Force a servo pick from the bus scan instead of letting the user
    guess/type an ID. Scans automatically the first time it's needed, and
    always offers a rescan in case a servo was just plugged in or powered."""
    while True:
        ids = scan_state.get("ids")
        if ids is None:
            ids = scan_servos(ser)
            scan_state["ids"] = ids

        if not ids:
            print(f"\nNo servos are currently detected on the bus, so there's nothing to pick for '{purpose}'.")
            choice = prompt_choice(
                "What would you like to do?",
                ["Rescan the bus", "Enter a servo ID manually (not detected - use with caution)"],
            )
            if choice.startswith("Rescan"):
                scan_state["ids"] = scan_servos(ser)
                continue
            return prompt_int(f"Servo ID to use for {purpose}", 1)

        options = [f"ID {i}" for i in ids] + ["Rescan the bus"]
        choice = prompt_choice(f"Select a servo for {purpose} (detected on the bus)", options)

        if choice == "Rescan the bus":
            scan_state["ids"] = scan_servos(ser)
            continue

        return int(choice.split()[1])


MODE_DESCRIPTIONS = [
    ("sniff", "Sniff raw traffic on the line"),
    ("scan", "Scan/rescan the bus for connected servos"),
    ("ping", "Ping a single servo (Feetech-style)"),
    ("ping-fe", "Ping a single servo (Elephant Robotics 0xFE framing)"),
    ("position", "Read a servo's current position and min/max angle limits (read-only)"),
    ("move", "Move a servo by +/- an amount from its current position"),
    ("jog", "Interactively nudge a servo back and forth to test a joint"),
    ("zero", "Move a servo to center/zero position (writes + moves hardware)"),
    ("set-zero", "Rewrite a servo's zero-offset EEPROM register without moving it"),
]


def select_mode() -> str:
    print("\nSelect a mode")
    for i, (key, desc) in enumerate(MODE_DESCRIPTIONS, start=1):
        print(f"  {i}) {key:10s} - {desc}")
    while True:
        raw = input("Select an option: ").strip()
        if not raw.isdigit():
            print("Please enter a number from the list.")
            continue
        idx = int(raw)
        if 1 <= idx <= len(MODE_DESCRIPTIONS):
            return MODE_DESCRIPTIONS[idx - 1][0]
        print("Out of range, try again.")


def open_serial(port: str, baud: int, timeout: float):
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
        print(f"\nOpened {port} @ {baud} baud")
        return ser
    except serial.SerialException as e:
        print(f"Could not open {port} at {baud} baud: {e}")
        return None


def run_mode(ser: serial.Serial, mode: str, scan_state: dict):
    if mode == "sniff":
        seconds = prompt_float("How many seconds to sniff for", 10.0)
        do_sniff(ser, seconds)

    elif mode == "scan":
        scan_state["ids"] = scan_servos(ser)

    elif mode == "ping":
        servo_id = select_servo_id(ser, scan_state, "ping")
        do_ping(ser, servo_id)

    elif mode == "ping-fe":
        servo_id = select_servo_id(ser, scan_state, "ping (0xFE framing)")
        do_ping_fe(ser, servo_id)

    elif mode == "position":
        servo_id = select_servo_id(ser, scan_state, "read position")
        do_read_position(ser, servo_id)

    elif mode == "move":
        servo_id = select_servo_id(ser, scan_state, "move")
        direction = prompt_choice("Direction", ["increase (+)", "decrease (-)"])
        amount = prompt_int("Amount to move (0-4095 scale, e.g. 100-300 for a small test nudge)", 100)
        delta = amount if direction.startswith("increase") else -amount
        speed = prompt_int("Goal speed (0-3400ish, conservative default)", 200)
        accel = prompt_int("Acceleration (0-254, conservative default)", 20)
        do_move_relative(ser, servo_id, delta, speed, accel)

    elif mode == "jog":
        servo_id = select_servo_id(ser, scan_state, "jog")
        step = prompt_int("Initial step size (0-4095 scale)", 50)
        speed = prompt_int("Goal speed (0-3400ish, conservative default)", 200)
        accel = prompt_int("Acceleration (0-254, conservative default)", 20)
        do_jog(ser, servo_id, step, speed, accel)

    elif mode == "zero":
        servo_id = select_servo_id(ser, scan_state, "zero")
        center = prompt_int("Target center position", 2048)
        speed = prompt_int("Goal speed (0-3400ish, conservative default)", 200)
        accel = prompt_int("Acceleration (0-254, conservative default)", 20)
        do_zero(ser, servo_id, center, speed, accel)

    elif mode == "set-zero":
        servo_id = select_servo_id(ser, scan_state, "set-zero")
        center = prompt_int("Target center value to write", 2048)
        do_set_zero(ser, servo_id, center)


def main():
    print("=" * 70)
    print(" mycobot_bus_probe - interactive menu")
    print(" Feetech vs Elephant Robotics serial bus diagnostic tool")
    print("=" * 70)

    port = select_port()
    baud = select_baud()
    timeout = prompt_float("Serial read timeout in seconds", 0.2)

    ser = open_serial(port, baud, timeout)
    if ser is None:
        sys.exit(1)

    # Scan for servos right away so every later menu can offer a real,
    # detected list of IDs instead of making the user guess one.
    scan_state = {"ids": scan_servos(ser)}

    try:
        while True:
            mode = select_mode()
            try:
                run_mode(ser, mode, scan_state)
            except serial.SerialException as e:
                print(f"Serial error while running '{mode}': {e}")

            if not prompt_yes_no("\nRun another action on this same connection?", default_no=False):
                break
    finally:
        ser.close()
        print("\nSerial port closed. Goodbye.")


if __name__ == "__main__":
    main()
