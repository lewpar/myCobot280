#!/usr/bin/env python3
"""
arm_server.py - TCP server for myCobot280 arm control.

Listens for TCP connections and relays commands to the arm via mycobot280.
One client at a time. Heartbeat kicks unresponsive clients after ~30s.

Commands:
    SCAN                         -> OK <id1,id2,...>
    COUNT                        -> <n>
    POS <id>                     -> <position>
    LIMITS <id>                  -> <min>,<max>
    INFO <id>                    -> pos:<pos> min:<min> max:<max>
    MOVE <id> <pos> [speed] [accel]   -> OK <new_pos>
    MOVE_REL <id> <delta> [speed] [accel]  -> OK <new_pos>
    TORQUE <id> <0|1>            -> OK
    CENTER <id> [pos] [speed] [accel]  -> OK <new_pos>
    SET_CENTER <id> [target]     -> OK (writes to servo EEPROM, no move)
    GET_CENTER <id>              -> <center> correction=<n>
    PING <id>                    -> OK / ERR no response
    ATOM_PING                    -> OK / ERR no response
    ATOM_COLOR <r> <g> <b>       -> OK
    ATOM_PIXEL <x> <y> <r> <g> <b> -> OK
    QUIT                         -> BYE
"""

import argparse
import signal
import socket
import sys
import threading
import time

from mycobot280 import MyCobot280


# ---------------------------------------------------------------------------
# SCS servo EEPROM helpers
# ---------------------------------------------------------------------------

_ADDR_POSITION_CORRECTION = 31   # EEPROM, 2 bytes, sign-magnitude, range -2047..+2047
_ADDR_LOCK = 55


def _encode_signed_11bit(value: int) -> bytes:
    """Feetech sign-magnitude 16-bit: bit 11 = sign, bits 0-10 = magnitude."""
    value = max(-2047, min(2047, value))
    if value < 0:
        word = 0x0800 | (-value)
    else:
        word = value & 0x07FF
    return bytes([word & 0xFF, (word >> 8) & 0xFF])


def _decode_signed_11bit(low: int, high: int) -> int:
    word = low | (high << 8)
    magnitude = word & 0x07FF
    if word & 0x0800:
        return -magnitude
    return magnitude


def _read_correction(bus, servo_id: int) -> int | None:
    """Read the Position Correction register from a servo's EEPROM."""
    with bus._lock:
        # Build read packet for address 31, length 2
        body = bytes([servo_id, 0x04, 0x02, _ADDR_POSITION_CORRECTION, 2])
        pkt = bytes([0xFF, 0xFF]) + body + bytes([((~sum(body)) & 0xFF)])
        bus._ser.reset_input_buffer()
        bus._ser.write(pkt)
        time.sleep(0.05)
        n = bus._ser.in_waiting
        resp = bus._ser.read(n) if n else b""
    if (len(resp) >= 7 and resp[0] == 0xFF and resp[1] == 0xFF
            and resp[3] >= 4):
        params = resp[5:5 + (resp[3] - 2)]
        return _decode_signed_11bit(params[0], params[1])
    return None


def _read_present_position(bus, servo_id: int) -> int | None:
    """Read present position (address 56, 2 bytes unsigned)."""
    with bus._lock:
        body = bytes([servo_id, 0x04, 0x02, 56, 2])
        pkt = bytes([0xFF, 0xFF]) + body + bytes([((~sum(body)) & 0xFF)])
        bus._ser.reset_input_buffer()
        bus._ser.write(pkt)
        time.sleep(0.05)
        n = bus._ser.in_waiting
        resp = bus._ser.read(n) if n else b""
    if (len(resp) >= 7 and resp[0] == 0xFF and resp[1] == 0xFF
            and resp[3] >= 4):
        params = resp[5:5 + (resp[3] - 2)]
        return params[0] | (params[1] << 8)
    return None


# ---------------------------------------------------------------------------
# Client connection handler
# ---------------------------------------------------------------------------

def handle_client(conn: socket.socket, addr: tuple, arm: MyCobot280,
                  active_conn: threading.Lock):
    conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    try:
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 10)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 5)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
    except (AttributeError, OSError):
        pass

    def reply(msg: str):
        try:
            conn.sendall((msg.rstrip() + "\r\n").encode())
        except OSError:
            pass

    heartbeat_stop = threading.Event()

    def heartbeat():
        while not heartbeat_stop.is_set():
            heartbeat_stop.wait(15)
            if heartbeat_stop.is_set():
                return
            try:
                conn.sendall(b"PING\r\n")
            except OSError:
                return

    threading.Thread(target=heartbeat, daemon=True).start()

    try:
        buf = b""
        while True:
            try:
                conn.settimeout(1.0)
                data = conn.recv(1024)
            except socket.timeout:
                continue
            if not data:
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                cmd = line.decode().strip()
                if not cmd or cmd.upper() == "PONG":
                    continue

                parts = cmd.split()
                op = parts[0].upper()

                # ---- SCAN ----
                if op == "SCAN":
                    ids = arm.scan()
                    reply(f"OK {','.join(str(i) for i in ids) if ids else ''}")

                # ---- COUNT ----
                elif op == "COUNT":
                    reply(str(arm.servo_count))

                # ---- POS <id> ----
                elif op == "POS":
                    if len(parts) < 2:
                        reply("ERR usage: POS <id>")
                        continue
                    pos = arm.get_position(int(parts[1]))
                    reply(str(pos) if pos is not None else "ERR no response")

                # ---- LIMITS <id> ----
                elif op == "LIMITS":
                    if len(parts) < 2:
                        reply("ERR usage: LIMITS <id>")
                        continue
                    lo, hi = arm.get_limits(int(parts[1]))
                    reply(f"{lo},{hi}")

                # ---- INFO <id> ----
                elif op == "INFO":
                    if len(parts) < 2:
                        reply("ERR usage: INFO <id>")
                        continue
                    sid = int(parts[1])
                    pos = arm.get_position(sid)
                    lo, hi = arm.get_limits(sid)
                    if pos is None:
                        reply("ERR no response")
                    else:
                        reply(f"pos:{pos} min:{lo} max:{hi}")

                # ---- MOVE <id> <pos> [speed] [accel] ----
                elif op == "MOVE":
                    if len(parts) < 3:
                        reply("ERR usage: MOVE <id> <pos> [speed] [accel]")
                        continue
                    sid    = int(parts[1])
                    target = int(parts[2])
                    speed  = int(parts[3]) if len(parts) > 3 else 600
                    accel  = int(parts[4]) if len(parts) > 4 else 20
                    ok, pos = arm.move(sid, target, speed, accel)
                    reply(f"OK {pos}" if ok else f"ERR move failed, pos={pos}")

                # ---- MOVE_REL <id> <delta> [speed] [accel] ----
                elif op == "MOVE_REL":
                    if len(parts) < 3:
                        reply("ERR usage: MOVE_REL <id> <delta> [speed] [accel]")
                        continue
                    sid   = int(parts[1])
                    delta = int(parts[2])
                    speed = int(parts[3]) if len(parts) > 3 else 600
                    accel = int(parts[4]) if len(parts) > 4 else 20
                    ok, pos = arm.move_rel(sid, delta, speed, accel)
                    reply(f"OK {pos}" if ok else "ERR move failed")

                # ---- TORQUE <id> <0|1> ----
                elif op == "TORQUE":
                    if len(parts) < 3:
                        reply("ERR usage: TORQUE <id> <0|1>")
                        continue
                    arm.set_torque(int(parts[1]), parts[2] == "1")
                    reply("OK")

                # ---- CENTER <id> [pos] [speed] [accel] ----
                elif op == "CENTER":
                    if len(parts) < 2:
                        reply("ERR usage: CENTER <id> [pos] [speed] [accel]")
                        continue
                    sid  = int(parts[1])
                    spd  = int(parts[3]) if len(parts) > 3 else 600
                    accl = int(parts[4]) if len(parts) > 4 else 20
                    if len(parts) > 2:
                        pos = int(parts[2])
                    else:
                        # No position given — read the servo's Position
                        # Correction register and compute the hardware center.
                        # center = 2048 - correction
                        correction = _read_correction(arm._bus, sid)
                        if correction is not None:
                            pos = 2048 - correction
                        else:
                            pos = 2048  # fallback
                    ok, new_pos = arm.center(sid, pos, spd, accl)
                    reply(f"OK {new_pos}" if ok else f"ERR center failed, pos={new_pos}")

                # ---- SET_CENTER <id> [target] ----
                # Rewrites the servo's Position Correction EEPROM register so
                # the current physical position maps to the target center
                # (default 2048).  This does NOT move the servo.
                elif op == "SET_CENTER":
                    if len(parts) < 2:
                        reply("ERR usage: SET_CENTER <id> [target_center]")
                        continue
                    sid = int(parts[1])
                    target_center = int(parts[2]) if len(parts) > 2 else 2048

                    # 1. Read current reported position
                    current_pos = _read_present_position(arm._bus, sid)
                    if current_pos is None:
                        reply(f"ERR could not read position of servo {sid}")
                        continue

                    # 2. Read existing Position Correction
                    old_correction = _read_correction(arm._bus, sid)
                    if old_correction is None:
                        reply(f"ERR could not read correction of servo {sid}")
                        continue

                    # 3. Calculate new correction so current_pos -> target_center
                    new_correction = old_correction + (current_pos - target_center)
                    new_correction = max(-2047, min(2047, new_correction))

                    # 4. Unlock EEPROM, write, re-lock
                    with arm._bus._lock:
                        arm._bus._write_raw(sid, _ADDR_LOCK, bytes([0]))
                        arm._bus._write_raw(sid, _ADDR_POSITION_CORRECTION,
                                            _encode_signed_11bit(new_correction))
                        arm._bus._write_raw(sid, _ADDR_LOCK, bytes([1]))
                    time.sleep(0.1)

                    # 5. Verify — read back position
                    verify_pos = _read_present_position(arm._bus, sid)

                    if verify_pos is not None and abs(verify_pos - target_center) <= 5:
                        reply(f"OK servo {sid} center set to {target_center} "
                              f"(correction={new_correction}, verified={verify_pos})")
                    else:
                        reply(f"WARN servo {sid} correction written ({new_correction}) "
                              f"but verify={verify_pos} != target {target_center}")

                # ---- GET_CENTER <id> ----
                elif op == "GET_CENTER":
                    if len(parts) < 2:
                        reply("ERR usage: GET_CENTER <id>")
                        continue
                    sid = int(parts[1])
                    correction = _read_correction(arm._bus, sid)
                    if correction is not None:
                        hardware_center = 2048 - correction
                        reply(f"{hardware_center} correction={correction}")
                    else:
                        reply(f"ERR could not read servo {sid}")

                # ---- PING <id> ----
                elif op == "PING":
                    if len(parts) < 2:
                        reply("ERR usage: PING <id>")
                        continue
                    reply("OK" if arm.servo_ping(int(parts[1])) else "ERR no response")

                # ---- ATOM_PING ----
                elif op == "ATOM_PING":
                    reply("OK" if arm.atom.ping() else "ERR no response")

                # ---- ATOM_COLOR <r> <g> <b> ----
                elif op == "ATOM_COLOR":
                    if len(parts) < 4:
                        reply("ERR usage: ATOM_COLOR <r> <g> <b>")
                        continue
                    arm.atom.set_color(int(parts[1]), int(parts[2]), int(parts[3]))
                    reply("OK")

                # ---- ATOM_PIXEL <x> <y> <r> <g> <b> ----
                elif op == "ATOM_PIXEL":
                    if len(parts) < 6:
                        reply("ERR usage: ATOM_PIXEL <x> <y> <r> <g> <b>")
                        continue
                    arm.atom.pixel(int(parts[1]), int(parts[2]),
                                   int(parts[3]), int(parts[4]), int(parts[5]))
                    reply("OK")

                # ---- QUIT ----
                elif op == "QUIT":
                    reply("BYE")
                    conn.close()
                    return

                else:
                    reply(f"ERR unknown command: {op}")

    except (ConnectionResetError, BrokenPipeError, OSError):
        pass
    finally:
        heartbeat_stop.set()
        try:
            conn.close()
        except OSError:
            pass
        active_conn.release()
        print(f"Client disconnected: {addr}")


# ---------------------------------------------------------------------------
# TCP server
# ---------------------------------------------------------------------------

def run_server(host: str, port: int, arm: MyCobot280):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.listen(1)
    print(f"Arm server listening on {host}:{port}")
    print(f"Detected servos: {arm.servo_ids}")

    running = True
    active_conn = threading.Lock()

    def shutdown(signum, frame):
        nonlocal running
        print("\nShutting down...")
        running = False
        sock.close()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        while running:
            try:
                sock.settimeout(1.0)
                conn, addr = sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            if not active_conn.acquire(blocking=False):
                try:
                    conn.sendall(b"BUSY - only one client at a time\r\n")
                except OSError:
                    pass
                conn.close()
                print(f"Rejected {addr}: server busy")
                continue

            print(f"Client connected: {addr}")
            t = threading.Thread(target=handle_client,
                                 args=(conn, addr, arm, active_conn), daemon=True)
            t.start()
    finally:
        print("Closing serial port...")
        arm.close()
        print("Done.")


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="myCobot280 Arm Server")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address")
    parser.add_argument("--port", type=int, default=5000, help="TCP port")
    parser.add_argument("--serial-port", default="/dev/ttyAMA0", help="Serial port")
    parser.add_argument("--serial-baud", type=int, default=1_000_000, help="Baud rate")
    args = parser.parse_args()

    try:
        arm = MyCobot280(args.serial_port, args.serial_baud)
    except Exception as e:
        print(f"Cannot open {args.serial_port}: {e}", file=sys.stderr)
        sys.exit(1)

    run_server(args.host, args.port, arm)


if __name__ == "__main__":
    main()
