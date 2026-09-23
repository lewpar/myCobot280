#!/usr/bin/env python3
"""
arm_server.py - TCP server for myCobot280 arm control.

Listens for TCP connections and relays commands to the arm via mycobot280.
One client at a time. Heartbeat kicks unresponsive clients after ~30s.

The first command must be AUTH with the server password (MYCOBOT_PASSWORD, --password, or the
one printed at startup). Three wrong attempts close the connection.

Commands:
    AUTH <password>              -> OK / ERR wrong password
    SCAN                         -> OK <id1,id2,...>
    COUNT                        -> <n>
    POS <id>                     -> <position>
    LIMITS <id>                  -> <min>,<max>
    INFO <id>                    -> pos:<pos> min:<min> max:<max>
    MOVE <id> <pos> [speed] [accel]   -> OK <new_pos>
    MOVE_REL <id> <delta> [speed] [accel]  -> OK <new_pos>
    TORQUE <id> <0|1>            -> OK
    CENTER <id> [pos] [speed] [accel]  -> OK <new_pos>
    SET_CENTER <id> <pos>        -> OK (saved to center_positions.json)
    GET_CENTER <id>              -> <pos> or ERR
    STOP                         -> OK (hold every joint; moves refused until RESUME)
    RESUME                       -> OK
    PING <id>                    -> OK / ERR no response
    ATOM_PING                    -> OK / ERR no response
    ATOM_COLOR <r> <g> <b>       -> OK
    ATOM_PIXEL <x> <y> <r> <g> <b> -> OK
    QUIT                         -> BYE

Moves are refused while stopped, and refused if the pose or the path to it would hit the table,
the base or the arm itself (see arm_model.py).
"""

import argparse
import hmac
import os
import secrets
import signal
import socket
import sys
import threading
import time

import arm_model as model
from mycobot280 import MyCobot280, SPEED_MIN, SPEED_MAX, ACCEL_MIN, ACCEL_MAX

_stopped = threading.Event()
MAX_AUTH_TRIES = 3


class BadArgs(Exception):
    pass


def _int(parts, i, name, lo, hi, default=None):
    """Parse parts[i] as an int in [lo, hi]; use ``default`` when missing (None = required)."""
    if i >= len(parts):
        if default is None:
            raise BadArgs(f"missing {name}")
        return default
    try:
        v = int(parts[i])
    except ValueError:
        raise BadArgs(f"{name} must be a whole number, got {parts[i]!r}")
    if not lo <= v <= hi:
        raise BadArgs(f"{name} must be between {lo} and {hi}, got {v}")
    return v


def _speed_accel(parts, i):
    return (_int(parts, i, "speed", SPEED_MIN, SPEED_MAX, 600),
            _int(parts, i + 1, "accel", ACCEL_MIN, ACCEL_MAX, 20))


def _guard(arm: MyCobot280, sid: int, target: int) -> str | None:
    """Reason a move must be refused, or None."""
    if _stopped.is_set():
        return "arm is stopped, send RESUME first"
    if sid not in model.JOINT_IDS:
        return None
    new = [target if s == sid else None for s in model.JOINT_IDS]
    why = model.check_tick_move(model.load_calibration(), arm.read_positions(model.JOINT_IDS), new)
    return f"move refused, {why}" if why else None


def _move(arm, sid, target, speed, accel):
    lo, hi = arm.get_limits(sid)
    target = max(lo, min(hi, target))
    why = _guard(arm, sid, target)
    if why:
        return f"ERR {why}"
    ok, pos = arm.move(sid, target, speed, accel, should_abort=_stopped.is_set)
    return f"OK {pos}" if ok else f"ERR move failed, pos={pos}"


def run_command(arm: MyCobot280, parts: list[str]) -> str:
    op = parts[0].upper()
    if op == "SCAN":
        ids = arm.scan()
        return f"OK {','.join(str(i) for i in ids) if ids else ''}"
    if op == "COUNT":
        return str(arm.servo_count)
    if op == "POS":
        pos = arm.get_position(_int(parts, 1, "id", 1, 253))
        return str(pos) if pos is not None else "ERR no response"
    if op == "LIMITS":
        lo, hi = arm.get_limits(_int(parts, 1, "id", 1, 253))
        return f"{lo},{hi}"
    if op == "INFO":
        sid = _int(parts, 1, "id", 1, 253)
        pos = arm.get_position(sid)
        lo, hi = arm.get_limits(sid)
        return "ERR no response" if pos is None else f"pos:{pos} min:{lo} max:{hi}"
    if op == "MOVE":
        sid = _int(parts, 1, "id", 1, 253)
        target = _int(parts, 2, "position", 0, 4095)
        return _move(arm, sid, target, *_speed_accel(parts, 3))
    if op == "MOVE_REL":
        sid = _int(parts, 1, "id", 1, 253)
        delta = _int(parts, 2, "delta", -4095, 4095)
        speed, accel = _speed_accel(parts, 3)
        cur = arm.get_position(sid)
        if cur is None:
            return "ERR no response"
        return _move(arm, sid, cur + delta, speed, accel)
    if op == "TORQUE":
        sid = _int(parts, 1, "id", 1, 253)
        on = _int(parts, 2, "state (0 or 1)", 0, 1)
        arm.set_torque(sid, bool(on))
        return "OK"
    if op == "CENTER":
        sid = _int(parts, 1, "id", 1, 253)
        pos = _int(parts, 2, "position", 0, 4095, model.load_centers().get(sid, 2048))
        return _move(arm, sid, pos, *_speed_accel(parts, 3))
    if op == "SET_CENTER":
        sid = _int(parts, 1, "id", 1, 253)
        pos = _int(parts, 2, "position", 0, 4095)
        centers = model.load_centers()
        centers[sid] = pos
        model.save_centers(centers)
        return f"OK center for servo {sid} set to {pos}"
    if op == "GET_CENTER":
        sid = _int(parts, 1, "id", 1, 253)
        pos = model.load_centers().get(sid)
        return str(pos) if pos is not None else f"ERR no saved center for servo {sid}"
    if op == "STOP":
        _stopped.set()
        arm.hold(arm.servo_ids or model.JOINT_IDS)
        return "OK stopped"
    if op == "RESUME":
        _stopped.clear()
        return "OK"
    if op == "PING":
        return "OK" if arm.servo_ping(_int(parts, 1, "id", 1, 253)) else "ERR no response"
    if op == "ATOM_PING":
        return "OK" if arm.atom.ping() else "ERR no response"
    if op == "ATOM_COLOR":
        r, g, b = (_int(parts, i, n, 0, 255) for i, n in ((1, "r"), (2, "g"), (3, "b")))
        arm.atom.set_color(r, g, b)
        return "OK"
    if op == "ATOM_PIXEL":
        x = _int(parts, 1, "x", 0, 4)
        y = _int(parts, 2, "y", 0, 4)
        r, g, b = (_int(parts, i, n, 0, 255) for i, n in ((3, "r"), (4, "g"), (5, "b")))
        arm.atom.pixel(x, y, r, g, b)
        return "OK"
    return f"ERR unknown command: {op}"


# ---------------------------------------------------------------------------
# Client connection handler
# ---------------------------------------------------------------------------

def handle_client(conn: socket.socket, addr: tuple, arm: MyCobot280,
                  active_conn: threading.Lock, password: str):
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
    authed, tries = False, 0

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
                cmd = line.decode(errors="replace").strip()
                if not cmd or cmd.upper() == "PONG":
                    continue
                parts = cmd.split()
                op = parts[0].upper()

                if op == "QUIT":
                    reply("BYE")
                    return
                if op == "AUTH":
                    given = cmd.split(None, 1)[1] if len(parts) > 1 else ""
                    if hmac.compare_digest(given.encode(), password.encode()):
                        authed = True
                        reply("OK")
                    else:
                        tries += 1
                        time.sleep(0.5)
                        reply("ERR wrong password")
                        if tries >= MAX_AUTH_TRIES:
                            print(f"Closing {addr}: too many wrong passwords")
                            return
                    continue
                if not authed:
                    reply("ERR auth required: send AUTH <password> first")
                    continue
                try:
                    reply(run_command(arm, parts))
                except BadArgs as e:
                    reply(f"ERR {e}")
                except Exception as e:   # keep the connection alive on unexpected errors
                    print(f"Error running {cmd!r}: {e}")
                    reply(f"ERR internal error: {e}")

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

def run_server(host: str, port: int, arm: MyCobot280, password: str):
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
                                 args=(conn, addr, arm, active_conn, password), daemon=True)
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
    parser.add_argument("--password", default=os.environ.get("MYCOBOT_PASSWORD", ""),
                        help="Client password (default: $MYCOBOT_PASSWORD, else a random one)")
    args = parser.parse_args()

    password = args.password
    if not password:
        password = secrets.token_urlsafe(9)
        print(f"No password set. Password for this run: {password}")
        print("Set MYCOBOT_PASSWORD or pass --password to choose your own.")

    try:
        arm = MyCobot280(args.serial_port, args.serial_baud)
    except Exception as e:
        print(f"Cannot open {args.serial_port}: {e}", file=sys.stderr)
        sys.exit(1)

    run_server(args.host, args.port, arm, password)


if __name__ == "__main__":
    main()
