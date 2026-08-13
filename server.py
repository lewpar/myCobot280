#!/usr/bin/env python3
"""
server.py - basic TCP server for the myCobot280 arm.

Thin, line-based protocol over TCP. One client at a time; a keepalive
"PING" is sent every 15 seconds and the client is expected to answer
"PONG".

Commands:
    SCAN                              -> OK 1,2,3,...
    COUNT                             -> <n>
    POS <id>                          -> <position>  (or ERR)
    MOVE <id> <pos> [speed] [accel]   -> OK <final_pos>   (blocks until settled)
    MOVE_ASYNC <id> <pos> [speed] [accel] -> OK <sent_pos> (returns immediately)
    WAIT <id> <target> [timeout]      -> OK <final_pos>   (wait for async move)
    SET_CENTER <id> [center]          -> OK correction=<n> verified=<pos>
                                         (writes the servo Position Correction
                                          EEPROM register, does NOT move)
    GET_CENTER <id>                   -> correction=<n> center=<pos>
    CENTER <id> [speed] [accel]       -> OK <final_pos>   (move to stored center)
    PING <id>                         -> OK / ERR
    ATOM_PING                         -> OK / ERR
    ATOM_COLOR <r> <g> <b>            -> OK
    QUIT                              -> BYE

The center position is stored in the servo's Position Correction register
(Feetech "set center position" / CalibrationOfs). Because that register
stores the *offset* rather than the numeric center target, the server also
caches the numeric center (default 2048) so the CENTER command knows where
to move.
"""

import argparse
import signal
import socket
import sys
import threading
import time

from mycobot280 import MyCobot280

DEFAULT_TIMEOUT = 15.0


# ---------------------------------------------------------------------------
# Client connection handler
# ---------------------------------------------------------------------------

def handle_client(conn: socket.socket, addr: tuple, arm: MyCobot280,
                  active_conn: threading.Lock, center_cache: dict):
    # Keep the TCP connection alive across idle periods.
    try:
        conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
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

    def parse_ints(parts, start, count, defaults):
        """Parse up to `count` optional ints from parts[start:]. Missing
        values fall back to `defaults`."""
        out = list(defaults)
        for i in range(count):
            idx = start + i
            if idx < len(parts):
                try:
                    out[i] = int(parts[idx])
                except ValueError:
                    out[i] = defaults[i]
        return out

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

                # ---- SCAN ----
                if op == "SCAN":
                    ids = arm.scan()
                    reply("OK " + ",".join(str(i) for i in ids) if ids else "OK")

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

                # ---- MOVE <id> <pos> [speed] [accel] ----
                elif op == "MOVE":
                    if len(parts) < 3:
                        reply("ERR usage: MOVE <id> <pos> [speed] [accel]")
                        continue
                    sid = int(parts[1])
                    target = int(parts[2])
                    speed, accel = parse_ints(parts, 3, 2, [600, 20])
                    ok, pos = arm.move(sid, target, speed, accel)
                    reply(f"OK {pos}" if ok else f"ERR move failed pos={pos}")

                # ---- MOVE_ASYNC <id> <pos> [speed] [accel] ----
                elif op == "MOVE_ASYNC":
                    if len(parts) < 3:
                        reply("ERR usage: MOVE_ASYNC <id> <pos> [speed] [accel]")
                        continue
                    sid = int(parts[1])
                    target = int(parts[2])
                    speed, accel = parse_ints(parts, 3, 2, [600, 20])
                    sent = arm.servo(sid).move_async(target, speed, accel)
                    reply(f"OK {sent}")

                # ---- WAIT <id> <target> [timeout] ----
                elif op == "WAIT":
                    if len(parts) < 3:
                        reply("ERR usage: WAIT <id> <target> [timeout]")
                        continue
                    sid = int(parts[1])
                    target = int(parts[2])
                    timeout = float(parts[3]) if len(parts) > 3 else DEFAULT_TIMEOUT
                    ok, pos = arm.servo(sid).wait_until_settled(target, timeout=timeout)
                    reply(f"OK {pos}" if ok else f"ERR timeout pos={pos}")

                # ---- SET_CENTER <id> [center] ----
                elif op == "SET_CENTER":
                    if len(parts) < 2:
                        reply("ERR usage: SET_CENTER <id> [center]")
                        continue
                    sid = int(parts[1])
                    center = int(parts[2]) if len(parts) > 2 else 2048

                    ok, correction = arm.servo(sid).set_center_register(center)
                    if not ok:
                        reply(f"ERR could not write center register for servo {sid}")
                        continue

                    # Verify the new reported position landed on the target.
                    time.sleep(0.15)
                    verify = arm.get_position(sid)
                    center_cache[sid] = center
                    if verify is not None and abs(verify - center) <= 5:
                        reply(f"OK correction={correction} verified={verify} center={center}")
                    else:
                        reply(f"WARN correction={correction} verify={verify} "
                              f"expected={center} (sign convention may be flipped)")

                # ---- GET_CENTER <id> ----
                elif op == "GET_CENTER":
                    if len(parts) < 2:
                        reply("ERR usage: GET_CENTER <id>")
                        continue
                    sid = int(parts[1])
                    correction = arm.servo(sid).get_center_register()
                    center = center_cache.get(sid, 2048)
                    if correction is not None:
                        reply(f"correction={correction} center={center}")
                    else:
                        reply(f"ERR could not read center register for servo {sid}")

                # ---- CENTER <id> [speed] [accel] ----
                elif op == "CENTER":
                    if len(parts) < 2:
                        reply("ERR usage: CENTER <id> [speed] [accel]")
                        continue
                    sid = int(parts[1])
                    speed, accel = parse_ints(parts, 2, 2, [600, 20])
                    center = center_cache.get(sid, 2048)
                    ok, pos = arm.move(sid, center, speed, accel)
                    reply(f"OK {pos}" if ok else f"ERR center failed pos={pos}")

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
                    r, g, b = (int(parts[1]), int(parts[2]), int(parts[3]))
                    arm.atom.set_color(r, g, b)
                    reply(f"OK rgb={r},{g},{b}")

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
    print(f"myCobot280 server listening on {host}:{port}")
    print(f"Detected servos: {arm.servo_ids}")

    running = True
    active_conn = threading.Lock()
    center_cache: dict[int, int] = {}

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
            threading.Thread(target=handle_client,
                             args=(conn, addr, arm, active_conn, center_cache),
                             daemon=True).start()
    finally:
        print("Closing serial port...")
        arm.close()
        print("Done.")


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="myCobot280 basic arm server")
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
