#!/usr/bin/env python3
"""
arm_server.py - TCP server for myCobot280 arm control.

Listens for TCP connections and controls Feetech-family servos over serial.
Protocol is line-based text commands. One command per line, one response per line.

Serial:  /dev/ttyAMA0 @ 1000000 baud
TCP:     localhost:5000 by default

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
    QUIT                         -> BYE
"""

import argparse
import signal
import socket
import sys
import threading
import time

try:
    import serial
except ImportError:
    print("This script needs pyserial. Install with: pip install pyserial --break-system-packages")
    sys.exit(1)


# ---------------------------------------------------------------------------
# Feetech protocol helpers
# ---------------------------------------------------------------------------

def feetech_checksum(payload_after_id: bytes) -> int:
    return (~sum(payload_after_id)) & 0xFF


def build_feetech_ping(servo_id: int) -> bytes:
    body = bytes([servo_id, 0x02, 0x01])
    chk = feetech_checksum(body)
    return bytes([0xFF, 0xFF]) + body + bytes([chk])


def build_feetech_write(servo_id: int, address: int, data: bytes) -> bytes:
    body = bytes([servo_id, 3 + len(data), 0x03, address]) + data
    chk = feetech_checksum(body)
    return bytes([0xFF, 0xFF]) + body + bytes([chk])


def build_feetech_read(servo_id: int, address: int, length: int) -> bytes:
    body = bytes([servo_id, 0x04, 0x02, address, length])
    chk = feetech_checksum(body)
    return bytes([0xFF, 0xFF]) + body + bytes([chk])


def parse_status_params(resp: bytes):
    if len(resp) >= 6 and resp[0] == 0xFF and resp[1] == 0xFF:
        length = resp[3]
        params = resp[5:5 + (length - 2)]
        return params
    return None


ADDR_MIN_ANGLE_LIMIT     = 9
ADDR_MAX_ANGLE_LIMIT     = 11
ADDR_TORQUE_ENABLE       = 40
ADDR_ACCELERATION        = 41
ADDR_GOAL_POSITION       = 42
ADDR_GOAL_SPEED          = 46
ADDR_POSITION_CORRECTION = 31
ADDR_LOCK                = 55
ADDR_PRESENT_POSITION    = 56

SERVO_MIN = 0
SERVO_MAX = 4095
SAFETY_BUFFER = 50


# ---------------------------------------------------------------------------
# Servo Controller (thread-safe, serialised via lock)
# ---------------------------------------------------------------------------

class ServoController:
    def __init__(self, port: str, baud: int):
        self._lock = threading.Lock()
        self._ser = serial.Serial(port, baud, timeout=0.2)
        self._limit_cache: dict[int, tuple[int, int]] = {}
        self.scan()

    def _read_response(self, wait: float = 0.05) -> bytes:
        time.sleep(wait)
        n = self._ser.in_waiting
        if n:
            return self._ser.read(n)
        return b""

    def _ping_one(self, servo_id: int) -> bool:
        packet = build_feetech_ping(servo_id)
        self._ser.reset_input_buffer()
        self._ser.write(packet)
        resp = self._read_response()
        return len(resp) >= 6 and resp[0] == 0xFF and resp[1] == 0xFF

    def _write_raw(self, servo_id: int, address: int, data: bytes):
        packet = build_feetech_write(servo_id, address, data)
        self._ser.reset_input_buffer()
        self._ser.write(packet)
        return self._read_response(0.05)

    def _read_raw(self, servo_id: int, address: int, length: int) -> bytes:
        packet = build_feetech_read(servo_id, address, length)
        self._ser.reset_input_buffer()
        self._ser.write(packet)
        return self._read_response(0.05)

    def _read_uint16(self, servo_id: int, address: int):
        resp = self._read_raw(servo_id, address, 2)
        params = parse_status_params(resp)
        if params and len(params) >= 2:
            return params[0] | (params[1] << 8)
        return None

    # ---- public API ----

    def _get_safe_limits(self, servo_id: int) -> tuple[int, int]:
        if servo_id not in self._limit_cache:
            return SERVO_MIN + SAFETY_BUFFER, SERVO_MAX - SAFETY_BUFFER
        return self._limit_cache[servo_id]

    def _clamp_target(self, servo_id: int, target: int) -> int:
        safe_min, safe_max = self._get_safe_limits(servo_id)
        return max(safe_min, min(safe_max, target))

    def scan(self):
        with self._lock:
            ids = []
            for sid in range(1, 51):
                if self._ping_one(sid):
                    ids.append(sid)
                time.sleep(0.015)
            self.servo_ids = ids
            self._limit_cache.clear()
            for sid in ids:
                lo = self._read_uint16(sid, ADDR_MIN_ANGLE_LIMIT)
                hi = self._read_uint16(sid, ADDR_MAX_ANGLE_LIMIT)
                if lo is None or hi is None or (lo == 0 and hi == 0):
                    self._limit_cache[sid] = (SERVO_MIN + SAFETY_BUFFER,
                                              SERVO_MAX - SAFETY_BUFFER)
                else:
                    self._limit_cache[sid] = (lo + SAFETY_BUFFER,
                                              hi - SAFETY_BUFFER)
        return self.servo_ids

    def get_position(self, servo_id: int):
        with self._lock:
            return self._read_uint16(servo_id, ADDR_PRESENT_POSITION)

    def get_limits(self, servo_id: int):
        with self._lock:
            lo = self._read_uint16(servo_id, ADDR_MIN_ANGLE_LIMIT)
            hi = self._read_uint16(servo_id, ADDR_MAX_ANGLE_LIMIT)
            return lo, hi

    def get_safe_limits(self, servo_id: int) -> tuple[int, int]:
        return self._get_safe_limits(servo_id)

    def get_info(self, servo_id: int):
        with self._lock:
            pos = self._read_uint16(servo_id, ADDR_PRESENT_POSITION)
            lo  = self._read_uint16(servo_id, ADDR_MIN_ANGLE_LIMIT)
            hi  = self._read_uint16(servo_id, ADDR_MAX_ANGLE_LIMIT)
            return {"pos": pos, "min": lo, "max": hi}

    def move(self, servo_id: int, target: int, speed: int = 200, accel: int = 20):
        with self._lock:
            target = self._clamp_target(servo_id, target)
            self._write_raw(servo_id, ADDR_TORQUE_ENABLE, bytes([1]))
            self._write_raw(servo_id, ADDR_ACCELERATION,  bytes([accel & 0xFF]))
            self._write_raw(servo_id, ADDR_GOAL_SPEED,
                            bytes([speed & 0xFF, (speed >> 8) & 0xFF]))
            self._write_raw(servo_id, ADDR_GOAL_POSITION,
                            bytes([target & 0xFF, (target >> 8) & 0xFF]))
            time.sleep(0.8)
            new_pos = self._read_uint16(servo_id, ADDR_PRESENT_POSITION)
        ok = new_pos is not None and abs(new_pos - target) <= 10
        return ok, new_pos

    def move_relative(self, servo_id: int, delta: int, speed: int = 200, accel: int = 20):
        with self._lock:
            current = self._read_uint16(servo_id, ADDR_PRESENT_POSITION)
            if current is None:
                return False, None, "could not read current position"
            target = self._clamp_target(servo_id, current + delta)
        ok, pos = self.move(servo_id, target, speed, accel)
        msg = f"moved from {current} to {pos}" if ok else "move failed"
        return ok, pos, msg

    def center(self, servo_id: int, center_pos: int = 2048, speed: int = 200, accel: int = 20):
        return self.move(servo_id, center_pos, speed, accel)

    def set_torque(self, servo_id: int, enable: bool):
        with self._lock:
            self._write_raw(servo_id, ADDR_TORQUE_ENABLE, bytes([1 if enable else 0]))

    def close(self):
        self._ser.close()


# ---------------------------------------------------------------------------
# Client connection handler (runs in its own thread)
# ---------------------------------------------------------------------------

def handle_client(conn: socket.socket, addr: tuple, ctrl: ServoController):
    def reply(msg: str):
        try:
            conn.sendall((msg.rstrip() + "\r\n").encode())
        except OSError:
            pass

    try:
        buf = b""
        while True:
            data = conn.recv(1024)
            if not data:
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                cmd = line.decode().strip()
                if not cmd:
                    continue

                parts = cmd.split()
                op = parts[0].upper()

                # ---- SCAN ----
                if op == "SCAN":
                    ids = ctrl.scan()
                    reply(f"OK {','.join(str(i) for i in ids) if ids else ''}")

                # ---- COUNT ----
                elif op == "COUNT":
                    reply(str(len(ctrl.servo_ids)))

                # ---- POS <id> ----
                elif op == "POS":
                    if len(parts) < 2:
                        reply("ERR usage: POS <id>")
                        continue
                    sid = int(parts[1])
                    pos = ctrl.get_position(sid)
                    reply(str(pos) if pos is not None else "ERR no response")

                # ---- LIMITS <id> ----
                elif op == "LIMITS":
                    if len(parts) < 2:
                        reply("ERR usage: LIMITS <id>")
                        continue
                    sid = int(parts[1])
                    lo, hi = ctrl.get_safe_limits(sid)
                    reply(f"{lo},{hi}")

                # ---- INFO <id> ----
                elif op == "INFO":
                    if len(parts) < 2:
                        reply("ERR usage: INFO <id>")
                        continue
                    sid = int(parts[1])
                    info = ctrl.get_info(sid)
                    if info["pos"] is None:
                        reply("ERR no response")
                    else:
                        reply(f"pos:{info['pos']} min:{info['min']} max:{info['max']}")

                # ---- MOVE <id> <pos> [speed] [accel] ----
                elif op == "MOVE":
                    if len(parts) < 3:
                        reply("ERR usage: MOVE <id> <pos> [speed] [accel]")
                        continue
                    sid    = int(parts[1])
                    target = int(parts[2])
                    speed  = int(parts[3]) if len(parts) > 3 else 200
                    accel  = int(parts[4]) if len(parts) > 4 else 20
                    ok, pos = ctrl.move(sid, target, speed, accel)
                    if ok:
                        reply(f"OK {pos}")
                    else:
                        reply(f"ERR move failed, pos={pos}")

                # ---- MOVE_REL <id> <delta> [speed] [accel] ----
                elif op == "MOVE_REL":
                    if len(parts) < 3:
                        reply("ERR usage: MOVE_REL <id> <delta> [speed] [accel]")
                        continue
                    sid   = int(parts[1])
                    delta = int(parts[2])
                    speed = int(parts[3]) if len(parts) > 3 else 200
                    accel = int(parts[4]) if len(parts) > 4 else 20
                    ok, pos, msg = ctrl.move_relative(sid, delta, speed, accel)
                    if ok:
                        reply(f"OK {pos}")
                    else:
                        reply(f"ERR {msg}")

                # ---- TORQUE <id> <0|1> ----
                elif op == "TORQUE":
                    if len(parts) < 3:
                        reply("ERR usage: TORQUE <id> <0|1>")
                        continue
                    sid = int(parts[1])
                    on  = parts[2] == "1"
                    ctrl.set_torque(sid, on)
                    reply("OK")

                # ---- CENTER <id> [pos] [speed] [accel] ----
                elif op == "CENTER":
                    if len(parts) < 2:
                        reply("ERR usage: CENTER <id> [pos] [speed] [accel]")
                        continue
                    sid       = int(parts[1])
                    center_pos = int(parts[2]) if len(parts) > 2 else 2048
                    speed     = int(parts[3]) if len(parts) > 3 else 200
                    accel     = int(parts[4]) if len(parts) > 4 else 20
                    ok, pos = ctrl.center(sid, center_pos, speed, accel)
                    if ok:
                        reply(f"OK {pos}")
                    else:
                        reply(f"ERR center move failed, pos={pos}")

                # ---- PING <id> ----
                elif op == "PING":
                    if len(parts) < 2:
                        reply("ERR usage: PING <id>")
                        continue
                    sid = int(parts[1])
                    alive = ctrl._ping_one(sid)
                    reply("OK" if alive else "ERR no response")

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
        try:
            conn.close()
        except OSError:
            pass


# ---------------------------------------------------------------------------
# TCP server
# ---------------------------------------------------------------------------

def run_server(host: str, port: int, ctrl: ServoController):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.listen(5)
    print(f"Arm server listening on {host}:{port}")
    print(f"Detected servos: {ctrl.servo_ids}")

    running = True

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
                print(f"Client connected: {addr}")
                t = threading.Thread(target=handle_client, args=(conn, addr, ctrl), daemon=True)
                t.start()
            except socket.timeout:
                continue
            except OSError:
                break
    finally:
        print("Closing serial port...")
        ctrl.close()
        print("Done.")


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="myCobot280 Arm Server")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=5000, help="TCP port (default: 5000)")
    parser.add_argument("--serial-port", default="/dev/ttyAMA0", help="Serial port (default: /dev/ttyAMA0)")
    parser.add_argument("--serial-baud", type=int, default=1_000_000, help="Serial baud rate (default: 1000000)")
    args = parser.parse_args()

    try:
        ctrl = ServoController(args.serial_port, args.serial_baud)
    except serial.SerialException as e:
        print(f"Cannot open serial port {args.serial_port}: {e}", file=sys.stderr)
        sys.exit(1)

    run_server(args.host, args.port, ctrl)


if __name__ == "__main__":
    main()
