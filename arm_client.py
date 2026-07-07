#!/usr/bin/env python3
"""
arm_client.py - Interactive menu client for myCobot280 arm control.

Run:
    python3 arm_client.py

Prompts for the server IP address and port, then provides a numbered
interactive menu for controlling the arm.
"""

import os
import socket
import sys
import termios
import time
import tty

# ---- ANSI helpers (no dependencies) ----

R  = "\033[0m"      # reset
B  = "\033[1m"      # bold
D  = "\033[2m"      # dim
RD = "\033[31m"     # red
GN = "\033[32m"     # green
YL = "\033[33m"     # yellow
BL = "\033[34m"     # blue
CY = "\033[36m"     # cyan

BAR  = BL + "═" * 44 + R
BAR2 = D  + "─" * 44 + R


def ok(msg=""):
    return f"{GN}✓{R} {msg}"

def fail(msg=""):
    return f"{RD}✗{R} {msg}"

def warn(msg):
    return f"{YL}{msg}{R}"


def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def send_command(sock: socket.socket, cmd: str) -> str:
    sock.sendall((cmd + "\n").encode())
    buf = b""
    while True:
        data = sock.recv(4096)
        if not data:
            raise ConnectionError("server closed connection")
        buf += data
        if b"\n" in buf:
            line, _ = buf.split(b"\n", 1)
            return line.decode().strip()


def prompt_int(prompt: str, default: int) -> int:
    raw = input(f"{D}{prompt} [{B}{default}{R}{D}]{R} ").strip()
    if raw == "":
        return default
    try:
        return int(raw)
    except ValueError:
        print(warn(f"Not a valid number, using default ({default})."))
        return default


def prompt_float(prompt: str, default: float) -> float:
    raw = input(f"{D}{prompt} [{B}{default}{R}{D}]{R} ").strip()
    if raw == "":
        return default
    try:
        return float(raw)
    except ValueError:
        print(warn(f"Not a valid number, using default ({default})."))
        return default


SERVO_LABELS = {
    1: "Base rotation",
    2: "Arm joint 2",
    3: "Arm joint 3",
    4: "Arm joint 4",
    5: "Arm joint 5",
    6: "End effector",
}

SAFETY_BUFFER = 50


def fetch_ids(sock: socket.socket) -> list[int]:
    resp = send_command(sock, "SCAN")
    if resp.startswith("ERR"):
        print(resp)
        return []
    prefix = "OK " if resp.startswith("OK ") else ""
    id_str = resp[len(prefix):]
    return [int(x) for x in id_str.split(",")] if id_str else []


def fetch_safe_limits(sock: socket.socket, cache: dict, servo_id: int) -> tuple[int, int]:
    if servo_id in cache:
        return cache[servo_id]
    resp = send_command(sock, f"LIMITS {servo_id}")
    if resp.startswith("ERR"):
        lo, hi = SAFETY_BUFFER, 4095 - SAFETY_BUFFER
    else:
        parts = resp.split(",")
        lo = int(parts[0])
        hi = int(parts[1])
        if lo == 0 and hi == 0:
            lo, hi = SAFETY_BUFFER, 4095 - SAFETY_BUFFER
    cache[servo_id] = (lo, hi)
    return lo, hi


def show_status(sock: socket.socket, ids: list[int]):
    print()
    print(f"{BL}{'ID':>4}  {'Joint':<16}  {'Position':>10}{R}")
    print(BAR2)
    for sid in ids:
        pos_resp = send_command(sock, f"POS {sid}")
        try:
            pos = int(pos_resp)
            label = SERVO_LABELS.get(sid, "")
            print(f"{B}{sid:>4}{R}  {label:<16}  {pos:>10}")
        except ValueError:
            label = SERVO_LABELS.get(sid, "")
            print(f"{B}{sid:>4}{R}  {label:<16}  {pos_resp:>10}")
        time.sleep(0.03)
    print()


def select_servo(ids: list[int]) -> int | None:
    if not ids:
        print(fail("No servos detected on the bus."))
        return None

    print()
    for sid in ids:
        label = SERVO_LABELS.get(sid, "")
        line = f"  {B}{sid}{R}) ID {sid} - {CY}{label}{R}" if label else f"  {B}{sid}{R}) ID {sid}"
        print(line)
    print(f"  {D}{ids[-1] + 1}) Cancel{R}")

    while True:
        raw = input(f"Choose [{ids[0]}-{ids[-1]}]: ").strip()
        if not raw:
            return ids[0]
        if raw.isdigit():
            choice = int(raw)
            if choice in ids:
                return choice
            if choice == ids[-1] + 1:
                return None
        print(warn("Invalid selection, try again."))


def print_banner():
    os.system("clear")
    print()
    print(CY + BAR)
    print(f"  {B}myCobot280 Arm Control{R}")
    print(CY + BAR + R)
    print()


def print_menu():
    print_banner()
    print(f"  {B}1{R})  Show servo status")
    print(f"  {B}2{R})  Read a servo's position")
    print(f"  {B}3{R})  Move a servo (absolute)")
    print(f"  {B}4{R})  Jog a servo (live +/-)")
    print(f"  {B}5{R})  Center a servo")
    print(f"  {B}6{R})  Enable/disable torque (single)")
    print(f"  {B}7{R})  Re-scan the bus")
    print(f"  {B}8{R})  Servo count")
    print(f"  {B}9{R})  Ping a servo")
    print(f"  {B}10{R}) Torque ALL servos on/off")
    print()
    print(f"  {CY}─── ATOM ───{R}")
    print(f"  {B}11{R}) Set ATOM LED color (all)")
    print(f"  {B}12{R}) Ping ATOM")
    print(f"  {B}13{R}) Set ATOM LED pixel (single)")
    print()
    print(f"  {D}0{R})  Quit")
    print()


def run_menu(sock: socket.socket):
    sys.stdout.write(f"{D}Scanning for servos...{R} ")
    sys.stdout.flush()
    ids = fetch_ids(sock)
    limit_cache: dict[int, tuple[int, int]] = {}
    if ids:
        print(ok(f"found {len(ids)} servo(s): {', '.join(str(i) for i in ids)}"))
    else:
        print(warn("no servos detected — try re-scanning from the menu"))
        ids = []

    print_menu()

    while True:
        try:
            choice = input(f"{B}> {R}").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nQuit")
            break

        # ---- STATUS ----
        if choice == "1":
            show_status(sock, ids)

        # ---- READ POSITION ----
        elif choice == "2":
            sid = select_servo(ids)
            if sid is not None:
                resp = send_command(sock, f"POS {sid}")
                label = SERVO_LABELS.get(sid, "")
                print(f"\n{B}Servo {sid}{R} ({D}{label}{R}) → {B}{resp}{R}")

        # ---- MOVE ABSOLUTE ----
        elif choice == "3":
            sid = select_servo(ids)
            if sid is not None:
                pos_resp = send_command(sock, f"POS {sid}")
                try:
                    cur = int(pos_resp)
                    safe_min, safe_max = fetch_safe_limits(sock, limit_cache, sid)
                    print(f"\n{B}Servo {sid}{R}  current: {B}{cur}{R}  "
                          f"safe range: {D}{safe_min}–{safe_max}{R}\n")
                except ValueError:
                    print(f"\nServo {sid} current position: {pos_resp}\n")

                target = prompt_int("Target position (0–4095)", 2048)
                speed  = prompt_int("Speed (0–3400)", 600)
                accel  = prompt_int("Acceleration (0–254)", 20)
                confirm = input(f"\nMove servo {sid} → {target}  "
                                f"speed={speed}  accel={accel}  [{B}y{R}/{D}N{R}] ").strip().lower()
                if confirm == "y":
                    sys.stdout.write(f"{D}Moving...{R}")
                    sys.stdout.flush()
                    resp = send_command(sock, f"MOVE {sid} {target} {speed} {accel}")
                    if resp.startswith("OK"):
                        print(f"\r{ok(resp)}   ")
                    else:
                        print(f"\r{fail(resp)}   ")
                else:
                    print("Cancelled.")

        # ---- MOVE RELATIVE (live jogging) ----
        elif choice == "4":
            sid = select_servo(ids)
            if sid is not None:
                speed = prompt_int("Speed (0–3400)", 600)
                accel = prompt_int("Acceleration (0–254)", 20)
                step  = prompt_int("Step size", 50)

                cur_resp = send_command(sock, f"POS {sid}")
                try:
                    cur = int(cur_resp)
                except ValueError:
                    cur = 0

                safe_min, safe_max = fetch_safe_limits(sock, limit_cache, sid)

                os.system("clear")
                print()
                print(f"  {B}Jogging servo {sid}{R}  "
                      f"step={B}{step}{R}  speed={B}{speed}{R}  accel={B}{accel}{R}")
                print(f"  Position: {B}{cur}{R}  "
                      f"(safe: {D}{safe_min}–{safe_max}{R})")
                print()
                print(f"  {B}+ / ={R}   move positive by step")
                print(f"  {B}-{R}       move negative by step")
                print(f"  {D}[num]{R}   change step size")
                print(f"  {D}q{R}       return to menu")
                print()

                try:
                    while True:
                        key = getch()
                        if key in ("q", "Q"):
                            break
                        elif key in ("+", "="):
                            delta = step
                        elif key == "-":
                            delta = -step
                        elif key.isdigit():
                            step_str = key
                            while True:
                                ch = getch()
                                if ch.isdigit():
                                    step_str += ch
                                else:
                                    key = ch
                                    break
                            try:
                                step = int(step_str)
                            except ValueError:
                                step = 50
                            sys.stdout.write(f"\r  Step size: {B}{step}{R}   ")
                            sys.stdout.flush()
                            if key in ("q", "Q"):
                                break
                            elif key in ("+", "="):
                                delta = step
                            elif key == "-":
                                delta = -step
                            else:
                                continue
                        else:
                            continue

                        target = max(safe_min, min(safe_max, cur + delta))
                        if target == cur:
                            sys.stdout.write(f"\r  {YL}At limit ({safe_min}–{safe_max}){R}     ")
                            sys.stdout.flush()
                            continue

                        resp = send_command(sock, f"MOVE_REL {sid} {delta} {speed} {accel}")
                        if resp.startswith("OK"):
                            try:
                                cur = int(resp.split()[1])
                            except (IndexError, ValueError):
                                cur = target
                        else:
                            cur = target

                        sys.stdout.write(f"\r  Position: {B}{cur}{R}   ")
                        sys.stdout.flush()
                finally:
                    pass
                print(f"\n{D}Jogging ended.{R}")

        # ---- CENTER ----
        elif choice == "5":
            sid = select_servo(ids)
            if sid is not None:
                center_pos = prompt_int("Center position", 2048)
                speed = prompt_int("Speed (0–3400)", 600)
                accel = prompt_int("Acceleration (0–254)", 20)
                confirm = input(f"\nCenter servo {sid} → {center_pos}  "
                                f"speed={speed}  accel={accel}  [{B}y{R}/{D}N{R}] ").strip().lower()
                if confirm == "y":
                    sys.stdout.write(f"{D}Moving...{R}")
                    sys.stdout.flush()
                    resp = send_command(sock, f"CENTER {sid} {center_pos} {speed} {accel}")
                    if resp.startswith("OK"):
                        print(f"\r{ok(resp)}   ")
                    else:
                        print(f"\r{fail(resp)}   ")
                else:
                    print("Cancelled.")

        # ---- TORQUE ----
        elif choice == "6":
            sid = select_servo(ids)
            if sid is not None:
                on_off = input(f"Torque on ({B}1{R}) or off ({D}0{R})? [{B}1{R}]: ").strip()
                if on_off == "":
                    on_off = "1"
                if on_off in ("0", "1"):
                    resp = send_command(sock, f"TORQUE {sid} {on_off}")
                    state = f"{GN}ON{R}" if on_off == "1" else f"{RD}OFF{R}"
                    print(f"\nServo {sid} torque: {state}")
                else:
                    print(warn("Invalid, enter 0 or 1."))

        # ---- SCAN ----
        elif choice == "7":
            sys.stdout.write(f"{D}Scanning...{R} ")
            sys.stdout.flush()
            ids = fetch_ids(sock)
            limit_cache.clear()
            if ids:
                print(f"\r{ok(f'found {len(ids)}: {ids}')}   ")
            else:
                print(f"\r{warn('no servos detected')}   ")

        # ---- COUNT ----
        elif choice == "8":
            print(f"\n{B}{len(ids)}{R} servo(s) detected")

        # ---- PING ----
        elif choice == "9":
            sid = select_servo(ids)
            if sid is not None:
                resp = send_command(sock, f"PING {sid}")
                alive = resp == "OK"
                label = SERVO_LABELS.get(sid, "")
                if alive:
                    print(f"\n{ok()} Servo {sid} ({label}) is {GN}alive{R}")
                else:
                    print(f"\n{fail()} Servo {sid} ({label}) {RD}no response{R}")

        # ---- QUIT ----
        elif choice == "0":
            try:
                send_command(sock, "QUIT")
            except Exception:
                pass
            print(f"\n{D}Disconnected.{R}")
            break

        # ---- TORQUE ALL ----
        elif choice == "10":
            if not ids:
                print(f"\n{warn('No servos detected.')}")
            else:
                on_off = input(f"Torque all servos on ({B}1{R}) or off ({D}0{R})? [{B}1{R}]: ").strip()
                if on_off == "":
                    on_off = "1"
                if on_off in ("0", "1"):
                    sys.stdout.write(f"{D}Setting torque...{R}")
                    sys.stdout.flush()
                    for sid in ids:
                        send_command(sock, f"TORQUE {sid} {on_off}")
                        time.sleep(0.03)
                    state = f"{GN}ON{R}" if on_off == "1" else f"{RD}OFF{R}"
                    print(f"\rAll {len(ids)} servos torque: {state}   ")
                else:
                    print(warn("Invalid, enter 0 or 1."))

        # ---- ATOM LED COLOR ----
        elif choice == "11":
            print(f"\n{CY}Set ATOM LED color{R}\n")
            try:
                r = int(input(f"  Red   {D}(0-255){R} [{B}255{R}]: ").strip() or "255")
                g = int(input(f"  Green {D}(0-255){R} [{B}0{R}]: ").strip() or "0")
                b = int(input(f"  Blue  {D}(0-255){R} [{B}0{R}]: ").strip() or "0")
            except ValueError:
                print(warn("Invalid number."))
            else:
                resp = send_command(sock, f"ATOM_COLOR {r} {g} {b}")
                if resp.startswith("OK"):
                    print(f"\n{ok(f'LED set to RGB({r},{g},{b})')}")
                else:
                    print(f"\n{fail(resp)}")

        # ---- ATOM INFO (PING) ----
        elif choice == "12":
            sys.stdout.write(f"{D}Pinging ATOM...{R} ")
            sys.stdout.flush()
            ping_resp = send_command(sock, "ATOM_PING")
            if ping_resp == "OK":
                sys.stdout.write(f"\r{ok('ATOM is reachable')}   \n")
            else:
                sys.stdout.write(f"\r{fail('ATOM not responding')}   \n")

        # ---- ATOM SET PIXEL ----
        elif choice == "13":
            print(f"\n{CY}Set ATOM LED pixel{R} (5×5 grid, x/y 0-4)\n")
            try:
                x = int(input(f"  X {D}(0-4){R}: ").strip())
                y = int(input(f"  Y {D}(0-4){R}: ").strip())
            except ValueError:
                print(warn("Invalid coordinate."))
            else:
                if 0 <= x <= 4 and 0 <= y <= 4:
                    try:
                        r = int(input(f"  Red   {D}(0-255){R} [{B}255{R}]: ").strip() or "255")
                        g = int(input(f"  Green {D}(0-255){R} [{B}0{R}]: ").strip() or "0")
                        b = int(input(f"  Blue  {D}(0-255){R} [{B}0{R}]: ").strip() or "0")
                    except ValueError:
                        print(warn("Invalid number."))
                    else:
                        resp = send_command(sock, f"ATOM_PIXEL {x} {y} {r} {g} {b}")
                        if resp.startswith("OK"):
                            print(f"\n{ok(f'Pixel ({x},{y}) set to RGB({r},{g},{b})')}")
                        else:
                            print(f"\n{fail(resp)}")
                else:
                    print(warn("Coordinates must be 0-4."))

        else:
            print(warn("Invalid choice. Enter a number from the menu (0–13)."))

        if choice != "0":
            input(f"\n{D}Press Enter to return to menu...{R}")
            print_menu()


def main():
    os.system("clear")
    print()
    print(CY + BAR)
    print(f"  {B}myCobot280 Arm Client{R}")
    print(CY + BAR + R)
    print()

    host = input(f"  Server IP address {D}[192.168.1.x]{R}: ").strip()
    if not host:
        print(fail("No IP address provided."))
        sys.exit(1)

    port_str = input(f"  Server port {D}[5000]{R}: ").strip()
    port = 5000
    if port_str:
        try:
            port = int(port_str)
        except ValueError:
            print(warn("Invalid port, using 5000."))

    sys.stdout.write(f"\n  {D}Connecting to {host}:{port}...{R} ")
    sys.stdout.flush()

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(60.0)
        sock.connect((host, port))
    except (ConnectionRefusedError, socket.timeout) as e:
        sys.stdout.write(f"\r  {fail(f'Cannot connect: {e}')}   \n")
        sys.exit(1)

    print(f"\r  {ok(f'Connected to {host}:{port}')}   \n")

    try:
        run_menu(sock)
    except (ConnectionError, BrokenPipeError) as e:
        print(f"\n{RD}Connection lost: {e}{R}")
        sys.exit(1)
    finally:
        try:
            sock.close()
        except OSError:
            pass


if __name__ == "__main__":
    main()
