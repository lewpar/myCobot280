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


SERVO_LABELS = {
    1: "Base rotation",
    2: "Arm joint 2",
    3: "Arm joint 3",
    4: "Arm joint 4",
    5: "Arm joint 5",
    6: "End effector",
}


def fetch_ids(sock: socket.socket) -> list[int]:
    resp = send_command(sock, "SCAN")
    if resp.startswith("ERR"):
        print(resp)
        return []
    prefix = "OK " if resp.startswith("OK ") else ""
    id_str = resp[len(prefix):]
    return [int(x) for x in id_str.split(",")] if id_str else []


SAFETY_BUFFER = 50


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
    print(f"\n{'ID':>4}  {'Joint':<16}  {'Position':>10}")
    print("-" * 44)

    for sid in ids:
        pos_resp = send_command(sock, f"POS {sid}")
        try:
            pos = int(pos_resp)
            label = SERVO_LABELS.get(sid, "")
            print(f"{sid:>4}  {label:<16}  {pos:>10}")
        except ValueError:
            label = SERVO_LABELS.get(sid, "")
            print(f"{sid:>4}  {label:<16}  {pos_resp:>10}")
        time.sleep(0.03)
    print()


def select_servo(ids: list[int]) -> int | None:
    """Pick a servo ID from a pre-fetched list (no bus scan)."""

    if not ids:
        print("No servos detected on the bus.")
        return None

    if not ids:
        print("No servos detected on the bus.")
        return None

    print("\nSelect a servo:")
    for sid in ids:
        label = SERVO_LABELS.get(sid, "")
        print(f"  {sid}) ID {sid} - {label}" if label else f"  {sid}) ID {sid}")
    print(f"  {ids[-1] + 1}) Cancel")

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
        print("Invalid selection, try again.")


def print_menu():
    os.system("clear")
    print()
    print("=" * 44)
    print("  myCobot280 Arm Control")
    print("=" * 44)
    print()
    print("  1) Show servo status (positions of all joints)")
    print("  2) Read a servo's position")
    print("  3) Move a servo (absolute position)")
    print("  4) Move a servo (relative +/- delta)")
    print("  5) Center a servo")
    print("  6) Enable/disable servo torque")
    print("  7) Re-scan the bus for servos")
    print("  8) Servo count")
    print("  9) Ping a servo")
    print(" 10) Torque ALL servos on/off")
    print("  0) Quit")
    print()


def run_menu(sock: socket.socket):
    print("Scanning for servos...")
    ids = fetch_ids(sock)
    limit_cache: dict[int, tuple[int, int]] = {}
    if not ids:
        print("No servos detected. Try re-scanning from the menu.")
        ids = []

    print_menu()

    while True:
        try:
            choice = input("> ").strip()
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
                print(f"\nServo {sid} position: {resp}")

        # ---- MOVE ABSOLUTE ----
        elif choice == "3":
            sid = select_servo(ids)
            if sid is not None:
                pos_resp = send_command(sock, f"POS {sid}")
                try:
                    cur = int(pos_resp)
                    safe_min, safe_max = fetch_safe_limits(sock, limit_cache, sid)
                    print(f"\nServo {sid} current position: {cur}   (safe range: {safe_min}-{safe_max})\n")
                except ValueError:
                    print(f"\nServo {sid} current position: {pos_resp}\n")

                target = prompt_int("Target position (0-4095)", 2048)
                speed  = prompt_int("Speed (0-3400)", 200)
                accel  = prompt_int("Acceleration (0-254)", 20)
                confirm = input(f"\nMove servo {sid} to {target} at speed={speed} accel={accel}? [y/N] ").strip().lower()
                if confirm == "y":
                    resp = send_command(sock, f"MOVE {sid} {target} {speed} {accel}")
                    print(f"\n{resp}")
                else:
                    print("Cancelled.")

        # ---- MOVE RELATIVE (live jogging) ----
        elif choice == "4":
            sid = select_servo(ids)
            if sid is not None:
                speed = prompt_int("Speed (0-3400)", 200)
                accel = prompt_int("Acceleration (0-254)", 20)
                step  = prompt_int("Step size", 50)

                cur_resp = send_command(sock, f"POS {sid}")
                try:
                    cur = int(cur_resp)
                except ValueError:
                    cur = 0

                safe_min, safe_max = fetch_safe_limits(sock, limit_cache, sid)

                os.system("clear")
                print(f"\nJogging servo {sid}  |  step={step}  speed={speed}  accel={accel}")
                print(f"Current position: {cur}   (safe range: {safe_min}-{safe_max})")
                print()
                print("  + / =   move positive by step")
                print("  -       move negative by step")
                print("  [number] change step size")
                print("  q       return to menu")
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
                            print(f"\rStep size: {step}   ", end="", flush=True)
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
                            sys.stdout.write(f"\rAt limit ({safe_min}-{safe_max}).   ")
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

                        sys.stdout.write(f"\rPosition: {cur}   ")
                        sys.stdout.flush()
                finally:
                    pass
                print("\nJogging ended.")

        # ---- CENTER ----
        elif choice == "5":
            sid = select_servo(ids)
            if sid is not None:
                center_pos = prompt_int("Center position", 2048)
                speed = prompt_int("Speed (0-3400)", 200)
                accel = prompt_int("Acceleration (0-254)", 20)
                confirm = input(f"\nCenter servo {sid} to {center_pos} at speed={speed} accel={accel}? [y/N] ").strip().lower()
                if confirm == "y":
                    resp = send_command(sock, f"CENTER {sid} {center_pos} {speed} {accel}")
                    print(f"\n{resp}")
                else:
                    print("Cancelled.")

        # ---- TORQUE ----
        elif choice == "6":
            sid = select_servo(ids)
            if sid is not None:
                on_off = input("Torque on (1) or off (0)? [1]: ").strip()
                if on_off == "":
                    on_off = "1"
                if on_off in ("0", "1"):
                    resp = send_command(sock, f"TORQUE {sid} {on_off}")
                    print(f"\n{resp}")
                else:
                    print("Invalid, enter 0 or 1.")

        # ---- SCAN ----
        elif choice == "7":
            ids = fetch_ids(sock)
            limit_cache.clear()
            print(f"\nOK {','.join(str(i) for i in ids) if ids else ''}")

        # ---- COUNT ----
        elif choice == "8":
            print(f"\n{len(ids)} servo(s) detected")

        # ---- PING ----
        elif choice == "9":
            sid = select_servo(ids)
            if sid is not None:
                resp = send_command(sock, f"PING {sid}")
                alive = resp == "OK"
                print(f"\nServo {sid}: {'alive' if alive else 'no response'}")

        # ---- QUIT ----
        elif choice == "0":
            try:
                send_command(sock, "QUIT")
            except Exception:
                pass
            print("Disconnected.")
            break

        # ---- TORQUE ALL ----
        elif choice == "10":
            if not ids:
                print("\nNo servos detected.")
            else:
                on_off = input("Torque all servos on (1) or off (0)? [1]: ").strip()
                if on_off == "":
                    on_off = "1"
                if on_off in ("0", "1"):
                    for sid in ids:
                        send_command(sock, f"TORQUE {sid} {on_off}")
                        time.sleep(0.03)
                    state = "ON" if on_off == "1" else "OFF"
                    print(f"\nAll servos torque: {state}")
                else:
                    print("Invalid, enter 0 or 1.")

        else:
            print("Invalid choice. Enter a number from the menu (0-10).")

        if choice != "0":
            input("\nPress Enter to return to menu...")
            print_menu()


def main():
    host = input("Server IP address: ").strip()
    if not host:
        print("No IP address provided.", file=sys.stderr)
        sys.exit(1)

    port_str = input("Server port [5000]: ").strip()
    port = 5000
    if port_str:
        try:
            port = int(port_str)
        except ValueError:
            print("Invalid port, using 5000.")

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((host, port))
    except (ConnectionRefusedError, socket.timeout) as e:
        print(f"Cannot connect to {host}:{port}: {e}", file=sys.stderr)
        sys.exit(1)

    try:
        run_menu(sock)
    except (ConnectionError, BrokenPipeError) as e:
        print(f"Connection lost: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        try:
            sock.close()
        except OSError:
            pass


if __name__ == "__main__":
    main()
