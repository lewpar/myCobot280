#!/usr/bin/env python3
"""
arm_client.py - Interactive menu client for myCobot280 arm control.

Run:
    python3 arm_client.py

Prompts for the server IP address and port, then provides a numbered
interactive menu for controlling the arm.
"""

import socket
import sys
import time


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


def show_status(sock: socket.socket):
    ids_resp = send_command(sock, "SCAN")
    if ids_resp.startswith("ERR"):
        print(ids_resp)
        return

    prefix = "OK " if ids_resp.startswith("OK ") else ""
    id_str = ids_resp[len(prefix):]
    ids = [int(x) for x in id_str.split(",")] if id_str else []

    if not ids:
        print("No servos detected.")
        return

    print(f"\n{'ID':>4}  {'Joint':<16}  {'Position':>10}")
    print("-" * 44)

    labels = {
        1: "Base rotation",
        2: "Arm joint 2",
        3: "Arm joint 3",
        4: "Arm joint 4",
        5: "Arm joint 5",
        6: "End effector",
    }
    for sid in ids:
        pos_resp = send_command(sock, f"POS {sid}")
        try:
            pos = int(pos_resp)
            label = labels.get(sid, "")
            print(f"{sid:>4}  {label:<16}  {pos:>10}")
        except ValueError:
            label = labels.get(sid, "")
            print(f"{sid:>4}  {label:<16}  {pos_resp:>10}")
        time.sleep(0.03)
    print()


def select_servo(sock: socket.socket) -> int | None:
    """Run a bus scan and let the user pick a servo ID from a numbered list."""
    ids_resp = send_command(sock, "SCAN")
    if ids_resp.startswith("ERR"):
        print(ids_resp)
        return None
    prefix = "OK " if ids_resp.startswith("OK ") else ""
    id_str = ids_resp[len(prefix):]
    ids = [int(x) for x in id_str.split(",")] if id_str else []

    if not ids:
        print("No servos detected on the bus.")
        return None

    labels = {
        1: "Base rotation",
        2: "Arm joint 2",
        3: "Arm joint 3",
        4: "Arm joint 4",
        5: "Arm joint 5",
        6: "End effector",
    }

    print("\nSelect a servo:")
    for sid in ids:
        label = labels.get(sid, "")
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


def run_menu(sock: socket.socket):
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
    print("  0) Quit")
    print()

    while True:
        try:
            choice = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nQuit")
            break

        # ---- STATUS ----
        if choice == "1":
            show_status(sock)

        # ---- READ POSITION ----
        elif choice == "2":
            sid = select_servo(sock)
            if sid is None:
                continue
            resp = send_command(sock, f"POS {sid}")
            print(f"\nServo {sid} position: {resp}\n")

        # ---- MOVE ABSOLUTE ----
        elif choice == "3":
            sid = select_servo(sock)
            if sid is None:
                continue
            target = prompt_int("Target position (0-4095)", 2048)
            speed  = prompt_int("Speed (0-3400)", 200)
            accel  = prompt_int("Acceleration (0-254)", 20)

            confirm = input(f"\nMove servo {sid} to {target} at speed={speed} accel={accel}? [y/N] ").strip().lower()
            if confirm != "y":
                print("Cancelled.\n")
                continue

            resp = send_command(sock, f"MOVE {sid} {target} {speed} {accel}")
            print(f"\n{resp}\n")

        # ---- MOVE RELATIVE ----
        elif choice == "4":
            sid = select_servo(sock)
            if sid is None:
                continue
            # show current position
            cur_resp = send_command(sock, f"POS {sid}")
            try:
                cur = int(cur_resp)
                print(f"Current position: {cur}")
            except ValueError:
                print(cur_resp)

            direction = ""
            while direction not in ("+", "-"):
                direction = input("Direction (+/-): ").strip()

            amount = prompt_int("Amount to move (0-4095 scale)", 100)
            delta = amount if direction == "+" else -amount
            speed = prompt_int("Speed (0-3400)", 200)
            accel = prompt_int("Acceleration (0-254)", 20)

            confirm = input(f"\nMove servo {sid} by {delta:+d} at speed={speed} accel={accel}? [y/N] ").strip().lower()
            if confirm != "y":
                print("Cancelled.\n")
                continue

            resp = send_command(sock, f"MOVE_REL {sid} {delta} {speed} {accel}")
            print(f"\n{resp}\n")

        # ---- CENTER ----
        elif choice == "5":
            sid = select_servo(sock)
            if sid is None:
                continue
            center_pos = prompt_int("Center position", 2048)
            speed = prompt_int("Speed (0-3400)", 200)
            accel = prompt_int("Acceleration (0-254)", 20)

            confirm = input(f"\nCenter servo {sid} to {center_pos} at speed={speed} accel={accel}? [y/N] ").strip().lower()
            if confirm != "y":
                print("Cancelled.\n")
                continue

            resp = send_command(sock, f"CENTER {sid} {center_pos} {speed} {accel}")
            print(f"\n{resp}\n")

        # ---- TORQUE ----
        elif choice == "6":
            sid = select_servo(sock)
            if sid is None:
                continue
            on_off = input("Torque on (1) or off (0)? [1]: ").strip()
            if on_off == "":
                on_off = "1"
            if on_off not in ("0", "1"):
                print("Invalid, enter 0 or 1.\n")
                continue
            resp = send_command(sock, f"TORQUE {sid} {on_off}")
            print(f"\n{resp}\n")

        # ---- SCAN ----
        elif choice == "7":
            resp = send_command(sock, "SCAN")
            print(f"\n{resp}\n")

        # ---- COUNT ----
        elif choice == "8":
            resp = send_command(sock, "COUNT")
            print(f"\n{resp} servo(s) detected\n")

        # ---- PING ----
        elif choice == "9":
            sid = select_servo(sock)
            if sid is None:
                continue
            resp = send_command(sock, f"PING {sid}")
            alive = resp == "OK"
            print(f"\nServo {sid}: {'alive' if alive else 'no response'}\n")

        # ---- QUIT ----
        elif choice == "0":
            try:
                send_command(sock, "QUIT")
            except Exception:
                pass
            print("Disconnected.")
            break

        else:
            print("Invalid choice. Enter a number from the menu (0-9).\n")


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
