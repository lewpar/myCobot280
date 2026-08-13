#!/usr/bin/env python3
"""
client.py - basic interactive client for the myCobot280 arm.

Run:
    python3 client.py

Prompts for the server IP and port, then shows a small numbered menu for
the core operations: absolute moves, async moves, center register
set/move, and ATOM LED colors.
"""

import queue
import socket
import sys
import threading

# ---- tiny ANSI helpers (no dependencies) ----

R = "\033[0m"
B = "\033[1m"
D = "\033[2m"
GN = "\033[32m"
RD = "\033[31m"
YL = "\033[33m"
CY = "\033[36m"


def ok(msg=""):
    return f"{GN}OK{R} {msg}".strip()


def fail(msg=""):
    return f"{RD}ERR{R} {msg}".strip()


def warn(msg=""):
    return f"{YL}{msg}{R}"


def show(resp: str) -> str:
    """Color a raw server response line based on its status prefix."""
    if resp.startswith("OK"):
        return f"{GN}{resp}{R}"
    if resp.startswith("WARN"):
        return f"{YL}{resp}{R}"
    if resp.startswith("ERR"):
        return f"{RD}{resp}{R}"
    return resp


def prompt_int(prompt, default):
    raw = input(f"{D}{prompt} [{B}{default}{R}{D}]{R} ").strip()
    if raw == "":
        return default
    try:
        return int(raw)
    except ValueError:
        print(warn(f"Not a number, using {default}."))
        return default


# ---------------------------------------------------------------------------
# Connection / protocol helpers
# ---------------------------------------------------------------------------

_resp_queue: queue.Queue | None = None


def reader_thread(sock: socket.socket, stop: threading.Event):
    global _resp_queue
    buf = b""
    while not stop.is_set():
        try:
            sock.settimeout(0.5)
            data = sock.recv(4096)
        except socket.timeout:
            continue
        except OSError:
            break
        if not data:
            break
        buf += data
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            msg = line.decode(errors="replace").strip()
            if msg == "PING":
                try:
                    sock.sendall(b"PONG\n")
                except OSError:
                    break
            else:
                _resp_queue.put(msg)
    _resp_queue.put(None)  # signal EOF
    stop.set()


def send_command(sock: socket.socket, cmd: str, timeout: float = 60.0) -> str:
    sock.sendall((cmd + "\n").encode())
    try:
        resp = _resp_queue.get(timeout=timeout)
    except queue.Empty:
        raise ConnectionError("no response from server")
    if resp is None:
        raise ConnectionError("server closed connection")
    return resp


def fetch_ids(sock: socket.socket) -> list[int]:
    resp = send_command(sock, "SCAN")
    if resp.startswith("ERR"):
        print(fail(resp))
        return []
    text = resp[3:].strip() if resp.startswith("OK") else resp.strip()
    return [int(x) for x in text.split(",") if x.strip().isdigit()]


def pick_servo(ids: list[int]) -> int | None:
    if not ids:
        print(warn("No servos detected."))
        return None
    print()
    for sid in ids:
        print(f"  {B}{sid}{R}) servo {sid}")
    while True:
        raw = input(f"Servo ID [{ids[0]}]: ").strip()
        if raw == "":
            return ids[0]
        if raw.isdigit() and int(raw) in ids:
            return int(raw)
        print(warn("Invalid servo ID."))


def print_menu():
    print()
    print(f"{CY}{'─' * 40}{R}")
    print(f"  {B}myCobot280 client{R}")
    print(f"{CY}{'─' * 40}{R}")
    print(f"  {B}1{R})  List servos")
    print(f"  {B}2{R})  Read position")
    print(f"  {B}3{R})  Move absolute (blocking)")
    print(f"  {B}4{R})  Move absolute (async)")
    print(f"  {B}5{R})  Set center in register")
    print(f"  {B}6{R})  Move to center")
    print(f"  {B}7{R})  Read center register")
    print(f"  {B}8{R})  Torque single servo on/off")
    print(f"  {B}9{R})  Torque all servos on/off")
    print(f"  {B}10{R}) Set ATOM color")
    print(f"  {B}11{R}) Ping ATOM")
    print(f"  {B}0{R})  Quit")
    print()


def run_menu(sock: socket.socket):
    ids = fetch_ids(sock)
    if ids:
        print(ok(f"found {len(ids)} servo(s): {', '.join(map(str, ids))}"))
    else:
        print(warn("no servos detected (choose option 1 to re-scan)"))

    print_menu()

    while True:
        try:
            choice = input(f"{B}> {R}").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nQuit")
            break

        if choice == "1":
            ids = fetch_ids(sock)
            if ids:
                print(ok(f"servos: {', '.join(map(str, ids))}"))
            else:
                print(warn("no servos detected"))

        elif choice == "2":
            sid = pick_servo(ids)
            if sid is not None:
                resp = send_command(sock, f"POS {sid}")
                print(f"\n  Servo {sid} position: {B}{resp}{R}")

        elif choice == "3":
            sid = pick_servo(ids)
            if sid is not None:
                target = prompt_int("Target position (0-4095)", 2048)
                speed = prompt_int("Speed", 600)
                accel = prompt_int("Acceleration", 20)
                print(f"  Moving servo {sid} to {target}...")
                resp = send_command(sock, f"MOVE {sid} {target} {speed} {accel}", timeout=60)
                print("  " + show(resp))

        elif choice == "4":
            sid = pick_servo(ids)
            if sid is not None:
                target = prompt_int("Target position (0-4095)", 2048)
                speed = prompt_int("Speed", 600)
                accel = prompt_int("Acceleration", 20)
                resp = send_command(sock, f"MOVE_ASYNC {sid} {target} {speed} {accel}")
                print("  " + show(resp))
                if resp.startswith("OK") and input(f"  Wait for it to settle? [{B}y{R}/{D}N{R}] ").strip().lower() == "y":
                    try:
                        sent = int(resp.split()[1])
                    except (IndexError, ValueError):
                        sent = target
                    resp2 = send_command(sock, f"WAIT {sid} {sent}", timeout=60)
                    print("  " + show(resp2))

        elif choice == "5":
            sid = pick_servo(ids)
            if sid is not None:
                print(f"\n  {D}This calibrates the servo's zero: the current physical{R}")
                print(f"  {D}position will report as the center value you enter.{R}")
                center = prompt_int("Center value", 2048)
                confirm = input(f"\n  Set servo {sid} center to {center} in register? [{B}y{R}/{D}N{R}] ").strip().lower()
                if confirm == "y":
                    resp = send_command(sock, f"SET_CENTER {sid} {center}", timeout=30)
                    print("  " + show(resp))
                else:
                    print("  Cancelled.")

        elif choice == "6":
            sid = pick_servo(ids)
            if sid is not None:
                speed = prompt_int("Speed", 600)
                accel = prompt_int("Acceleration", 20)
                print(f"  Moving servo {sid} to stored center...")
                resp = send_command(sock, f"CENTER {sid} {speed} {accel}", timeout=60)
                print("  " + show(resp))

        elif choice == "7":
            sid = pick_servo(ids)
            if sid is not None:
                resp = send_command(sock, f"GET_CENTER {sid}")
                print(f"\n  Servo {sid} center register: {B}{resp}{R}")

        elif choice == "8":
            sid = pick_servo(ids)
            if sid is not None:
                on_off = input(f"  Torque on ({B}1{R}) or off ({D}0{R})? [{B}1{R}]: ").strip()
                if on_off == "":
                    on_off = "1"
                if on_off not in ("0", "1"):
                    print(warn("Enter 0 or 1."))
                else:
                    resp = send_command(sock, f"TORQUE {sid} {on_off}")
                    print("  " + show(resp))

        elif choice == "9":
            if not ids:
                print(warn("No servos detected."))
            else:
                on_off = input(f"  Torque ALL servos on ({B}1{R}) or off ({D}0{R})? [{B}1{R}]: ").strip()
                if on_off == "":
                    on_off = "1"
                if on_off not in ("0", "1"):
                    print(warn("Enter 0 or 1."))
                else:
                    resp = send_command(sock, f"TORQUE_ALL {on_off}")
                    print("  " + show(resp))

        elif choice == "10":
            print(f"\n  {CY}Set ATOM LED color{R}")
            try:
                r = int(input(f"  Red   {D}[255]{R}: ").strip() or "255")
                g = int(input(f"  Green {D}[0]{R}: ").strip() or "0")
                b = int(input(f"  Blue  {D}[0]{R}: ").strip() or "0")
            except ValueError:
                print(warn("Invalid number."))
            else:
                resp = send_command(sock, f"ATOM_COLOR {r} {g} {b}")
                print("  " + show(resp))

        elif choice == "11":
            resp = send_command(sock, "ATOM_PING")
            print("  " + show(resp))

        elif choice == "0":
            try:
                send_command(sock, "QUIT", timeout=3)
            except Exception:
                pass
            print("\nDisconnected.")
            break

        else:
            print(warn("Invalid choice (0-11)."))

        if choice != "0":
            input(f"\n{D}Press Enter to return to the menu...{R}")
            print_menu()


def main():
    print()
    print(f"{CY}{'─' * 40}{R}")
    print(f"  {B}myCobot280 client{R}")
    print(f"{CY}{'─' * 40}{R}\n")

    host = input(f"  Server IP {D}[192.168.1.10]{R}: ").strip()
    if not host:
        print(fail("no server address"))
        sys.exit(1)

    port = 5000
    port_str = input(f"  Port {D}[5000]{R}: ").strip()
    if port_str:
        try:
            port = int(port_str)
        except ValueError:
            print(warn("invalid port, using 5000"))

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10.0)
        sock.connect((host, port))
    except (ConnectionRefusedError, socket.timeout, OSError) as e:
        print(fail(f"cannot connect to {host}:{port}: {e}"))
        sys.exit(1)

    print(ok(f"connected to {host}:{port}"))

    global _resp_queue
    _resp_queue = queue.Queue()
    stop = threading.Event()
    threading.Thread(target=reader_thread, args=(sock, stop), daemon=True).start()

    try:
        run_menu(sock)
    except (ConnectionError, BrokenPipeError, OSError) as e:
        print(f"\n{RD}Connection lost: {e}{R}")
        sys.exit(1)
    finally:
        stop.set()
        try:
            sock.close()
        except OSError:
            pass


if __name__ == "__main__":
    main()
