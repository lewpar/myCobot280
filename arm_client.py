#!/usr/bin/env python3
"""
arm_client.py - TCP client for myCobot280 arm control.

Connects to the arm server and provides both an interactive CLI and
one-shot command-line usage.

Usage:
    python3 arm_client.py                         # interactive mode
    python3 arm_client.py STATUS                  # show all servo positions
    python3 arm_client.py POS 1                   # get servo 1 position
    python3 arm_client.py MOVE 1 2048 200 20      # move servo 1 to 2048
    python3 arm_client.py --host 192.168.1.5 POS 1

Interactive commands:
    scan          re-scan the bus for servos
    count         show number of connected servos
    pos <id>      read current position
    limits <id>   read min/max angle limits
    info <id>     read full servo info
    move <id> <pos> [speed] [accel]     absolute move
    mrel <id> <delta> [speed] [accel]   relative move
    torque <id> <0|1>   enable/disable torque
    center <id> [pos] [speed] [accel]   move to center
    ping <id>     ping a servo
    status        show all servo positions at once
    help          show this help
    quit          disconnect
"""

import argparse
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


def format_servo_list(ids: list[int]) -> str:
    if not ids:
        return "(none)"
    return ", ".join(str(i) for i in ids)


def interactive(sock: socket.socket):
    print("myCobot280 Arm Client")
    print("Type 'help' for commands, 'quit' to exit.\n")

    while True:
        try:
            raw = input("arm> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nquit")
            break

        if not raw:
            continue

        parts = raw.split()
        op = parts[0].lower()

        if op == "quit" or op == "exit":
            sock.sendall(b"QUIT\n")
            resp = send_command(sock, "QUIT")
            print(resp)
            break

        elif op == "help":
            print("""Commands:
  scan                       re-scan the bus for servos
  count                      number of connected servos
  pos <id>                   read current servo position
  limits <id>                read min/max angle limits
  info <id>                  read full servo info
  move <id> <pos> [speed] [accel]    absolute move (pos 0-4095)
  mrel <id> <delta> [speed] [accel]  relative move
  torque <id> <0|1>          enable/disable torque
  center <id> [pos] [speed] [accel]  move servo to center (default 2048)
  ping <id>                  ping a servo
  status                     show all servo positions
  help                       show this help
  quit                       disconnect""")

        elif op == "scan":
            resp = send_command(sock, "SCAN")
            print(resp)

        elif op == "count":
            resp = send_command(sock, "COUNT")
            print(f"{resp} servo(s)")

        elif op == "pos":
            if len(parts) < 2:
                print("usage: pos <id>")
                continue
            resp = send_command(sock, raw.upper())
            try:
                print(f"Servo {parts[1]} position: {resp}")
            except ValueError:
                print(resp)

        elif op == "limits":
            if len(parts) < 2:
                print("usage: limits <id>")
                continue
            resp = send_command(sock, raw.upper())
            if resp.startswith("ERR"):
                print(resp)
            else:
                lo, hi = resp.split(",")
                print(f"Servo {parts[1]} limits: min={lo} max={hi}")

        elif op == "info":
            if len(parts) < 2:
                print("usage: info <id>")
                continue
            resp = send_command(sock, raw.upper())
            print(resp)

        elif op == "move":
            if len(parts) < 3:
                print("usage: move <id> <pos> [speed] [accel]")
                continue
            resp = send_command(sock, raw.upper())
            print(resp)

        elif op == "mrel":
            if len(parts) < 3:
                print("usage: mrel <id> <delta> [speed] [accel]")
                continue
            resp = send_command(sock, "MOVE_REL " + " ".join(parts[1:]).upper())
            print(resp)

        elif op == "torque":
            if len(parts) < 3:
                print("usage: torque <id> <0|1>")
                continue
            resp = send_command(sock, raw.upper())
            print(resp)

        elif op == "center":
            if len(parts) < 2:
                print("usage: center <id> [pos=2048] [speed] [accel]")
                continue
            resp = send_command(sock, raw.upper())
            print(resp)

        elif op == "ping":
            if len(parts) < 2:
                print("usage: ping <id>")
                continue
            resp = send_command(sock, raw.upper())
            print(f"Servo {parts[1]}: {'alive' if resp == 'OK' else 'no response'}")

        elif op == "status":
            ids_resp = send_command(sock, "SCAN")
            if ids_resp.startswith("ERR"):
                print(ids_resp)
                continue
            prefix = "OK " if ids_resp.startswith("OK ") else ""
            id_str = ids_resp[len(prefix):]
            ids = [int(x) for x in id_str.split(",")] if id_str else []
            if not ids:
                print("No servos detected.")
                continue
            print(f"{'ID':>4}  {'Position':>10}")
            print("-" * 22)
            for sid in ids:
                pos_resp = send_command(sock, f"POS {sid}")
                try:
                    pos = int(pos_resp)
                    print(f"{sid:>4}  {pos:>10}")
                except ValueError:
                    print(f"{sid:>4}  {pos_resp:>10}")
                time.sleep(0.03)

        else:
            print(f"Unknown command: {raw}. Type 'help' for available commands.")


def main():
    parser = argparse.ArgumentParser(description="myCobot280 Arm Client")
    parser.add_argument("--host", default="127.0.0.1", help="Server address (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=5000, help="Server port (default: 5000)")
    parser.add_argument("command", nargs="*", help="Command to run (omit for interactive mode)")
    args = parser.parse_args()

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((args.host, args.port))
    except (ConnectionRefusedError, socket.timeout) as e:
        print(f"Cannot connect to {args.host}:{args.port}: {e}", file=sys.stderr)
        sys.exit(1)

    try:
        if args.command:
            cmd = " ".join(args.command)
            if cmd.upper() == "STATUS":
                ids_resp = send_command(sock, "SCAN")
                prefix = "OK " if ids_resp.startswith("OK ") else ""
                id_str = ids_resp[len(prefix):]
                ids = [int(x) for x in id_str.split(",")] if id_str else []
                if not ids:
                    print("No servos detected.")
                else:
                    print(f"{'ID':>4}  {'Position':>10}")
                    print("-" * 22)
                    for sid in ids:
                        pos_resp = send_command(sock, f"POS {sid}")
                        try:
                            pos = int(pos_resp)
                            print(f"{sid:>4}  {pos:>10}")
                        except ValueError:
                            print(f"{sid:>4}  {pos_resp:>10}")
                        time.sleep(0.03)
            else:
                resp = send_command(sock, cmd)
                print(resp)
        else:
            interactive(sock)
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
