"""Test script for myCobot280 using the pymycobot library.

Uses the official 0xFE 0xFE binary protocol over /dev/ttyAMA0 at 1,000,000 baud.
Run with: python test.py
"""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "pymycobot"))

import pymycobot

PORT = getattr(pymycobot, "PI_PORT", "/dev/ttyAMA0")
BAUD = getattr(pymycobot, "PI_BAUD", 1000000)


def main():
    print(f"Connecting to {PORT} at {BAUD} baud...")

    arm = pymycobot.MyCobot280(PORT, BAUD)

    print("Connected. Querying servo IDs...")
    try:
        ids = arm.get_servo_ids()
        print(f"Found servos: {ids}")
    except Exception as e:
        print(f"get_servo_ids failed: {e}")

    print("\nReading joint angles...")
    try:
        angles = arm.get_angles()
        print(f"Angles: {angles}")
    except Exception as e:
        print(f"get_angles failed: {e}")

    print("\nReading joint radians...")
    try:
        radians = arm.get_radians()
        print(f"Radians: {radians}")
    except Exception as e:
        print(f"get_radians failed: {e}")

    print("\nDone.")


if __name__ == "__main__":
    main()
