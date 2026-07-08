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

    print("Checking if controller connected...")
    try:
        connected = arm.is_controller_connected()
        print(f"Controller connected: {connected}")
    except Exception as e:
        print(f"is_controller_connected failed: {e}")

    print("\nChecking power status...")
    try:
        power = arm.is_power_on()
        print(f"Power on: {power}")
    except Exception as e:
        print(f"is_power_on failed: {e}")

    print("\nReading joint angles...")
    try:
        angles = arm.get_angles()
        print(f"Angles: {angles}")
    except Exception as e:
        print(f"get_angles failed: {e}")

    print("\nReading servo status...")
    try:
        status = arm.get_servo_status()
        print(f"Servo status: {status}")
    except Exception as e:
        print(f"get_servo_status failed: {e}")

    print("\nDone.")


if __name__ == "__main__":
    main()
