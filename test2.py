"""Test script that tries enabling GPIO pins before talking to the arm.

The myArm_Pi_Base_V1.0 board uses logic gates (tri-state buffers) that may
need a GPIO enable signal to connect the Pi's TX to the servo bus.

Run with: python test2.py
Use -p to try a specific pin: python test2.py -p 19
Use -l 0 to try LOW instead of HIGH: python test2.py -l 0 -p 19
"""

import os
import sys
import time
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "pymycobot"))


try:
    import RPi.GPIO as GPIO
except ImportError:
    print("ERROR: RPi.GPIO not available. Run this on the Raspberry Pi.")
    sys.exit(1)

GPIOS_TO_TRY = [17, 18, 27, 22, 23, 24, 25, 5, 6, 12, 13, 16, 19, 26, 7, 8, 11, 4]

import pymycobot

PORT = getattr(pymycobot, "PI_PORT", "/dev/ttyAMA0")
BAUD = getattr(pymycobot, "PI_BAUD", 1000000)


def try_arm():
    """Attempt to connect and check controller. Returns (success, message)."""
    try:
        arm = pymycobot.MyCobot280(PORT, BAUD, thread_lock=False)
    except Exception as e:
        return False, f"MyCobot280 init failed: {e}"

    try:
        connected = arm.is_controller_connected()
        if connected == 1:
            angles = arm.get_angles()
            return True, f"Connected, angles: {angles}"
        else:
            return False, f"Controller connected: {connected}"
    except Exception as e:
        return False, f"Communication failed: {e}"
    finally:
        try:
            arm._serial_port.close()
        except Exception:
            pass


def try_pins(pins, level):
    for pin in pins:
        lvl_name = "HIGH" if level else "LOW"
        print(f"\n--- GPIO {pin} {lvl_name} ---")

        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, level)
        time.sleep(0.02)

        ok, msg = try_arm()
        if ok:
            print(f"  SUCCESS: {msg}")
            print(f"\nFound working config: GPIO {pin} {lvl_name}")
            return True
        else:
            print(f"  FAIL: {msg}")

        GPIO.setup(pin, GPIO.IN)
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--pin", type=int, help="Try only this GPIO (BCM number)")
    parser.add_argument("-l", "--level", type=int, choices=[0, 1],
                        help="Logic level: 1=HIGH, 0=LOW (default: try both)")
    args = parser.parse_args()

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    pins = [args.pin] if args.pin else GPIOS_TO_TRY

    print(f"Port: {PORT}, Baud: {BAUD}")
    print(f"Pins to try: {pins}")
    print("=" * 50)

    if args.level is not None:
        try_pins(pins, args.level)
    else:
        if try_pins(pins, 1):
            pass
        elif try_pins(pins, 0):
            pass
        else:
            print("\nNo GPIO combination worked.")
            print("The board may not use GPIO direction control,")
            print("or servos may need power, or baud rate may be wrong.")

    GPIO.cleanup()


if __name__ == "__main__":
    main()
