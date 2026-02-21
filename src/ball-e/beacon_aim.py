#!/usr/bin/env python3
"""
Beacon reader + servo commander for three IR beacons on a Raspberry Pi.
Reads BCM pins for left/mid/right beacons and sends servo microsecond
commands to the Arduino over serial to aim at the detected beacon.
"""
import asyncio
import sys
import time

import RPi.GPIO as GPIO
import serial

# ----------------- CONFIG -----------------
PIN_LEFT = 14   # BCM pin for left beacon input
PIN_MID = 15    # BCM pin for middle beacon input
PIN_RIGHT = 18  # BCM pin for right beacon input

POLL_HZ = 10.0  # how often to poll beacons
VERBOSE = True  # set False to reduce console output

SERIAL_PORT = "/dev/ttyACM0"
BAUD = 115200

SERVO_LEFT = 1000
SERVO_MID = 1500
SERVO_RIGHT = 2000
# ------------------------------------------


def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    for pin in (PIN_LEFT, PIN_MID, PIN_RIGHT):
        GPIO.setup(pin, GPIO.IN)


def open_serial():
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.2, write_timeout=0.2)
    time.sleep(0.2)
    ser.reset_input_buffer()
    return ser


def read_beacons():
    """Return a dict of beacon states (True = detected)."""
    return {
        "left": GPIO.input(PIN_LEFT) == GPIO.LOW,
        "mid": GPIO.input(PIN_MID) == GPIO.LOW,
        "right": GPIO.input(PIN_RIGHT) == GPIO.LOW,
    }


def choose_servo_target(states):
    """Priority: left > mid > right. Adjust as needed."""
    if states["left"]:
        return SERVO_LEFT
    if states["mid"]:
        return SERVO_MID
    if states["right"]:
        return SERVO_RIGHT
    return None


async def poll_and_command(ser):
    interval = 1.0 / POLL_HZ
    last_sent = None
    last_states = None
    while True:
        states = read_beacons()
        if VERBOSE and states != last_states:
            print(f"States L/M/R = {states}")
            last_states = states
        target = choose_servo_target(states)
        if target is not None and target != last_sent:
            try:
                # send preset command (P,sel) instead of raw microseconds
                if target == SERVO_LEFT:
                    ser.write(b"<P,0>")
                elif target == SERVO_MID:
                    ser.write(b"<P,1>")
                elif target == SERVO_RIGHT:
                    ser.write(b"<P,2>")
                ser.flush()
                last_sent = target
                print(f"Commanded servo to {target} us based on {states}")
            except Exception as e:
                print(f"Serial error sending servo cmd: {e}")
        elif target is None and VERBOSE:
            print("No beacon detected; holding position")
        await asyncio.sleep(interval)


async def main():
    print("Beacon/Servo Control Ready")
    setup_gpio()
    ser = open_serial()
    try:
        await poll_and_command(ser)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.close()
        except Exception:
            pass
        GPIO.cleanup()
        sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
