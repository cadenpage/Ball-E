#!/usr/bin/env python3
import serial
import time
import numpy as np
from sendStringScript import sendString

from enum import Enum, auto
# import time  # duplicate, removed

class State(Enum):
    CENTERED = auto()
    LEFT = auto()
    RIGHT = auto()
    STOP = auto()
    INTERSECTION = auto()
    CROSSCOUNT = auto()
    TURN = auto()

# Tunables
BASE_SPEED = 180
LIMIT      = 255
Kp         = 0.02
MID        = 3500
BAND       = 250
INTERSECTION_COOLDOWN = 1.0
TURN_TIME  = 0.6

# FSM state
state = State.CENTERED
lines_hit = 0
t_mark = 0.0

def clamp(u, lo=0, hi=LIMIT):
    return max(lo, min(hi, int(u)))

def centered(z):
    return abs(z - MID) <= BAND

def is_stop(s):
    return s[2] and s[3] and s[4] and s[5]

def is_all_high(s, z):  # CHANGED: include z>=7000 as a cross
    return all(s) or (z >= 7000)

def sense_left(s):
    return (s[4] or s[5]) and not (s[2] or s[3])

def sense_right(s):
    return (s[2] or s[3]) and not (s[4] or s[5])

def drive_follow(z):
    err = (z - MID)
    L = BASE_SPEED + Kp*err
    R = BASE_SPEED - Kp*err
    return clamp(L), clamp(R)

def step_state_machine(z, sensors):
    global state, lines_hit, t_mark
    now = time.monotonic()

    # ---------- transitions ----------
    if is_all_high(sensors, z):            # CHANGED
        if state != State.INTERSECTION:
            state = State.INTERSECTION
            lines_hit += 1
            t_mark = now
    elif is_stop(sensors):
        state = State.STOP
    else:
        if sense_left(sensors):
            state = State.LEFT
        elif sense_right(sensors):
            state = State.RIGHT
        elif centered(z):
            state = State.CENTERED

    # ---------- actions ----------
    if state == State.CENTERED:
        return drive_follow(z)

    if state == State.LEFT:
        L, R = drive_follow(z)
        return clamp(L - 10), clamp(R + 10)

    if state == State.RIGHT:
        L, R = drive_follow(z)
        return clamp(L + 10), clamp(R - 10)

    if state == State.STOP:
        return 0, 0

    if state == State.INTERSECTION:
        dt = now - t_mark
        if dt < 0.30:
            return 0, 0
        elif dt < INTERSECTION_COOLDOWN:
            return 100, 100
        else:
            state = State.CENTERED
            return drive_follow(z)

    if state == State.TURN:
        dt = now - t_mark
        if dt < TURN_TIME:
            return clamp(+180), clamp(0)
        else:
            state = State.CENTERED
            return drive_follow(z)

    return clamp(BASE_SPEED), clamp(BASE_SPEED)

leftMotor = 100
rightMotor = 100

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 115200)
    ser.reset_input_buffer()

    while True:
        # --- READ & PARSE FIRST ---
        if ser.in_waiting > 0:
            try:
                raw = ser.readline().decode('utf-8').strip()
                parts = raw.split(',')
                # x,y,z,s0..s7
                x = int(parts[0]); y = int(parts[1])
                z = int(parts[2])                 # CHANGED: always numeric
                s0 = int(parts[3]); s1 = int(parts[4]); s2 = int(parts[5]); s3 = int(parts[6])
                s4 = int(parts[7]); s5 = int(parts[8]); s6 = int(parts[9]); s7 = int(parts[10])

                print([x,y,z,s0,s1,s2,s3,s4,s5,s6,s7])

                sensors = [int(bool(s0)), int(bool(s1)), int(bool(s2)), int(bool(s3)),
                           int(bool(s4)), int(bool(s5)), int(bool(s6)), int(bool(s7))]

                # --- COMPUTE with FSM ---
                leftMotor, rightMotor = step_state_machine(z, sensors)

            except Exception as ex:
                print("packet dropped:", ex)

        # --- THEN SEND COMMAND ---
        sendString('/dev/ttyACM0', 115200, f'<{leftMotor},{rightMotor}>', 0.0001)
