#!/usr/bin/env python3
import serial
import time
import numpy as np
from sendStringScript import sendString

# --- Mission controller variables ---
cross_count = 0
prev_is_cross = 0
last_cross_time = 0.0
CROSS_COOLDOWN = 0.8  # seconds to avoid double counting while sitting on a junction

turning = False
turn_until = 0.0
TURN_RIGHT_SPEED = (220, 80)  # (left, right) => turn right
TURN_DURATION = 0.7           # seconds of right turn; tune on your bot

mission_done = False


leftMotor=int(100)
rightMotor=int(100)

class State(object):
    """
    We define a state object which provides some utility functions for the
    individual states within the state machine.
    """

    def __init__(self):
        print('Processing current state:', str(self))

    def on_event(self, event):
        """
        Handle events that are delegated to this State.
        """
        pass

    def __repr__(self):
        """
        Leverages the __str__ method to describe the State.
        """
        return self.__str__()

    def __str__(self):
        """
        Returns the name of the State.
        """
        return self.__class__.__name__

class LeftOfLine(State):
    """Robot is left of the line (needs to turn right)"""
    def on_event(self, event):
        if event == "centered":
            return Center()
        elif event == "right_detected":
            return RightOfLine()
        elif event == "intersection":
            return Intersection()
        elif event == "stop":
            return Stop()
        return self


class RightOfLine(State):
    """Robot is right of the line (needs to turn left)"""
    def on_event(self, event):
        if event == "centered":
            return Center()
        elif event == "left_detected":
            return LeftOfLine()
        elif event == "intersection":
            return Intersection()
        elif event == "stop":
            return Stop()
        return self


class Center(State):
    """Robot is centered on the line (go straight)"""
    def on_event(self, event):
        if event == "left_detected":
            return LeftOfLine()
        elif event == "right_detected":
            return RightOfLine()
        elif event == "intersection":
            return Intersection()
        elif event == "stop":
            return Stop()
        return self


class Intersection(State):
    """Robot detects an intersection"""
    def on_event(self, event):
        if event == "centered":
            return Center()
        elif event == "left_detected":
            return LeftOfLine()
        elif event == "right_detected":
            return RightOfLine()
        elif event == "stop":
            return Stop()
        return self


class Stop(State):
    """Robot is stopped"""
    def on_event(self, event):
        if event == "left_detected":
            return LeftOfLine()
        elif event == "right_detected":
            return RightOfLine()
        elif event == "centered":
            return Center()
        elif event == "intersection":
            return Intersection()
        return self


class LineFollower(object):
    """Line following state machine"""
    def __init__(self):
        self.state = Stop()

    def on_event(self, event):
        self.state = self.state.on_event(event)

if __name__ == '__main__':
    ser=serial.Serial('/dev/ttyACM0',115200)
    ser.reset_input_buffer() #clears anything the arduino has been sending while the Rpi isnt prepared to recieve.

    leftMotor = 0
    rightMotor = 0

    line_follower = LineFollower()



# initialize HUD vars that get printed every loop
    x = 0  # linePosition default so HUD doesn't crash

    while True:
        if ser.in_waiting > 0:
            try:
                raw = ser.readline().decode('utf-8').strip()
                if not raw:
                    continue
                fields = raw.split(',')
                if len(fields) < 12:
                    continue

                # -------------------- Parse telemetry --------------------
                x = int(fields[0])          # linePosition (0 or ~1000..5000)
                y = int(fields[1])          # isCross (0/1)
                z = int(fields[2])          # echoed left motor
                s3 = int(fields[6])
                s4 = int(fields[7])
                sensors = [int(val) for val in fields[3:11]]  # s0..s7

                # Keep your debug print format
                print([x, y, z, s3, s4, sensors])

                # ==================================================
                # SENSOR INTERPRETATION → EVENT  (your logic)
                # ==================================================
                if y == 1:
                    event = "intersection"
                elif (sensors[0] == 0 and sensors[7] == 0) and (sensors[3] > 800 or sensors[4] > 800):
                    event = "centered"
                elif sum(sensors[:3]) > sum(sensors[5:]):
                    event = "left_detected"
                elif sum(sensors[5:]) > sum(sensors[:3]):
                    event = "right_detected"
                else:
                    event = "stop"

                # ==================================================
                # STATE MACHINE UPDATE (prints on transitions)
                # ==================================================
                line_follower.on_event(event)

                # ==================================================
                # MISSION: 2 crosses → right turn, 3rd cross → stop
                # ==================================================
                now = time.time()
                is_cross = y

                # Rising-edge with cooldown
                if is_cross == 1 and prev_is_cross == 0 and (now - last_cross_time) > CROSS_COOLDOWN:
                    cross_count += 1
                    last_cross_time = now
                    print(f"--- CROSS #{cross_count} detected ---")

                    if cross_count == 2 and not turning and not mission_done:
                        print("Initiating right turn (after 2nd cross)")
                        turning = True
                        turn_until = now + TURN_DURATION
                        # Immediate kick so we don't sit on the cross
                        leftMotor, rightMotor = TURN_RIGHT_SPEED
                        ser.write(f"<{leftMotor},{rightMotor}>".encode('ascii'))
                        ser.flush()

                    elif cross_count >= 3 and not mission_done:
                        print("Mission complete (3rd cross) → stopping")
                        mission_done = True

                prev_is_cross = is_cross

                # ==================================================
                # MOTOR CONTROL PRIORITY:
                #   mission_done > turning > normal FSM state
                # ==================================================
                if mission_done:
                    leftMotor, rightMotor = 0, 0

                elif turning:
                    if time.time() < turn_until:
                        leftMotor, rightMotor = TURN_RIGHT_SPEED
                    else:
                        turning = False
                        leftMotor, rightMotor = 150, 130  # resume forward

                else:
                    # Normal FSM outputs (unchanged)
                    if isinstance(line_follower.state, LeftOfLine):
                        leftMotor, rightMotor = 80, 200
                    elif isinstance(line_follower.state, RightOfLine):
                        leftMotor, rightMotor = 200, 80
                    elif isinstance(line_follower.state, Center):
                        leftMotor, rightMotor = 150, 130
                    elif isinstance(line_follower.state, Intersection):
                        # Pass through early crosses
                        if cross_count < 2:
                            leftMotor, rightMotor = 150, 150
                        elif cross_count >= 3:
                            leftMotor, rightMotor = 0, 0
                        else:
                            # safety if race before 'turning' arms
                            leftMotor, rightMotor = 140, 140
                    else:  # Stop
                        leftMotor, rightMotor = 0, 0

            except Exception as e:
                print("packet dropped:", e)

        # HUD every loop (safe: x was initialized before loop)
        print(f"\rSTATE:{line_follower.state}  X:{x}  CROSSES:{cross_count}  TURN:{turning}  DONE:{mission_done}  CMD:({leftMotor},{rightMotor})", end="")

        # Always send latest command via the already-open handle
        ser.write(f"<{leftMotor},{rightMotor}>".encode('ascii'))
        ser.flush()

        time.sleep(0.02)
