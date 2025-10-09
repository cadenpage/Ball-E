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
                x = int(fields[0])            # linePosition (0 or ~1000..5000)
                y = int(fields[1])            # isCross (0/1) -- keep as 'y' for your prints
                z = int(fields[2])            # echoed left motor -- keep for your prints
                # keep your s3/s4 debug prints exactly as you had them
                s3 = int(fields[6])
                s4 = int(fields[7])

                sensors = [int(val) for val in fields[3:11]]  # s0..s7

                # Keep your debug print format intact
                print([x, y, z, s3, s4, sensors])

                # ==================================================
                # SENSOR INTERPRETATION → EVENT  (kept intact)
                # ==================================================
                event = None
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
                # STATE MACHINE UPDATE (unchanged printing behavior)
                # ==================================================
                line_follower.on_event(event)

                # ==================================================
                # MISSION: 2 crosses → right turn, 3rd cross → stop
                # ==================================================
                now = time.time()

                # Rising-edge cross detect with cooldown
                is_cross = y
                if is_cross == 1 and prev_is_cross == 0 and (now - last_cross_time) > CROSS_COOLDOWN:
                    cross_count += 1
                    last_cross_time = now
                    print(f"--- CROSS #{cross_count} detected ---")
                    if cross_count == 2 and not turning and not mission_done:
                        print("Initiating right turn (after 2nd cross)")
                        turning = True
                        turn_until = now + TURN_DURATION
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
                    if now < turn_until:
                        leftMotor, rightMotor = TURN_RIGHT_SPEED
                    else:
                        turning = False
                        # After turn, resume forward; the FSM will re-center
                        leftMotor, rightMotor = 150, 130

                else:
                    # Your original state-based motor outputs (no sleeps)
                    if isinstance(line_follower.state, LeftOfLine):
                        leftMotor = int(80)
                        rightMotor = 200
                    elif isinstance(line_follower.state, RightOfLine):
                        leftMotor = 200
                        rightMotor = 80
                    elif isinstance(line_follower.state, Center):
                        leftMotor = 150
                        rightMotor = 130
                    elif isinstance(line_follower.state, Intersection):
                        # Hold while on a cross (non-blocking)
                        print("Intersection detected – holding")
                        leftMotor = 0
                        rightMotor = 0
                    elif isinstance(line_follower.state, Stop):
                        leftMotor = 0
                        rightMotor = 0

            except Exception as e:
                print("packet dropped:", e)

        # Always send the latest command each loop (keeps Arduino in sync)
        print(f"\rSTATE:{line_follower.state}  X:{x}  CROSSES:{cross_count}  TURN:{turning}  DONE:{mission_done}  CMD:({leftMotor},{rightMotor})", end="")

        sendString('/dev/ttyACM0', 115200, f'<{leftMotor},{rightMotor}>', 0.0001)

