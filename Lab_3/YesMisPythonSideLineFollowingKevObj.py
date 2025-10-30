#!/usr/bin/env python3
import serial
import time
import numpy as np
from sendStringScript import sendString

# ---- Motion tunables ----
STOP_HOLD      = 0.20   # seconds to fully stop before turning
TURN_TIME      = .5   # seconds to execute the turn
STRAIGHT_TIME  = 0.30   # pass-through window at 1st cross

TURN_RIGHT_SPEED = (-200, 200)   # (left, right) → right turn
TURN_LEFT_SPEED  = (200, -200)   # (left, right) → left turn
STRAIGHT_SPEED   = (150, 150)  # straight drive for pass-through



leftMotor=int(0)
rightMotor=int(0)

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

class TurnRight(State):
    """Stop briefly, then perform a right turn, then return to Center."""
    def __init__(self):
        super().__init__()
        self.t0 = None

    def handle_action(self, follower):
        now = time.time()
        if self.t0 is None:
            self.t0 = now
            print("TurnRight: init")
            # immediate stop on entry
            follower.leftMotor = 0
            follower.rightMotor = 0
            return self

        dt = now - self.t0
        if dt < STOP_HOLD:
            # hold stop to settle
            follower.leftMotor = 0
            follower.rightMotor = 0
            return self
        elif dt < STOP_HOLD + TURN_TIME:
            # turning window
            follower.leftMotor, follower.rightMotor = TURN_RIGHT_SPEED
            return self
        else:
            # done turning → resume following
            print("TurnRight: complete")
            return Center()

    def on_event(self, event):
        # Ignore events during the deterministic turn
        return self


class TurnLeft(State):
    """Stop briefly, then perform a left turn, then return to Center."""
    def __init__(self):
        super().__init__()
        self.t0 = None

    def handle_action(self, follower):
        now = time.time()
        if self.t0 is None:
            self.t0 = now
            print("TurnLeft: init")
            follower.leftMotor = 0
            follower.rightMotor = 0
            return self

        dt = now - self.t0
        if dt < STOP_HOLD:
            follower.leftMotor = 0
            follower.rightMotor = 0
            return self
        elif dt < STOP_HOLD + TURN_TIME:
            follower.leftMotor, follower.rightMotor = TURN_LEFT_SPEED
            return self
        else:
            print("TurnLeft: complete")
            return Center()

    def on_event(self, event):
        return self



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
    """Robot detects an intersection and chooses the action based on cross_count."""
    def __init__(self):
        super().__init__()
        self.pause_start = None  # used for timing in pass-through only

    def handle_action(self, follower):
        count = follower.cross_count
        now = time.time()

        if count == 1:
            # ---- PASS THROUGH: short straight window ----
            if self.pause_start is None:
                self.pause_start = now
                print("Intersection: pass-through (cross #1)")
            if now - self.pause_start < STRAIGHT_TIME:
                follower.leftMotor, follower.rightMotor = STRAIGHT_SPEED
                return self
            else:
                self.pause_start = None
                return Center()

        elif count == 2:
            # ---- TURN RIGHT at 2nd cross ----
            print("Intersection: hand-off to TurnRight (cross #2)")
            return TurnRight()

        else:  # count >= 3
            # ---- STOP at 3rd cross or later ----
            print("Intersection: stopping (cross #3+)")
            follower.leftMotor = 0
            follower.rightMotor = 0
            return TurnLeft()

    def on_event(self, event):
        # While in Intersection we let handle_action drive transitions.
        return self


    # def on_event(self, event):
    #     if event == "centered":
    #         return Center()
    #     elif event == "left_detected":
    #         return LeftOfLine()
    #     elif event == "right_detected":
    #         return RightOfLine()
    #     elif event == "stop":
    #         return Stop()
    #     return self


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
        self.state = Stop()
        self.cross_count = 0
        self.in_cross = False
        self.last_cross_time = 0.0
        self.leftMotor = 0
        self.rightMotor = 0

    def on_event(self, event):
        self.state = self.state.on_event(event)

if __name__ == '__main__':
    ser=serial.Serial('/dev/ttyACM0',115200)
    ser.reset_input_buffer() #clears anything the arduino has been sending while the Rpi isnt prepared to recieve.

    leftMotor = 0
    rightMotor = 0

    line_follower = LineFollower()

    while True:
        #sendString('/dev/ttyACM0',115200,'<'+str(leftMotor)+','+str(rightMotor)+'>',0.0001)

        if ser.in_waiting > 0:  #we wait until the arduino has sent something to us before we try to read anything from the serial port.
            line = ser.readline().decode('utf-8')
            line=line.split(',')
            #this splits the incoming string up by commas
            try:
                x=int(line[0])
                y=int(line[1])
                z=int(line[2]) #we dont convert this to a float becasue we went to be able to recieve the message that we are at a cross, which wont be an int.
                sensors = [int(val) for val in line[3:11]] #s0-s7

                print([x,y,z,sensors, "Crosses:", line_follower.cross_count, "Motors:", leftMotor, rightMotor])

                # ==================================================
                # SENSOR INTERPRETATION → EVENT
                # ==================================================

                #new logic for turn / cross counting logic
                now = time.time()

                # Rising edge: just entered a new intersection
                if y == 1 and not line_follower.in_cross:
                    line_follower.in_cross = True

                    # Debounce so we don't count the same cross multiple times
                    if now - line_follower.last_cross_time > 0.7:
                        line_follower.cross_count += 1
                        line_follower.last_cross_time = now
                        print(f"Cross #{line_follower.cross_count} detected")

                        # Optionally trigger event now (you’ll still set it below)
                        event = "intersection"

                # Falling edge: left the cross area
                elif y == 0 and line_follower.in_cross:
                    line_follower.in_cross = False



                event = None
                #if z >= 7000:  # intersection detected
                if y==1:
                    event = "intersection"
                elif (sensors[0] == 0 and sensors[7] == 0) and (sensors[3] > 800 or sensors[4] > 800):  # middle sensors see line
                    event = "centered"
                elif sum(sensors[:3]) > sum(sensors[5:]):  # stronger on left side
                    event = "left_detected"
                elif sum(sensors[5:]) > sum(sensors[:3]):  # stronger on right side
                    event = "right_detected"
                else:
                    event = "stop"
                #state = state.on_event(event)  #bullshit
                #print("Current state:", state)
                # ==================================================
                # STATE MACHINE UPDATE
                # ==================================================
                line_follower.on_event(event)

                # ==================================================
                # ==================================================
                # MOTOR CONTROL BASED ON STATE
                # ==================================================
                if isinstance(line_follower.state, LeftOfLine):
                    leftMotor  = 20
                    rightMotor = 30 #updated values 

                elif isinstance(line_follower.state, RightOfLine):
                    leftMotor  = 20
                    rightMotor = 10

                elif isinstance(line_follower.state, Center):
                    leftMotor  = 30
                    rightMotor = 30

                elif isinstance(line_follower.state, (Intersection, TurnRight, TurnLeft)):
                    # Let action states decide motor outputs and transitions
                    line_follower.state = line_follower.state.handle_action(line_follower)
                    # Pull the commanded motors from the FSM object (fallback to existing values)
                    leftMotor  = getattr(line_follower, "leftMotor", leftMotor)
                    rightMotor = getattr(line_follower, "rightMotor", rightMotor)

                elif isinstance(line_follower.state, Stop):
                    leftMotor  = 0
                    rightMotor = 0


            except (IndexError, ValueError):
                print("packet dropped") #this is designed to catch when python shoves bits on top of each other.

        sendString('/dev/ttyACM0',115200,'<'+str(leftMotor)+','+str(rightMotor)+'>',0.0001)


            #define high value and low value variable just to see what vibe were working with


#centerVal = Array Logic
#Leftofline = array logic
#right of line = array logic
# stop
#counter variables, t, and cross



                #Following is my control law, we're keeping it basic for now, writing good control law is your job
                #ok so high numbers(highest 7000) on the line follwing mean I am too far to the LEFT,
                #low numbers mean I am too far on the RIGHT, 3500 means I am at the middle
                #below is a basic control law you can send to your motors, with an exeption if z is a value greater than 7000, meaning the arduino code sees that the line sensor is on a cross. Feel free to take insperation from this,
            #but you will need to impliment a state machine similar to what you made in lab 2 (including a way of counting time without blocking)

                # if not z < 7000: #im assuming that in your arduino code you will be setting z to the int 8000 if you sense a cross, dont feel obligated to do it this way.
                #     leftMotor=100+.02*z #now that we are SURE that z isnt the string cross, we cast z to an int and recalculate leftMotor and rightMotor,
                #     rightMotor=250-.02*z
                # else:
                #     print('at intersection')
                    #do something here like incrimenting a value you call 'lines_hit' to one higher, and writing code to make sure that some time (1 second should do it)
                    # passes between being able to incriment lines_hit so that it wont be incrimented a bunch of times when you hit your first cross. IE give your robot time to leave a cross
                    #before allowing lines_hit to be incrimented again.
