#!/usr/bin/env python3
import serial
import time
import numpy as np
from sendStringScript import sendString




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
    def __init__(self):
        super().__init__()
        self.pause_start = None  # timestamp marker
        self.action_done = False

    def handle_action(self, follower):
        """Perform action based on current cross count"""
        count = follower.cross_count
        now = time.time()

        # Initialize timing when we first enter this intersection
        if self.pause_start is None:
            self.pause_start = now
            print(f"At cross #{count}")

        # ========= 1️⃣  FIRST CROSS: Pass through =========
        if count == 1:
            # For ~0.3 sec, just drive straight
            if now - self.pause_start < 0.3:
                follower.leftMotor = 150
                follower.rightMotor = 150
                return self
            else:
                # Done passing through → go back to following
                self.pause_start = None
                self.action_done = True
                return Center()

        # ========= 2️⃣  SECOND CROSS: Turn right =========
        elif count == 2:
            # For ~0.6 sec, perform right turn
            if now - self.pause_start < 0.6:
                follower.leftMotor = 200
                follower.rightMotor = 80
                return self
            else:
                # Finished turn, back to line following
                self.pause_start = None
                self.action_done = True
                return Center()

        # ========= 3️⃣  THIRD CROSS: Stop =========
        elif count >= 3:
            follower.leftMotor = 0
            follower.rightMotor = 0
            print("Third intersection reached — stopping robot")
            return Stop()

        # Safety: stay in Intersection if nothing else triggered
        return self

    def on_event(self, event):
        # Normal transitions are ignored here; handle_action controls it
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
                # s0 = int(line[3])
                # s1 = int(line[4])
                # s2 = int(line[5])
                s3 = int(line[6])
                s4 = int(line[7])
                # s5 = int(line[8])
                # s6 = int(line[9])
                # s7 = int(line[10])
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
                # MOTOR CONTROL BASED ON STATE
                # ==================================================
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
                    # Let the Intersection class decide what to do
                    line_follower.state = line_follower.state.handle_action(line_follower)

                    # Update motor outputs from the FSM object (if stored there)
                    leftMotor = getattr(line_follower, "leftMotor", leftMotor)
                    rightMotor = getattr(line_follower, "rightMotor", rightMotor)


                elif isinstance(line_follower.state, Stop):
                    leftMotor = 0
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
