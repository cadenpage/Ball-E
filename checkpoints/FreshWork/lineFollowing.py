#!/usr/bin/env python3
import serial
import time
import numpy as np
from minimal.sendStringScript import sendString
leftMotor=int(10)
rightMotor=int(10)
MID = 3600
THRESH = 600
TURNING = False
pos = MID

if __name__ == '__main__':
    ser=serial.Serial('/dev/ttyACM0',115200)
    ser.reset_input_buffer() #clears anything the arduino has been sending while the Rpi isnt prepared to recieve.
    
    while True:
        # sendString('/dev/ttyACM0',115200,'<'+str(leftMotor)+','+str(rightMotor)+'>',0.0001)
        ser.write(('<'+str(leftMotor)+ ',' + str(rightMotor)+'>').encode())
        if ser.in_waiting > 0:  #we wait until the arduino has sent something to us before we try to read anything from the serial port.
                
                line = ser.readline().decode('utf-8')
                print(line)    
                line=line.split(',')

                if TURNING:
                    time.sleep(3)
                    TURNING = False
                 #this splits the incoming string up by commas
                try:
                    
                    pos=int(line[0])
                    cross = int(line[1])

                except:
                     print("packet dropped") #this is designed to catch when python shoves bits on top of each other. 


            
            
                if not TURNING:

                    if pos < MID - THRESH:
                        print('left')
                        leftMotor= 8
                        rightMotor= 10
                    if pos > MID + THRESH:
                        print('right')
                        leftMotor= 10
                        rightMotor= 8
                    
                    if cross == 1:
                        print('TURN RIGHT')
                        TURNING = True
                        leftMotor = 10
                        rightMotor = -10
                    
                    #do something here like incrimenting a value you call 'lines_hit' to one higher, and writing code to make sure that some time (1 second should do it) 
                    # passes between being able to incriment lines_hit so that it wont be incrimented a bunch of times when you hit your first cross. IE give your robot time to leave a cross
                    #before allowing lines_hit to be incrimented again.