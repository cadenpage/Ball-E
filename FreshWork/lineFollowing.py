#!/usr/bin/env python3
import serial
import time
import numpy as np
from sendStringScript import sendString
leftMotor=int(100)
rightMotor=int(100)


if __name__ == '__main__':
    ser=serial.Serial('/dev/ttyACM0',115200)
    ser.reset_input_buffer() #clears anything the arduino has been sending while the Rpi isnt prepared to recieve.

    while True:
        # sendString('/dev/ttyACM0',115200,'<'+str(leftMotor)+','+str(rightMotor)+'>',0.0001)
        ser.write(('<'+str(leftMotor)+str(rightMotor)+'>').encode())
        if ser.in_waiting > 0:  #we wait until the arduino has sent something to us before we try to read anything from the serial port.
                 
                line = ser.readline().decode('utf-8')
                print(line)
                #line=line.split(',')


            #     #this splits the incoming string up by commas
            #     try:
                    
            #         x=int(line[0])
                    
            #     except:
            #          print("packet dropped") #this is designed to catch when python shoves bits on top of each other. 


            
            
            #     #Following is my control law, we're keeping it basic for now, writing good control law is your job
            #     #ok so high numbers(highest 7000) on the line follwing mean I am too far to the RIGHT,
            #     #low numbers mean I am too far on the RIGHT, 3500 means I am at the middle
            #     #below is a basic control law you can send to your motors, with an exeption if z is a value greater than 7000, meaning the arduino code sees that the line sensor is on a cross. Feel free to take insperation from this,
            # #but you will need to impliment a state machine similar to what you made in lab 2 (including a way of counting time without blocking)
            
            #     if x < 70001: #im assuming that in your arduino code you will be setting z to the int 8000 if you sense a cross, dont feel obligated to do it this way.  
            #         leftMotor=400-((x/7000)*300) #will be between 100 and 400, HIGh if z is low
            #         rightMotor=((x/7000)*300)+100 #will be between 100 and 400, LOW if z is low
            #     else:
            #         print('at intersetion')
            #         #do something here like incrimenting a value you call 'lines_hit' to one higher, and writing code to make sure that some time (1 second should do it) 
            #         # passes between being able to incriment lines_hit so that it wont be incrimented a bunch of times when you hit your first cross. IE give your robot time to leave a cross
            #         #before allowing lines_hit to be incrimented again.