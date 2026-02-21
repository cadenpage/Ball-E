#!/usr/bin/env python3
import serial
import time
import numpy as np

from sendStringScript import sendString
leftMotor=int(100)
rightMotor=int(100)

x=int(1)
y=int(1)
z=int(1) #a bump sensor that is unactivated starts at 1 (because they are pullups), hence why these are all one
a=int(1)
b=int(1)
c=int(1)


if __name__ == '__main__':
    ser=serial.Serial('/dev/ttyACM0',115200)
    #every time the serial port is opened, the arduino program will restart, very convient!
    ser.reset_input_buffer()
    ready = 0
    

    while True:
        
        #think of the below line as the default condition where no pairs of sensors are triggered as state 0, where the robot moves forward
        sendString('/dev/ttyACM0',115200,'<'+str(leftMotor)+','+str(rightMotor)+'>',0.0005)
        #ser.write(b'<'+bytes(str(leftMotor),'utf-8')+b','+bytes(str(rightMotor),'utf-8')+b'>')


        #why so I append '<' and '>' to the beginning and end of my message that I send to the arduino?

        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8')
                #ive just called 2 methods from the ser object, what do they do? read the documentation and find out!
            line=line.split(',')
                #this one i wont ask you about this one is pretty self explanitory

            try:
                    
                x=int(line[0])
                y=int(line[1])
                z=int(line[2])

                a=int(line[3])  
                b=int(line[4])
                c=int(line[5])

                print([x,y,z,a,b,c])
                
            except:
                print("packetLost") 
                #why do I have this exepction? 



       
            #rudimentery state machine
        if x < 1 and y < 1:
            sendString('/dev/ttyACM0',115200,'<'+str(-leftMotor)+','+str(-rightMotor)+'>',0.0005)
            time.sleep(2)
            sendString('/dev/ttyACM0',115200,'<'+str(-leftMotor)+','+str(rightMotor)+'>',0.0005)
            time.sleep(.5)
            sendString('/dev/ttyACM0',115200,'<'+str(leftMotor)+','+str(rightMotor)+'>',0.0005)
            x=1
            y=1
        if z < 1 and a < 1:
           #your code here
           z=1
           a=1
        if b < 1 and c < 1:
            #your code here
            b=1
            c=1