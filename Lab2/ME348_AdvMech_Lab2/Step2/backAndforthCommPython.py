#!/usr/bin/env python3
import serial
import time
import numpy as np


Str2Send = '<handshake is happening>'
#why so I append '<' and '>' to the beginning and end of my message that I send to the arduino?

if __name__ == '__main__':
    ser=serial.Serial("/dev/ttyACM0",115200)
    ser.reset_input_buffer()
    ser.reset_output_buffer() #we clear the input and output buffer at the beginning of running any program to make sure
                             #that any bits left over in the buffer dont show up
    while True: 
            
            ser.write(Str2Send.encode('utf-8'))


            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8')
                print(line)
                #ive just called 2 methods from an instance of ser, what do they do?