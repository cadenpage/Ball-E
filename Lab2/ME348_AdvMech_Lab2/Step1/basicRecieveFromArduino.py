#!/usr/bin/env python3
import serial
import time
import numpy as np

if __name__ == '__main__':
    ser=serial.Serial('/dev/ttyACM0',115200)
    ser.reset_input_buffer() #we clear the input buffer at the beginning of running any program to make sure
                             #that any bits left over in the buffer dont show up


    while True: #this statement causes our program to loop like the loop(){} function in arduino
        line = ser.readline().decode('utf-8')
        print(line)