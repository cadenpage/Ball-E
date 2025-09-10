#!/usr/bin/env python3
import serial
import time
import numpy as np

from sendStringScript import sendString

String2Send='<150,10>'  #these are my motor commands

if __name__ == '__main__':
    ser=serial.Serial("/dev/ttyACM0",115200)
    #every time the serial port is opened, the arduino program will restart, very convient!
    ser.reset_input_buffer()
    ser.reset_output_buffer() #we clear the input and output buffer at the beginning of running any program to make sure
                             #that any bits left over in the buffer dont show up

    while True: 
            ser.write(String2Send.encode('utf-8'))



            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8')
                print(line)
                #ive just called 2 methods from the ser object, what do they do?