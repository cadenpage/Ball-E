#!/usr/bin/env python3
import serial
import time
import numpy as np
import RPi.GPIO as GPIO

sensor = 14

GPIO.setmode(GPIO.BCM)
GPIO.setup(sensor,GPIO.IN)

print('IR Sensor Ready')
print()

try:
	while True:
		if GPIO.input(sensor):
			print('NOT DETECTED')
				
		else:
			print('DETECTED')
		time.sleep(1)

except KeyboardInterrupt:
	GPIO.cleanup()