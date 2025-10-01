#!/usr/bin/env python3
import serial
from time import sleep
import numpy as np
import RPi.GPIO as GPIO
import asyncio
import sys

#assign GPIO pins for motor
motor_channel = (13,16,19,20)

GPIO.setmode(GPIO.BCM)
GPIO.setup(motor_channel,GPIO.OUT)


async def StepperFull():
	print('clockwise\n') 
	GPIO.output(motor_channel, (GPIO.HIGH,GPIO.LOW,GPIO.LOW,GPIO.LOW))
	await asyncio.sleep(0.002)
	GPIO.output(motor_channel, (GPIO.LOW,GPIO.HIGH,GPIO.LOW,GPIO.LOW))
	await asyncio.sleep(0.002)
	GPIO.output(motor_channel, (GPIO.LOW,GPIO.LOW,GPIO.HIGH,GPIO.LOW))
	await asyncio.sleep(0.002)
	GPIO.output(motor_channel, (GPIO.LOW,GPIO.LOW,GPIO.LOW,GPIO.HIGH))
	await asyncio.sleep(0.002)

async def main():
	motor_direction = input('Select motor direction: c=clockwise\n') # you add counterclockwise option

	while True:
		try:
			if(motor_direction == 'c'):
				await StepperFull()

		except KeyboardInterrupt as e: # have to put ctrl+c for this to stop
			motor_direction = input('Select motor direction: c=clockwise or q to quit\n') # you add counterclockwise option
			if(motor_direction == 'q'):
				print('Motor Stopped')

asyncio.run(main())
				

