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


async def StepperFull(direction):
	print(f'{direction} full step\n')
	sequence = [
		(GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.LOW),
		(GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.LOW),
		(GPIO.LOW, GPIO.LOW, GPIO.HIGH, GPIO.LOW),
		(GPIO.LOW, GPIO.LOW, GPIO.LOW, GPIO.HIGH)
	]
	if direction == 'cc':
		sequence = sequence[::-1]
	for step in sequence:
		GPIO.output(motor_channel, step)
		await asyncio.sleep(0.002)
		print('Step executed')

async def StepperHalf(direction):
	print(f'{direction} half step\n')
	sequence = [
		(GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.LOW),
		(GPIO.HIGH, GPIO.HIGH, GPIO.LOW, GPIO.LOW),
		(GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.LOW),
		(GPIO.LOW, GPIO.HIGH, GPIO.HIGH, GPIO.LOW),
		(GPIO.LOW, GPIO.LOW, GPIO.HIGH, GPIO.LOW),
		(GPIO.LOW, GPIO.LOW, GPIO.HIGH, GPIO.HIGH),
		(GPIO.LOW, GPIO.LOW, GPIO.LOW, GPIO.HIGH),
		(GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.HIGH)
	]
	if direction == 'cc':
		sequence = sequence[::-1]
	for step in sequence:
		GPIO.output(motor_channel, step)
		await asyncio.sleep(0.002)
		print('Step executed')

async def main():
	while True:
		motor_mode = input('Select stepping mode: f=full, h=half, q=quit\n')
		if motor_mode == 'q':
			print('Exiting motor control.')
			break
		direction = input('Select motor direction: c=clockwise, cc=counterclockwise\n')

		if direction not in ['c', 'cc'] or motor_mode not in ['f', 'h']:
			print('Invalid input. Try again.')
			continue

		try:
			if motor_mode == 'f':
				await StepperFull(direction)
			elif motor_mode == 'h':
				await StepperHalf(direction)
		except KeyboardInterrupt:
			print('Interrupted. Use \"q\" to quit.')

asyncio.run(main())
				

