#!/usr/bin/env python3
import serial
import time
import numpy as np
import RPi.GPIO as GPIO
import asyncio

sensor = 14

GPIO.setmode(GPIO.BCM)
GPIO.setup(sensor,GPIO.IN)


async def IRcheck():
	await asyncio.sleep(1)
	irValue = GPIO.input(sensor)
	if irValue:
		print('NOT DETECTED')
				
	else:
		print('DETECTED')

async def main():
	print('IR Sensor Ready')

	try:
		while True:
			await IRcheck()
	except KeyboardInterrupt:
		GPIO.cleanup()

asyncio.run(main())
