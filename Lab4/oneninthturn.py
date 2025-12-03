import RPi.GPIO as GPIO
import asyncio

#assign GPIO pins for motor
motor_channel = (13,16,19,20)

GPIO.setmode(GPIO.BCM)
GPIO.setup(motor_channel,GPIO.OUT)

async def cFeedIndex():
	print('clockwise full 40deg turn\n') 
	steps=2048/9/2/2
	for _ in range(int(steps)): #steps for 40 degree turn
		GPIO.output(motor_channel, (GPIO.HIGH,GPIO.LOW,GPIO.LOW,GPIO.LOW))
		await asyncio.sleep(0.002)
		GPIO.output(motor_channel, (GPIO.LOW,GPIO.HIGH,GPIO.LOW,GPIO.LOW))
		await asyncio.sleep(0.002)
		GPIO.output(motor_channel, (GPIO.LOW,GPIO.LOW,GPIO.HIGH,GPIO.LOW))
		await asyncio.sleep(0.002)
		GPIO.output(motor_channel, (GPIO.LOW,GPIO.LOW,GPIO.LOW,GPIO.HIGH))
		await asyncio.sleep(.002)

async def main():
    await cFeedIndex()
    print('Done')
    GPIO.cleanup()
	
asyncio.run(main())