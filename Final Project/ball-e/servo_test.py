from gpiozero import AngularServo
from time import sleep

from gpiozero.pins.pigpio import PiGPIOFactory
factory = PiGPIOFactory()

# servo = AngularServo(13, min_pulse_width=0.0005, max_pulse_width=0.0025) #input servo pin 12 = aim 13 = shoot
servo = AngularServo(12, min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=factory) #input servo pin 12 = aim 13 = shoot

while (True):
    servo.angle = 0
    sleep (2)

 