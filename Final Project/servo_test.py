from gpiozero import AngularServo
from time import sleep

servo = AngularServo(12, min_pulse_width=0.0005, max_pulse_width=0.0025)

while (True):
    servo.angle = 0
    sleep (10)
    servo.angle = 30
    sleep (1)
    servo.angle = -30
    sleep (1)