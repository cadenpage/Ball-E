from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep

factory = PiGPIOFactory()
servo = AngularServo(12, min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=factory)

def move_slowly(target_angle, step=2, delay=0.05):
    current = servo.angle or 0  # default to 0 if None
    # decide direction
    if target_angle > current:
        angles = range(int(current), int(target_angle) + 1, step)
    else:
        angles = range(int(current), int(target_angle) - 1, -step)

    for a in angles:
        servo.angle = a
        sleep(delay)

# Example: sequence of single-angle commands
while True:
    move_slowly(0)     # go to 0°
    sleep(1)
    move_slowly(30)    # go to 30°
    sleep(1)
    move_slowly(-30)   # go to -30°
    sleep(1)