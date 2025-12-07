import RPi.GPIO as GPIO
import time

STEPPER_PINS = (22, 23, 24, 25) # [STEPPER1, STEPPER2, STEPPER3, STEPPER4]

GPIO.setmode(GPIO.BCM)
for pin in STEPPER_PINS:
    GPIO.setup(pin, GPIO.OUT)

def FeederIndex():
    print('clockwise feed turn\n')

    steps = 2048 / 9 / 2 / 2   # your original math
    delay = 0.002              # 2 ms per step

    for _ in range(int(steps)):  # steps for 40 degree turn
        GPIO.output(STEPPER_PINS, (GPIO.HIGH, GPIO.LOW,  GPIO.LOW,  GPIO.LOW))
        time.sleep(delay)

        GPIO.output(STEPPER_PINS, (GPIO.LOW,  GPIO.HIGH, GPIO.LOW,  GPIO.LOW))
        time.sleep(delay)

        GPIO.output(STEPPER_PINS, (GPIO.LOW,  GPIO.LOW,  GPIO.HIGH, GPIO.LOW))
        time.sleep(delay)

        GPIO.output(STEPPER_PINS, (GPIO.LOW,  GPIO.LOW,  GPIO.LOW,  GPIO.HIGH))
        time.sleep(delay)

while 1:
    time.sleep(2.0) # pause and load ball
    FeederIndex()   # Advance feeder