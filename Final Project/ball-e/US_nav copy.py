#!/usr/bin/env python3
import serial
import time
import math
from gpiozero import AngularServo
import RPi.GPIO as GPIO

from gpiozero.pins.pigpio import PiGPIOFactory
factory = PiGPIOFactory()

####################################################################
# CONFIG (edit these easily)
####################################################################
portname = '/dev/ttyACM0'   # Serial port for Arduino
VERBOSE = False             # Set True for extra logs
TELEMETRY_TIMEOUT = 1.0     # seconds to wait for a telemetry line

# Init sequence parameters (time-based, since no IMU/encoders)
SPIN_SPEED = 45             # Motor speed for initial scan
SEEK_SPIN_SPEED = 45        # Motor speed when re-seeking closest wall (slower to avoid overshoot)
SCAN_DURATION = 8.0         # seconds to scan for closest wall (spin longer to sample more)
MIN_HIT_TOL = 2.0           # cm tolerance around best distance
SEEK_TIMEOUT = 10.0         # seconds while re-seeking the closest wall
NEAR_HITS_REQUIRED = 8      # consecutive samples within tolerance to count as facing wall
SETTLE_DELAY = 0.8          # seconds to pause after stopping a spin
TURN_90_S = 0.9             # seconds for ~90° turn (tune)
TURN_SPEED = 60            # motor speed for timed turns
DRIVE_SPEED = 60           # forward speed toward centerline
HALF_DISTANCE = 78.0        # target centerline distance (cm)
DIST_TOL = 5.0              # tolerance (cm)
DRIVE_TIMEOUT = 8.0         # cap straight drive time (s)
CENTER_NUDGE_CM = 5.0       # drive this many cm past nominal half-distance before stopping

# IR Sensor & Servos
POLL_HZ = 10.0  # how often to poll beacons
BAUD = 115200

SERVO_LEFT = 1000
SERVO_MID = 1500
SERVO_RIGHT = 2000

ANGLE_LEFT = 30
ANGLE_MID = 0
ANGLE_RIGHT = -30

REST_ANGLE = -90 #might be negative
FIRE_ANGLE = 90 #Should deflect ruler and let go

SHOT_COUNT = 0 #initiating count for shots taken just like me fr

# Actuator RPI GPIO Pins
IR_PINS = [14, 15, 18] #[PIN_LEFT, PIN_MID, PIN_RIGHT]
STEPPER_PINS = (22, 23, 24, 25) # [STEPPER1, STEPPER2, STEPPER3, STEPPER4]

# Post-init behavior
USE_LINE_FOLLOW = True      # True => use line following, False => drive straight to 56 cm
STOP_FRONT_CM = 56.0        # stop distance if driving straight

# Motor bias (tuned near comp day)
LEFT_BIAS = 1.12
RIGHT_BIAS = 1.0
####################################################################

#IR Commands

# Begin main logic
#Setup GPIO pins
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    for pin in IR_PINS:
        GPIO.setup(pin, GPIO.IN)
    for pin in STEPPER_PINS:
        GPIO.setup(pin, GPIO.OUT)

def read_beacons():
    """Return states for L/M/R where True means beacon detected."""
    return {
        "left": GPIO.input(IR_PINS[0]) == GPIO.LOW, # BCM pin for left beacon input
        "mid":  GPIO.input(IR_PINS[1])  == GPIO.LOW, # BCM pin for middle beacon input
        "right": GPIO.input(IR_PINS[2]) == GPIO.LOW # BCM pin for right beacon input
    }

def choose_servo_angle(states):
    """Priority L > M > R."""
    if states["left"]:
        return ANGLE_LEFT
    if states["mid"]:
        return ANGLE_MID
    if states["right"]:
        return ANGLE_RIGHT
    return None

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



#Phase 7: Robot is stopped, we begin to read beacons, aim, and shoot
print(f"[PHASE] Monitoring IR Beacon Telemetry & shooting system")
setup_gpio()
print("GPIO ready (BCM mode)")
prev_detected = False  # Remember if beacon was detected last loop
aim_servo = AngularServo(12, min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=factory) # Define the actual servo w/ PWM values from spec sheet
#Pin 12 will be used for aiming servo
shoot_servo = AngularServo(13, min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=factory) #"       "
#Pin 13 will be used for shooting servo
aim_servo.angle = 60  # Initial rest position for sensing

while SHOT_COUNT < 10:
    states = read_beacons()
    detected = any(states.values())  # Any IR beacon active?

    # -------- SHOOT ONLY ON THE RISING EDGE --------
    if detected and not prev_detected:
        # New detection → fire once
        target_angle = choose_servo_angle(states)
        if target_angle is not None:
            aim_servo.angle = target_angle
            time.sleep(2.0)

            # FIRE
            shoot_servo.angle = FIRE_ANGLE
            time.sleep(1.0)
            shoot_servo.angle = REST_ANGLE
            time.sleep(2.0)

            SHOT_COUNT += 1
            time.sleep(2.0) # pause and load ball
            FeederIndex()   # Advance feeder
            time.sleep(1.0) # pause after feeding
            aim_servo.angle = 60  # Reset to rest position
            print(f"[SHOOT] Fired shot #{SHOT_COUNT}")

    # update memory
    prev_detected = detected

    time.sleep(0.5)