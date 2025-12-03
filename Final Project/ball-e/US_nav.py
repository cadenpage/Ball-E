#!/usr/bin/env python3
import serial
import time
import math
from gpiozero import AngularServo
import RPi.GPIO as GPIO



def send_packet(ser, packet, char_delay=0.0001):
    """Send a packet like <left,right>.

    This first tries a single write of the full packet. If that times out,
    falls back to sending char-by-char. Exceptions are caught and logged.
    Returns True on success, False on failure.
    """
    data = packet.encode('utf-8')
    # Try a single write first (faster and less likely to interleave)
    try:
        ser.write(data)
        ser.flush()
        return True
    except serial.SerialTimeoutException:
        log(f"Write timeout sending packet '{packet}' (single-write). Falling back to char-by-char")
    except serial.SerialException as e:
        log(f"Serial error on write: {e}")
        return False

    # Fallback: send char-by-char with smasll delays; catch timeouts per-byte
    for b in data:
        try:
            ser.write(bytes([b]))
        except serial.SerialTimeoutException:
            log(f"Write timeout while sending packet '{packet}' (char). Aborting")
            return False
        except serial.SerialException as e:
            log(f"Serial error while sending packet: {e}")
            return False
        time.sleep(char_delay)

    try:
        ser.flush()
    except Exception:
        pass
    return True

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


def log(msg):
    if VERBOSE:
        print(msg)


def wait_for_arduino_ready(ser, timeout=5.0):
    """Wait for the Arduino to print a line enclosed in <> (ready heartbeat)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if getattr(ser, 'in_waiting', 0) == 0:
            time.sleep(0.05)
            continue
        line = ser.readline().decode('utf-8', errors='ignore').strip()

        if not line:
            continue
        log(f"Arduino boot: {line}")
        if line == "<Arduino is ready>" or (line.startswith('<') and line.endswith('>')):
            return True
    return False


def read_latest_telemetry(ser, timeout=TELEMETRY_TIMEOUT):
    """Blocking-ish read for latest telemetry line <front,left>."""
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        if getattr(ser, 'in_waiting', 0) == 0:
            if last is not None:
                return last
            time.sleep(0.02)
            continue
        raw = ser.readline().decode('utf-8', errors='ignore').strip()
        if not raw or not (raw.startswith('<') and raw.endswith('>')):
            continue
        payload = raw.strip('<>')
        parts = payload.split(',')
        try:
            front = float(parts[0])
            left = float(parts[1]) if len(parts) > 1 else float('nan')
            line = float(parts[2]) if len(parts) > 2 else float('nan')
            busy = int(parts[3]) if len(parts) > 3 else 0
        except ValueError:
            continue
        last = {'front': front, 'left': left, 'line': line, 'busy': busy}
    return last


def wait_until_idle(ser, timeout=5.0):
    """Wait until telemetry reports busy=0 (action complete)."""
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        tele = read_latest_telemetry(ser, timeout=0.2)
        if tele:
            last = tele
            if tele.get('busy', 0) == 0:
                return tele
    return last


# Begin main logic
#Setup GPIO pins
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    for pin in IR_PINS:
        GPIO.setup(pin, GPIO.IN)
    for pin in STEPPER_PINS:
        GPIO.setup(pin, GPIO.OUT)

#IR Commands

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

# Setup Serial
def timed_drive(left, right, duration):
    send_packet(ser, f"<S,{left},{right}>")
    time.sleep(duration)
    send_packet(ser, "<S,0,0>")


def cmd_speed(left, right):
    send_packet(ser, f"<S,{left},{right}>")


def cmd_bias(left_bias, right_bias):
    send_packet(ser, f"<B,{left_bias:.3f},{right_bias:.3f}>")


def cmd_drive_cm(cm, speed):
    send_packet(ser, f"<D,{cm:.2f},{speed}>")


def cmd_turn_deg(deg, speed):
    send_packet(ser, f"<T,{deg:.1f},{speed}>")


def cmd_max_caps(left_max, right_max):
    send_packet(ser, f"<M,{int(left_max)},{int(right_max)}>")

def cmd_line_follow_start():
    send_packet(ser, "<L>")


def cmd_line_follow_stop():
    send_packet(ser, "<X>")


def cmd_drive_to_front(stop_cm, speed):
    send_packet(ser, f"<F,{stop_cm:.1f},{speed}>")


def cmd_invert_line(flag):
    send_packet(ser, f"<I,{int(flag)}>")



print("Ball-E Control System Ready")
setup_gpio()
print("GPIO ready (BCM mode)")

# Open serial port with modest timeouts and a write timeout
ser = serial.Serial(portname, BAUD, timeout=0.5, write_timeout=3)

# Toggle DTR/RTS to hard-reset the Arduino (no unplug needed)
ser.setDTR(False)
ser.setRTS(False)
time.sleep(0.5)
ser.setDTR(True)
ser.setRTS(True)

# Give it time to reboot and flush any startup text
print("Resetting Arduino...")
time.sleep(1.0)
ser.reset_input_buffer()

print("Waiting for Arduino telemetry...")
if not wait_for_arduino_ready(ser, timeout=8.0):
    print("Warning: no ready signal from Arduino; continuing regardless.")

print("Beginning Python-controlled initialization. Press Ctrl+C to abort.\n")
ser.reset_input_buffer()

# Apply initial bias
cmd_bias(LEFT_BIAS, RIGHT_BIAS)


try:
    # Phase 1: spin scan to find min front distance
    print("[PHASE] Spin scan for closest wall")
    best_front = float('inf')
    spin_start = time.time()
    cmd_speed(-SPIN_SPEED, SPIN_SPEED)  # spin left
    while time.time() - spin_start < SCAN_DURATION:
        tele = read_latest_telemetry(ser, timeout=TELEMETRY_TIMEOUT)
        if tele:
            f = tele.get('front', float('inf'))
            if f > 0 and f < best_front:
                best_front = f
    cmd_speed(0, 0)
    time.sleep(SETTLE_DELAY)
    if best_front == float('inf'):
        print("No front sensor data during scan; aborting.")
        raise SystemExit(1)
    print(f"[PHASE] Closest wall measured at ~{best_front:.2f} cm")

    # Phase 2: spin again until near that minimum
    print("[PHASE] Seek closest wall again")
    cmd_speed(-SEEK_SPIN_SPEED, SEEK_SPIN_SPEED)
    near_hits = 0
    seek_start = time.time()
    last_good_f = float('inf')
    while time.time() - seek_start < SEEK_TIMEOUT and near_hits < NEAR_HITS_REQUIRED:
        tele = read_latest_telemetry(ser, timeout=TELEMETRY_TIMEOUT)
        if not tele:
            continue
        f = tele.get('front', float('nan'))
        if not math.isfinite(f) or f <= 0:
            continue  # ignore invalid readings
        last_good_f = f
        if f <= (best_front + MIN_HIT_TOL):
            near_hits += 1
        else:
            near_hits = 0
    cmd_speed(0, 0)
    time.sleep(SETTLE_DELAY)
    print(f"[PHASE] Facing closest wall (front={last_good_f:.2f} cm, hits={near_hits}/{NEAR_HITS_REQUIRED})")

    # Phase 3: classify side using left sensor (looking at right wall now)
    tele = read_latest_telemetry(ser, timeout=TELEMETRY_TIMEOUT)
    left_read = tele.get('left', float('nan')) if tele else float('nan')
    print(f"[PHASE] Left sensor now sees right wall: {left_read:.2f} cm")
    position = "center"
    if math.isfinite(left_read):
        if left_read > (HALF_DISTANCE + DIST_TOL):
            position = "left"
        elif left_read < (HALF_DISTANCE - DIST_TOL):
            position = "right"
    else:
        position = "unknown"  # no valid side reading; default to driving
    print(f"[PHASE] Classified position: {position.upper()}")

    # Phase 4: turn toward the center (left if on left side, right if on right)
    if position == "center":
        skip_drive = True
        turn_dir = -1  # default left for the pair of turns
        print("[PHASE] Near center; skipping center drive.")
    else:
        skip_drive = False
        turn_dir = -1 if position == "left" else 1

    print(f"[PHASE] Turning { 'left' if turn_dir==-1 else 'right' } 90°")
    cmd_turn_deg(90 * turn_dir, TURN_SPEED)
    wait_until_idle(ser, timeout=6.0)

    # Phase 5: drive toward centerline if not skipping
    if not skip_drive:
        print("[PHASE] Driving toward centerline...")
        drive_start = time.time()
        cmd_speed(DRIVE_SPEED, DRIVE_SPEED)
        last_front = float('inf')
        target_center = max(0.0, HALF_DISTANCE - CENTER_NUDGE_CM)
        while time.time() - drive_start < DRIVE_TIMEOUT:
            tele = read_latest_telemetry(ser, timeout=TELEMETRY_TIMEOUT)
            if not tele:
                continue
            last_front = tele.get('front', float('inf'))
            print(f"[DRIVE] front={last_front:.2f} cm (target {target_center:.2f}±{DIST_TOL:.2f})")
            if last_front <= (target_center + DIST_TOL):
                break
        cmd_speed(0, 0)
        print(f"[PHASE] Centerline approach complete, front={last_front:.2f} cm")

    # Phase 6: turn again to face away from back wall & Follow Line
    print(f"[PHASE] Turning { 'left' if turn_dir==-1 else 'right' } 90° to face away")
    cmd_turn_deg(90 * turn_dir, TURN_SPEED)
    wait_until_idle(ser, timeout=6.0)

    if USE_LINE_FOLLOW:
        print("\nInitialization complete. Starting line follow to goal...")
        cmd_line_follow_start()
    else:
        print("\nInitialization complete. Driving straight to stop distance...")
        cmd_drive_to_front(STOP_FRONT_CM, DRIVE_SPEED)

    #Phase 7: Robot is stopped, we begin to read beacons, aim, and shoot
    print(f"[PHASE] Monitoring IR Beacon Telemetry & shooting system")

    prev_detected = False  # Remember if beacon was detected last loop
    aim_servo = AngularServo(12, min_pulse_width=0.0005, max_pulse_width=0.0025) # Define the actual servo w/ PWM values from spec sheet
    #Pin 12 will be used for aiming servo
    shoot_servo = AngularServo(13, min_pulse_width=0.0005, max_pulse_width=0.0025) #"       "
    #Pin 13 will be used for shooting servo
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

                print(f"[SHOOT] Fired shot #{SHOT_COUNT}")

        # update memory
        prev_detected = detected

        time.sleep(0.5)

    # =================THIS IS WHAT I WROTE but im worried itll overcount shots so i added a debouncing version above^^ =====
    # while (SHOT_COUNT < 10): #10 total shots before being drunk
    #     states = read_beacons()
    #     target_angle = choose_servo_angle(states)
    #     if target_angle is not None:
    #         aim_servo.angle = target_angle
    #         # print(f"[BEACON] Detected states L/M/R = {states}, aiming servo to {target_angle}°")
    #         time.sleep(2.0)  # Pause to allow servo to reach position
    #         # ===== Activate shooting servo ==========
    #         # print("[SHOOT] Activating shooting servo")
    #         shoot_servo.angle = FIRE_ANGLE  # Move to shoot position
    #         time.sleep(1.0)         # Hold position briefly
    #         shoot_servo.angle = REST_ANGLE    # Return to rest position
    #         time.sleep(3.0)         # Allow time to return
    #         SHOT_COUNT += 1
    #     else:
    #         print(f"[BEACON] No beacon detected; returning to home")

    #     time.sleep(0.5)  # Polling interval


    # Keep the script alive to monitor until the robot reports idle
    print("[MONITOR] Watching telemetry... (Ctrl+C to stop)")
    while True:
        tele = read_latest_telemetry(ser, timeout=0.5)
        if tele:
            front = tele.get('front', float('nan'))
            busy = tele.get('busy', 0)
            print(f"[TEL] front={front:.2f} cm | busy={busy}")
            if busy == 0:
                print("[MONITOR] Robot reported idle; stopping monitor.")
                break
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nAborted by user.")
finally:
    GPIO.cleanup()
    cmd_speed(0, 0)
    ser.close()
    print("Serial closed.")
