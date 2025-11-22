#!/usr/bin/env python3
import serial
import time
import numpy as np
import RPi.GPIO as GPIO


def write_packet_charwise(serial_port: serial.Serial, packet: str, char_delay: float = 0.0001) -> None:
    encoded = packet.encode('utf-8')
    for byte in encoded:
        serial_port.write(bytes([byte]))
        time.sleep(char_delay)
    serial_port.flush()

portname = '/dev/ttyACM0'
sensor = 14  # IR Sensor pin
VERBOSE = False


def log(msg: str) -> None:
    if VERBOSE:
        print(msg)


def wait_for_arduino_ready(serial_port: serial.Serial, timeout: float = 5.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if serial_port.in_waiting == 0:
            time.sleep(0.05)
            continue
        line = serial_port.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue
        log(f"Arduino boot: {line}")
        if line == "<Arduino is ready>" or (line.startswith('<') and line.endswith('>')):
            return True
    return False


def normalize_delta(new_angle: float, old_angle: float) -> float:
    diff = new_angle - old_angle
    if diff > 180.0:
        diff -= 360.0
    elif diff < -180.0:
        diff += 360.0
    return diff


def shortest_angle_diff(target_angle: float, current_angle: float) -> float:
    diff = target_angle - current_angle
    if diff > 180.0:
        diff -= 360.0
    elif diff < -180.0:
        diff += 360.0
    return diff


def read_latest_telemetry(serial_port: serial.Serial, timeout: float = 2.0):
    """Read and return the most recent telemetry packet within timeout.
    Returns dict with keys 'theta', 'front', 'left' or None if none found.
    """
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        while serial_port.in_waiting > 0:
            line = serial_port.readline().decode('utf-8', errors='ignore').strip()
            if not line or not (line.startswith('<') and line.endswith('>')):
                continue
            parts = line.strip('<>').split(',')
            try:
                theta = float(parts[0])
                front = float(parts[1]) if len(parts) >= 2 else float('nan')
                left = float(parts[2]) if len(parts) >= 3 else float('nan')
            except ValueError:
                continue
            last = {'theta': theta, 'front': front, 'left': left}
        if last is not None:
            return last
        time.sleep(0.02)
    return last


# def align_to_heading(serial_port: serial.Serial, target_theta: float, current_theta: float,
#                      angle_tolerance: float = 3.0, spin_interval: float = 0.05,
#                      settle_samples: int = 8, timeout: float = 30.0,
#                      status_interval: float = 5.0) -> float:
#     """Align robot to target_theta using the same control strategy as Phase 1.
#     Returns the last observed `current_theta` (float)."""
#     alignment_start = time.time()
#     settle_counter = 0
#     next_align_status = alignment_start
#     current = current_theta

#     while time.time() - alignment_start < timeout and settle_counter < settle_samples:
#         angle_error = shortest_angle_diff(target_theta, current)

#         drive_left = 0
#         drive_right = 0
#         if abs(angle_error) <= angle_tolerance:
#             write_packet_charwise(serial_port, '<1,0,0>')
#             settle_counter += 1
#         else:
#             settle_counter = 0
#             speed = int(max(30, min(150, abs(angle_error) * 2.0)))
#             if angle_error > 0:
#                 drive_left = speed
#                 drive_right = -speed
#             else:
#                 drive_left = -speed
#                 drive_right = speed
#             write_packet_charwise(serial_port, f'<1,{drive_left},{drive_right}>')

#         time.sleep(spin_interval)

#         # consume any telemetry and update current heading
#         while serial_port.in_waiting > 0:
#             line = serial_port.readline().decode('utf-8', errors='ignore').strip()
#             if not line or not (line.startswith('<') and line.endswith('>')):
#                 continue
#             parts = line.strip('<>').split(',')
#             try:
#                 current = float(parts[0])
#             except ValueError:
#                 continue
#             angle_error = shortest_angle_diff(target_theta, current)

#         if time.time() >= next_align_status:
#             log(f"Aligning: heading {current:7.2f}°, error {angle_error:6.2f}°")
#             next_align_status = time.time() + status_interval

#     write_packet_charwise(serial_port, '<1,0,0>')
#     time.sleep(0.2)
#     return current
def align_to_heading(serial_port: serial.Serial, target_theta: float, current_theta: float,
                     angle_tolerance: float = 3.0, spin_interval: float = 0.05,
                     settle_samples: int = 4, timeout: float = 25.0,
                     status_interval: float = 5.0) -> float:
    """Align robot to target_theta, using capped low speeds.

    - Speed is capped at 60 in all cases
    - Only spins in place (no forward drive)
    """
    alignment_start = time.time()
    settle_counter = 0
    next_status_time = alignment_start
    current = current_theta

    while time.time() - alignment_start < timeout and settle_counter < settle_samples:
        angle_error = shortest_angle_diff(target_theta, current)

        if abs(angle_error) <= angle_tolerance:
            write_packet_charwise(serial_port, '<1,0,0>')
            settle_counter += 1
        else:
            settle_counter = 0
            # base 30, cap at 60
            raw_speed = abs(angle_error) * 1.5
            speed = int(max(30, min(60, raw_speed)))

            if angle_error > 0:
                left_cmd = speed
                right_cmd = -speed
            else:
                left_cmd = -speed
                right_cmd = speed

            write_packet_charwise(serial_port, f'<1,{left_cmd},{right_cmd}>')

        time.sleep(spin_interval)

        tele = read_latest_telemetry(serial_port, timeout=0.3)
        if tele is not None:
            current = tele['theta']

        if time.time() >= next_status_time:
            print(f"[ALIGN] heading {current:7.2f}°, error {angle_error:6.2f}°")
            next_status_time = time.time() + status_interval

    write_packet_charwise(serial_port, '<1,0,0>')
    time.sleep(0.2)
    return current


# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(sensor, GPIO.IN)

# Setup Serial
print("Ball-E Control System Ready")

ser = serial.Serial(portname, 115200, timeout=1)

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

print("\nPhase 1: double spin + align")
input("Press ENTER to begin...")

ser.reset_input_buffer()

SPIN_SPEED = -45


SPIN_INTERVAL = 0.05  # seconds between read attempts
ROTATION_TARGET_DEG = 720.0  # two full rotations
KEEPALIVE_INTERVAL = 2.0  # seconds between re-sending spin command
STATUS_INTERVAL = 5.0  # seconds between progress prints
ANGLE_TOLERANCE = 3.0  # degrees
TURN_TIMEOUT = 30.0  # seconds
SETTLE_SAMPLES_REQUIRED = 8

accumulated_rotation = 0.0
prev_theta = None
current_theta = None
min_front = float('inf')
min_theta = 0.0
samples = []
start_time = time.time()
iteration = 0

spin_cmd = f'<1,{SPIN_SPEED},{-SPIN_SPEED}>'
write_packet_charwise(ser, spin_cmd)
last_spin_command = time.time()
next_status_time = time.time() + STATUS_INTERVAL

while accumulated_rotation < ROTATION_TARGET_DEG and (time.time() - start_time) < 120:
    iteration += 1
    
    if time.time() - last_spin_command >= KEEPALIVE_INTERVAL:
        write_packet_charwise(ser, spin_cmd)
        last_spin_command = time.time()
    
    received = False
    while ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line or not (line.startswith('<') and line.endswith('>')):
            continue
        received = True
        parts = line.strip('<>').split(',')
        if len(parts) < 2:
            continue
        try:
            theta = float(parts[0])


            front = float(parts[1])
            left = float(parts[2]) if len(parts) >= 3 else float('nan')
        except ValueError:
            continue
        
        current_theta = theta
        if prev_theta is not None:
            delta = normalize_delta(theta, prev_theta)
            accumulated_rotation += abs(delta)
        prev_theta = theta
        
        samples.append({'theta': theta, 'front': front, 'left': left})
        print(f"[SPIN] sample {len(samples):04d} | θ={theta:7.2f}° | front={front:6.2f} cm | left={left:6.2f} cm")
        
        if front > 0 and front < min_front:
            min_front = front
            min_theta = theta
    
    if time.time() >= next_status_time:
        print(f"Spin progress: {accumulated_rotation:5.0f}° rotated | best {min_front:6.2f} cm @ {min_theta:6.2f}°")
        next_status_time = time.time() + STATUS_INTERVAL
    time.sleep(SPIN_INTERVAL)

write_packet_charwise(ser, '<1,0,0>')
time.sleep(0.5)

if not samples:
    print("\nNo sensor data received during spin. Check connections.")
    ser.close()
    GPIO.cleanup()
    exit()

print("\n" + "=" * 60)
print("SPIN SUMMARY")
print("=" * 60)
print(f"Total rotation accumulated : {accumulated_rotation:.1f}°")
print(f"Total samples collected    : {len(samples)}")
print(f"Closest wall distance      : {min_front:.2f} cm")
print(f"Heading at closest wall    : {min_theta:.2f}°\n")

target_theta = min_theta
print(f"Target heading (face wall) : {target_theta:.2f}°")
print("\nAligning robot to target heading...")

alignment_start = time.time()
settle_counter = 0
current_theta = current_theta if current_theta is not None else min_theta
next_align_status = alignment_start

while time.time() - alignment_start < TURN_TIMEOUT and settle_counter < SETTLE_SAMPLES_REQUIRED:
    angle_error = shortest_angle_diff(target_theta, current_theta)
    
    drive_left = 0
    drive_right = 0
    if abs(angle_error) <= ANGLE_TOLERANCE:
        write_packet_charwise(ser, '<1,0,0>')
        settle_counter += 1
    else:
        settle_counter = 0
        #speed = int(max(30, min(150, abs(angle_error) * 2.0)))
        speed = 45
        if angle_error > 0:
            drive_left = speed
            drive_right = -speed
        else:
            drive_left = -speed
            drive_right = speed
        write_packet_charwise(ser, f'<1,{drive_left},{drive_right}>')
    
    time.sleep(SPIN_INTERVAL)
    
    updated = False
    while ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line or not (line.startswith('<') and line.endswith('>')):
            continue
        updated = True
        parts = line.strip('<>').split(',')
        try:
            current_theta = float(parts[0])
        except ValueError:
            continue
        angle_error = shortest_angle_diff(target_theta, current_theta)
    
    if time.time() >= next_align_status:
        print(f"Aligning: heading {current_theta:7.2f}°, error {angle_error:6.2f}°")
        next_align_status = time.time() + STATUS_INTERVAL

write_packet_charwise(ser, '<1,0,0>')
time.sleep(0.5)

print("\n" + "=" * 60)
print("PHASE 1 COMPLETE")
print("=" * 60)
print(f"Final heading              : {current_theta:.2f}°")
print(f"Target heading             : {target_theta:.2f}°")
print(f"Closest wall distance      : {min_front:.2f} cm\n")
print("Phase 1 finished. Ready for next phases.\n")


print("\n===== CENTERLINE APPROACH SEQUENCE =====\n")

HALF_DISTANCE = 81.0  # cm
DIST_TOL = 5.0        # cm tolerance around centerline
STRAIGHT_SPEED = 60   # max straight speed


def drive_until_distance(serial_port: serial.Serial,
                         target_dist_cm: float,
                         forward_speed: int = STRAIGHT_SPEED,
                         timeout: float = 12.0) -> float:
    """Drive straight until front US is at or inside target_dist_cm.

    Stops as soon as front <= target_dist_cm,
    or when timeout elapses. Returns the last
    measured front distance.
    """
    start = time.time()
    last_front = float('nan')

    # start driving forward
    fwd = int(max(20, min(STRAIGHT_SPEED, abs(forward_speed))))
    write_packet_charwise(serial_port, f"<1,{fwd},{fwd}>")

    while time.time() - start < timeout:
        tele = read_latest_telemetry(serial_port, timeout=0.15)
        if tele is None:
            continue

        front = tele.get('front', float('nan'))
        if not np.isfinite(front):
            continue

        last_front = front
        print(f"[CENTERLINE DRIVE] front = {front:.2f} cm (target {target_dist_cm:.2f} cm)")

        if front <= target_dist_cm:
            break

    # stop
    write_packet_charwise(serial_port, "<1,0,0>")
    return last_front


# At this point, robot has just finished facing the closest wall.
current_theta = current_theta if current_theta is not None else target_theta

# Step 1: Rotate to face the farthest (orthogonal) wall
check_theta = (target_theta - 90.0) % 360.0
print(f"Turning left 90° to face far-side wall @ {check_theta:.2f}°")
current_theta = align_to_heading(ser,
                                 target_theta=check_theta,
                                 current_theta=current_theta,
                                 angle_tolerance=ANGLE_TOLERANCE,
                                 spin_interval=SPIN_INTERVAL,
                                 settle_samples=4,
                                 timeout=15.0,
                                 status_interval=STATUS_INTERVAL)

# Step 2: Now that we are facing the far-side wall, read distance and classify
tele = read_latest_telemetry(ser, timeout=2.0)
if tele is None:
    print("Warning: no telemetry received for side-check; assuming centerline (no drive).")
    front_check = float('nan')
else:
    front_check = tele.get('front', float('nan'))

print(f"Side-check front reading: {front_check:.2f} cm (half target {HALF_DISTANCE:.2f} ± {DIST_TOL:.2f})")

position = "center"
if np.isfinite(front_check):
    if front_check > HALF_DISTANCE + DIST_TOL:
        position = "left"
    elif front_check < HALF_DISTANCE - DIST_TOL:
        position = "right"
    else:
        position = "center"

print(f"Classified robot position along corridor: {position.upper()}")


# Step 3: Move to centerline if necessary
if position == "left":
    print("On LEFT side: driving forward until ~81 cm from side wall.")
    final_front = drive_until_distance(ser, target_dist_cm=HALF_DISTANCE)
    print(f"Final side-wall distance after drive: {final_front}")
elif position == "right":
    print("On RIGHT side: turning 180°, then driving until ~81 cm from opposite wall.")

    # rotate 180° in place from current heading
    rotate_theta = (current_theta + 180.0) % 360.0
    current_theta = align_to_heading(ser,
                                     target_theta=rotate_theta,
                                     current_theta=current_theta,
                                     angle_tolerance=ANGLE_TOLERANCE,
                                     spin_interval=SPIN_INTERVAL,
                                     settle_samples=4,
                                     timeout=20.0,
                                     status_interval=STATUS_INTERVAL)

    final_front = drive_until_distance(ser, target_dist_cm=HALF_DISTANCE)
    print(f"Final opposite-wall distance after drive: {final_front}")
else:
    print("Near centerline already; no straight motion needed.")


# Step 4: Face away from the closest wall (180° from min_theta)
final_theta = (min_theta + 180.0) % 360.0
print(f"Final step: orienting 180° away from closest wall -> {final_theta:.2f}°")
current_theta = align_to_heading(ser,
                                 target_theta=final_theta,
                                 current_theta=current_theta,
                                 angle_tolerance=ANGLE_TOLERANCE,
                                 spin_interval=SPIN_INTERVAL,
                                 settle_samples=6,
                                 timeout=30.0,
                                 status_interval=STATUS_INTERVAL)

print("\n===== CENTERLINE APPROACH COMPLETE =====")
print(f"Closest wall distance      : {min_front:.2f} cm")
print(f"Closest wall heading       : {min_theta:.2f}°")
print(f"Final robot heading        : {current_theta:.2f}° (away from closest wall)")

ser.close()
GPIO.cleanup()
print("Done!")

