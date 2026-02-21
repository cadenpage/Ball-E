#!/usr/bin/env python3
import serial
import time


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
SPIN_SPEED = 40             # Motor speed for spins (keep < 60)
SCAN_DURATION = 4.0         # seconds to scan for closest wall
MIN_HIT_TOL = 3.0           # cm tolerance around best distance
SEEK_TIMEOUT = 5.0          # seconds while re-seeking the closest wall
TURN_90_S = 1.1             # seconds for ~90° turn (tune)
TURN_SPEED = 50             # motor speed for timed turns (keep < 60)
DRIVE_SPEED = 55            # forward speed toward centerline (keep < 60)
HALF_DISTANCE = 81.0        # target centerline distance (cm)
DIST_TOL = 5.0              # tolerance (cm)
DRIVE_TIMEOUT = 8.0         # cap straight drive time (s)

# Motor bias (tuned near comp day)
LEFT_BIAS = 1.0
RIGHT_BIAS = 1.5
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


# Begin main logic


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


def cmd_turn_gain(gain):
    send_packet(ser, f"<G,{gain:.3f}>")


print("Ball-E Control System Ready")

# Open serial port with modest timeouts and a write timeout
ser = serial.Serial(portname, 115200, timeout=0.5, write_timeout=3)

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
    if best_front == float('inf'):
        print("No front sensor data during scan; aborting.")
        raise SystemExit(1)
    print(f"[PHASE] Closest wall measured at ~{best_front:.2f} cm")

    # Phase 2: spin again until near that minimum
    print("[PHASE] Seek closest wall again")
    cmd_speed(-SPIN_SPEED, SPIN_SPEED)
    near_hits = 0
    seek_start = time.time()
    while time.time() - seek_start < SEEK_TIMEOUT and near_hits < 3:
        tele = read_latest_telemetry(ser, timeout=TELEMETRY_TIMEOUT)
        if not tele:
            continue
        f = tele.get('front', float('inf'))
        if f > 0 and f <= (best_front + MIN_HIT_TOL):
            near_hits += 1
        else:
            near_hits = 0
    cmd_speed(0, 0)
    print(f"[PHASE] Facing closest wall (front={f if 'f' in locals() else float('nan'):.2f} cm)")

    # Phase 3: classify side using left sensor (looking at right wall now)
    tele = read_latest_telemetry(ser, timeout=TELEMETRY_TIMEOUT)
    left_read = tele.get('left', float('nan')) if tele else float('nan')
    print(f"[PHASE] Left sensor now sees right wall: {left_read:.2f} cm")
    position = "center"
    if left_read > (HALF_DISTANCE + DIST_TOL):
        position = "left"
    elif left_read < (HALF_DISTANCE - DIST_TOL):
        position = "right"
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
    timed_drive(turn_dir * TURN_SPEED, -turn_dir * TURN_SPEED, TURN_90_S)

    # Phase 5: drive toward centerline if not skipping
    if not skip_drive:
        print("[PHASE] Driving toward centerline...")
        drive_start = time.time()
        cmd_speed(DRIVE_SPEED, DRIVE_SPEED)
        last_front = float('inf')
        while time.time() - drive_start < DRIVE_TIMEOUT:
            tele = read_latest_telemetry(ser, timeout=TELEMETRY_TIMEOUT)
            if not tele:
                continue
            last_front = tele.get('front', float('inf'))
            print(f"[DRIVE] front={last_front:.2f} cm (target {HALF_DISTANCE:.2f}±{DIST_TOL:.2f})")
            if last_front <= (HALF_DISTANCE + DIST_TOL):
                break
        cmd_speed(0, 0)
        print(f"[PHASE] Centerline approach complete, front={last_front:.2f} cm")

    # Phase 6: turn again to face away from back wall
    print(f"[PHASE] Turning { 'left' if turn_dir==-1 else 'right' } 90° to face away")
    timed_drive(turn_dir * TURN_SPEED, -turn_dir * TURN_SPEED, TURN_90_S)

    print("\nInitialization sequence complete. Ready for next steps.")

except KeyboardInterrupt:
    print("\nAborted by user.")
finally:
    cmd_speed(0, 0)
    ser.close()
    print("Serial closed.")
