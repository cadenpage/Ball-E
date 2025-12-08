#!/usr/bin/env python3
import serial
import time
import math

"""Standalone align-only tester.

Copies just enough config and helpers from US_nav.py so we can
run the alignment phase without importing US_nav (which runs
its own main routine on import).
"""

# --- Minimal config (copied from US_nav) ---
portname = '/dev/ttyACM0'
BAUD = 115200
VERBOSE = False
TELEMETRY_TIMEOUT = 1.0

SPIN_SPEED = 60
SEEK_SPIN_SPEED = 45
ALIGN_SCAN_DURATION = 1.5
MIN_HIT_TOL = 2.0
ALIGN_TIMEOUT = 10.0
NEAR_HITS_REQUIRED = 8
SETTLE_DELAY = 0.8

LEFT_BIAS = 1.12
RIGHT_BIAS = 1.0


def log(msg: str) -> None:
    if VERBOSE:
        print(msg)


def send_packet(ser: serial.Serial, packet: str, char_delay: float = 0.0001) -> bool:
    """Send a packet like <left,right> over serial.

    Matches the behavior in US_nav: try one write, fall back to
    char-by-char if needed. Returns True on success.
    """
    data = packet.encode("utf-8")
    try:
        ser.write(data)
        ser.flush()
        return True
    except serial.SerialTimeoutException:
        log(f"Write timeout sending packet '{packet}' (single-write). Falling back to char-by-char")
    except serial.SerialException as e:
        log(f"Serial error on write: {e}")
        return False

    # Fallback: char-by-char
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


def read_latest_telemetry(ser: serial.Serial, timeout: float = TELEMETRY_TIMEOUT):
    """Blocking-ish read for latest telemetry line <front,left,line,busy>.

    Copied from US_nav so align.py is self-contained.
    """
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        if getattr(ser, "in_waiting", 0) == 0:
            if last is not None:
                return last
            time.sleep(0.02)
            continue
        raw = ser.readline().decode("utf-8", errors="ignore").strip()
        if not raw or not (raw.startswith("<") and raw.endswith(">")):
            continue
        payload = raw.strip("<>")
        parts = payload.split(",")
        try:
            front = float(parts[0])
            left = float(parts[1]) if len(parts) > 1 else float("nan")
            line = float(parts[2]) if len(parts) > 2 else float("nan")
            busy = int(parts[3]) if len(parts) > 3 else 0
        except ValueError:
            continue
        last = {"front": front, "left": left, "line": line, "busy": busy}
    return last


def wait_for_arduino_ready(ser: serial.Serial, timeout: float = 5.0) -> bool:
    """Wait for an Arduino line enclosed in <> (ready heartbeat)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if getattr(ser, "in_waiting", 0) == 0:
            time.sleep(0.05)
            continue
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            continue
        log(f"Arduino boot: {line}")
        if line == "<Arduino is ready>" or (line.startswith("<") and line.endswith(">")):
            return True
    return False


def cmd_speed(left: int, right: int) -> None:
    send_packet(ser, f"<S,{left},{right}>")


def cmd_bias(left_bias: float, right_bias: float) -> None:
    send_packet(ser, f"<B,{left_bias:.3f},{right_bias:.3f}>")


def align_to_goal(ser: serial.Serial) -> bool:
    """Run just the phase-7 style alignment using LEFT sensor."""
    print("[ALIGN] Sweeping with LEFT sensor to find goal alignment")
    best_left = float('inf')

    # Sweep left
    print("[ALIGN] Sweeping left")
    spin_start = time.time()
    cmd_speed(-SPIN_SPEED, SPIN_SPEED)
    while time.time() - spin_start < ALIGN_SCAN_DURATION:
        tele = read_latest_telemetry(ser, timeout=TELEMETRY_TIMEOUT)
        if tele:
            d = tele.get('left', float('inf'))
            if d > 0 and d < best_left:
                best_left = d

    cmd_speed(0, 0)
    time.sleep(SETTLE_DELAY)

    # Sweep right (symmetric duration)
    print("[ALIGN] Sweeping right")
    spin_start = time.time()
    cmd_speed(SPIN_SPEED, -SPIN_SPEED)
    while time.time() - spin_start < (2 * ALIGN_SCAN_DURATION):
        tele = read_latest_telemetry(ser, timeout=TELEMETRY_TIMEOUT)
        if tele:
            d = tele.get('left', float('inf'))
            if d > 0 and d < best_left:
                best_left = d

    cmd_speed(0, 0)
    time.sleep(SETTLE_DELAY)

    if best_left == float('inf'):
        print("[ALIGN] No LEFT sensor data during alignment sweep; aborting.")
        return False

    print(f"[ALIGN] Closest side-wall (LEFT sensor) measured at ~{best_left:.2f} cm")

    # Re-seek the closest left distance
    print("[ALIGN] Seeking closest side-wall again")
    cmd_speed(-SEEK_SPIN_SPEED, SEEK_SPIN_SPEED)
    near_hits = 0
    seek_start = time.time()
    last_good = float('inf')

    while time.time() - seek_start < ALIGN_TIMEOUT and near_hits < NEAR_HITS_REQUIRED:
        tele = read_latest_telemetry(ser, timeout=TELEMETRY_TIMEOUT)
        if not tele:
            continue
        d = tele.get('left', float('nan'))
        if not math.isfinite(d) or d <= 0:
            continue
        last_good = d
        if d <= (best_left + MIN_HIT_TOL):
            near_hits += 1
        else:
            near_hits = 0

    cmd_speed(0, 0)
    time.sleep(SETTLE_DELAY)

    print(f"[ALIGN] Facing closest side-wall (LEFT={last_good:.2f} cm, hits={near_hits}/{NEAR_HITS_REQUIRED})")
    return near_hits >= NEAR_HITS_REQUIRED


if __name__ == "__main__":
    print("[ALIGN] Align-only tester starting up")
    global ser
    ser = serial.Serial(portname, BAUD, timeout=0.5, write_timeout=3)

    # Reset Arduino like in US_nav
    ser.setDTR(False)
    ser.setRTS(False)
    time.sleep(0.5)
    ser.setDTR(True)
    ser.setRTS(True)

    print("[ALIGN] Resetting Arduino...")
    time.sleep(1.0)
    ser.reset_input_buffer()

    print("[ALIGN] Waiting for Arduino telemetry...")
    if not wait_for_arduino_ready(ser, timeout=8.0):
        print("[ALIGN] Warning: no ready signal from Arduino; continuing regardless.")

    ser.reset_input_buffer()

    # Apply same motor bias
    cmd_bias(LEFT_BIAS, RIGHT_BIAS)

    try:
        ok = align_to_goal(ser)
        if not ok:
            print("[ALIGN] Alignment did not complete cleanly.")
        else:
            print("[ALIGN] Alignment complete.")
    except KeyboardInterrupt:
        print("\n[ALIGN] Aborted by user.")
    finally:
        cmd_speed(0, 0)
        ser.close()
        print("[ALIGN] Serial closed.")
