#!/usr/bin/env python3
import serial, time

PORT = '/dev/ttyACM0'
BAUD = 115200
SEND_HZ = 50.0
DT = 1.0 / SEND_HZ

# desired linear speeds in inches/sec (example values)
leftMotor  = 10.0
rightMotor = 10.0
BASE = 10.0

# --- minimal line-following knobs ---
MID = 3000       # center of linePosition scale (your Arduino sends ~1000..5000)
THRESH = 400     # deadband

if __name__ == '__main__':
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    t0 = time.time()
    try:
        while True:
            
            # Send desired velocities as <L,R>
            ser.write(f"<{leftMotor},{rightMotor}>\n".encode("ascii"))

            # (Optional) read one line of telemetry if available
            try:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    print(line)  # e.g., "pos,isCross,PWM_L,PWM_R,velL,velR,desL,desR"

                    # --- minimal dynamic change based on linePosition ---
                    parts = line.split(',')
                    if len(parts) >= 2:
                        pos = int(parts[0])    # linePosition from Arduino
                        # adjust for next cycle (very simple proof)
                        if pos > 0:  # only act on valid readings
                            if pos < MID - THRESH:
                                # line is left → slow left wheel slightly
                                leftMotor, rightMotor = BASE*0.8, BASE*1.0
                            elif pos > MID + THRESH:
                                # line is right → slow right wheel slightly
                                leftMotor, rightMotor = BASE*1.0, BASE*0.8
                            else:
                                # centered
                                leftMotor = rightMotor = BASE
            except UnicodeDecodeError:
                pass  # skip bad partial lines
            except ValueError:
                pass  # skip malformed numeric parse

            # pace the sender
            sleep_left = DT - ((time.time() - t0) % DT)
            if sleep_left > 0:
                time.sleep(sleep_left)
    finally:
        ser.close()
