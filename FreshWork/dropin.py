#!/usr/bin/env python3
import serial, time

PORT = '/dev/ttyACM0'
BAUD = 115200
SEND_HZ = 50.0
DT = 1.0 / SEND_HZ

# desired linear speeds in inches/sec (example values; change live if you want)
leftMotor  = 10
rightMotor = 10

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
                line = ser.readline().decode("utf-8").strip()
                if line:
                    print(line)  # e.g., "pos,isCross,PWM_L,PWM_R,velL,velR,desL,desR"
            except UnicodeDecodeError:
                pass  # skip bad partial lines

            # pace the sender
            sleep_left = DT - ((time.time() - t0) % DT)
            if sleep_left > 0:
                time.sleep(sleep_left)
    finally:
        ser.close()
