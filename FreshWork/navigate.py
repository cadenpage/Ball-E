import serial
import time

# Change this to your Arduino port
arduino_port = "/dev/ttyACM0"  # Linux/Mac
# arduino_port = "COM3"        # Windows
baud_rate = 115200

ser = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2)  # wait for Arduino reset

# Example route sent dynamically
route = ["F", "R", "F", "L"]

for cmd in route:
    print(f"Sending command: {cmd}")
    ser.write(cmd.encode())
    time.sleep(0.5)  # give Arduino time to start movement
