#!/usr/bin/env python3
import serial
import time
import numpy as np
import RPi.GPIO as GPIO

portname = '/dev/ttyACM0'
sensor = 14

GPIO.setmode(GPIO.BCM)
GPIO.setup(sensor, GPIO.IN)

print("=" * 60)
print("Ball-E - Spin & Find Minimum Distance")
print("=" * 60)

ser = serial.Serial(portname, 115200, timeout=1)
ser.reset_input_buffer()
time.sleep(2)

print("\nFlushing Arduino startup...")
time.sleep(1)
while ser.in_waiting > 0:
    ser.readline()

print("\nStarting 1 full spin (120 seconds)...")
print("Motor speed: 50 (slow)")
print("Will find angle where front US distance is minimum")
print()

spinStartTime = time.time()
minFrontDist = float('inf')
minFrontAngle = 0.0
count = 0
allData = []

print(f"{'Time (s)':>8} | {'Angle (°)':>8} | {'Front (cm)':>10} | Status")
print("-" * 60)

while time.time() - spinStartTime < 120.0:
    elapsed = time.time() - spinStartTime
    
    # Send spin command
    cmd = b'<1,50,-50>'
    ser.write(cmd)
    ser.flush()
    
    # Wait and read
    time.sleep(0.1)
    
    while ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        
        if line.startswith('<') and line.endswith('>'):
            try:
                parts = line.strip('<>').split(',')
                theta = float(parts[0])
                front = float(parts[1])
                left = float(parts[2])
                
                count += 1
                allData.append({'theta': theta, 'front': front, 'elapsed': elapsed})
                
                status = ""
                if front > 0 and front < minFrontDist:
                    minFrontDist = front
                    minFrontAngle = theta
                    status = "*** MINIMUM ***"
                
                print(f"{elapsed:8.1f} | {theta:8.1f} | {front:10.2f} | {status}")
                
            except:
                pass
    
    time.sleep(0.9)

print("\n" + "=" * 60)
print("SPIN COMPLETE")
print("=" * 60)

# Stop motors
ser.write(b'<1,0,0>')
time.sleep(0.5)

print(f"\nTotal readings: {count}")
print(f"\nMinimum front distance: {minFrontDist:.2f} cm")
print(f"Angle at minimum: {minFrontAngle:.2f}°")

# Calculate target angle (90 degrees to the right)
targetAngle = (minFrontAngle + 90.0) % 360.0
print(f"Target angle (90° right): {targetAngle:.2f}°")

print("\n" + "=" * 60)
print("ROTATING TO TARGET ANGLE")
print("=" * 60)

# Now rotate to target angle
rotateStartTime = time.time()
print(f"\nCurrent angle: {minFrontAngle:.2f}°")
print(f"Target angle: {targetAngle:.2f}°")
print(f"Will rotate slowly until reaching target...\n")

print(f"{'Time (s)':>8} | {'Angle (°)':>8} | {'Angle Diff':>12} | Status")
print("-" * 60)

reachedTarget = False
while not reachedTarget and time.time() - rotateStartTime < 60.0:  # 60 second timeout
    elapsed = time.time() - rotateStartTime
    
    # Send slow turn command toward target
    angleDiff = targetAngle - minFrontAngle
    # Normalize to -180 to 180
    if angleDiff > 180:
        angleDiff -= 360
    elif angleDiff < -180:
        angleDiff += 360
    
    # Turn in the right direction
    if angleDiff > 2.0:  # 2 degree tolerance
        cmd = b'<1,40,-40>'  # Turn left
        direction = "LEFT"
    elif angleDiff < -2.0:
        cmd = b'<1,-40,40>'  # Turn right
        direction = "RIGHT"
    else:
        cmd = b'<1,0,0>'  # Stop
        direction = "STOP - REACHED"
        reachedTarget = True
    
    ser.write(cmd)
    ser.flush()
    
    time.sleep(0.1)
    
    while ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        
        if line.startswith('<') and line.endswith('>'):
            try:
                parts = line.strip('<>').split(',')
                theta = float(parts[0])
                
                minFrontAngle = theta  # Update current angle
                newDiff = targetAngle - minFrontAngle
                if newDiff > 180:
                    newDiff -= 360
                elif newDiff < -180:
                    newDiff += 360
                
                print(f"{elapsed:8.1f} | {theta:8.1f} | {newDiff:12.2f} | {direction}")
                
                if abs(newDiff) <= 2.0:
                    reachedTarget = True
                
            except:
                pass
    
    time.sleep(0.9)

# Stop
ser.write(b'<1,0,0>')
time.sleep(0.5)

print("\n" + "=" * 60)
print("MISSION COMPLETE")
print("=" * 60)
print(f"\nClosest wall was at: {minFrontAngle:.2f}°")
print(f"Robot is now facing: {minFrontAngle + 90:.2f}°")
print(f"(90° to the right from closest wall)")

ser.close()
GPIO.cleanup()
print("\nDone!")
