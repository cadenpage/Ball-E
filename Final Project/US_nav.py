#!/usr/bin/env python3
import serial
import time
import numpy as np
from sendStringScript import sendString
import RPi.GPIO as GPIO

portname = '/dev/ttyACM0'
# IR Sensor Setup
sensor = 14
GPIO.setmode(GPIO.BCM)
GPIO.setup(sensor, GPIO.IN)

# Main program starts here
ser = serial.Serial(portname, 115200)
ser.reset_input_buffer()

# Debug mode selection
print("=" * 50)
print("Ball-E Control System")
print("=" * 50)
print("\nSelect mode:")
print("1. Manual motor debug (test motor speeds)")
print("2. Auto State 1 spin (autonomous)")
choice = input("Enter choice (1 or 2): ").strip()

if choice == "1":
    print("\n=== MANUAL MOTOR DEBUG MODE ===")
    print("You can now manually set motor speeds")
    print("Range: -400 to 400")
    print("Type 'exit' to quit\n")
    
    while True:
        try:
            left_input = input("Enter left motor speed (-400 to 400) or 'exit': ").strip()
            if left_input.lower() == 'exit':
                break
            
            right_input = input("Enter right motor speed (-400 to 400): ").strip()
            
            leftMotor = int(left_input)
            rightMotor = int(right_input)
            
            # Clamp values
            leftMotor = int(np.clip(leftMotor, -400, 400))
            rightMotor = int(np.clip(rightMotor, -400, 400))
            
            # Send to Arduino using persistent serial connection
            command = '<0,' + str(leftMotor) + ',' + str(rightMotor) + '>'
            print(f"Sending: {command}")
            ser.write(command.encode())
            time.sleep(0.001)
            
            # Read response
            if ser.in_waiting > 0:
                response = ser.readline().decode('utf-8').strip()
                print(f"Arduino: {response}\n")
            
        except ValueError:
            print("Invalid input! Use numbers between -400 and 400\n")
    
    print("Exiting debug mode...")
    ser.close()
    GPIO.cleanup()
    exit()

else:
    print("\n=== AUTO MODE: State 1 Spin ===\n")

# Initialize variables
crossCount = 0
isCross = 0
leftMotor = 0
rightMotor = 0
currentState = 1  # 1: initialization, 2: line following, 3: IR aiming/shooting

# Position tracking from encoders
xPosition = 0.0
yPosition = 0.0
robotAngle = 0.0

# Encoder tracking
prevLeftEncoderCount = 0
prevRightEncoderCount = 0
encoderResolution = 1440  # counts per revolution
wheelDiameter = 2.7559055  # inches
wheelCircumference = np.pi * wheelDiameter
robotTrackWidth = 5.0  # distance between wheels in inches

# State 1 spin data logging
spinData = []
spinPhase = "waiting"  # waiting, spinning, analyzing, navigating
state1Complete = False
minFrontDist = float('inf')
minFrontAngle = 0
leftDistAtMin = 0
rightDistAtMin = 0
navigationPhase = 0  # 0: return to min angle, 1: turn 90, 2: advance to center
navigationStartTime = time.time()

# Current sensor readings
frontDist = -1
leftDist = -1

# Motor command constraints
motorMin = -400
motorMax = 400

# State 3 IR control
irDetected = False
lastIRCheckTime = 0
irCheckInterval = 0.1  # Check IR every 100ms

print('IR Sensor Ready')
print('Starting main control loop...')
print('State 1: Spin to find closest wall')

# Initialize State 1 - start spinning immediately
spinStartTime = time.time()
state1SpinStarted = False

while True:
    # ===== STATE 1: SPINNING =====
    if currentState == 1 and not state1Complete:
        # Start spin if not already spinning
        if not state1SpinStarted:
            state1SpinStarted = True
            spinStartTime = time.time()
            print("Starting spin phase - commanding motors to spin...")
        
        # Keep spinning until timeout or completion signal
        elapsedSpinTime = time.time() - spinStartTime
        if elapsedSpinTime < 4.0:  # Spin for 4 seconds (360 degrees)
            leftMotor = 150
            rightMotor = -150
        else:
            # Spin complete, stop motors
            leftMotor = 0
            rightMotor = 0
            state1Complete = True
            print("Spin phase complete")
    
    # Check IR sensor for State 3
    if currentState == 3:
        currentTime = time.time()
        if currentTime - lastIRCheckTime >= irCheckInterval:
            irValue = GPIO.input(sensor)
            if irValue:
                print('IR: NOT DETECTED')
                irDetected = False
            else:
                print('IR: DETECTED')
                irDetected = True
            lastIRCheckTime = currentTime
    
    # Constrain motor commands to -400 to 400 range
    leftMotor_cmd = int(np.clip(leftMotor, motorMin, motorMax))
    rightMotor_cmd = int(np.clip(rightMotor, motorMin, motorMax))
    
    # Send command to Arduino: <state,left_motor,right_motor>
    command = '<' + str(currentState) + ',' + str(leftMotor_cmd) + ',' + str(rightMotor_cmd) + '>'
    print(f"Sending to Arduino: {command}")
    ser.write(command.encode())
    time.sleep(0.001)
    
    if ser.in_waiting > 0:
        # Read data from Arduino
        recievedData = ser.readline().decode('utf-8').strip()
        ser.reset_input_buffer()

        # Parse encoder and US sensor data from Arduino
        try:
            recievedData_clean = recievedData.strip('<>')
            recievedData_parts = recievedData_clean.split(',')
            
            if len(recievedData_parts) >= 4:
                leftEncoderCount = int(recievedData_parts[0])
                rightEncoderCount = int(recievedData_parts[1])
                frontDist = float(recievedData_parts[2])
                leftDist = float(recievedData_parts[3])
                
                # During State 1 spin, log the sensor readings
                if currentState == 1 and not state1Complete:
                    spinData.append({
                        'frontDist': frontDist,
                        'leftDist': leftDist
                    })
                    print(f"Spin: front={frontDist:.1f}cm, left={leftDist:.1f}cm")
                    
                    # Find minimum front distance during spin
                    if frontDist > 0 and frontDist < minFrontDist:
                        minFrontDist = frontDist
                
                # Update position from encoder counts
                deltaLeftEncoder = leftEncoderCount - prevLeftEncoderCount
                deltaRightEncoder = rightEncoderCount - prevRightEncoderCount
                
                # Convert encoder counts to distance traveled in inches
                leftDistance = (deltaLeftEncoder / encoderResolution) * wheelCircumference
                rightDistance = (deltaRightEncoder / encoderResolution) * wheelCircumference
                
                # Average distance traveled (for forward motion)
                avgDistance = (leftDistance + rightDistance) / 2.0
                
                # Difference in distances (for rotation)
                deltaDistance = rightDistance - leftDistance
                
                # Update robot angle (theta)
                robotAngle += deltaDistance / robotTrackWidth
                
                # Update position based on current angle and average distance
                xPosition += avgDistance * np.cos(robotAngle)
                yPosition += avgDistance * np.sin(robotAngle)
                
                # Remember encoder counts for next update
                prevLeftEncoderCount = leftEncoderCount
                prevRightEncoderCount = rightEncoderCount
                
                # Print position and sensor data for debugging
                print(f"Position: x={xPosition:.2f}in, y={yPosition:.2f}in, angle={robotAngle:.3f}rad")
                print(f"Front: {frontDist:.1f}cm, Left: {leftDist:.1f}cm")
        except:
            print(f"packet lost or parsing error: {recievedData}")
            continue

try:
    GPIO.cleanup()
except:
    pass
