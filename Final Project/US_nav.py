#!/usr/bin/env python3
import serial
import time
import numpy as np
from sendStringScript import sendString
import RPi.GPIO as GPIO
import asyncio
import json
from datetime import datetime

sensor = 14

GPIO.setmode(GPIO.BCM)
GPIO.setup(sensor,GPIO.IN)


if __name__ == '__main__':

    async def IRcheck():
        await asyncio.sleep(1)
        irValue = GPIO.input(sensor)
        if irValue:
            print('NOT DETECTED')
                    
        else:
            print('DETECTED')

    async def main():
        print('IR Sensor Ready')

        try:
            while True:
                await IRcheck()
        except KeyboardInterrupt:
            GPIO.cleanup()

    asyncio.run(main())
    ser=serial.Serial('/dev/ttyACM2',115200) #sometimes is /dev/ttyACM0' or ttyACM02 unsure what deliniates
    ser.reset_input_buffer() #clears anything the arduino has been sending while the Rpi isnt prepared to recieve.

    # Initialize variables
    crossCount = 0
    isCross = 0
    leftMotor = 0
    rightMotor = 0
    currentState = 1  # 1: initialization, 2: line following, 3: IR only
    
    # Position tracking from encoders
    xPosition = 0.0
    yPosition = 0.0
    robotAngle = 0.0
    
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

    while True:
        sendString('/dev/ttyACM2',115200,'<'+str(currentState)+','+str(leftMotor)+','+str(rightMotor)+'>',0.0001)
        #ser.write(('<'+str(leftMotor)+str(rightMotor)+'>').encode())
        if ser.in_waiting > 0:  #we wait until the arduino has sent something to us before we try to read anything from the serial port.
                 
            #first we read the data from the arduino using the pySerial library
            recievedData = ser.readline().decode('utf-8').strip()
            ser.reset_input_buffer() #clears anything the arduino has been sending while the Rpi isnt prepared to recieve.

            # ===== STATE 1 SPIN PHASE =====
            if currentState == 1 and not state1Complete:
                if recievedData == "SPIN_START":
                    spinPhase = "spinning"
                    spinData = []
                    minFrontDist = float('inf')
                    print("Starting spin phase...")
                    continue
                    
                elif recievedData == "SPIN_END":
                    spinPhase = "analyzing"
                    print("Spin complete! Analyzing data...")
                    
                    # Find minimum front distance and corresponding angles
                    for data in spinData:
                        if data['frontDist'] > 0 and data['frontDist'] < minFrontDist:
                            minFrontDist = data['frontDist']
                            minFrontAngle = data['angle']
                            leftDistAtMin = data['leftDist']
                            rightDistAtMin = data['rightDist']
                    
                    print(f"Found minimum front distance: {minFrontDist:.1f}cm at angle {minFrontAngle}°")
                    print(f"  Left distance at min: {leftDistAtMin:.1f}cm")
                    print(f"  Right distance at min: {rightDistAtMin:.1f}cm")
                    
                    # Determine which side is closer
                    if leftDistAtMin > rightDistAtMin:
                        print(f"Left side is more open (left={leftDistAtMin:.1f}cm > right={rightDistAtMin:.1f}cm), will turn LEFT")
                    else:
                        print(f"Right side is more open or equal (right={rightDistAtMin:.1f}cm >= left={leftDistAtMin:.1f}cm), will turn RIGHT")
                    
                    # Save spin data to file
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    filename = f"spin_data_{timestamp}.json"
                    with open(filename, 'w') as f:
                        json.dump({
                            'spin_data': spinData,
                            'minFrontDist': minFrontDist,
                            'minFrontAngle': minFrontAngle,
                            'leftDistAtMin': leftDistAtMin,
                            'rightDistAtMin': rightDistAtMin,
                            'timestamp': timestamp
                        }, f, indent=2)
                    print(f"Spin data saved to {filename}")
                    
                    navigationPhase = 0
                    navigationStartTime = time.time()
                    continue
                    
                elif recievedData.startswith("SPIN,"):
                    # Parse spin data: SPIN,angle,frontDist,leftDist,rightDist
                    spinPhase = "spinning"
                    try:
                        parts = recievedData.split(',')
                        if len(parts) >= 5:
                            angle = float(parts[1])
                            frontDist = float(parts[2]) if parts[2] != "-1" else -1
                            leftDist = float(parts[3]) if parts[3] != "-1" else -1
                            rightDist = float(parts[4]) if parts[4] != "-1" else -1
                            
                            spinData.append({
                                'angle': angle,
                                'frontDist': frontDist,
                                'leftDist': leftDist,
                                'rightDist': rightDist
                            })
                            print(f"Spin: angle={angle:.0f}°, front={frontDist:.1f}cm, left={leftDist:.1f}cm, right={rightDist:.1f}cm")
                    except ValueError:
                        pass
                    continue
                
                # ===== STATE 1 NAVIGATION PHASE =====
                elif spinPhase == "analyzing":
                    # Python controls navigation, Arduino just provides sensor data
                    elapsedNav = time.time() - navigationStartTime
                    
                    if navigationPhase == 0:
                        print("Phase 0: Returning to minimum front distance angle...")
                        # Calculate how long it takes to rotate back to minFrontAngle
                        # Assuming 1 second for ~90 degree turn, scale accordingly
                        rotateTimeNeeded = (minFrontAngle / 90.0) * 1.0
                        
                        if elapsedNav < rotateTimeNeeded:
                            # Rotate back
                            leftMotor = 50
                            rightMotor = 200
                        else:
                            leftMotor = 0
                            rightMotor = 0
                            navigationPhase = 1
                            navigationStartTime = time.time()
                            print("Phase 1: Turning 90 degrees toward center...")
                        continue
                    
                    elif navigationPhase == 1:
                        # Turn 90 degrees based on which side is more open
                        if elapsedNav < 1.0:  # 1 second turn
                            if leftDistAtMin > rightDistAtMin:
                                # Turn left
                                leftMotor = 200
                                rightMotor = 0
                            else:
                                # Turn right
                                leftMotor = 0
                                rightMotor = 200
                        else:
                            leftMotor = 0
                            rightMotor = 0
                            navigationPhase = 2
                            navigationStartTime = time.time()
                            print("Phase 2: Advancing to center (front distance = 50cm)...")
                        continue
                    
                    elif navigationPhase == 2:
                        # Advance forward until front distance ~= 50cm
                        # Use current sensor readings from Arduino
                        leftMotor = 200
                        rightMotor = 200
                        # Python will update this based on actual sensor feedback in State 2
                        # For now, assume it takes ~2 seconds
                        if elapsedNav > 2.0:
                            leftMotor = 0
                            rightMotor = 0
                            state1Complete = True
                            print("State 1 navigation complete!")
                        continue

            #this below variable keeps track of what isCross was in the previos iteration of the whileloop, will be used later
            isCrossLastTimestep=isCross

            #now we split that data up into a list, using commas to deliniate between elements
            try:
                recievedData_parts=recievedData.split(',')
                
                # Parse based on state indicator
                if recievedData_parts[0] == "STATE1":
                    # STATE1,frontDist,leftDist,rightDist,xPos,yPos,angle
                    frontDist = float(recievedData_parts[1]) if recievedData_parts[1] != "No Echo" else -1
                    leftDist = float(recievedData_parts[2]) if recievedData_parts[2] != "No Echo" else -1
                    rightDist = float(recievedData_parts[3]) if recievedData_parts[3] != "No Echo" else -1
                    xPosition = float(recievedData_parts[4])
                    yPosition = float(recievedData_parts[5])
                    robotAngle = float(recievedData_parts[6])
                    
                elif recievedData_parts[0] == "STATE2":
                    # STATE2,linePose,isCross,xPos,yPos,angle
                    linePose = int(recievedData_parts[1])
                    isCross = int(recievedData_parts[2])
                    xPosition = float(recievedData_parts[3])
                    yPosition = float(recievedData_parts[4])
                    robotAngle = float(recievedData_parts[5])
                else:
                    # Fallback for old format (shouldn't happen with new code)
                    linePose = int(recievedData_parts[0])
                    isCross = int(recievedData_parts[1])
                    leftMotorrcvd=recievedData_parts[2] if len(recievedData_parts) > 2 else 0
                    rightMotorrcvd=recievedData_parts[3] if len(recievedData_parts) > 3 else 0
            except:
                print(f"packet lost or parsing error: {recievedData}")
                continue

            #incrimenting seeing a cross: we only want to count each cross once, so we will use crossCount changing from 0->1 as our inclination that we have hit a new cross
            #this will prevent us from counting the same cross twice
            if isCrossLastTimestep == 0 and isCross ==1:
                 #meaning we were NOT on a cross last timestep but ARE on a cross now. this makes sure we dont count the same cross more than once
                crossCount=crossCount+1
            
            
            #now that we store our position on the line as well as how many crosses we have hit, we can write a state machine, which can incriment based on crossCount.
            #Determine current state based on progress
            if crossCount < 2:
                currentState = 1  # Still in initialization/finding center
            elif crossCount < 4:
                currentState = 2  # Line following to second cross
            else:
                currentState = 3  # IR sensors only (reached goal)
            
            # Print position data for debugging
            print(f"Position: x={xPosition:.2f}in, y={yPosition:.2f}in, angle={robotAngle:.3f}rad")
            
            #STATE 1: WE HAVENT HIT THE SECOND CROSS YET, we translate forward using proportonal control
            if crossCount < 2:
                #rightMotor=int(((linePose-1000)/4000)*300 + 100) #maps leftmotor to be between 100 and 400, will be LOW if robot is to thr right of the line. keep these as ints, less bits being sent the better
                #leftMotor=int(400-(((linePose-1000)/4000)*300))
                rightMotor=int(np.interp(linePose,[1000,5000],[100,400])-40) #interpelation functions that map our linesenor values to motorcommand values, made convient by the numpy library
                leftMotor=int(np.interp(linePose,[1000,5000],[400,100])+40)

            if crossCount==2: #rotates the robot to the right until the line sensor lines up with the new line, which incriments crossCounter and moves me into my next state
                rightMotor=0
                leftMotor=0
                print("reached second cross")
                #time.sleep(500) #these pauses are for me to be able to clearly see when I change states
                rightMotor=-150
                leftMotor=150
            
            if crossCount ==3: #we should be facing the right side of the board now, and we want to translate forward until we hit the next cross
                #time.sleep(1)
                rightMotor=int(np.interp(linePose,[1000,5000],[100,400])-40)
                leftMotor=int(np.interp(linePose,[1000,5000],[400,100])+40)
                
            
            if crossCount ==4: #now we've hit the right front cross, lets rotate to the left until the line sensor alignes with the line paralell to the goal board and causing the crossCount to incriment, which will exit us out of this state and into the next one
                #time.sleep(1)
                rightMotor=150
                leftMotor=-150
            
            if crossCount > 4: #once we have alighned with the line paralell to the goal board, we stop.
                rightMotor=0
                leftMotor=0
            
            print('left:'+str(leftMotorrcvd)+'right:'+str(rightMotorrcvd))
            print(linePose)
            
            #     #Following is my control law, we're keeping it basic for now, writing good control law is your job
            #     #ok so high numbers(highest 7000) on the line follwing mean I am too far to the RIGHT,
            #     #low numbers mean I am too far on the RIGHT, 3500 means I am at the middle
            #     #below is a basic control law you can send to your motors, with an exeption if z is a value greater than 7000, meaning the arduino code sees that the line sensor is on a cross. Feel free to take insperation from this,
            # #but you will need to impliment a state machine similar to what you made in lab 2 (including a way of counting time without blocking)
            
            #     if x < 70001: #im assuming that in your arduino code you will be setting z to the int 8000 if you sense a cross, dont feel obligated to do it this way.  
            #         leftMotor=400-((x/7000)*300) #will be between 100 and 400, HIGh if z is low
            #         rightMotor=((x/7000)*300)+100 #will be between 100 and 400, LOW if z is low
            #     else:
            #         print('at intersetion')
            #         #do something here like incrimenting a value you call 'lines_hit' to one higher, and writing code to make sure that some time (1 second should do it) 
            #         # passes between being able to incriment lines_hit so that it wont be incrimented a bunch of times when you hit your first cross. IE give your robot time to leave a cross
            #         #before allowing lines_hit to be incrimented again.