# Startup Sequence - Ball-E Navigation System

## Initial Power-On (Arduino First)

```
1. Arduino boots up
   ↓
2. setup() runs:
   - Serial.begin(115200)
   - Configures ultrasonic sensor pins
   - Prints: "<Arduino is ready>"
   ↓
3. Arduino loop() starts continuously:
   - Waits for serial data from Raspberry Pi
   - Motors idle (0 speed) until commands arrive
   - Ready to receive: <state,leftMotor,rightMotor>
```

## Python Script Starts

```
1. Python connects to serial port /dev/ttyACM2
2. Initializes variables:
   - currentState = 1 (initialization mode)
   - leftMotor = 0
   - rightMotor = 0
   - Position tracking: x=0, y=0, angle=0
3. Starts infinite main loop
```

## Main Communication Loop

### **Python Side** (each iteration):
```
1. SEND to Arduino: <1,0,0>  (state=1, left=0, right=0)
2. WAIT for response
3. Parse received data
4. Print position and sensor data
5. Update motor commands based on state
6. Loop back to step 1
```

### **Arduino Side** (each iteration):
```
1. Check for serial data
2. If data received:
   - Parse: state, leftMotor, rightMotor
   - Set newData = false
3. Read encoders
4. Update position from encoder counts
5. Send sensor data + position to Python
   - Format: STATE1,front,left,right,x,y,angle
6. Apply motor commands via analogWrite()
7. Loop back to step 1
```

## State 1: Initialization (Finding Map Center)

### **Arduino Behavior:**
- When `currentState == 1 && !state1Initialized`:
  - Calls `state1Spin()`
  - Rotates robot full 360° while reading ultrasonic sensors
  - Sends: `SPIN_START` → spin data → `SPIN_END`
  - Motors controlled internally: left=200, right=50

### **Python Behavior:**
- Receives `SPIN_START`
- Captures spin data array with 360 readings
- Receives `SPIN_END`
- Analyzes data to find:
  - **Minimum front distance** (closest wall)
  - **Angle at minimum**
  - **Left vs right distances** at that angle
- Saves data to `spin_data_YYYYMMDD_HHMMSS.json`
- Enters navigation phase (3 phases):
  - **Phase 0**: Rotate back to minimum angle
  - **Phase 1**: Turn 90° toward open side
  - **Phase 2**: Move forward until centered
- Sets `state1Initialized = true`

### **Console Output:**
```
Starting spin phase...
Spin: angle=0°, front=45.2cm, left=30.1cm, right=48.5cm
Spin: angle=10°, front=42.8cm, left=28.9cm, right=50.2cm
...
Spin complete! Analyzing data...
Found minimum front distance: 22.5cm at angle 45°
  Left distance at min: 35.2cm
  Right distance at min: 18.9cm
Right side is more open or equal (right=18.9cm >= left=35.2cm), will turn RIGHT
Spin data saved to spin_data_20251118_143025.json
Phase 0: Returning to minimum front distance angle...
Phase 1: Turning 90 degrees toward center...
Phase 2: Advancing to center (front distance = 50cm)...
State 1 navigation complete!
Position: x=5.23in, y=-2.41in, angle=1.571rad
```

## State 2: Line Following

### **Python Behavior:**
- When `crossCount < 2` and `state1Initialized`:
  - Sets `currentState = 2`
  - Commands motors for proportional line following
  - Waits for cross detection
  
- When first cross detected:
  - `crossCount` increments to 1
  - Continues line following
  
- When second cross detected:
  - `crossCount` increments to 2
  - Robot rotates to align with next line
  - `crossCount` increments to 3
  - Continues to next cross
  
- Until `crossCount >= 4`:
  - Robot has navigated the field
  - Ready for State 3 (IR sensor only)

### **Console Output:**
```
Position: x=15.23in, y=8.42in, angle=0.156rad
Position: x=20.15in, y=9.87in, angle=0.142rad
reached second cross
Position: x=25.33in, y=11.20in, angle=1.571rad
```

## State 3: IR Sensors Only

- Robot has reached goal area
- Stops reading line sensors and ultrasonic sensors
- Only IR sensors active (for obstacle detection)
- Minimal data sent to Python

## Key Data Flows

### Arduino Sends to Python:
```
SPIN_START                                    (State 1 spin beginning)
SPIN,angle,front,left,right                   (State 1 spin data)
SPIN_END                                      (State 1 spin complete)
STATE1,front,left,right,x,y,angle             (State 1 after init)
STATE2,linePos,isCross,x,y,angle              (State 2 line following)
```

### Python Sends to Arduino:
```
<state,leftMotor,rightMotor>                  (Every 0.0001 seconds)
```

## Startup Checklist

- [ ] Arduino connected via USB (should see `/dev/ttyACM2` or similar)
- [ ] Python script has correct serial port
- [ ] Motors respond to `analogWrite()` commands
- [ ] Encoders produce tick counts
- [ ] Ultrasonic sensors return valid distances
- [ ] Robot has power and is in safe testing area
- [ ] Monitor serial console for "Arduino is ready" message

## Troubleshooting

| Issue | Check |
|-------|-------|
| No serial connection | Is Arduino plugged in? Run: `ls /dev/ttyACM*` |
| Motors don't spin | Check pin assignments (3, 2) - are they correct? |
| No sensor data | Check ultrasonic pins are correct |
| Position stuck at 0,0 | Check encoders are connected and reading |
| Spin never completes | Check if stuck in loop - may need timeout |
| Python crashes | Check serial port availability |

