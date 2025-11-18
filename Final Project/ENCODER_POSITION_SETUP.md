# Encoder Position Tracking Setup

## Overview
The US navigation code now uses encoder data from the drive wheels to calculate the robot's position (x, y coordinates and angle). This is pulled from the geometry and encoder constants in `distanceCalc.ino`.

## Hardware Constants Added to US_nav.ino

### Encoder Configuration
- **Left Encoder Pins**: A8, B11
- **Right Encoder Pins**: A15, B16
- **Encoder Resolution**: 1440 counts per revolution
- **Wheel Diameter**: 2.7559055 inches
- **Robot Track Width**: 5.0 inches (distance between wheels - ADJUST IF NEEDED)

### Key Definitions
```cpp
#define PI 3.141592653589
int encoderResolution = 1440;  // counts per rev
double wheelDiameter = 2.7559055;  // inches
double wheelCircumference = PI * wheelDiameter;
double robotTrackWidth = 5.0;  // ADJUST THIS VALUE
```

## Position Tracking Variables

The Arduino tracks:
- **xPosition**: X coordinate in inches (0 = starting position)
- **yPosition**: Y coordinate in inches (0 = starting position)
- **robotAngle**: Robot heading in radians (0 = facing forward, π/2 = rotated 90°)

## Functions Added

### `updatePositionFromEncoders(leftCount, rightCount)`
Called with current encoder counts to calculate:
1. Distance traveled by each wheel
2. Average forward distance (odometry)
3. Rotation angle from differential wheel speeds
4. Updates x, y position based on current angle

**Formula:**
- `avgDistance = (leftDistance + rightDistance) / 2`
- `deltaDistance = rightDistance - leftDistance`
- `robotAngle += deltaDistance / robotTrackWidth`
- `xPosition += avgDistance * cos(robotAngle)`
- `yPosition += avgDistance * sin(robotAngle)`

### `resetPosition()`
Resets all position tracking to origin (0, 0, 0 radians)

## Data Format Sent to Python

### State 1 (After Initialization)
```
STATE1,frontDist,leftDist,rightDist,xPosition,yPosition,robotAngle
```

### State 2 (Line Following)
```
STATE2,linePosition,isCross,xPosition,yPosition,robotAngle
```

## Python Side (US_nav.py)

Position data is automatically parsed and stored:
```python
xPosition = float(recievedData_parts[4])
yPosition = float(recievedData_parts[5])
robotAngle = float(recievedData_parts[6])
```

Printed for each sensor update:
```
Position: x=12.45in, y=8.32in, angle=0.785rad
```

## Important Notes

1. **Robot Track Width**: The value 5.0 inches is the distance between the left and right wheels. Measure your robot and update this if different.

2. **Angle Convention**: 
   - 0 radians = facing forward
   - π/2 radians = rotated 90° counterclockwise
   - Use `angle_in_degrees = robotAngle * 180 / PI`

3. **Units**: All distances are in inches, angles in radians

4. **Accumulation**: Position accumulates from encoder counts, so errors compound over time (typical odometry limitation)

## Tuning

If position tracking seems inaccurate:
1. Verify encoder pin assignments (check with oscilloscope)
2. Verify encoder resolution (1440 CPR for your motor encoder)
3. Verify wheel diameter (measure actual diameter, not nominal)
4. Verify robot track width (measure wheel-to-wheel distance)
5. Check for encoder slip (wheels slipping on floor)

## Example Usage in US_nav.py

```python
# Print position for debugging
print(f"Position: x={xPosition:.2f}in, y={yPosition:.2f}in, angle={robotAngle:.3f}rad")

# Use position for navigation decisions
distance_from_start = (xPosition**2 + yPosition**2) ** 0.5
print(f"Distance from start: {distance_from_start:.2f} inches")

# Check if at a specific location
if abs(xPosition - 50.0) < 2.0 and abs(yPosition - 0.0) < 2.0:
    print("Reached target position!")
```
