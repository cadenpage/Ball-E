#include <QTRSensors.h>
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t linePosition;

// FRONT Sensor
#define TRIG_FRONT 14
#define ECHO_FRONT 17

// LEFT Sensor
#define TRIG_LEFT 12
#define ECHO_LEFT 1    // TX pin (OK for now)

// RIGHT Sensor
#define TRIG_RIGHT 0   // RX pin (OK for now)
#define ECHO_RIGHT 19


const byte numChars = 32;
char receivedChars[numChars];
char tempChar[numChars]; // temporary array used for parsing

//motor command variables
int leftMotor; //int leftMotor
int rightMotor;
int currentState = 1; // 1: initialization with US, 2: line following, 3: IR only
int isCross=0;

// State 1 initialization variables
boolean state1Initialized = false;
boolean isSpinning = false;  // Currently spinning
unsigned long spinStartTime = 0;
const int FULL_SPIN_TIME = 4000;  // Time for full 360 degree spin in ms

// ===== GEOMETRY AND ENCODER CONSTANTS (from distanceCalc.ino) =====
#define PI 3.141592653589

// Encoder configuration
const int encoderRightPinA = 15;
const int encoderRightPinB = 16;
const int encoderLeftPinA = 8;
const int encoderLeftPinB = 11;

// Encoder and wheel parameters
int encoderResolution = 1440;  // counts per revolution
double wheelDiameter = 2.7559055;  // wheel diameter in inches
double wheelCircumference = PI * wheelDiameter;  // circumference in inches

// Robot geometry for position tracking
double robotTrackWidth = 5.0;  // distance between left and right wheels in inches (ADJUST THIS VALUE)

// Position tracking variables
double xPosition = 0.0;  // x position in inches
double yPosition = 0.0;  // y position in inches
double robotAngle = 0.0;  // robot heading angle in radians (0 = facing forward)
int prevLeftEncoderCount = 0;
int prevRightEncoderCount = 0;

boolean newData = false;

//=====================================================

void setup() {
   pinMode(3, OUTPUT); //left motor
   pinMode(2,OUTPUT); //left motor
    Serial.begin(115200);
    // qtr.setTypeRC(); //this allows us to read the line sensor from didgital pins


    // //arduino pin sensornames I am using: 7, 18, 19, 20, 21, 22, 23, 6. UNHOOK THE BLUE JUMPER LABELED BUZZER ON THE ASTAR or pin 6 will cause the buzzer to activate.
    // qtr.setSensorPins((const uint8_t[]){7, 18, 19, 20, 21, 22, 23, 6}, SensorCount);

    // calibrateSensors();
      // Set sensor pins
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  
    Serial.println("<Arduino is ready>");
    delay(500);
}

//====================================================

void loop() {

    recvWithStartEndMarkers(); //this function is in charge of taking a peice of data that looks like <1,200,150> 
                               //turning it into a string looking like 1,200,150 and then setting newdata to true
                 
    if (newData == true) {
      strcpy(tempChar, receivedChars); //copy recievedChars for parsing
      parseData(); //parses state, leftMotor, rightMotor
      newData = false;
    }
    
    // ===== POSITION TRACKING FROM ENCODERS =====
    // Read encoder counts and update position
    // NOTE: You'll need to add encoder library includes and encoder object declarations
    // int leftEncoderCount = encoderLeft.read();
    // int rightEncoderCount = encoderRight.read();
    // updatePositionFromEncoders(leftEncoderCount, rightEncoderCount);
    
    // ===== SEND DATA TO RPI =====
    sendDataToRpi(); //sends sensor data and position to Python
                   
    // ===== CONTROL MOTORS =====
    commandMotors(); //applies leftMotor and rightMotor commands
}


//======================================================
float readUltrasonic(int trigPin, int echoPin) {
  // trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(4);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // wait for echo, 30 ms timeout
  long duration = pulseIn(echoPin, HIGH, 30000);  

  if (duration == 0) return -1;  // No echo detected

  return (duration * 0.0343f) / 2.0f;
}
//======================================================

void updatePositionFromEncoders(int leftEncoderCount, int rightEncoderCount) {
  // Calculate distance traveled by each wheel since last update
  int deltaLeftEncoder = leftEncoderCount - prevLeftEncoderCount;
  int deltaRightEncoder = rightEncoderCount - prevRightEncoderCount;
  
  // Convert encoder counts to distance traveled in inches
  double leftDistance = (deltaLeftEncoder / (double)encoderResolution) * wheelCircumference;
  double rightDistance = (deltaRightEncoder / (double)encoderResolution) * wheelCircumference;
  
  // Average distance traveled (for forward motion)
  double avgDistance = (leftDistance + rightDistance) / 2.0;
  
  // Difference in distances (for rotation)
  double deltaDistance = rightDistance - leftDistance;
  
  // Update robot angle (rotate based on difference between wheels)
  // Robot rotates around its center, so each wheel contributes half the rotation
  robotAngle += deltaDistance / robotTrackWidth;
  
  // Update position based on current angle and average distance
  xPosition += avgDistance * cos(robotAngle);
  yPosition += avgDistance * sin(robotAngle);
  
  // Remember encoder counts for next update
  prevLeftEncoderCount = leftEncoderCount;
  prevRightEncoderCount = rightEncoderCount;
}

void resetPosition() {
  // Reset position to origin
  xPosition = 0.0;
  yPosition = 0.0;
  robotAngle = 0.0;
  prevLeftEncoderCount = 0;
  prevRightEncoderCount = 0;
}

//======================================================

void parseData(){


  char *strtokIndexer; //doing char * allows strtok to increment across my string properly frankly im not sure why... something to do with pointers that I dont expect students to understand

  
  strtokIndexer = strtok(tempChar,","); //sets strtokIndexer = to everything up to the first comma in tempChar /0 //this line is broken
  currentState = atoi(strtokIndexer); //converts strtokIndexer into a int for state
  

  strtokIndexer= strtok(NULL, ","); //setting the first input to null causes strtok to continue looking for commas in tempChar starting from where it left off, im not really sure why 
  leftMotor = atoi(strtokIndexer);

  strtokIndexer= strtok(NULL, ",");
  rightMotor = atoi(strtokIndexer);

  
  //now that we have extracted the data from the Rpi as floats, we can use them to command actuators somewhere else in the code
  
}

//==========================================

void state1Spin() {
  // Perform full 360 degree spin while sending US sensor values to Python
  
  if (!isSpinning) {
    // Start spin
    isSpinning = true;
    spinStartTime = millis();
    Serial.println("SPIN_START");
  }
  
  unsigned long elapsedTime = millis() - spinStartTime;
  
  if (elapsedTime < FULL_SPIN_TIME) {
    // Calculate current angle (0-360 degrees)
    int spinAngle = (elapsedTime * 360) / FULL_SPIN_TIME;
    
    // Spin counterclockwise - left motor forward, right motor backward
    analogWrite(3, 200);   // Left motor
    analogWrite(2, 50);    // Right motor (slower to create turning motion)
    
    // Read sensors
    float frontDist = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
    float leftDist = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
    float rightDist = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
    
    // Send data to Python: SPIN,angle,frontDist,leftDist,rightDist
    Serial.print("SPIN,");
    Serial.print(spinAngle);
    Serial.print(',');
    Serial.print(frontDist);
    Serial.print(',');
    Serial.print(leftDist);
    Serial.print(',');
    Serial.println(rightDist);
    
  } else {
    // Spin complete
    isSpinning = false;
    analogWrite(3, 0);
    analogWrite(2, 0);
    delay(500);
    Serial.println("SPIN_END");
  }
}

//==========================================

void sendDataToRpi() {

  // STATE 1: Initialization with US sensors - spin and let Python analyze
  if (currentState == 1 && !state1Initialized) {
    state1Spin();  // Just spin and send raw data
  } 
  // STATE 1: After initialization complete, send sensor values + position
  else if (currentState == 1 && state1Initialized) {
    float frontDist = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
    float leftDist  = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
    float rightDist = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

    Serial.print("STATE1,");
    if (frontDist < 0) Serial.print("No Echo");
    else Serial.print(frontDist);
    Serial.print(',');

    if (leftDist < 0) Serial.print("No Echo");
    else Serial.print(leftDist);

    Serial.print(',');
    if (rightDist < 0) Serial.print("No Echo");
    else Serial.print(rightDist);
    
    // Also send position data
    Serial.print(',');
    Serial.print(xPosition);
    Serial.print(',');
    Serial.print(yPosition);
    Serial.print(',');
    Serial.println(robotAngle);
  }
  // STATE 2: Line following to second cross
  else if (currentState == 2) {
    linePosition = qtr.readLineBlack(sensorValues);
    
    // Check for cross
    if(sensorValues[7] > 750 && sensorValues[0] > 750) {
      isCross = 1;
    } else {
      isCross = 0;
    }
    
    Serial.print("STATE2,");
    Serial.print(linePosition);
    Serial.print(',');
    Serial.print(isCross);
    
    // Also send position data
    Serial.print(',');
    Serial.print(xPosition);
    Serial.print(',');
    Serial.print(yPosition);
    Serial.print(',');
    Serial.println(robotAngle);
  }
  // STATE 3: IR sensors only (no line or US sensor reading)
  else if (currentState == 3) {
    // Only IR sensors are read on Rpi side, Arduino sends minimal data
    Serial.println("3");
  }

  delay(250);
  newData = false;

}

//=======================================

void commandMotors(){
  analogWrite(3, leftMotor);   // Left motor
  analogWrite(2, rightMotor);  // Right motor
}


//=========================================================


void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
                                                               
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
                                                             
                                                                  
        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminates the string, frankly unsure why I need this
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//==================================================================

void calibrateSensors(){

  //THE SENSORS ONLY CALIBRATE WHEN YOU UPLOAD NEW ARDUINO CODE TO THE ASTAR. after that the sensors STAY calibrated as long as the Astar has power.

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
                                   ///while calibrating, move the sensor over the line a couple times

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 200 times to make calibration take about 5 seconds.
  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  
}
