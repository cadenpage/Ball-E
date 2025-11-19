#include <QTRSensors.h>
#include <Encoder.h>
#include <AStar32U4Motors.h>

QTRSensors qtr;
AStar32U4Motors m;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t linePosition;

// FRONT Sensor
#define TRIG_FRONT 14
#define ECHO_FRONT 17

// LEFT Sensor
#define TRIG_LEFT 12
#define ECHO_LEFT 1    // TX pin (OK for now)




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
const int encoderLeftPinA = 8;
const int encoderLeftPinB = 11;
const int encoderRightPinA = 15;
const int encoderRightPinB = 16;

// Create encoder objects
Encoder encoderLeft(encoderLeftPinA, encoderLeftPinB);
Encoder encoderRight(encoderRightPinA, encoderRightPinB);

// Encoder and wheel parameters
int encoderResolution = 1440;  // counts per revolution
double wheelDiameter = 2.7559055;  // wheel diameter in inches
double wheelCircumference = PI * wheelDiameter;  // circumference in inches

// Robot geometry for position tracking
double robotTrackWidth = 5.0;  // distance between left and right wheels in inches (ADJUST THIS VALUE)

// Position tracking variables
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

void parseData(){
  char *strtokIndexer;
  
  strtokIndexer = strtok(tempChar,",");
  currentState = atoi(strtokIndexer);
  
  strtokIndexer= strtok(NULL, ",");
  leftMotor = atoi(strtokIndexer);

  strtokIndexer= strtok(NULL, ",");
  rightMotor = atoi(strtokIndexer);

  // Debug output to verify parsing
  Serial.print("PARSED: state=");
  Serial.print(currentState);
  Serial.print(" leftMotor=");
  Serial.print(leftMotor);
  Serial.print(" rightMotor=");
  Serial.println(rightMotor);
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

    // Send data to Python: SPIN,angle,frontDist,leftDist
    Serial.print("SPIN,");
    Serial.print(spinAngle);
    Serial.print(',');
    Serial.print(frontDist);
    Serial.print(',');
    Serial.println(leftDist);
    
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
  // Send raw encoder counts and US sensor readings
  // Format: <encoder_left,encoder_right,front_US,left_US>
  // Python handles all state management and calculations (angle, position, etc)
  
  // Read both encoder counts
  int leftEncoderCount = encoderLeft.read();
  int rightEncoderCount = encoderRight.read();
  
  // Read US sensors
  float frontDist = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  float leftDist = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
  
  // Send data to Python
  Serial.print('<');
  Serial.print(leftEncoderCount);
  Serial.print(',');
  Serial.print(rightEncoderCount);
  Serial.print(',');
  Serial.print(frontDist);
  Serial.print(',');
  Serial.print(leftDist);
  Serial.println('>');
  
  delay(50);  // ~20Hz update rate
  newData = false;

}

//=======================================

void commandMotors(){
  // Command motors using AStar32U4Motors library
  // Range: -400 to 400
  // M1 = right motor, M2 = left motor
  m.setM1Speed(rightMotor);
  m.setM2Speed(leftMotor);
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
