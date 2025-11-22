#include <Arduino.h>
#include <QTRSensors.h>
#include <Encoder.h>
#include <AStar32U4Motors.h>
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"

QTRSensors qtr;
AStar32U4Motors m;
MPU6050 mpu;

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
int leftMotor = 0;
int rightMotor = 0;
int currentState = 1; // 1: initialization with US, 2: line following, 3: IR only
int isCross = 0;

// ===== IMU VARIABLES =====
bool dmpReady = false;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
float currentYaw = 0.0;  // Current yaw angle in degrees (0-360)

// Position tracking variables
int prevLeftEncoderCount = 0;
int prevRightEncoderCount = 0;

boolean newData = false;
const unsigned long TELEMETRY_INTERVAL_MS = 40; // ~25 Hz stream
unsigned long lastTelemetryMillis = 0;

//=====================================================

void setup() {
    pinMode(3, OUTPUT); //left motor
    pinMode(2, OUTPUT); //right motor
    Serial.begin(115200);
    delay(1000);
    
    // Initialize IMU
    Serial.println("Starting Wire...");
    Wire.begin();
    delay(500);

    Serial.println("Initializing MPU6050...");
    mpu.initialize();
    delay(500);

    if (!mpu.testConnection()) {
        Serial.println("ERROR: MPU6050 connection failed!");
        while (1) delay(1000);
    }
    Serial.println("MPU6050 connected!");

    // Set calibrated offsets
    mpu.setXAccelOffset(10123);
    mpu.setYAccelOffset(11011);
    mpu.setZAccelOffset(-6750);
    mpu.setXGyroOffset(187);
    mpu.setYGyroOffset(129);
    mpu.setZGyroOffset(-55);

    uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        Serial.println("DMP Ready!");
        mpu.setDMPEnabled(true);
        dmpReady = true;
    } else {
        Serial.print("ERROR: DMP init failed. Code: ");
        Serial.println(devStatus);
        while (1) delay(1000);
    }
    
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

    recvWithStartEndMarkers();
                 
    if (newData == true) {
      strcpy(tempChar, receivedChars);
      parseData();
      newData = false;
    }
    
    // Update IMU yaw
    updateIMU();
    
    // ===== CONTROL MOTORS =====
    commandMotors();
    
    // ===== CONTINUOUS TELEMETRY STREAM =====
    unsigned long now = millis();
    if (now - lastTelemetryMillis >= TELEMETRY_INTERVAL_MS) {
      sendDataToRpi();
      lastTelemetryMillis = now;
    }
}

//======================================================
void updateIMU() {
  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Convert yaw from radians to degrees (0-360)
    float yaw_deg = ypr[0] * 180.0f / M_PI;
    if (yaw_deg < 0) yaw_deg += 360.0f;
    
    currentYaw = yaw_deg;
  }
}

//======================================================
float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(4);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);  

  if (duration == 0) return -1;

  return (duration * 0.0343f) / 2.0f;
}

//======================================================

void parseData(){
  char *strtokIndexer;

  strtokIndexer = strtok(tempChar,",");
  currentState = atoi(strtokIndexer);
  
  strtokIndexer = strtok(NULL, ",");
  leftMotor = atoi(strtokIndexer);

  strtokIndexer = strtok(NULL, ",");
  rightMotor = atoi(strtokIndexer);

  // Clamp motor values to valid range
  leftMotor = constrain(leftMotor, -400, 400);
  rightMotor = constrain(rightMotor, -400, 400);
}

//==========================================

void sendDataToRpi() {
  // Send: <theta_yaw,front_US,left_US>
  // Python uses IMU theta and US data for state management
  
  float frontDist = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  float leftDist = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
  
  // Send data to Python
  Serial.print('<');
  Serial.print(currentYaw, 2);
  Serial.print(',');
  Serial.print(frontDist, 2);
  Serial.print(',');
  Serial.print(leftDist, 2);
  Serial.println('>');
}

//=======================================

void commandMotors(){
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
                receivedChars[ndx] = '\0';
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
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
}