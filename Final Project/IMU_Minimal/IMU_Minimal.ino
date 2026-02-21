#include <Arduino.h>
#include <AStar32U4Motors.h>

#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"

// ================== CONFIG ==================
#define USE_IMU 1        // set to 0 to disable IMU completely

// Pins
#define TRIG_FRONT 14
#define ECHO_FRONT 17

#define IMU_INT_PIN 7    // MPU6050 INT wired here

// Telemetry
const unsigned long TELEMETRY_INTERVAL_MS = 40; // ~25 Hz
const unsigned long HEARTBEAT_INTERVAL_MS = 1000;

// Motor command range: -400..400 (AStar32U4Motors)
int leftMotor = 0;
int rightMotor = 0;

// ================== IMU / DMP ==================
MPU6050 mpu;

bool dmpReady = false;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
float currentYaw = 0.0f;  // 0–360 degrees
uint16_t packetSize = 0;

// Interrupt flag from MPU
volatile bool mpuInterrupt = false;

void dmpDataReadyISR() {
  mpuInterrupt = true;
}

// ================== SERIAL PARSING ==================
const byte numChars = 32;
char receivedChars[numChars];
char tempChar[numChars];
bool newData = false;

void recvWithStartEndMarkers();
void parseData();

// ================== OBJECTS ==================
AStar32U4Motors m;

// ================== FORWARD DECS ==================
void updateIMU();
float readUltrasonic(int trigPin, int echoPin);
void sendDataToRpi();
void commandMotors();

// ===================================================

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  delay(1000);

#if USE_IMU
  Serial.println(F("Starting Wire / IMU..."));
  Wire.begin();
  Wire.setClock(100000);   // keep it conservative for stability
  delay(200);

  Serial.println(F("Initializing MPU6050..."));
  mpu.initialize();
  delay(200);

  if (!mpu.testConnection()) {
    Serial.println(F("ERROR: MPU6050 connection failed! Running without IMU."));
    dmpReady = false;
  } else {
    // Your calibrated offsets
    mpu.setXAccelOffset(10123);
    mpu.setYAccelOffset(11011);
    mpu.setZAccelOffset(-6750);
    mpu.setXGyroOffset(187);
    mpu.setYGyroOffset(129);
    mpu.setZGyroOffset(-55);

    uint8_t devStatus = mpu.dmpInitialize();

    if (devStatus == 0) {
      Serial.println(F("DMP Ready, enabling..."));
      mpu.setDMPEnabled(true);

      // Attach interrupt for data-ready
      pinMode(IMU_INT_PIN, INPUT);
      attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), dmpDataReadyISR, RISING);

      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();

      Serial.print(F("DMP packet size: "));
      Serial.println(packetSize);
    } else {
      Serial.print(F("ERROR: DMP init failed. Code: "));
      Serial.println(devStatus);
      dmpReady = false;
    }
  }
#else
  dmpReady = false;
#endif

  // Ultrasonic
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  Serial.println(F("<Arduino is ready>"));
  delay(500);
}

// ===================================================

void loop() {
  static unsigned long lastHeartbeat = 0;
  static unsigned long lastTelemetryMillis = 0;
  unsigned long now = millis();

  // ----- Heartbeat -----
  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
    Serial.println(F("<HB>"));
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    lastHeartbeat = now;
  }

  // ----- Serial commands -----
  recvWithStartEndMarkers();
  if (newData) {
    strncpy(tempChar, receivedChars, numChars - 1);
    tempChar[numChars - 1] = '\0';
    parseData();
    Serial.print(F("<RC>"));
    Serial.print(tempChar);
    Serial.println(F(">"));
    newData = false;
  }

  // ----- IMU update -----
  updateIMU();

  // ----- Motor control -----
  commandMotors();

  // ----- Telemetry (yaw + front ultrasonic) -----
  if (now - lastTelemetryMillis >= TELEMETRY_INTERVAL_MS) {
    sendDataToRpi();
    lastTelemetryMillis = now;
  }
}

// ===================================================
// IMU: non-blocking DMP read using INT flag
void updateIMU() {
#if USE_IMU
  if (!dmpReady || packetSize == 0) return;

  // only do work when interrupt says data is ready
  if (!mpuInterrupt) return;
  mpuInterrupt = false;

  uint8_t mpuIntStatus = mpu.getIntStatus();
  uint16_t fifoCount = mpu.getFIFOCount();

  // overflow?
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    // Optional: debug
    // Serial.println(F("IMU FIFO overflow, resetting"));
    return;
  }

  // DMP data ready?
  if (!(mpuIntStatus & 0x02)) {
    return; // nothing to do
  }

  // if not enough bytes yet, bail; we'll catch next interrupt
  if (fifoCount < packetSize) {
    return;
  }

  // read exactly one packet. no while loops.
  mpu.getFIFOBytes(fifoBuffer, packetSize);

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  float yaw_deg = ypr[0] * 180.0f / M_PI;
  if (yaw_deg < 0) yaw_deg += 360.0f;

  currentYaw = yaw_deg;
#endif
}

// ===================================================
// Ultrasonic (front)
// Called only at telemetry rate to reduce blocking
float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Shorter timeout to reduce worst-case blocking
  unsigned long duration = pulseIn(echoPin, HIGH, 10000UL); // 10 ms
  if (duration == 0) return -1.0f;
  return (duration * 0.0343f) / 2.0f;
}

// ===================================================

void sendDataToRpi() {
  float frontDist = readUltrasonic(TRIG_FRONT, ECHO_FRONT);

  Serial.print('<');
  Serial.print(currentYaw, 2);
  Serial.print(',');
  Serial.print(frontDist, 2);
  Serial.println('>');
}

// ===================================================

void commandMotors() {
  // Your standard polarity; adjust if needed
  m.setM1Speed(leftMotor);
  m.setM2Speed(rightMotor);
}

// ===================================================
// Serial receive with <...> markers
void recvWithStartEndMarkers() {
  static bool recvInProgress = false;
  static byte ndx = 0;
  const char startMarker = '<';
  const char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress) {
      if (rc != endMarker) {
        if (ndx < (numChars - 1)) {
          receivedChars[ndx++] = rc;
        }
      } else {
        receivedChars[ndx] = '\0';
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
      ndx = 0;
    }
  }
}

// ===================================================

void parseData() {
  char *tok = strtok(tempChar, ",");
  if (!tok) return;
  long parsedLeft = strtol(tok, NULL, 10);

  tok = strtok(NULL, ",");
  if (!tok) return;
  long parsedRight = strtol(tok, NULL, 10);

  leftMotor  = constrain((int)parsedLeft,  -400, 400);
  rightMotor = constrain((int)parsedRight, -400, 400);

  Serial.print(F("[CMD] left="));
  Serial.print(leftMotor);
  Serial.print(F(" right="));
  Serial.println(rightMotor);
}
