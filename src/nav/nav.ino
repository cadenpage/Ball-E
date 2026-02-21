#include <Arduino.h>
#include <AStar32U4Motors.h>
#include <Encoder.h>

AStar32U4Motors m;

// FRONT Sensor
#define TRIG_FRONT 14
#define ECHO_FRONT 17

// LEFT Sensor (not wired now, but we keep the defines)
#define TRIG_LEFT 12
#define ECHO_LEFT 1

// Serial command parsing
const byte numChars = 32;
char receivedChars[numChars];
char tempChar[numChars];

// Motor command variables
int leftMotor = 0;
int rightMotor = 0;
boolean newData = false;
const unsigned long TELEMETRY_INTERVAL_MS = 40; // ~25 Hz stream
unsigned long lastTelemetryMillis = 0;
float lastFrontDist = -1.0f;
float lastLeftDist = -1.0f;
int16_t linePosition = -1;
bool actionActive = false;

// Encoder + geometry (from lab template)
const int encoderRightPinA = 15;
const int encoderRightPinB = 16;
const int encoderLeftPinA = 19;
const int encoderLeftPinB = 11;
Encoder encoderRight(encoderRightPinA, encoderRightPinB);
Encoder encoderLeft(encoderLeftPinA, encoderLeftPinB);

const long encoderResolution = 1440;       // counts per rev
const float wheelDiameterIn = 2.7559055f;  // inches
const float wheelDiameterCm = wheelDiameterIn * 2.54f;
const float trackWidthCm = 14.0f;          // measured center-to-center wheel spacing
const float countsPerCm = encoderResolution / (PI * wheelDiameterCm);

// Bias and default speeds (tunable via commands)
float leftBias = 1.0f;
float rightBias = 1.0f;
int leftMaxCmd = 400;   // per-motor command caps (tune if one wheel is stronger)
int rightMaxCmd = 400;
int defaultDriveSpeed = 120;
int defaultTurnSpeed = 100;

// Forward decls
void recvWithStartEndMarkers();
void parseData();
float readUltrasonic(int trigPin, int echoPin);
void updateLineSensors();
void sendDataToRpi();
void commandMotors();
void resetEncoders();
void driveDistanceCm(float cm, int speed);
void turnDegrees(float deg, int speed);
int applyLeftBias(int cmd);
int applyRightBias(int cmd);

//=====================================================

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    delay(1000);

    // Set ultrasonic sensor pins
    pinMode(TRIG_FRONT, OUTPUT);
    pinMode(ECHO_FRONT, INPUT);

    pinMode(TRIG_LEFT, OUTPUT);
    pinMode(ECHO_LEFT, INPUT);   // harmless if not wired

    Serial.println("<Arduino is ready>");
    delay(500);
}

//====================================================

void loop() {
    // Heartbeat indicator (1s)
    static unsigned long lastHeartbeat = 0;
    unsigned long now = millis();
    if (now - lastHeartbeat >= 1000) {
      Serial.println("<HB>");
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      lastHeartbeat = now;
    }

    // Read serial commands (non-blocking)
    recvWithStartEndMarkers();
    if (newData == true) {
      // copy safely
      strncpy(tempChar, receivedChars, numChars - 1);
      tempChar[numChars - 1] = '\0';
      parseData();
      // echo received command for debugging
      Serial.print("<RC>");
      Serial.print(tempChar);
      Serial.println(">");
      newData = false;
    }
    // ===== CONTROL MOTORS =====
    commandMotors();

    // ===== CONTINUOUS TELEMETRY STREAM =====
    if (now - lastTelemetryMillis >= TELEMETRY_INTERVAL_MS) {
      sendDataToRpi();
      lastTelemetryMillis = now;
    }
}

//======================================================
float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(4);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // 20 ms timeout to avoid long blocking
  long duration = pulseIn(echoPin, HIGH, 20000);
  if (duration == 0) return -1;
  return (duration * 0.0343f) / 2.0f;
}

//======================================================

void updateLineSensors() {
  // TODO: wire in your line sensor reading here.
  // For now, leave as -1 to indicate unavailable.
  linePosition = -1;
}

//======================================================

void parseData(){
  char *tok = strtok(tempChar, ",");
  if (tok == NULL) return;

  // First token selects mode
  if (tok[0] == 'S') {  // manual speed command: S,left,right
    long parsedLeft = strtol(strtok(NULL, ","), NULL, 10);
    long parsedRight = strtol(strtok(NULL, ","), NULL, 10);
    leftMotor = constrain(applyLeftBias((int)parsedLeft), -400, 400);
    rightMotor = constrain(applyRightBias((int)parsedRight), -400, 400);
    actionActive = false;
    Serial.print("[CMD] Manual L=");
    Serial.print(leftMotor);
    Serial.print(" R=");
    Serial.println(rightMotor);
  } else if (tok[0] == 'B') { // bias command: B,leftBias,rightBias
    char *lTok = strtok(NULL, ",");
    char *rTok = strtok(NULL, ",");
    if (lTok && rTok) {
      leftBias = atof(lTok);
      rightBias = atof(rTok);
      Serial.print("[CMD] Bias set L=");
      Serial.print(leftBias, 3);
      Serial.print(" R=");
      Serial.println(rightBias, 3);
    }
  } else if (tok[0] == 'M') { // max command caps: M,leftMax,rightMax
    char *lTok = strtok(NULL, ",");
    char *rTok = strtok(NULL, ",");
    if (lTok && rTok) {
      leftMaxCmd = constrain(atoi(lTok), 0, 400);
      rightMaxCmd = constrain(atoi(rTok), 0, 400);
      Serial.print("[CMD] Max set L=");
      Serial.print(leftMaxCmd);
      Serial.print(" R=");
      Serial.println(rightMaxCmd);
    }
  } else if (tok[0] == 'D') { // drive distance cm: D,cm,speed
    char *distTok = strtok(NULL, ",");
    char *spdTok = strtok(NULL, ",");
    if (distTok) {
      float cm = atof(distTok);
      int spd = spdTok ? atoi(spdTok) : defaultDriveSpeed;
      driveDistanceCm(cm, spd);
    }
  } else if (tok[0] == 'T') { // turn degrees: T,deg,speed
    char *degTok = strtok(NULL, ",");
    char *spdTok = strtok(NULL, ",");
    if (degTok) {
      float deg = atof(degTok);
      int spd = spdTok ? atoi(spdTok) : defaultTurnSpeed;
      turnDegrees(deg, spd);
    }
  } else if (tok[0] == 'H') { // halt
    leftMotor = 0;
    rightMotor = 0;
    actionActive = false;
    Serial.println("[CMD] Halt");
  }
}

//==========================================

void sendDataToRpi() {
  // Refresh ultrasonic readings right before telemetry
  float frontDist = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  if (frontDist > 0) lastFrontDist = frontDist;
  float leftDist = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
  if (leftDist > 0) lastLeftDist = leftDist;
  updateLineSensors();

  Serial.print("<");
  Serial.print(lastFrontDist, 2);
  Serial.print(",");
  Serial.print(lastLeftDist, 2);
  Serial.print(",");
  Serial.print(linePosition);
  Serial.print(",");
  Serial.print(actionActive ? 1 : 0);
  Serial.println(">");
}

//=======================================

void commandMotors(){
  if (actionActive) {
    return; // action routines drive motors directly
  }
  // Note: AStar M1 = right motor, M2 = left motor (per Pololu wiring)
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
          if (ndx < (numChars - 1)) {
            receivedChars[ndx++] = rc;
          }
        } else {
          receivedChars[ndx] = '\0';
          recvInProgress = false;
          ndx = 0;
          newData = true;
        }
      }
      else if (rc == startMarker) {
        recvInProgress = true;
        ndx = 0;
      }
    }
}

//==================================================================

void resetEncoders() {
  encoderLeft.write(0);
  encoderRight.write(0);
}

// Blocking drive for distance (cm). Positive = forward, negative = backward.
void driveDistanceCm(float cm, int speed) {
  speed = constrain(speed, -400, 400);
  resetEncoders();
  actionActive = true;

  long targetCounts = (long)(abs(cm) * countsPerCm);
  int dir = (cm >= 0) ? 1 : -1;
  int leftCmd = applyLeftBias(dir * speed);
  int rightCmd = applyRightBias(dir * speed);

  m.setM1Speed(rightCmd); // M1 = right
  m.setM2Speed(leftCmd);  // M2 = left

  while (true) {
    long lCounts = abs(encoderLeft.read());
    long rCounts = abs(encoderRight.read());
    if (lCounts >= targetCounts && rCounts >= targetCounts) break;
  }

  m.setM1Speed(0);
  m.setM2Speed(0);
  actionActive = false;
}

// Blocking in-place turn based on encoders. Positive deg = left (CCW).
void turnDegrees(float deg, int speed) {
  speed = constrain(speed, 0, 400);
  resetEncoders();
  actionActive = true;

  float arcCm = (trackWidthCm * PI) * (abs(deg) / 360.0f);
  long targetCounts = (long)(arcCm * countsPerCm);
  int dir = (deg >= 0) ? 1 : -1; // left positive

  int leftCmd = applyLeftBias(dir * speed);
  int rightCmd = applyRightBias(-dir * speed);

  m.setM1Speed(rightCmd); // M1 = right
  m.setM2Speed(leftCmd);  // M2 = left

  while (true) {
    long lCounts = abs(encoderLeft.read());
    long rCounts = abs(encoderRight.read());
    if (lCounts >= targetCounts && rCounts >= targetCounts) break;
  }

  m.setM1Speed(0);
  m.setM2Speed(0);
  actionActive = false;
}

int applyLeftBias(int cmd) {
  float scaled = cmd * leftBias;
  return (int)constrain((int)scaled, -leftMaxCmd, leftMaxCmd);
}

int applyRightBias(int cmd) {
  float scaled = cmd * rightBias;
  return (int)constrain((int)scaled, -rightMaxCmd, rightMaxCmd);
}
