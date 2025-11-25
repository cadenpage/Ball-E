#include <Arduino.h>
#include <AStar32U4Motors.h>
#include <Encoder.h>
#include <QTRSensors.h>
#include <Servo.h>
#include <math.h>

AStar32U4Motors m;
QTRSensors qtr;

// FRONT Sensor
#define TRIG_FRONT 14
#define ECHO_FRONT 17

// LEFT Sensor (not wired now, but we keep the defines)
#define TRIG_LEFT 12
#define ECHO_LEFT 1

// Line sensor (QTR-8RC)
const uint8_t LineSensorCount = 8;
uint16_t lineSensorValues[LineSensorCount];
uint16_t linePosition = 0;

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
bool actionActive = false;
bool lineFollowActive = false;

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
const float trackWidthCm = 14.0f;          // measured center-to-center wheel spacing (tune for turn accuracy)
const float countsPerCm = encoderResolution / (PI * wheelDiameterCm);
float turnGain = 0.92f;                    // scale commanded degrees (1.0 = no scale; <1 reduces turn)

// Bias and default speeds (tunable via commands)
float leftBias = 1.12f;
float rightBias = 1.0f;
int leftMaxCmd = 400;   // per-motor command caps (tune if one wheel is stronger)
int rightMaxCmd = 400;
int defaultDriveSpeed = 60;  // keep raw commands <= 60 for controlled motion
int defaultTurnSpeed = 60;
int lineBaseSpeed = 90;        // base forward speed for line follow (matches working sketch)
const int lineMinForward = 60; // minimum forward that overcomes friction
const float lineStopFrontCm = 56.0f; // stop line following when front US within this
const int lineStopHitsRequired = 3;   // consecutive valid hits required to stop
bool invertLine = false;   // set true if array is reversed (changes error sign)

// Servo (feetech) control
const uint8_t SERVO_PIN = 9;     // Servo signal pin (use the same style as working sweep sketch)
Servo shooterServo;
int servoLeftUs = 1000;          // adjust for your left target
int servoMidUs = 1500;           // adjust for center
int servoRightUs = 2000;         // adjust for right target
unsigned long lastServoStepMs = 0;

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
void driveUntilFront(float targetCm, int speed);
int applyLeftBias(int cmd);
int applyRightBias(int cmd);
void initLineSensors();
void lineFollowStep();
void startLineFollow();
void stopLineFollow();
void setServoUs(int us);

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

    // Line sensor init and calibration
    initLineSensors();

    // Servo init
    shooterServo.attach(SERVO_PIN);
    setServoUs(servoMidUs);

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
  // ===== LINE FOLLOWING =====
  if (lineFollowActive) {
    lineFollowStep();
  } else {
    // ===== CONTROL MOTORS =====
    commandMotors();
  }

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
  // Read sensors (emitters on during read), remove low noise
  linePosition = qtr.readLineBlack(lineSensorValues, QTRReadMode::On);
  for (int i = 0; i < LineSensorCount; i++) {
    if (lineSensorValues[i] < 300) {
      lineSensorValues[i] = 0;
    }
  }
}

//======================================================

void parseData(){
  char *tok = strtok(tempChar, ",");
  if (tok == NULL) return;

  // First token selects mode
  if (tok[0] == 'S') {  // manual speed command: S,left,right
    long parsedLeft = strtol(strtok(NULL, ","), NULL, 10);
    long parsedRight = strtol(strtok(NULL, ","), NULL, 10);
    parsedLeft = constrain(parsedLeft, -60, 60);
    parsedRight = constrain(parsedRight, -60, 60);
    leftMotor = constrain(applyLeftBias((int)parsedLeft), -400, 400);
    rightMotor = constrain(applyRightBias((int)parsedRight), -400, 400);
    actionActive = false;
    lineFollowActive = false;
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
      spd = constrain(spd, -60, 60);
      driveDistanceCm(cm, spd);
    }
  } else if (tok[0] == 'T') { // turn degrees: T,deg,speed
    char *degTok = strtok(NULL, ",");
    char *spdTok = strtok(NULL, ",");
    if (degTok) {
      float deg = atof(degTok);
      int spd = spdTok ? atoi(spdTok) : defaultTurnSpeed;
      spd = constrain(spd, 0, 60);
      turnDegrees(deg, spd);
    }
  } else if (tok[0] == 'G') { // turn gain: G,gain (scale degrees)
    char *gTok = strtok(NULL, ",");
    if (gTok) {
      turnGain = atof(gTok);
      if (turnGain < 0.1f) turnGain = 0.1f;
      if (turnGain > 2.0f) turnGain = 2.0f;
      Serial.print("[CMD] Turn gain set to ");
      Serial.println(turnGain, 3);
    }
  } else if (tok[0] == 'L') { // start line follow
    startLineFollow();
  } else if (tok[0] == 'X') { // stop line follow
    stopLineFollow();
  } else if (tok[0] == 'F') { // drive until front distance: F,targetCm,speed
    char *tTok = strtok(NULL, ",");
    char *sTok = strtok(NULL, ",");
    if (tTok) {
      float targetCm = atof(tTok);
      int spd = sTok ? atoi(sTok) : defaultDriveSpeed;
      spd = constrain(spd, 0, 60);
      driveUntilFront(targetCm, spd);
    }
  } else if (tok[0] == 'I') { // invert line error: I,0/1
    char *iTok = strtok(NULL, ",");
    if (iTok) {
      int flag = atoi(iTok);
      invertLine = (flag != 0);
    }
  } else if (tok[0] == 'V') { // servo microseconds: V,us
    char *uTok = strtok(NULL, ",");
    if (uTok) {
      int us = atoi(uTok);
      setServoUs(us);
    }
  } else if (tok[0] == 'P') { // beacon aim preset: P,0/1/2 (left/mid/right)
    char *bTok = strtok(NULL, ",");
    if (bTok) {
      int sel = atoi(bTok);
      if (sel == 0) setServoUs(servoLeftUs);
      else if (sel == 1) setServoUs(servoMidUs);
      else if (sel == 2) setServoUs(servoRightUs);
    }
  } else if (tok[0] == 'H') { // halt
    leftMotor = 0;
    rightMotor = 0;
    actionActive = false;
    lineFollowActive = false;
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
  Serial.print((actionActive || lineFollowActive) ? 1 : 0);
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

// Blocking forward drive until front US <= targetCm or timeout
void driveUntilFront(float targetCm, int speed) {
  speed = constrain(speed, 0, 60);
  actionActive = true;
  lineFollowActive = false;

  int lCmd = applyLeftBias(speed);
  int rCmd = applyRightBias(speed);

  m.setM1Speed(rCmd);
  m.setM2Speed(lCmd);

  unsigned long start = millis();
  const unsigned long timeoutMs = 12000;
  while (millis() - start < timeoutMs) {
    float front = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
    if (front > 0 && front <= targetCm) {
      break;
    }
  }

  m.setM1Speed(0);
  m.setM2Speed(0);
  actionActive = false;
}

//=========================================================
// ================== LINE FOLLOWING (from line_following_new_PID_CP) =========

// PID parameters (match working line_following_new_PID_CP.ino)
float Kp = 2.0f;
float Ki = 1.0f;
float Kd = 1.0f;
const float integralMin = -5000.0f;
const float integralMax = 5000.0f;
float ItermAccum = 0.0f;
float prevError = 0.0f;
unsigned long lastPidMs = 0;
bool useSimpleLine = false; // use full PID like working sketch

void initLineSensors() {
  // Line sensor pins and emitter (QTR-8RC)
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){7, 18, 23, 20, 21, 22, 8, 6}, LineSensorCount);
  qtr.setEmitterPin(4);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void startLineFollow() {
  lineFollowActive = true;
  actionActive = false;
  ItermAccum = 0.0f;
  prevError = 0.0f;
  lastPidMs = millis();
}

void stopLineFollow() {
  lineFollowActive = false;
  m.setM1Speed(0);
  m.setM2Speed(0);
}

void lineFollowStep() {
  unsigned long now = millis();
  const unsigned long minLoopMs = 50; // match working sketch PID interval
  if (now - lastPidMs < minLoopMs) {
    return;
  }
  lastPidMs = now;
  float dt = 0.05f; // fixed 50ms loop like working code

  updateLineSensors();
  static float filtError = 0.0f;
  int rawErr = invertLine ? ((int)linePosition - 3500) : (3500 - (int)linePosition); // target center of array (~3500)
  filtError = 0.7f * filtError + 0.3f * (float)rawErr; // simple low-pass
  int error = (int)filtError;

  // Proportional
  float output;
  if (useSimpleLine) {
    // Simple proportional steering (close to original example behavior)
    output = Kp * (float)error;
  } else {
    float Pterm = Kp * (float)error;
    // Integral with windup guard
    ItermAccum += ((float)error) * dt;
    if (ItermAccum > integralMax) ItermAccum = integralMax;
    if (ItermAccum < integralMin) ItermAccum = integralMin;
    float Iterm = Ki * ItermAccum;
    // Derivative
    float Dterm = Kd * (((float)error - prevError) / dt);
    prevError = (float)error;
    output = Pterm + Iterm + Dterm;
  }

  const float deadband = 8.0f;
  if (fabs(output) < deadband) output = 0.0f;

  float maxCorrection = (float)lineBaseSpeed - (float)lineMinForward;
  if (maxCorrection < 0.0f) maxCorrection = 0.0f;
  if (output > maxCorrection) output = maxCorrection;
  if (output < -maxCorrection) output = -maxCorrection;

  float desiredLeftF = (float)lineBaseSpeed - output;
  float desiredRightF = (float)lineBaseSpeed + output;
  desiredLeftF = constrain(desiredLeftF, (float)lineMinForward, 255.0f);
  desiredRightF = constrain(desiredRightF, (float)lineMinForward, 255.0f);

  // Use raw motor commands to match working line-follow sketch (no bias)
  int lCmd = (int)desiredLeftF;
  int rCmd = (int)desiredRightF;

  m.setM1Speed(lCmd); // match working sketch mapping (M1=left)
  m.setM2Speed(rCmd); // M2=right

  // Stop condition: front US within threshold
  float front = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  static int stopHits = 0;
  if (front > 0 && front <= lineStopFrontCm) {
    stopHits++;
  } else {
    stopHits = 0;
  }
  if (stopHits >= lineStopHitsRequired) {
    stopLineFollow();
    stopHits = 0;
  }
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

  float arcCm = (trackWidthCm * PI) * (abs(deg) * turnGain / 360.0f);
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

// Servo helper
void setServoUs(int us) {
  // constrain to broad analog servo range; adjust if needed
  us = constrain(us, 500, 2500);
  shooterServo.writeMicroseconds(us);
}
