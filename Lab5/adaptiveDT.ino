#include <AStar32U4Motors.h>
#include <Encoder.h>
#include <QTRSensors.h>

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t linePosition;

const byte numChars = 32;
char receivedChars[numChars];
char tempChar[numChars];
boolean newData = false;

// ----------------- MOTOR / ENCODER -----------------
AStar32U4Motors m;

#define PI 3.141592653589

// Commanded outputs
int leftMotor = 0;
int rightMotor = 0;
int isCross = 0;

double leftMotorMax  = 36;   // in/s  (find from max test)
double rightMotorMax = 30;   // in/s

// Encoder pins (keep as your working PID)
const int encoderRightPinA = 15;
const int encoderRightPinB = 16;
const int encoderLeftPinA  = 19;
const int encoderLeftPinB  = 11;

Encoder encoderRight(encoderRightPinA, encoderRightPinB);
Encoder encoderLeft(encoderLeftPinA, encoderLeftPinB);

int encoderResolution = 1440;      // counts / rev
double d = 2.7559055;              // wheel diameter [in]

volatile int posLeftCount = 0;
volatile int posRightCount = 0;
int posLeftCountLast  = 0;
int posRightCountLast = 0;

double delta_right = 0.0, delta_left = 0.0;
double posLeftRad = 0.0, posRightRad = 0.0;
double velLeft = 0.0, velRight = 0.0;           // in/s
double newVelLeft = 0.0, newVelRight = 0.0;     // in/s

// ----------------- TIMING -----------------
unsigned long previousMillis = 0;      // for velocity dt
unsigned long priorTimeL = 0, priorTimeR = 0;   // PID dt
unsigned long lastPrint = 0;           // throttle serial

// ----------------- PID -----------------
double lastSpeedErrorL = 0.0, lastSpeedErrorR = 0.0;
double cumErrorL = 0.0,     cumErrorR = 0.0;
double maxErr = 20.0;                      // anti-windup clamp (units: (in/s)*s)
double desVelL = 10.0;                     // setpoints [in/s]
double desVelR = 10.0;

// LEFT gains
double kpL = 3.0;
double kiL = 0.7;
double kdL = 1.0;
// RIGHT gains
double kpR = 3.0;
double kiR = 0.7;
double kdR = 1.0;

// ----------------- QTR PINS -----------------
// Do NOT use 16 here (it’s encoderRightPinB).
// If you have the buzzer jumper on pin 6, remove it or avoid pin 6 as a sensor.
void setupQTR() {
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){7, 18, 23, 20, 21, 22, 8, 6}, SensorCount);
  qtr.setEmitterPins(4, 5); // <- avoid 16 to prevent encoder conflict
}

// ----------------- SETUP -----------------
void setup() {
  Serial.begin(115200);

  // Motors idle
  m.setM1Speed(0);
  m.setM2Speed(0);

  // Timers init
  unsigned long now = millis();
  previousMillis = now;
  priorTimeL = now;
  priorTimeR = now;

  // QTR
  setupQTR();
  calibrateSensors();

  Serial.println(F("<Arduino ready>"));
}

// ----------------- LOOP -----------------
void loop() {
  // line sensing (non-blocking)
  readThatBitchAssLine();

  // receive setpoints like <12.5,12.5>
  recvWithStartEndMarkers();
  if (newData) {
    strcpy(tempChar, receivedChars);
    parseData();
    newData = false;
  }

  // run velocity PID at whatever rate the loop allows (uses real dt)
  runPID();

  // throttle prints to reduce jitter
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.print(desVelL); Serial.print(',');
    Serial.print(desVelR); Serial.print(" || ");
    Serial.print(velLeft); Serial.print(',');
    Serial.print(velRight); Serial.print(" || ");
    Serial.print(newVelLeft); Serial.print(',');
    Serial.print(newVelRight); Serial.print(" || ");
    Serial.print(linePosition); Serial.print(',');
    Serial.println(isCross);
  }
}

// ===================== MOTOR COMMANDS =====================
void CommandMotors() {
  m.setM1Speed(rightMotor);
  m.setM2Speed(leftMotor);
}

int motorVelToSpeedCommand(double v, double vmax) {
  v = constrain(v, -vmax, vmax);
  double cmd = (v / vmax) * 400.0;  // scale to [-400, 400]
  if (cmd > 400.0) cmd = 400.0;
  if (cmd < -400.0) cmd = -400.0;
  return (int)cmd;
}

// ===================== CONTROL LOOP =====================
void runPID() {
  unsigned long now = millis();
  double dt = (now - previousMillis) / 1000.0;    // seconds
  if (dt <= 0.0) return;                          // guard for first loop / timer wrap
  previousMillis = now;

  // Read encoders
  posRightCount = encoderRight.read();
  posLeftCount  = encoderLeft.read();

  // Deltas
  delta_right = (double)(posRightCount - posRightCountLast);
  delta_left  = (double)(posLeftCount  - posLeftCountLast);

  // Angular velocity [rad/s]
  posRightRad = (delta_right / (double)encoderResolution) * (2.0 * PI) / dt;
  posLeftRad  = (delta_left  / (double)encoderResolution) * (2.0 * PI) / dt;

  // Tangential velocity [in/s]
  velRight = - (d * 0.5) * posRightRad;
  velLeft  = - (d * 0.5) * posLeftRad;

  // PID per side (each uses its own dt based on priorTime*)
  newVelRight = drivePIDR(velRight);
  newVelLeft  = drivePIDL(velLeft);

  // Command motors
  rightMotor = motorVelToSpeedCommand(newVelRight, rightMotorMax);
  leftMotor  = motorVelToSpeedCommand(newVelLeft,  leftMotorMax);

  posRightCountLast = posRightCount;
  posLeftCountLast  = posLeftCount;

  CommandMotors();
}

// ===================== PID CORES =====================
double drivePIDL(double curr) {
  unsigned long t = millis();
  double dt = (t - priorTimeL) / 1000.0;
  if (dt <= 0.0) dt = 1e-3;

  double error = desVelL - curr;
  cumErrorL += error * dt;
  if (cumErrorL >  maxErr) cumErrorL =  maxErr;   // anti-windup
  if (cumErrorL < -maxErr) cumErrorL = -maxErr;

  double rateError = (error - lastSpeedErrorL) / dt;
  double out = kpL * error + kiL * cumErrorL + kdL * rateError;

  lastSpeedErrorL = error;
  priorTimeL = t;
  return out;
}

double drivePIDR(double curr) {
  unsigned long t = millis();
  double dt = (t - priorTimeR) / 1000.0;
  if (dt <= 0.0) dt = 1e-3;

  double error = desVelR - curr;
  cumErrorR += error * dt;
  if (cumErrorR >  maxErr) cumErrorR =  maxErr;   // anti-windup
  if (cumErrorR < -maxErr) cumErrorR = -maxErr;

  double rateError = (error - lastSpeedErrorR) / dt;
  double out = kpR * error + kiR * cumErrorR + kdR * rateError;

  lastSpeedErrorR = error;
  priorTimeR = t;
  return out;
}

// ===================== SERIAL IN: <L,R> =====================
void parseData() {
  // DO NOT wipe integrators every packet.
  char *tok = strtok(tempChar, ",");
  if (!tok) return;
  double newDesVelL = atof(tok);

  tok = strtok(NULL, ",");
  if (!tok) return;
  double newDesVelR = atof(tok);

  // Optional: reset integrators only on large setpoint steps
  if (fabs(newDesVelL - desVelL) > 2.0) cumErrorL = 0.0;
  if (fabs(newDesVelR - desVelR) > 2.0) cumErrorR = 0.0;

  desVelL = newDesVelL;
  desVelR = newDesVelR;
}

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  const char startMarker = '<';
  const char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if (recvInProgress) {
      if (rc != endMarker) {
        receivedChars[ndx++] = rc;
        if (ndx >= numChars) ndx = numChars - 1;
      } else {
        receivedChars[ndx] = '\0';
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

// ===================== QTR SENSORS =====================
void calibrateSensors() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(1);
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void readThatBitchAssLine() {
  // Read with emitters (library handles them after setEmitterPins)
  linePosition = qtr.readLineBlack(sensorValues);

  // Filter small noise
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] < 300) sensorValues[i] = 0;
  }

  // All zero -> position zero
  bool allZero = true;
  for (int i = 0; i < 8; i++) if (sensorValues[i] != 0) { allZero = false; break; }
  if (allZero) linePosition = 0;

  // Edge-only clamps
  if (sensorValues[0] > 0 &&
      sensorValues[1]==0 && sensorValues[2]==0 && sensorValues[3]==0 &&
      sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0 && sensorValues[7]==0) {
    linePosition = 1000;
  }
  if (sensorValues[7] > 0 &&
      sensorValues[0]==0 && sensorValues[1]==0 && sensorValues[2]==0 &&
      sensorValues[3]==0 && sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0) {
    linePosition = 5000;
  }

  if (linePosition > 5000) linePosition = 5000;
  if (linePosition < 1000 && linePosition > 0) linePosition = 1000;

  // Cross detection
  if (sensorValues[7] > 500 && sensorValues[0] > 500) isCross = 1;
  else isCross = 0;
}
