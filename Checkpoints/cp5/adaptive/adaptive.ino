#include <AStar32U4Motors.h>
#include <Encoder.h>
#include <QTRSensors.h>

QTRSensors qtr;

// ---------- Serial CSV timing ----------
unsigned long lastCSV = 0;
const uint16_t CSV_PERIOD_MS = 20;   // ~50 Hz

// ---------- QTR read timing (don’t block too often) ----------
unsigned long lastQTR = 0;
const uint16_t QTR_PERIOD_MS = 25;   // ~40 Hz

// ---------- Sensors ----------
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t linePosition = 0;
int isCross = 0;

// ---------- Motor & encoder ----------
AStar32U4Motors m;

// Command outputs (what we send to AStar library)
int leftMotor = 0;
int rightMotor = 0;

#define PI 3.141592653589

double leftMotorMax  = 36.0; // in/s  (measure on your robot)
double rightMotorMax = 30.0; // in/s

// Encoders (keep your working pins)
const int encoderRightPinA = 15;
const int encoderRightPinB = 16;
const int encoderLeftPinA  = 19;
const int encoderLeftPinB  = 11;

Encoder encoderRight(encoderRightPinA, encoderRightPinB);
Encoder encoderLeft(encoderLeftPinA, encoderLeftPinB);

const int encoderResolution = 1440;    // counts / rev
const double d = 2.7559055;            // wheel diameter [in]

// Encoder counters
int posLeftCount = 0,  posRightCount = 0;
int posLeftCountLast = 0, posRightCountLast = 0;

// Kinematics
double delta_right = 0.0, delta_left = 0.0;
double posLeftRad = 0.0, posRightRad = 0.0;
double velLeft = 0.0, velRight = 0.0;        // in/s
double newVelLeft = 0.0, newVelRight = 0.0;  // in/s

// Loop timing
unsigned long prevMillis = 0;   // for velocity dt
unsigned long priorTimeL = 0;   // PID dt per side
unsigned long priorTimeR = 0;

// ---------- PID ----------
double desVelL = 0.0;   // in/s (set via serial "<L,R>")
double desVelR = 0.0;

double kpL = 3.0, kiL = 0.7, kdL = 1.0;
double kpR = 3.0, kiR = 0.7, kdR = 1.0;

double lastSpeedErrorL = 0.0, lastSpeedErrorR = 0.0;
double cumErrorL = 0.0, cumErrorR = 0.0;
double maxErr = 20.0; // anti-windup clamp (units: (in/s)*s)

// ---------- serial RX buffer ----------
const byte numChars = 32;
char receivedChars[numChars];
char tempChar[numChars];
bool newData = false;

// ---------- Forward decl ----------
void emitCSV();
void runPID();
void readLineThrottled();
void parseData();
void recvWithStartEndMarkers();
int  motorVelToSpeedCommand(double v, double vmax);
double drivePIDL(double curr);
double drivePIDR(double curr);
void calibrateSensors();

void setupQTR() {
  qtr.setTypeRC();
  // Avoid encoder pins (15,16,19,11). Pin 6 is OK only if buzzer jumper removed.
  qtr.setSensorPins((const uint8_t[]){7, 18, 23, 20, 21, 22, 8, 6}, SensorCount);
  qtr.setEmitterPin(13);      // <- do NOT use 16 (conflicts with encoder B)
  qtr.setTimeout(1000);          // shorter RC timing (us)
  qtr.setSamplesPerSensor(2);    // fewer samples to reduce blocking
}

void setup() {
  Serial.begin(115200);

  m.setM1Speed(0);
  m.setM2Speed(0);

  unsigned long now = millis();
  prevMillis = now;
  priorTimeL = now;
  priorTimeR = now;
  lastCSV = now;
  lastQTR = now;

  setupQTR();
  calibrateSensors();

  // No extra Serial prints (Python expects clean CSV only)
}

void loop() {
  // 1) Receive setpoints "<L,R>"
  recvWithStartEndMarkers();
  if (newData) {
    strcpy(tempChar, receivedChars);
    parseData();
    newData = false;
  }

  // 2) Run velocity PID (uses real dt)
  runPID();

  // 3) Read QTR at a slower rate (reduces missed encoder counts)
  readLineThrottled();

  // 4) Emit CSV for Python
  if (millis() - lastCSV >= CSV_PERIOD_MS) {
    lastCSV = millis();
    emitCSV();
  }
}

// ========================= Motor command =========================
void CommandMotors() {
  m.setM1Speed(rightMotor);
  m.setM2Speed(leftMotor);
}

int motorVelToSpeedCommand(double v, double vmax) {
  if (v >  vmax) v =  vmax;
  if (v < -vmax) v = -vmax;
  double cmd = (v / vmax) * 400.0;   // scale to [-400,400]
  if (cmd >  400.0) cmd =  400.0;
  if (cmd < -400.0) cmd = -400.0;
  return (int)cmd;
}

// ========================= Control loop =========================
void runPID() {
  unsigned long now = millis();
  double dt = (now - prevMillis) / 1000.0;  // seconds
  if (dt <= 0.0) return;
  prevMillis = now;

  // Encoders
  posRightCount = encoderRight.read();
  posLeftCount  = encoderLeft.read();

  delta_right = (double)(posRightCount - posRightCountLast);
  delta_left  = (double)(posLeftCount  - posLeftCountLast);

  // Angular velocity [rad/s]
  posRightRad = (delta_right / (double)encoderResolution) * (2.0 * PI) / dt;
  posLeftRad  = (delta_left  / (double)encoderResolution) * (2.0 * PI) / dt;

  // Tangential velocity [in/s]
  velRight = - (d * 0.5) * posRightRad;
  velLeft  = - (d * 0.5) * posLeftRad;

  // PID per side
  newVelRight = drivePIDR(velRight);
  newVelLeft  = drivePIDL(velLeft);

  // Map to motor commands
  rightMotor = motorVelToSpeedCommand(newVelRight, rightMotorMax);
  leftMotor  = motorVelToSpeedCommand(newVelLeft,  leftMotorMax);

  posRightCountLast = posRightCount;
  posLeftCountLast  = posLeftCount;

  CommandMotors();
}

// ========================= PID cores =========================
double drivePIDL(double curr) {
  unsigned long t = millis();
  double dt = (t - priorTimeL) / 1000.0;
  if (dt <= 0.0) dt = 1e-3;

  double error = desVelL - curr;
  cumErrorL += error * dt;
  if (cumErrorL >  maxErr) cumErrorL =  maxErr;
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
  if (cumErrorR >  maxErr) cumErrorR =  maxErr;
  if (cumErrorR < -maxErr) cumErrorR = -maxErr;

  double rateError = (error - lastSpeedErrorR) / dt;
  double out = kpR * error + kiR * cumErrorR + kdR * rateError;

  lastSpeedErrorR = error;
  priorTimeR = t;
  return out;
}

// ========================= Serial RX: "<L,R>" =========================
void parseData() {
  char *tok = strtok(tempChar, ",");
  if (!tok) return;
  double newDesVelL = atof(tok);

  tok = strtok(NULL, ",");
  if (!tok) return;
  double newDesVelR = atof(tok);

  // Reset integrator only on big steps (>2 in/s)
  double stepL = newDesVelL - desVelL;
  double stepR = newDesVelR - desVelR;
  if (stepL > 2.0 || stepL < -2.0) cumErrorL = 0.0;
  if (stepR > 2.0 || stepR < -2.0) cumErrorR = 0.0;

  desVelL = newDesVelL;
  desVelR = newDesVelR;
}

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

// ========================= QTR =========================
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
  linePosition = qtr.readLineBlack(sensorValues);

  // Filter small noise
  for (int i = 0; i < 8; i++) if (sensorValues[i] < 300) sensorValues[i] = 0;

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

  // Cross detection via outer sensors
  isCross = (sensorValues[7] > 500 && sensorValues[0] > 500) ? 1 : 0;
}

// Throttled wrapper
void readLineThrottled() {
  unsigned long now = millis();
  if (now - lastQTR >= QTR_PERIOD_MS) {
    lastQTR = now;
    readThatBitchAssLine();
  }
}

// ========================= CSV out =========================
void emitCSV() {
  // Format matches Python:
  // linePosition, isCross, leftMotor, s0..s7, rightMotor
  Serial.print(linePosition); Serial.print(',');
  Serial.print(isCross);      Serial.print(',');
  Serial.print(leftMotor);    Serial.print(',');
  for (int i = 0; i < 8; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(',');
  }
  Serial.println(rightMotor);
}
