#include <QTRSensors.h>
#include <AStar32U4Motors.h>
#include <Encoder.h>
#include <math.h>
#include <Arduino.h>

AStar32U4Motors m;
QTRSensors qtr;

// ===== Line Following =====
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t linePosition = 0;
int isCross = 0;

// ===== Serial Parsing =====
const byte numChars = 32;
char receivedChars[numChars];
char tempChar[numChars];
bool newData = false;

// ===== Encoders / Kinematics =====
const int encoderRightPinA = 15;
const int encoderRightPinB = 16;
const int encoderLeftPinA  = 10;
const int encoderLeftPinB  = 11;

Encoder encoderRight(encoderRightPinA, encoderRightPinB);
Encoder encoderLeft (encoderLeftPinA , encoderLeftPinB );

const long encoderResolution = 1440;     // counts / rev
const double wheelDiam_in = 2.7559055;   // inches
const double wheelRadius_in = wheelDiam_in * 0.5;

// ===== Control Timing =====
const unsigned long outerInterval_ms = 5; // ~200 Hz outer loop (velocity calc + apply PWM)
unsigned long lastOuter_ms = 0;

// For PID timing (seconds)
unsigned long priorTimeL_us = 0;
unsigned long priorTimeR_us = 0;

// ===== State =====
long posRightCount = 0, posLeftCount = 0;
long posRightCountLast = 0, posLeftCountLast = 0;

double velRight = 0.0; // in/s
double velLeft  = 0.0; // in/s

// ===== Setpoints (in/s) =====
double desVelL = 10.0;
double desVelR = 10.0;

// ===== Limits =====
double leftVelMax_inps  = 36.0;   // max achievable speed (for clamping setpoint)
double rightVelMax_inps = 30.0;

// ===== PID (velocity -> PWM) =====
// PID outputs are DIRECTLY PWM commands (±400)
double kpL = 8.0,  kiL = 1.2, kdL = 0.8;
double kpR = 8.0,  kiR = 1.2, kdR = 0.8;

double lastErrL = 0.0, lastErrR = 0.0;
double intErrL  = 0.0, intErrR  = 0.0;
double intLimit = 100.0; // anti-windup clamp

// ===== Prototypes =====
void LineFollow();
void recvWithStartEndMarkers();
void parseData();
void CommandMotors(int pwmL, int pwmR);
int  clipPWM(double u);
double drivePIDL(double meas_inps);
double drivePIDR(double meas_inps);

// ===== Setup =====
void setup() {
  pinMode(3, OUTPUT);
  pinMode(2, OUTPUT);
  Serial.begin(115200);

  // QTR setup (RC mode, single emitter pin)
  qtr.setTypeRC();
  qtr.setEmitterPin(4);
  qtr.setSensorPins((const uint8_t[]){7, 18, 23, 20, 21, 22, 8, 6}, SensorCount);

  calibrateSensors();

  // Prime timers
  lastOuter_ms  = millis();
  priorTimeL_us = micros();
  priorTimeR_us = micros();

  // Small initial spin (optional)
  m.setM1Speed(0);
  m.setM2Speed(0);

  Serial.println("<Arduino is ready>");
}

// ===== Loop =====
void loop() {
  // 1) Non-blocking serial receive
  recvWithStartEndMarkers();
  if (newData) {
    strcpy(tempChar, receivedChars);
    parseData();        // updates desVelL / desVelR (in/s)
    newData = false;
  }

  // 2) Line follow sensors (if you still need them for logging/cross-detect)
  LineFollow();

  // 3) Outer control/velocity update at fixed pace
  unsigned long now_ms = millis();
  if (now_ms - lastOuter_ms >= outerInterval_ms) {
    lastOuter_ms = now_ms;

    // Read encoders
    long cR = encoderRight.read();
    long cL = encoderLeft.read();

    long dR = cR - posRightCountLast;
    long dL = cL - posLeftCountLast;

    // Use actual dt (seconds) for velocity
    static unsigned long lastVel_us = micros();
    unsigned long now_us = micros();
    double dt = (now_us - lastVel_us) * 1e-6;
    if (dt <= 0) dt = outerInterval_ms / 1000.0;
    lastVel_us = now_us;

    // counts -> rad/s
    double wR = ((double)dR / (double)encoderResolution) * (2.0 * PI) / dt;
    double wL = ((double)dL / (double)encoderResolution) * (2.0 * PI) / dt;

    // rad/s -> in/s  (check signs for your wiring)
    velRight = -(wheelRadius_in) * wR;
    velLeft  = -(wheelRadius_in) * wL;

    // Clamp setpoints to achievable
    if (desVelL >  leftVelMax_inps) desVelL =  leftVelMax_inps;
    if (desVelL < -leftVelMax_inps) desVelL = -leftVelMax_inps;
    if (desVelR >  rightVelMax_inps) desVelR =  rightVelMax_inps;
    if (desVelR < -rightVelMax_inps) desVelR = -rightVelMax_inps;

    // PID -> PWM (direct)
    int pwmR = clipPWM(drivePIDR(velRight));
    int pwmL = clipPWM(drivePIDL(velLeft));

    // Apply
    CommandMotors(pwmL, pwmR);

    // Save counts
    posRightCountLast = cR;
    posLeftCountLast  = cL;

    // Telemetry (one line; easy for Python .readline())
    Serial.print(linePosition); Serial.print(',');
    Serial.print(isCross);      Serial.print(',');
    Serial.print(pwmL);         Serial.print(',');
    Serial.print(pwmR);         Serial.print(',');
    Serial.print(velLeft, 3);   Serial.print(',');
    Serial.print(velRight, 3);  Serial.print(',');
    Serial.print(desVelL, 3);   Serial.print(',');
    Serial.println(desVelR, 3);
  }
}

// ===== PID (seconds timebase; output = PWM) =====
double drivePIDL(double meas_inps) {
  unsigned long now_us = micros();
  double dt = (now_us - priorTimeL_us) * 1e-6;
  if (dt <= 0) dt = outerInterval_ms / 1000.0;
  priorTimeL_us = now_us;

  static double lastSet = desVelL;
  if (fabs(desVelL - lastSet) > 0.5) {  // bumpless change
    intErrL = 0.0;
    lastErrL = 0.0;
  }
  lastSet = desVelL;

  double err = desVelL - meas_inps;
  intErrL += err * dt;
  if (intErrL >  intLimit) intErrL =  intLimit;
  if (intErrL < -intLimit) intErrL = -intLimit;

  double dErr = (err - lastErrL) / dt;
  lastErrL = err;

  double u = kpL * err + kiL * intErrL + kdL * dErr;  // PWM units
  return u;
}

double drivePIDR(double meas_inps) {
  unsigned long now_us = micros();
  double dt = (now_us - priorTimeR_us) * 1e-6;
  if (dt <= 0) dt = outerInterval_ms / 1000.0;
  priorTimeR_us = now_us;

  static double lastSet = desVelR;
  if (fabs(desVelR - lastSet) > 0.5) {  // bumpless change
    intErrR = 0.0;
    lastErrR = 0.0;
  }
  lastSet = desVelR;

  double err = desVelR - meas_inps;
  intErrR += err * dt;
  if (intErrR >  intLimit) intErrR =  intLimit;
  if (intErrR < -intLimit) intErrR = -intLimit;

  double dErr = (err - lastErrR) / dt;
  lastErrR = err;

  double u = kpR * err + kiR * intErrR + kdR * dErr;  // PWM units
  return u;
}

int clipPWM(double u) {
  if (u >  400.0) return  400;
  if (u < -400.0) return -400;
  return (int)u;
}

// ===== Motor Command =====
void CommandMotors(int pwmL, int pwmR){
  m.setM1Speed(pwmL);
  m.setM2Speed(pwmR);
}

// ===== Serial Receive (unchanged frame <L,R>) =====
void recvWithStartEndMarkers() {
  static bool recvInProgress = false;
  static byte ndx = 0;
  const char startMarker = '<';
  const char endMarker   = '>';
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

// Parse into setpoints (in/s)
void parseData() {
  char *tok = strtok(tempChar, ",");
  if (!tok) return;
  desVelL = atof(tok);

  tok = strtok(NULL, ",");
  if (!tok) return;
  desVelR = atof(tok);
}

// ===== Line Following (your repaired version) =====
void LineFollow() {
  linePosition = qtr.readLineBlack(sensorValues);

  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] < 400) sensorValues[i] = 0;
  }

  if (sensorValues[0]==0 && sensorValues[1]==0 && sensorValues[2]==0 && sensorValues[3]==0 &&
      sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0 && sensorValues[7]==0) {
    linePosition = 0;
  }

  if (sensorValues[0] > 0 && sensorValues[1]==0 && sensorValues[2]==0 && sensorValues[3]==0 &&
      sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0 && sensorValues[7]==0) {
    linePosition = 1000;
  }
  if (sensorValues[7] > 0 && sensorValues[0]==0 && sensorValues[1]==0 && sensorValues[2]==0 &&
      sensorValues[3]==0 && sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0) {
    linePosition = 5000;
  }

  if (linePosition > 5000) linePosition = 5000;
  if (linePosition < 1000 && linePosition > 0) linePosition = 1000;

  if ((sensorValues[7] > 500) && (sensorValues[0] > 500)) isCross = 1;
  else                                                    isCross = 0;
}

// ===== Sensor Calibration =====
void calibrateSensors() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++) qtr.calibrate();
  digitalWrite(LED_BUILTIN, LOW);
}
