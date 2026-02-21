//#include <Arduino.h>
#include <QTRSensors.h>
#include <AStar32U4Motors.h>
#include <Encoder.h>
AStar32U4Motors m; //motor assignment
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t linePosition;

const byte numChars = 32;
char receivedChars[numChars];
char tempChar[numChars]; // temporary array used for parsing

//motor command variables
int leftMotor=0; //int leftMotor
int rightMotor=0;
bool intersection=false; //is the robot at an intersection?

//PID parameters
float Kp = 2.0;
float Ki = 1.0;
float Kd = 1.0;

uint8_t multiP = 2.5;
uint8_t multiI = 1;
uint8_t multiD = 1.2;
uint8_t Kpfinal;
uint8_t Kifinal;
uint8_t Kdfinal;
int error;
int lfspeed = 90; //base speed of the motors
const float integralMin = -5000.0f;
const float integralMax = 5000.0f;
float I = 0.0f;
float P, D, previousError, PIDvalue;
const float PID_dt = 0.05f; // 50 ms loop time for pid
int lsp, rsp;

// --- Function declarations --- we need to declare functions before using them I have come to find out
void robot_control();
void PID_Linefollow(int error);
void commandMotors(int left, int right);
void calibrateSensors();

boolean newData = false;
// PID timing
unsigned long lastPidMs = 0;
const unsigned long PID_INTERVAL_MS = 50; // 50 ms PID loop
unsigned long lastPrintMs = 0;
const unsigned long PRINT_INTERVAL_MS = 1000; // 1s debug print

void setup() {
    Serial.begin(115200);
    Serial.println("Arduino is Ready");
    m.setM1Speed(0);
    m.setM2Speed(0);
    //Connecting to line follower sensor
    qtr.setTypeRC(); //this allows us to read the line sensor from didgital pins
    //arduino pin sensornames I am using: 7, 18, A5, 20, 21, 22, 8, 6.
    qtr.setSensorPins((const uint8_t[]){7, 18, 23, 20, 21, 22, 8, 6}, SensorCount);
    qtr.setEmitterPin(4); //can get away with a single emitter pin providing power to both emitters
    calibrateSensors();
 
    Serial.println("<Arduino is ready>");
    delay(3000);
    Serial.println("<Starting Line Following>");
}

//====================================================

void loop() {
//////REPAIRING THE READLINEBLACK FUNCTION /////////////////////////////////////
  // read sensors (emitters on during read)
  linePosition = qtr.readLineBlack(sensorValues, QTRReadMode::On);
  // QTRReadMode::On tells the read call to enable emitters for the measurement

//*******REPAIR********
//Remove low reading noise
for (int i=0; i<= 7; i++){
   if (sensorValues[i] <300){
       sensorValues[i]=0;
    }
}

  unsigned long now = millis();

//----------------------------Debug (non-blocking)---------------------
//
//   if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
//     lastPrintMs = now;
//     Serial.print(linePosition);
//     Serial.print("\t");
//     for (int i = 0; i < SensorCount; i++) {
//       Serial.print(sensorValues[i]);
//       Serial.print("\t");
//     }
//     Serial.println();
//   }

//-----------------ACTUAL SCRIPT OR SOME SHIT LIKE THAT------------------

  // Run PID at fixed interval (non-blocking)
  if (now - lastPidMs >= PID_INTERVAL_MS) {
    lastPidMs = now;
    int err = 3500 - (int)linePosition; // same error calc used in robot_control
    PID_Linefollow(err);
  }


}

//////////////////////////////////////////////////////////////////////

void PID_Linefollow(int error){
  float e = (float)error;
  Serial.println(e);
  // Proportional term
  float Pterm = Kp * e;

  // Integral term (with anti-windup)
  I += e * PID_dt;
  if (I > integralMax) I = integralMax;
  if (I < integralMin) I = integralMin;
  float Iterm = Ki * I;

  // Derivative term (on error)
  D = (e - previousError) / PID_dt;
  float Dterm = Kd * D;

  // PID output
  float output = Pterm + Iterm + Dterm;
  previousError = e;

  // Compute and constrain motor speeds
  //int lsp = (int)constrain((float)lfspeed - output, -255.0f, 255.0f);
  //int rsp = (int)constrain((float)lfspeed + output, -255.0f, 255.0f);

  const int minForward =60;
  float maxCorrection =(float)lfspeed - (float)minForward;
  if (maxCorrection < 0.0f) maxCorrection = 0.0f;
  const float deadband = 5.0f;
  if (fabs(output)<deadband) output = 0.0f;

  if (output > maxCorrection) output = maxCorrection;
  if (output < -maxCorrection) output = -maxCorrection;

  float desiredLeftF = (float) lfspeed - output;
  float desiredRightF = (float) lfspeed + output;

  desiredLeftF = constrain(desiredLeftF, (float)minForward, 255.0f);
  desiredRightF = constrain(desiredRightF, (float)minForward, 255.0f);

  int lsp = (int)desiredLeftF;
  int rsp = (int)desiredRightF;
  commandMotors(lsp, rsp);
  
}

//////////////////////////////////////////////////////////

void commandMotors(int left, int right){
  m.setM1Speed(left);
  m.setM2Speed(right);
}

//==================================================================

void calibrateSensors(){
  //THE SENSORS ONLY CALIBRATE WHEN YOU UPLOAD NEW ARDUINO CODE TO THE ASTAR. after that the sensors STAY calibrated as long as the Astar has power.

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
                                   ///while calibrating, move the sensor over the line a couple times

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10s seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

}

//==================================================================
