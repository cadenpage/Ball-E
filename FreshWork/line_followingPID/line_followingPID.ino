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
int isCross=0;

//PID parameters
float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.0;

uint8_t multiP = 1;
uint8_t multiI = 1;
uint8_t multiD = 1;

int lfspeed = 200;
int previousError = 0;
int IvalueAccum = 0;

//Python interface route
char route[] = {'F','R', 'F', 'L'};
int stepIndex = 0;
// --- Non-blocking turn variables ---
bool turning = false;
char turnDirection = ' ';
unsigned long turnStartTime = 0;
unsigned long turnDuration = 10;

// --- Function declarations ---
void robot_control();
void PID_Linefollow(int error);
void motor_drive(int left, int right);
void startTurn(char dir, unsigned long duration_ms);

boolean newData = false;

void setup() {

    Serial.begin(115200);
    m.setM1Speed(0);
    m.setM2Speed(0);

    qtr.setTypeRC(); //this allows us to read the line sensor from didgital pins
    //arduino pin sensornames I am using: 7, 18, A5, 20, 21, 22, 8, 6. 
    qtr.setSensorPins((const uint8_t[]){7, 18, 23, 20, 21, 22, 8, 6}, SensorCount);

    calibrateSensors();
    qtr.setEmitterPin(4); //can get away with a single emitter pin providing power to both emitters
    QTRReadMode::On; //emitters on measures active reflectance instead of ambient light levels, better becasue the ambient light level will change as the robot moves around the board but the reflectance levels will not
    Serial.println("<Arduino is ready>");
}

//====================================================

void loop() {

linePosition = qtr.readLineBlack(sensorValues);

//check serial commands for nav
  if(Serial.available()) {
    char cmd = Serial.read();
    if(!turning){  // Ignore new commands while turning
      if(cmd == 'L') startTurn('L', 400);
      else if(cmd == 'R') startTurn('R', 400);
      else if(cmd == 'F') startTurn('F', 300);
    }
  }

  // Handle ongoing turn
  if(turning && millis() - turnStartTime >= turnDuration){
    turning = false;
    motor_drive(0,0); // stop after turn
  }

  // PID line following runs continuously if not turning
  if(!turning){
    robot_control();
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////END OF REPAIRS TO LINEPOSITION///////////////////////////////////////////////////////////////////////////

void robot_control() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = 2000 - position;

  // Intersection detection: all sensors over threshold
  bool intersection = true;
  for(uint8_t i=0;i<SensorCount;i++){
    if(sensorValues[0] && sensorValues[7] < 2000){
      intersection = false;
      break;
    }
  }

  // Handle intersection without blocking
  if(intersection && !turning){
    if(stepIndex < sizeof(route)){
      char move = route[stepIndex++];
      if(move == 'L') startTurn('L', 400);
      else if(move == 'R') startTurn('R', 400);
      else if(move == 'F') startTurn('F', 300);
    }
  }

  PID_Linefollow(error);
}

void PID_Linefollow(int error){
  int P = error;
  IvalueAccum += error;
  int D = error - previousError;

  float Pvalue = (Kp/pow(10,multiP))*P;
  float Ivalue = (Ki/pow(10,multiI))*IvalueAccum;
  float Dvalue = (Kd/pow(10,multiD))*D;

  float PIDvalue = Pvalue + Ivalue + Dvalue;
  previousError = error;

  int lsp = lfspeed - PIDvalue;
  int rsp = lfspeed + PIDvalue;

  // Clamp values
  if(lsp > 100) lsp = 100; if(lsp < -100) lsp = -100;
  if(rsp > 100) rsp = 100; if(rsp < -100) rsp = -100;

  commandMotors(lsp,rsp);
}  

void commandMotors(int left, int right){
  if(left>=0){m.setM1Speed(leftMotor);
  if(right>=0){m.setM2Speed(rightMotor);
}
  }
}

void startTurn(char dir, unsigned long duration_ms){
  turning = true;
  turnDirection = dir;
  turnStartTime = millis();
  turnDuration = duration_ms;

  if(dir == 'R') motor_drive(200,-200);
  else if(dir == 'L') motor_drive(-200,200);
  else if(dir == 'F') motor_drive(180,180);
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
