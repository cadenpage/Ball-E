
#include <AStar32U4Motors.h>
#include <Encoder.h>
#include <QTRSensors.h>
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

  
boolean newData = false;

AStar32U4Motors m; //read the documentation of this library to understand what functions to use to drive the motors and how to use them

#define PI 3.141592653589

double leftMotorMax = 36; // **students should find this variable themselves**
double rightMotorMax = 30;

const int encoderRightPinA = 15;
const int encoderRightPinB = 16;

const int encoderLeftPinA = 19; 
const int encoderLeftPinB = 11;

Encoder encoderRight(encoderRightPinA,encoderRightPinB);
Encoder encoderLeft(encoderLeftPinA,encoderLeftPinB);

int encoderResolution = 1440; // counts per rev
double d = 2.7559055; //wheel diameter in inches

int posLeftCount = 0;
double delta_right = 0.0;
double delta_left = 0.0;
int posRightCount = 0;
int posLeftCountLast = 0;
int posRightCountLast = 0;
double posLeftRad = 0.0; // this will need to be converted to rad/sec
double posRightRad = 0.0; // this will need to be converted to rad/sec
double velLeft = 0; // this will be omegaLeft*d/2;
double velRight = 0; // this will be omegaRight*d/2 will be in inches per sec;
double newVelLeft = 0; // this will be omegaLeft*d/2;
double newVelRight = 0; // this will be omegaRight*d/2 will be in inches per sec;

// MOTOR LOOP CONSTANTS
double interval = 5; // 5 ms means 200Hz loop
unsigned long previousMillis = 0;
unsigned long priorTimeL,priorTimeR; // We need to keep track of time for each PID controller separately
double lastSpeedErrorL,lastSpeedErrorR; //same with error
double cumErrorL, cumErrorR;
double maxErr = 20; // chosen arbitrarily for now, students can tune. 
double desVelL = 0; // will be in inches per sec
double desVelR = 0;

// PID CONSTANTS
// LEFT MOTOR - you need to find values. FYI I found good responses with Kp ~ 10x bigger than Ki, and ~3x bigger than Kd. My biggest value was <2.
double kpL = 3;
double kiL = .7;
double kdL = 1;
// Right MOTOR - assumes we need to tune them differently
double kpR = 3;
double kiR = .7;
double kdR = 1;

double speedmag = 15; //in/s
bool start = true;


//=====================================================



void setup() {
  Serial.begin(115200);
  
    m.setM1Speed(0);  // MAX IS 400 FYI. You should set this first to see max speed in in/s after you convert the values
    m.setM2Speed(0);  
    unsigned long startTime = millis();
    cumErrorL = cumErrorR = 0;
    priorTimeL = priorTimeR = millis();
    lastSpeedErrorL = lastSpeedErrorR = 0;
    //pinMode(3, OUTPUT); //left motor
   // pinMode(2,OUTPUT); //left motor
    qtr.setTypeRC(); //this allows us to read the line sensor from didgital pins

    //arduino pin sensornames I am using: 7, 18, 23 aka A5. note:PIN A1 DID NOT WORK WITH ANY SENSOR!!, 20, 21, 22, 8, 6. UNHOOK THE BLUE JUMPER LABELED BUZZER ON THE ASTAR or pin 6 will cause the buzzer to activate.
//    qtr.setEmitterPins(4, 5);
//     QTRReadMode::On;
    qtr.setSensorPins((const uint8_t[]){7, 18, 23, 20, 21,22,8,6}, SensorCount); // changed pins - removed conflicts

    calibrateSensors();
    qtr.setEmitterPins(4,16);//et away with a single emitter pin providing power to both emitters originally (4,5) for future reference
    QTRReadMode::On; //emitters on measures active reflectance instead of ambient light levels, better becasue the ambient light level will change as the robot moves around the board but the reflectance levels will not
    Serial.println("<Arduino is ready>");
}


void loop() {
 
   readThatBitchAssLine();

   
   recvWithStartEndMarkers(); //this function is in charge of taking a peice of data that looks like <17,16> 
                               //turning it into a string looking like 17,16 and then setting newdata to true,
                               //letting the rest of the program know a packet of data is ready to be analyzed, does all this without blocking
    if (newData == true) { //newData will be true when recvWithStartEndMarkers(); has finished recieving a whole set of data from Rpi (a set of data is denoted as being containted between <>)
      
      strcpy(tempChar, receivedChars); //this line makes a copy of recievedChars for parsing in parseData, I do this becasue strtok() will alter any string I give it,I want to preserve the origonal data
      parseData(); //right now parseData only parses a string of 2 numbers seperated by commas into floats
                   //so the string 17.5,16 becomes two floats; 17.5 and 16


      //below this comment and between setting newData to false is where you want to send the Rpi whatever data you want.
      Serial.print(linePosition);
      Serial.print(",");
      Serial.print(isCross);
      Serial.print(",");
      Serial.print(leftMotor);
      Serial.print(",");
      Serial.print(sensorValues[0]);
      Serial.print(",");
      Serial.print(sensorValues[1]);
      Serial.print(",");
      Serial.print(sensorValues[2]);
      Serial.print(",");
      Serial.print(sensorValues[3]);
      Serial.print(",");
      Serial.print(sensorValues[4]);
      Serial.print(",");
      Serial.print(sensorValues[5]);
      Serial.print(",");
      Serial.print(sensorValues[6]);
      Serial.print(",");
      Serial.print(sensorValues[7]);
      Serial.print(",");
      Serial.println(rightMotor);

      newData = false;

      
      //sendDataToRpi(); //unused
                   
    }
runPID();

}




//PID MOTOR COMMAND FUNCTIONS DONT EDIT
//===========================================================================================================================
//SEND MOTOR COMMANDS
void CommandMotors(){  
  m.setM1Speed(rightMotor);
  m.setM2Speed(leftMotor);
}

//MAPS OUT MOTOR SPEED TO COMMAND
int motorVelToSpeedCommand(double Vel, double maxVel){
    int newSpeed = 0;
    Vel = constrain(Vel,-1*maxVel, maxVel);
    newSpeed = map(Vel,-1*maxVel, maxVel, -400, 400);
    return newSpeed;
}

//USES KINEMATICS TO CONTROL SPEED
void runPID(){
  
   unsigned long currentMillis = millis();
      posRightCount = encoderRight.read(); 
      posLeftCount = encoderLeft.read();
      
   if (currentMillis - previousMillis >= interval){
     previousMillis = currentMillis;

     delta_right = posRightCount - posRightCountLast;
     delta_left = posLeftCount - posLeftCountLast;
     
     posRightRad = (((delta_right/encoderResolution)*(2*PI)) / (interval / 1000)); // Write expression to get Rad/sec. Pi is defined above FYI.
     posLeftRad = (((delta_left/encoderResolution)*(2*PI)) / (interval/ 1000)); // Same - Rad/sec
     
     velRight = - (d/2) *posRightRad; // Now convert to get inches/sec (tangential velocity)
     velLeft = - (d/2) * posLeftRad; // Same - Inches/sec

     newVelRight = drivePIDR(velRight);
     newVelLeft = drivePIDL(velLeft);  
      
//      Serial.print("RIGHT: ");
//      Serial.print(velRight);
//      Serial.print(',');
////      Serial.print(newVelRight);
//      Serial.print("  ===  LEFT: ");




      rightMotor = motorVelToSpeedCommand(newVelRight,rightMotorMax);
      leftMotor = motorVelToSpeedCommand(newVelLeft,leftMotorMax);
     
      posRightCountLast = posRightCount;
      posLeftCountLast = posLeftCount;
      CommandMotors();
      
   }
}
//===========================================================================================================================

//PID FUNCTIONS DONT EDIT
//===========================================================================================================================
double drivePIDL(double curr){
    double rateError;
    double error;
    unsigned long currentTime;
    unsigned long elapsedTime;
  
    currentTime = millis();                               //get current time
    elapsedTime = (double)(currentTime - priorTimeL);     // compute elasped time for this control period

    error = desVelL - curr;                               // Error
    cumErrorL += error*elapsedTime;                       // Cumulative Error(since we add this outside the loop, needs to be unique to the motor controlled)

    // INTEGRAL WINDUP                                    // REMOVE WINDUP
    if(cumErrorL>maxErr)
    cumErrorL = maxErr;
    else if (cumErrorL<-1*maxErr)
      cumErrorL = -1*maxErr;

    rateError = (error-lastSpeedErrorL)/elapsedTime;      // Derivative Error

    double out = kpL*error+kiL*cumErrorL+kdL*rateError;   // PID output

    lastSpeedErrorL = error;                              // remember current error
    priorTimeL = currentTime;                             // remember current time
    return out;                                           // return the needed motor speed. 
}

double drivePIDR(double curr){
    double rateError;
    double error;
    unsigned long currentTime;
    unsigned long elapsedTime;
  
    currentTime = millis();                               //get current time
    elapsedTime = (double)(currentTime - priorTimeR);      // compute elasped time for this control period

    error = desVelR - curr;                                   // Error
    cumErrorR += error*elapsedTime;                       // Cumulative Error(since we add this outside the loop, needs to be unique to the motor controlled)

    // INTEGRAL WINDUP                                    // REMOVE WINDUP
    if(cumErrorR>maxErr)
    cumErrorR = maxErr;
    else if (cumErrorR<-1*maxErr)
      cumErrorR = -1*maxErr;

    rateError = (error-lastSpeedErrorR)/elapsedTime;      // Derivative Error

    double out = kpR*error+kiR*cumErrorR+kdR*rateError;   // PID output

    lastSpeedErrorR = error;                              // remember current error
    priorTimeR = currentTime;                             // remember current time
    return out;                                           // return the needed motor speed. 
}

//PYTHON INTERFACE
//===========================================================================================================================

void parseData(){
  


  char *strtokIndexer; //doing char * allows strtok to increment across my string properly frankly im not sure why... something to do with pointers that I dont expect students to understand

  
  strtokIndexer = strtok(tempChar,","); //sets strtokIndexer = to everything up to the first comma in tempChar /0 //this line is broken
  //leftMotor = atoi(strtokIndexer); //converts strtokIndexer into a int
  desVelL = atoi(strtokIndexer);

  

  strtokIndexer= strtok(NULL, ","); //setting the first input to null causes strtok to continue looking for commas in tempChar starting from where it left off, im not really sure why 
  //rightMotor = atoi(strtokIndexer);
  desVelR = atoi(strtokIndexer);

  
  //now that we have extracted the data from the Rpi as floats, we can use them to command actuators somewhere else in the code
  
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
//===========================================================================================================================



//SENSOR FUNCTIONS
//===========================================================================================================================
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

void readThatBitchAssLine(){
  linePosition = qtr.readLineBlack(sensorValues);

//REPAIR NUMBER ONE:  every sensor reading under 300 is a noisy and messes up the lineposition measurment, so this for loop filters it out
for (int i=0; i<= 7; i++){
   if (sensorValues[i] <300){
       sensorValues[i]=0;     
    }
}


// REPAIR NUMBER 2: checking if all my sensorvalues are zero and then setting lineposition to zero,sometimes if all sensorvalues are zero lineposition will be set to 7k and this fixes that
if (sensorValues[0]==0 && sensorValues[1]==0 && sensorValues[2]==0 && sensorValues[3]==0 && sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0 && sensorValues[7]==0){
  linePosition=0;
}

//REPAIR NUMBER THREE: if only sensor 0 or sensor 8 are reading measurments, then the low level function that calculates linePosition will set it to values that dare not representative of where the sensor array
//is actually located relative to the line, the below loops fix that by setting linePosition to  1000 if ONLY sensor 0 sees anything, and setting linePosition to 5000 if ONLY sensor 7 sees anything
if (sensorValues[0] >0 && sensorValues[1]==0 && sensorValues[2]==0 && sensorValues[3]==0 && sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0 && sensorValues[7]==0){
  linePosition=1000;
} 
if (sensorValues[7] >0 && sensorValues[0]==0 && sensorValues[1]==0 && sensorValues[2]==0 && sensorValues[3]==0 && sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0){
  linePosition=5000;
}

//REPAIR NUMBER 4: there are still situations where linePosition is somehow greater than 5000 or 0<linePosition<1000, so I am hard capping linePosition to be between 1000 and 5000 when linePosition is greater
//than zero.
if (linePosition > 5000){
  linePosition = 5000;
}
if (linePosition < 1000 && linePosition > 0){
  linePosition = 1000;
}

//this loop uses the leftmost and rightmost sensors to determin if the robot is at a cross. If both of those sensors read high, then the robot is at a cross. 
if ((sensorValues[7] > 500) && (sensorValues[0] > 500)){
  isCross = 1;
}else{
    isCross = 0;
  }
}

// THIS FUNCTION NEEDS TO READ ULTRASOUND AND LOG VALUE IN "SensorValue[8]"