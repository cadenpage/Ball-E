
#include <AStar32U4Motors.h>
#include <Encoder.h>

AStar32U4Motors m; //read the documentation of this library to understand what functions to use to drive the motors and how to use them

#define PI 3.141592653589

int leftMotor;
int rightMotor;

double leftMotorMax = 32.47; // students should find this variable themselves
double rightMotorMax = 32.47;

const int encoderRightPinA = 15;
const int encoderRightPinB = 16;

const int encoderLeftPinA = 8; //15;
const int encoderLeftPinB = 11;

Encoder encoderRight(encoderRightPinA,encoderRightPinB);
Encoder encoderLeft(encoderLeftPinA,encoderLeftPinB);

int encoderResolution = 1440; // counts per rev
double d = 2.7559055; //wheel diameter in inches
//double c = pi*d; // wheel circumference
// this will be omegaLeft*d/2;
int posLeftCount = 0;
int posRightCount = 0;
int posLeftCountLast = 0;
int posRightCountLast = 0;
double posLeftRad = 0.0; // this needs to be converted to rad/sec
double posRightRad = 0.0; // this needs to be converted to rad/sec
double velLeft = 0; // this will be omegaLeft*d/2;
double velRight = 0; // this will be omegaRight*d/2 will be in inches per sec;
double newVelLeft = 0; // this will be omegaLeft*d/2;
double newVelRight = 0; // this will be omegaRight*d/2 will be in inches per sec;

//variables carl needed to declare for communication
int leftMotorRcvd; //revieved motor commands from Rpi
int rightMotorRcvd; //..
const byte numChars = 32; //variables used for parsing
char receivedChars[numChars]; //..
char tempChar[numChars]; //..
boolean newData = false; //..
//end of variables carl needed to declare for communication

// MOTOR LOOP CONSTANTS
double interval = 5.0; // 5 ms means 200Hz loop
unsigned long previousMillis = 0;
unsigned long priorTimeL,priorTimeR;
double lastSpeedErrorL,lastSpeedErrorR;
double cumErrorL, cumErrorR;
double maxErr = 20; // chosen arbitrarily for now
double desVelL = 0.0; // will be in inches per sec
double desVelR = 0.0;

// PID CONSTANTS
// LEFT MOTOR - assumes we need to tune them differently
double kpL = 1.5;
double kiL = 0.02;
double kdL = 0.5;
// Right MOTOR - assumes we need to tune them differently
double kpR = 1.5;
double kiR = 0.02;
double kdR = 0.5                                                                                                                                                                                              ;

//=====================================================

void setup() {
  Serial.begin(115200);
    m.setM1Speed(0);
    m.setM2Speed(0);  
 

}

//================================// this will be omegaLeft*d/2;=====================

void loop() {

   recvWithStartEndMarkers();

   if (newData == true) {

    strcpy(tempChar, receivedChars); //copy recieved packet ti parse it
    parseData();
    
    sendDataToRpi();
   }

   unsigned long currentMillis = millis();

      posRightCount = encoderRight.read(); 
      posLeftCount = encoderLeft.read();

   if (currentMillis - previousMillis >= interval){
      previousMillis = currentMillis;


     posRightRad = 2*PI*(((posRightCount-posRightCountLast)/(interval/1000))/(double) encoderResolution); // Write expression to get Rad/sec
     posLeftRad = 2*PI*(((posLeftCount-posLeftCountLast)/(interval/1000))/(double) encoderResolution);
     velRight = (posRightRad)*d/2; // Inches/sec
     velLeft = (posLeftRad)*d/2; // Inches/sec

     // HERE WILL DO PID AND CREATE NEW MOTOR COMMAND MAPPED TO -400-400 based on max. 

     newVelRight = drivePIDR(velRight);
     newVelLeft = drivePIDL(velLeft);

      //Serial.print("RIGHT: ");
      
      //moved the print statements to senddatatorpi function

      rightMotor = motorVelToSpeedCommand(newVelRight,rightMotorMax);
      leftMotor = motorVelToSpeedCommand(newVelLeft,leftMotorMax);

      
     
     posRightCountLast = posRightCount;
     posLeftCountLast = posLeftCount;
    

      CommandMotors();
   }
}

void CommandMotors(){  

  //read the documentation for the functions that drive the motors in the astar library

  m.setM1Speed(rightMotor);
  m.setM2Speed(leftMotor);
  //uncomment to drive motors
}

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

int motorVelToSpeedCommand(double Vel, double maxVel){
    int newSpeed = 0;
    Vel = constrain(Vel,-1*maxVel, maxVel);
    newSpeed = map(Vel,-1*maxVel, maxVel, -400, 400);
    return newSpeed;
}

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
                receivedChars[ndx] = '\0'; // terminates the string, frankly unsure why I need 
                                           //this but it breaks if I remove it. Bonus points if you find out why
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


void parseData(){


  char *strtokIndexer; //doing char * allows strtok to increment across my string properly frankly im not sure why... my kingdom for a proper c++ class

  strtokIndexer = strtok(tempChar,","); //sets strtokIndexer = to everything up to the first comma in tempChar /0 //this line is broken
  leftMotorRcvd = atoi(strtokIndexer); //converts strtokIndexer into a int
  
  strtokIndexer= strtok(NULL, ","); //setting the first input to null causes strtok to continue looking for commas in tempChar starting from where it left off, im not really sure why 
  rightMotorRcvd = atoi(strtokIndexer);
  //these are the recieved commands from the RPI, unsure what varable to change in order to tell the rest of this program to go that speed

}

void sendDataToRpi(){
    Serial.print('<');
      Serial.print(velRight);
      Serial.print(',');
      Serial.print(newVelRight);
      //Serial.print("  ===  LEFT: ");
      Serial.print(velLeft);
      Serial.print(',');
      Serial.println(newVelLeft);
      Serial.println('>');
}
