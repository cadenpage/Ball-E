#include <AStar32U4Motors.h>

AStar32U4Motors m; //read the documentation of this library to understand what functions to use to drive the motors and how to use them


const byte numChars = 32;
char receivedChars[numChars];
char tempChar[numChars]; // temporary array used for parsing

boolean newData = false;
int leftMotor;
int rightMotor;


  void setup() {
    Serial.begin(115200);
 
}   

void loop() {

  recvWithStartEndMarkers();


  if (newData == true){
        
    parseData();
    commandMotors();
    sendRecievedData();
    newData = false;
    //why am I setting newdata equil to false after I send data back to the rpi.
    //what would happen if this line was not here?
    
}
}


void recvWithStartEndMarkers() {
//this function is the most important one of the whole lab, read the blog post made my Robin2
//some questions:
      //whats the purpose of the start and end markers?
      //Why bother making this code unblocking?
      //why not use the Arduino built in functions for reading serial data?
      
    static boolean recvInProgress = false;
    //what is the purpose of this boolean?
    
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

//====================================

void parseData(){
  
strcpy(tempChar,receivedChars); //copying recievedChar into tempChar so we dont alter recievedChar

char *strIndexer = strtok(tempChar,","); //dont worry about the *, this isnt a C class and I dont expect you to know how pointers work

leftMotor = atoi(strIndexer);

strIndexer = strtok(NULL,",");

rightMotor = atoi(strIndexer);

}

//=====================================

void sendRecievedData(){

  
  Serial.print(leftMotor);
  Serial.print(',');
  Serial.println(rightMotor);
}

//=======================================


void commandMotors(){

  //read the documentation for the functions that drive the motors in the astar library

  m.setM1Speed(leftMotor);
  m.setM2Speed(rightMotor);
  //uncomment to drive motors
}
