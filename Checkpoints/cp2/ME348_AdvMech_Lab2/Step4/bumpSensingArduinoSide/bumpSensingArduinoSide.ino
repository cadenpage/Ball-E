#include <AStar32U4Motors.h>

AStar32U4Motors m; //read the documentation of this library to understand what functions to use to drive the motors and how to use them

const uint8_t BMP_FR 0;
const uint8_t BMP_L 1;
const uint8_t BMP_MR 4;
const uint8_t BMP_ML 5;
const uint8_t BMP_L 7;
const uint8_t BMP_FL 8;


enum states {
    NONE,
    KEEPDRIVING,
    HEADON,
    AVOIDRIGHT,
    AVOIDLEFT,
}

states prior_state state;

uint32_t initialT;
uint16_t counter


int leftMotor;
int rightMotor;
int FAST = 150;
int STOP = 0;
int MED = 75;

uint32_t INCREMENT

const byte numChars = 32;
char receivedChars[numChars];
char tempChar[numChars]; // temporary array used for parsing

boolean newData = false;

int FR = 0;
int R = 0;
int MR = 0;

int ML = 0;
int L = 0;
int FL =0;


//=====================================================

void setup() {
  Serial.begin(115200);
  pinMode(BMP_FR, INPUT_PULLUP);
  pinMode(BMP_R, INPUT_PULLUP); //might have to reassign these to pins that the astar knows
  pinMode(BMP_MR, INPUT_PULLUP);

  pinMode(BMP_ML, INPUT_PULLUP);
  pinMode(BMP_L, INPUT_PULLUP);
  pinMode(BMP_FL, INPUT_PULLUP);

  prior_state = NONE;
  state = KEEPDRIVING

}

//=====================================================
void keepdriving(){
  if (state != to prior_state) {
    prior_state = state;
    leftMotor = FAST;
    rightMotor = FAST;
    CommandMotors()
    
  }
}

void headon(){
  if (state != to prior_state) {
    prior_state = state;
    
  }  
}

void avoidright(){
  if (state != to prior_state) {
    prior_state = state;
  }  
}

void avoidleft(){
  if (state != to prior_state) {
    prior_state = state;
  }  
}

void stop(){
  
}





void loop() {


  recvWithStartEndMarkers();

  if (newData == true){
    
    strcpy(tempChar, receivedChars); //make a copy of recievedChars so we can make changes to it and not change recievedChars 
    parseData();  
    SendBumpData();

//start of FSM CODE ************
    switch (state) {
        case KEEPDRIVING:
          keepdriving();
          break
        case HEADON:
          headon();
          break

        case AVOIDRIGHT:
          avoidright();
          break
        case AVOIDLEFT:
          avoidleft();
          break
          
        case STOP:
          stop();
          break
        
    }

// ************************************************************
    
    //SendRecievedData(); //uncomment this (and make the neccicary changes to the Rpi code) to have the arduino send the Rpi back what it sent. Viewing the message the
                          //Rpi sends the arduino is important to make sure the Rpi isnt sending garbage
    newData = false;
    
  }

    
  
//printBumpData(); //for testing the arduino code with the bump sensors on its own, comment this in to test the 
//                   //bump sensors without doing any communication with the RPI
  
CommandMotors();

}

//============================================

void recvWithStartEndMarkers() {
//this function is the most important one of the whole lab, read the blog post made my Robin2
//some questions:
      //whats the purpose of the start and end markers?
      //Why bother making this code unblocking?
      //
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

//============================================

void parseData(){ //this function takes a string and seperates it by the comma into 2 strings, then casts those strings to ints
  
strcpy(tempChar,receivedChars); //copying recievedChar into tempChar so we dont alter recievedChar

char *strIndexer = strtok(tempChar,",");

leftMotor = atoi(strIndexer);

strIndexer = strtok(NULL,",");

rightMotor = atoi(strIndexer);

}  

//============================================
//void SendBumpData(){
//  
//  x=digitalRead(0);
//  y=digitalRead(1);
//  z=digitalRead(4);
//
//  a=digitalRead(5);
//  b=digitalRead(7);
//  c=digitalRead(8);
//  
//
//  Serial.print(x);
//  Serial.print(',');
//  Serial.print(y);
//  Serial.print(',');
//  Serial.print(z);
//  Serial.print(',');
//  Serial.print(a);
//  Serial.print(',');
//  Serial.print(b);
//  Serial.print(',');
//  Serial.println(c);
//}

//============================================

void CommandMotors(){  

  //read the documentation for the functions that drive the motors in the astar library

  m.setM1Speed(rightMotor);
  m.setM2Speed(leftMotor);
  //uncomment to drive motors
}







//===========================FUNCTIONS USED FOR TESTING, NOT NEEDED FOR LAB====================================





void SendRecievedData(){
  Serial.println(receivedChars);
}


void printBumpData(){
  
  x=digitalRead(0);
  y=digitalRead(1);
  z=digitalRead(4);

  a=digitalRead(5);
  b=digitalRead(7); //gonna have to set these also
  c=digitalRead(8);
  

  //Serial.print(receivedChars);
  Serial.print(x);
  Serial.print(',');
  Serial.print(y);
  Serial.print(',');
  Serial.print(z);
  Serial.print(',');
  Serial.print(a);
  Serial.print(',');
  Serial.print(b);
  Serial.print(',');
  Serial.println(c);
  delay(1000);
}
