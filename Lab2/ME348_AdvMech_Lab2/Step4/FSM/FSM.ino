#include <AStar32U4Motors.h>
#include <AStar32U4.h>
AStar32U4Motors m;

int leftMotor;
int rightMotor;
int FAST = 150;
int OFF = 0;
int MED = 75;

const uint8_t BMP_FR = 0;
const uint8_t BMP_R = 1;
const uint8_t BMP_MR = 4;
const uint8_t BMP_ML = 5;
const uint8_t BMP_L = 7;
const uint8_t BMP_FL = 8;

//============================================================================================

enum states {
    NONE,
    STOP,
    KEEPDRIVING,
    HEADON,
    AVOIDRIGHT,
    AVOIDLEFT,
};


states prior_state, state;
uint32_t initialT;
uint16_t counter;
uint32_t t;

const unsigned long INCREMENT=1000;

const byte numChars = 32;
char receivedChars[numChars];
char tempChar[numChars]; // temporary array used for parsing

boolean newData = false;

bool FR; // = digitalRead(BMP_FR) == HIGH;
bool R; //= digitalRead(BMP_R)  == HIGH;
bool MR; //= digitalRead(BMP_MR) == HIGH;

bool ML; //= digitalRead(BMP_ML) == HIGH;
bool L; //= digitalRead(BMP_L) == HIGH;
bool FL; //= digitalRead(BMP_FL) == HIGH;

//============================================================================================

void stateChecker() {
  
  if (FL && L && ML && MR && R && FR) {
    state = STOP;  
  } else if (ML && MR) {
    state = HEADON;
  } else if ( FL || L || ML ) {
    state = AVOIDLEFT;
  } else if (FR || R || MR) {
    state = AVOIDRIGHT;
  } else {
    state = KEEPDRIVING;
  }
    
}

//============================================================================================

void keepdriving(){
  ledGreen(1);
  if (state != prior_state) {
    prior_state = state;
    leftMotor = FAST;
    rightMotor = FAST;
    m.setM1Speed(rightMotor);
    m.setM2Speed(leftMotor);
  
     
  } stateChecker();
}

void headon(){
  if (state != prior_state) {
    ledRed(1);
    ledGreen(0);
    initialT = millis();
    prior_state = state;
    
    leftMotor = -MED;
    rightMotor = -MED;
    m.setM1Speed(rightMotor);
    m.setM2Speed(leftMotor);
 
  } stateChecker();
  t = millis();
  if (t > (initialT + INCREMENT)){
    state = KEEPDRIVING;
    ledRed(0); 
  }  
}

void avoidright(){
  if (state != prior_state) {
    ledYellow(1);
    ledGreen(0);
    initialT = millis();
    prior_state = state;
    leftMotor = OFF;
    rightMotor = FAST;
    m.setM1Speed(rightMotor);
    m.setM2Speed(leftMotor);
  } stateChecker();
  t = millis();
  if (t > (initialT + INCREMENT)){
    state = KEEPDRIVING;
    ledYellow(0); 
  }  
}

void avoidleft(){
  if (state != prior_state) {
    ledYellow(1);
    ledGreen(0);
    initialT = millis();
    prior_state = state;
    leftMotor = FAST;
    rightMotor = OFF;
    m.setM1Speed(rightMotor);
    m.setM2Speed(leftMotor);
     
  } stateChecker();
  t = millis();
  if (t > (initialT + INCREMENT)){
    state = KEEPDRIVING;
    ledYellow(0); 
  }  
}

void stop(){
  if (state != prior_state) {
    ledRed(1);
    initialT = millis();
    prior_state = state;
    leftMotor = OFF;
    rightMotor = OFF;
    m.setM1Speed(rightMotor);
    m.setM2Speed(leftMotor);
    

  }  
}

//============================================================================================
void setup() {
  Serial.begin(115200);
  pinMode(BMP_FR, INPUT_PULLUP);
  pinMode(BMP_R, INPUT_PULLUP); //might have to reassign these to pins that the astar knows
  pinMode(BMP_MR, INPUT_PULLUP);

  pinMode(BMP_ML, INPUT_PULLUP);
  pinMode(BMP_L, INPUT_PULLUP);
  pinMode(BMP_FL, INPUT_PULLUP);

  

  prior_state = NONE;
  state = KEEPDRIVING;

}
//============================================================================================
void loop() {
  
  FR  = digitalRead(BMP_FR) == LOW;
  R = digitalRead(BMP_R)  == LOW;
  MR = digitalRead(BMP_MR) == LOW;
  ML = digitalRead(BMP_ML) == LOW;
  L = digitalRead(BMP_L) == LOW;
  FL = digitalRead(BMP_FL) == LOW;

    switch (state) {
        case KEEPDRIVING:
          keepdriving();
          break;
        case HEADON:
          headon();
          break;

        case AVOIDRIGHT:
          avoidright();
          break;
        case AVOIDLEFT:
          avoidleft();
          break;
          
        case STOP:
          stop();
          break;
        
    }
}
