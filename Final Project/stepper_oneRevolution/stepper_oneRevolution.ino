
/*
 Stepper Motor Control - one revolution

 This program drives a unipolar or bipolar stepper motor.
 The motor is attached to digital pins 8 - 11 of the Arduino.

 The motor should revolve one revolution in one direction, then
 one revolution in the other direction.


 Created 11 Mar. 2007
 Modified 30 Nov. 2009
 by Tom Igoe

 */

#include <Stepper.h>
#include <Servo.h>

Servo ftServo;


const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
const float ball = 22.22; //ball evolution 
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 2, 3, 5, 9);

void setup() {
  // set the speed at 60 rpm:
  myStepper.setSpeed(20);
  // initialize the serial port:
  ftServo.attach(13);
  Serial.begin(9600);
}

void loop() {
  // step one revolution  in one direction:
  ftServo.write(0);
  delay(500);
  Serial.println("clockwise");
  myStepper.step(ball);
  
  delay(2000);
  
  ftServo.write(180);
  delay(1000);
  
//
//  // step one revolution in the other direction:
//  Serial.println("counterclockwise");
//  myStepper.step(-stepsPerRevolution);
//  delay(500);
}
