#include <Arduino.h>

int sensorPin = A0; //potentiometer analog signal is wire to pin A0
int sensorValue = 0;
int button_counter = 0;
int button_Pin = 14; //Button is considered gpio 14
int buttonState = 0;
int lastButtonState = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT); //Sets the built in LED mode to output
  Serial.begin(115200); //Initializs serial signal rate
  digitalWrite(LED_BUILTIN, HIGH); //Starts off with LED on

}

void loop() {
  buttonState = digitalRead(button_Pin); //Starts off loop with reading button pin (pressed = HIGH, not pressed = LOW)
  if (buttonState != lastButtonState) //If it is different from the last signal (did someone press or release it)
    if (buttonState == HIGH) //If someone did press it
      button_counter ++; //Add 1 to the button counter
    lastButtonState = buttonState; //Update the last button state to track if its been pressed or released
  if (button_counter >=2) //if the button has been pressed twice
    sensorValue = analogRead(sensorPin); //Read analog value from potentiometer
    digitalWrite(LED_BUILTIN,HIGH); //Turn on LED
    delay(sensorValue); //wait depending on sensor value
    Serial.println(sensorValue); //Show sensor value so we know potentiometer is doing its job

    digitalWrite(LED_BUILTIN, LOW);//Turn off LED
    delay(sensorValue); //wait depending on sensor value. Larger value, slower flashing frequency

}
