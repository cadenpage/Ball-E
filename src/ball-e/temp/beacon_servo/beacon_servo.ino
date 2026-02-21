#include <Arduino.h>
#include <Servo.h>

const uint8_t SERVO_PIN   = 9;    // signal pin for servo
const int SERVO_LEFT_US   = 1000; // adjust to your target angles
const int SERVO_MID_US    = 1500;
const int SERVO_RIGHT_US  = 2000;

Servo s;
char buf[32];
uint8_t idx = 0;
bool inFrame = false;

void setup() {
  Serial.begin(115200);
  s.attach(SERVO_PIN);
  s.writeMicroseconds(SERVO_MID_US); // center on boot
}

void applyCommand(const char *cmd) {
  if (cmd[0] == 'V') {             // raw microseconds
    int us = atoi(cmd + 2);
    us = constrain(us, 500, 2500);
    s.writeMicroseconds(us);
  } else if (cmd[0] == 'P') {      // preset left/mid/right
    int sel = atoi(cmd + 2);
    if (sel == 0) s.writeMicroseconds(SERVO_LEFT_US);
    else if (sel == 1) s.writeMicroseconds(SERVO_MID_US);
    else if (sel == 2) s.writeMicroseconds(SERVO_RIGHT_US);
  }
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '<') { inFrame = true; idx = 0; continue; }
    if (!inFrame) continue;
    if (c == '>') {
      buf[idx] = '\0';
      applyCommand(buf);
      inFrame = false;
      idx = 0;
      continue;
    }
    if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    }
  }
}
