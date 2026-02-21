
#include <Arduino.h>
// One Ultrasonic Sensor Test
// Trig = 14, Echo = 17 (A-Star 32U4 digital pins)

#define TRIG_PIN 14
#define ECHO_PIN 17

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.println("Ultrasonic Test Ready");
}

float readUltrasonic() {
  // Ensure trigger is low
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(4);

  // 10 µs pulse to trigger
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo time in microseconds
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30 ms timeout

  // Convert to distance (cm)
  if (duration == 0) {
    return -1.0; // no reading
  }

  float distance_cm = duration * 0.0343 / 2.0;
  return distance_cm;
}

void loop() {
  float d = readUltrasonic();

  if (d < 0) {
    Serial.println("No reading (timeout)");
  } else {
    Serial.print("Distance: ");
    Serial.print(d);
    Serial.println(" cm");
  }

  delay(250);
}
