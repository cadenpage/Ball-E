// =========================
// Ultrasonic Pin Setup
// =========================

// FRONT Sensor
#define TRIG_FRONT 14
#define ECHO_FRONT 17

// LEFT Sensor
#define TRIG_LEFT 12
#define ECHO_LEFT 1    // TX pin (OK for now)

// RIGHT Sensor
#define TRIG_RIGHT 0   // RX pin (OK for now)
#define ECHO_RIGHT 19

// =========================
// Function: Read ultrasonic sensor distance (cm)
// =========================
float readUltrasonic(int trigPin, int echoPin) {
  // trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(4);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // wait for echo, 30 ms timeout
  long duration = pulseIn(echoPin, HIGH, 30000);  

  if (duration == 0) return -1;  // No echo detected

  return (duration * 0.0343f) / 2.0f;
}

// =========================
// Setup
// =========================
void setup() {
  Serial.begin(115200);

  // Set sensor pins
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  Serial.println("=== Ultrasonic Test Start ===");
  delay(500);
}

// =========================
// Loop
// =========================
void loop() {
  float frontDist = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  float leftDist  = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
  float rightDist = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  Serial.print("Front (cm): ");
  if (frontDist < 0) Serial.print("No Echo");
  else Serial.print(frontDist);

  Serial.print("   |   Left (cm): ");
  if (leftDist < 0) Serial.print("No Echo");
  else Serial.print(leftDist);

  Serial.print("   |   Right (cm): ");
  if (rightDist < 0) Serial.print("No Echo");
  else Serial.print(rightDist);

  Serial.println();

  delay(250);
}
