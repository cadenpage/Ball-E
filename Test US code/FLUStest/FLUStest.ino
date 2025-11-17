// =========================
// Ultrasonic Pin Setup
// =========================

// FRONT ultrasonic sensor
#define TRIG_FRONT 4
#define ECHO_FRONT 7

// LEFT ultrasonic sensor
#define TRIG_LEFT 12
#define ECHO_LEFT 1   // NOTE: TX pin on some boards, OK for testing

// =========================
// Function: Read ultrasonic distance in cm
// =========================
float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(4);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);  // 30 ms timeout

  if (duration == 0) return -1;  // failed reading

  return (duration * 0.0343f) / 2.0f;  // convert to cm
}

// =========================
// Setup
// =========================
void setup() {
  Serial.begin(115200);

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  Serial.println("Ultrasonic Sensor Test Starting...");
}

// =========================
// Loop
// =========================
void loop() {
  float frontDist = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  float leftDist  = readUltrasonic(TRIG_LEFT, ECHO_LEFT);

  Serial.print("Front (cm): ");
  if (frontDist < 0) Serial.print("No Echo");
  else Serial.print(frontDist);

  Serial.print("   |   Left (cm): ");
  if (leftDist < 0) Serial.print("No Echo");
  else Serial.print(leftDist);

  Serial.println();

  delay(250);
}
