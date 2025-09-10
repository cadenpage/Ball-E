  
void setup() {
  delay(10000);
    Serial.begin(115200);
    Serial.println("resetting arduino");
 
}   

void loop() {
  Serial.println("<hello from arduino>");
  delay(1000);

}
