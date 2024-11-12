


const char *outString = "ABCD";

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT); // LED pin
}

void loop() {
  if (Serial.available()) {
    char received = Serial.read();
    if (received == 'a') {
      digitalWrite(2, HIGH);
    }
    if (received == 'b') {
      digitalWrite(2, LOW);
    }
    if ((received >= 'A') && (received <= 'Z')) {
      Serial.write(received);
    }
    if (received == 'c') {
      Serial.println(outString);
    }
  }

delay(unsigned long ms)
  
}
