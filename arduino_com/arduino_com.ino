
void setup() {
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming byte
    String message = Serial.readString();
    //Print the received message back to the serial monitor
    Serial.print("Received: ");
    Serial.println(message);

    
    
  }
}