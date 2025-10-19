void setup() {
  Serial.begin(115200);       // For output to your PC via USB
  Serial1.begin(115200);      // For communication with the XBee (using pins RX1=19, TX1=18)
  Serial.println("Ready to receive from Xbee");
}

void loop() {
  static String line = "";  // Stores one line of incoming text

  while (Serial1.available()) {  // While there's data from the XBee
    char c = Serial1.read();     // Read one character

    if (c == '\n') {             // If it's a newline (end of line)
      unsigned long t = millis();      // Get the current time in milliseconds
      Serial.print(t);                // Print the timestamp
      Serial.print("\t");             // Separator
      Serial.println(line);           // Print the full received line
      line = "";                      // Reset the line for the next one
    } else if (c != '\r') {     // If it's not a carriage return, add to line
      line += c;                // Build up the line character by character
    }
  }
}
