void setup()
{
	Serial.begin(115200);       // For output to your PC via USB
	Serial1.begin(115200);      // For communication with the XBee (RX1=19, TX1=18)
	Serial.println("Ready to receive from Xbee");
}

uint8_t simple_checksum(const String &data)
{
	// Sum of all characters mod 256
	uint32_t sum = 0;

	for (size_t i = 0; i < data.length(); i++) {
		sum += (uint8_t)data[i];
	}

	return sum % 256;
}

void loop()
{
	static String line = "";  // Stores one line of incoming text

	while (Serial1.available()) {  // While there's data from the XBee
		char c = Serial1.read();

		if (c == '\n') {  // End of packet
			bool valid = false;
			float vals[3] = {0, 0, 0}; // Store x, y, z

			if (line.length() > 0 && line[0] == '$') {
				int starIndex = line.indexOf('*');

				if (starIndex > 0) {
					String dataPart = line.substring(1, starIndex);       // Exclude $
					String checksumPart = line.substring(starIndex + 1);  // After *

					// Convert checksum string to integer
					int receivedChecksum = checksumPart.toInt();
					int computedChecksum = simple_checksum(dataPart);

					if (receivedChecksum == computedChecksum) {
						valid = true;

						// Split dataPart (tab-delimited) into floats
						int idx = 0;
						int start = 0;

						for (int i = 0; i <= dataPart.length(); i++) {
							if (i == dataPart.length() || dataPart[i] == '\t') {
								String valStr = dataPart.substring(start, i);

								if (idx < 3) { vals[idx] = valStr.toFloat(); }

								idx++;
								start = i + 1;
							}
						}
					}
				}
			}

			// Print time in milliseconds
			Serial.print(millis());
			Serial.print("\t");

			// Print the floats followed by valid flag
			for (int i = 0; i < 3; i++) {
				Serial.print(vals[i], 3); // Print with 3 decimal places
				Serial.print("\t");
			}

			Serial.println(valid ? 1 : 0);

			line = "";  // Reset for next packet

		} else if (c != '\r') {
			line += c;  // Build the line
		}
	}
}
