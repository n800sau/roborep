const int boot_pin = 11;
const int reset_pin = 12;

// the setup routine runs once when you press reset:
void setup() {
	Serial.begin(115200);

	// initialize the digital pin as input (work mode)
	pinMode(boot_pin, OUTPUT);
	pinMode(reset_pin, OUTPUT);
	digitalWrite(boot_pin, HIGH);
	digitalWrite(reset_pin, HIGH);
	Serial.println("Ready");
}

void loop() {
	// see if there's incoming serial data:
	if (Serial.available() > 0) {
		// read the oldest byte in the serial buffer:
		int incomingByte = Serial.read();
		if (incomingByte == 'W') {
			pinMode(boot_pin, LOW);
			pinMode(reset_pin, LOW);
			delay(100);
			pinMode(reset_pin, HIGH);
			Serial.println("W mode");
		} else if (incomingByte == 'P') {
			pinMode(boot_pin, HIGH);
			pinMode(reset_pin, LOW);
			delay(100);
			pinMode(reset_pin, HIGH);
			Serial.println("P mode");
		} 
	}
}
