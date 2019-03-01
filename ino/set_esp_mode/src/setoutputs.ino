const int boot_pin = 11;
const int reset_pin = 12;

// the setup routine runs once when you press reset:
void setup() {
	Serial.begin(115200);

	// initialize the digital pin as input (work mode)
	pinMode(boot_pin, INPUT_PULLUP);
	pinMode(reset_pin, INPUT_PULLUP);
	Serial.println("Ready");
}

void loop() {
	// see if there's incoming serial data:
	if (Serial.available() > 0) {
		// read the oldest byte in the serial buffer:
		int incomingByte = Serial.read();
		if (incomingByte == 'W' || incomingByte == 'w') {
			// work mode
			pinMode(boot_pin, INPUT_PULLUP);
//			digitalWrite(boot_pin, HIGH);
			pinMode(reset_pin, OUTPUT);
			digitalWrite(reset_pin, LOW);
			delay(300);
			pinMode(reset_pin, INPUT_PULLUP);
//			digitalWrite(reset_pin, HIGH);
			Serial.println("W mode");
		} else if (incomingByte == 'P' || incomingByte == 'p') {
			// programming mode
			pinMode(boot_pin, OUTPUT);
			digitalWrite(boot_pin, LOW);
			pinMode(reset_pin, OUTPUT);
			digitalWrite(reset_pin, LOW);
			delay(300);
			pinMode(reset_pin, INPUT);
//			digitalWrite(reset_pin, HIGH);
			Serial.println("P mode");
		} else if (incomingByte == 'R' || incomingByte == 'r') {
			pinMode(reset_pin, OUTPUT);
			digitalWrite(reset_pin, LOW);
			delay(300);
			pinMode(reset_pin, INPUT_PULLUP);
			digitalWrite(reset_pin, HIGH);
			Serial.println("Reset");
		} 
	}
}
