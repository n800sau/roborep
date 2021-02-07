const int boot_pin = 3;
const int reset_pin = 2;
const int pin1 = 5;
const int pin2 = 6;

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
			digitalWrite(boot_pin, LOW);
			digitalWrite(reset_pin, LOW);
			delay(300);
			digitalWrite(reset_pin, HIGH);
			Serial.println("W mode");
		} else if (incomingByte == 'P') {
			digitalWrite(boot_pin, HIGH);
			digitalWrite(reset_pin, LOW);
			delay(300);
			digitalWrite(reset_pin, HIGH);
			Serial.println("P mode");
		} else if (incomingByte == 'R') {
			digitalWrite(reset_pin, LOW);
			delay(300);
			digitalWrite(reset_pin, HIGH);
			Serial.println("Reset");
		} else if (incomingByte == '1') {
			digitalWrite(pin1, LOW);
			delay(300);
			digitalWrite(pin1, HIGH);
			Serial.println("P1 Reset");
		} else if (incomingByte == '2') {
			digitalWrite(pin2, LOW);
			delay(300);
			digitalWrite(pin2, HIGH);
			Serial.println("P2 Reset");
		} 
	}
}
