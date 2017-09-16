const int RESET_PIN = 8;
const int PROGRAM_PIN = A5;

// the setup routine runs once when you press reset:
void setup() {
	Serial.begin(115200);

	pinMode(RESET_PIN, OUTPUT);
	digitalWrite(RESET_PIN, HIGH);
	pinMode(PROGRAM_PIN, OUTPUT);
	digitalWrite(PROGRAM_PIN, LOW);
	// initialize the digital pin as input (work mode)
	Serial.println("1 - Start programming");
	Serial.println("0 - Stop programming");
	Serial.println("Ready");
}

void reset_stm32()
{
	Serial.println("Reset");
	digitalWrite(RESET_PIN, LOW);
	delay(200);
	digitalWrite(RESET_PIN, HIGH);
}

void loop()
{
	// see if there's incoming serial data:
	if (Serial.available() > 0) {
		// read the oldest byte in the serial buffer:
		int incomingByte = Serial.read();
		if (incomingByte == '1') {
			Serial.println("Programming mode");
			digitalWrite(PROGRAM_PIN, HIGH);
			reset_stm32();
		} else if (incomingByte == '0') {
			Serial.println("Work mode");
			digitalWrite(PROGRAM_PIN, LOW);
			reset_stm32();
		} else if (incomingByte == 'R') {
			reset_stm32();
		} 
	}
}
