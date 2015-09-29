const int io8 = 8;
const int io9 = 9;

int io8mode = INPUT;
int io9mode = INPUT;

// the setup routine runs once when you press reset:
void setup() {
	Serial.begin(9600);

	digitalWrite(io8, LOW);
	digitalWrite(io9, LOW);
	// initialize the digital pin as input (work mode)
	pinMode(io8, io8mode);
	pinMode(io9, io9mode);
	Serial.println("Ready");
}

void loop() {
	// keep them low so they never become +5v
	digitalWrite(io8, LOW);
	digitalWrite(io9, LOW);
	// see if there's incoming serial data:
	if (Serial.available() > 0) {
		// read the oldest byte in the serial buffer:
		int incomingByte = Serial.read();
		if (incomingByte == '8' || incomingByte == '0') {
			io8mode = OUTPUT;
			pinMode(io8, io8mode);
		} 
		if (incomingByte == '9' || incomingByte == '0') {
			io9mode = OUTPUT;
			pinMode(io9, io9mode);
		} 
		Serial.println(String("io8 is in ") + ((io8mode == INPUT) ? "work mode" : "programming mode"));
		Serial.println(String("io9 is in ") + ((io9mode == INPUT) ? "work mode" : "programming mode"));
	}
}
