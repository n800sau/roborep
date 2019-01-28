const int io11 = 11;
const int io12 = 12;

int io11mode = INPUT;
int io12mode = INPUT;

// the setup routine runs once when you press reset:
void setup() {
	Serial.begin(115200);

	digitalWrite(io11, LOW);
	digitalWrite(io12, LOW);
	// initialize the digital pin as input (work mode)
	pinMode(io11, io11mode);
	pinMode(io12, io12mode);
	Serial.println("Ready");
}

void loop() {
	// keep them low so they never become +5v
	digitalWrite(io11, LOW);
	digitalWrite(io12, LOW);
	// see if there's incoming serial data:
	if (Serial.available() > 0) {
		// read the oldest byte in the serial buffer:
		int incomingByte = Serial.read();
		if (incomingByte == '1' || incomingByte == '0') {
			io11mode = OUTPUT;
			pinMode(io11, io11mode);
		} 
		if (incomingByte == '2' || incomingByte == '0') {
			io12mode = OUTPUT;
			pinMode(io12, io12mode);
		} 
		Serial.println(String("io11 is in ") + ((io11mode == INPUT) ? "work mode" : "programming mode"));
		Serial.println(String("io12 is in ") + ((io12mode == INPUT) ? "work mode" : "programming mode"));
	}
}
