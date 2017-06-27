#define BAUD_RATE 9600

#define RESET_PIN 10
#define OutSerial Serial3

void setup()
{
	pinMode(RESET_PIN, OUTPUT);
	Serial.begin(115200);
	OutSerial.begin(BAUD_RATE);
	digitalWrite(RESET_PIN, LOW);
	delay(500);
	digitalWrite(RESET_PIN, HIGH);
	delay(500);
	int n = OutSerial.available();
	for(int i=0; i< n; i++) {
		OutSerial.read();
	}
	Serial.println("Ready ...");
}

void loop()
{
	int n = OutSerial.available();
	for(int i=0; i< n; i++) {
		Serial.write(OutSerial.read());
	}
//	while(Serial.available()) OutSerial.write(Serial.read());
}
