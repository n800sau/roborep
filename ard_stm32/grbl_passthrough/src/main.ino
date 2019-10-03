
int inByte;

void setup()
{
	Serial.begin(115200); // Ignored by Maple. But needed by boards using hardware serial via a USB to Serial adaptor
	Serial1.begin(115200);
}

void loop()
{
	while(Serial.available() || Serial1.available()) {
		if(Serial1.available()) {
			inByte = Serial1.read();
			Serial.write(inByte);
		}
		if(Serial.available()) {
			inByte = Serial.read();
			Serial1.write(inByte);
		}
	}
}
