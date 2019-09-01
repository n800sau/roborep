
const uint8_t oline[] = {5, 6, 7};
const uint8_t iline[] = {8, 9, 10};

void setup()
{

	for(byte i=0; i<sizeof(oline); i++) {
		pinMode(oline[i], OUTPUT);
		digitalWrite(oline[i], HIGH);
	}

	for(byte i=0; i<sizeof(iline); i++) {
		pinMode(iline[i], INPUT_PULLUP);
	}

	Serial.begin(115200);
	Serial.println("\nButton test");
}


void loop()
{
	byte key = 0;
	for(byte i=0; i<sizeof(oline) && key == 0; i++) {
		digitalWrite(oline[i], LOW);
		for(byte j=0; j<sizeof(iline); j++) {
			if(digitalRead(iline[j]) == LOW) {
				key = i<<4 | j;
				break;
			}
		}
		digitalWrite(oline[i], HIGH);
	}
	if(key != 0) {
		Serial.println(key, HEX);
	}

}
