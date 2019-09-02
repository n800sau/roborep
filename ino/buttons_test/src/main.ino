
const uint8_t oline[] = {11, 12, 13};
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

byte last_key = 0;
unsigned long  last_change_millis = 0;
const unsigned long delta_millis = 500;

byte read_key()
{
	byte rs = 0;
	for(byte i=0; i<sizeof(oline) && rs == 0; i++) {
		digitalWrite(oline[i], LOW);
		for(byte j=0; j<sizeof(iline); j++) {
			if(digitalRead(iline[j]) == LOW) {
				rs = (i+1)<<4 | (j+1);
				break;
			}
		}
		digitalWrite(oline[i], HIGH);
	}
	return rs;
}

void loop()
{
	byte key = read_key();
	if(last_key != key) {
		unsigned long m = millis();
		if(key == 0 && delta_millis + last_change_millis > m) {
			key = last_key;
			last_change_millis = m;
		} else {
			last_key = key;
		}
		Serial.println(key, HEX);
	}
}
