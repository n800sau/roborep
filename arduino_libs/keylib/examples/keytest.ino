#include <keylib.h>

const uint8_t oline[] = {PA15, PB3, PB4};
const uint8_t iline[] = {PB5, PB6, PB7};
readKey keylib;

void setup()
{
	keylib.begin(sizeof(oline), sizeof(iline), oline, iline);
}

void loop()
{
	static byte last_key = 0;
	byte key = keylib.read_key_debounce();
	if(last_key != key) {
		Serial.println(key, HEX);
		last_key = key;
	}
}
